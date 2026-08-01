#include "control.h"
#include "app_config.h"
#include "eight_ir.h"
#include "encoder.h"
#include "icm20948.h"
#include "moto.h"
#include "pid.h"
#include "ssd1306.h"
#include "ti_msp_dl_config.h"
#include "uart_bt.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define OLED_PERIOD_MS        (125UL)
#define IMU_PERIOD_MS         (20UL)
#define AUTO_START_RAMP_MS    (2000UL)
#define CURVE_BASE_PERCENT    (82.0f)
#define CURVE_YAW_RATE_DPS    (12.0f)
#define FINISH_BRAKE_HOLD_MS  (160UL)
#define LAP_YAW_ARM_DEG       (330.0f)
#define LAP_YAW_SLOW_DEG      (340.0f)
#define LAP_YAW_NOMINAL_DEG   (360.0f)
#define LAP_YAW_FALLBACK_DEG  (370.0f)
#define FINISH_BASE_PERCENT   (65.0f)
#define START_PATTERN_YAW_GATE_DEG (350.0f)
#define LOST_LINE_HOLD_PERCENT (55)

static Control_State g_control;
static PID_Controller g_line_pid;
static PID_Controller g_speed_pid;
static PID_Controller g_yaw_pid;
static IMU_Data g_imu;
static int16_t g_diag_power;
static int16_t g_manual_left;
static int16_t g_manual_right;
static int16_t g_last_track_left;
static int16_t g_last_track_right;
static uint32_t g_last_control_ms;
static uint32_t g_last_oled_ms;
static uint32_t g_last_oled_recovery_ms;
static uint32_t g_last_imu_ms;
static uint32_t g_imu_period_ms;
static uint32_t g_line_lost_ms;
static uint32_t g_marker_clear_ms;
static uint32_t g_auto_elapsed_ms;
static uint32_t g_finish_brake_start_ms;
static float g_lap_yaw_accum_deg;
static float g_lap_yaw_last_deg;
static uint8_t g_start_marker_armed;
static uint8_t g_finish_braking;
static uint8_t g_lap_yaw_tracking;
static uint8_t g_finish_yaw_armed;
static uint8_t g_start_ir_raw;
static uint8_t g_start_ir_active_count;
static uint8_t g_start_ir_valid;
static uint8_t g_button_last_raw;
static uint8_t g_button_stable;
static uint32_t g_button_change_ms;
static char g_last_command[22];

static const char *mode_name(Control_Mode mode)
{
    switch (mode) {
        case CONTROL_SAFE:
            return "SAFE";
        case CONTROL_DIAGNOSTIC:
            return "DIAG";
        case CONTROL_AUTO_TRACK:
            return "AUTO";
        case CONTROL_FAULT:
            return "FAULT";
        default:
            return "?";
    }
}

static int16_t configured_pwm_limit(void)
{
    if (g_control_tuning.pwm_limit_permille < 0) {
        return 0;
    }
    if (g_control_tuning.pwm_limit_permille > 550) {
        return 550;
    }
    return g_control_tuning.pwm_limit_permille;
}

static int16_t configured_pwm_value(int16_t value)
{
    int16_t limit = configured_pwm_limit();

    if (value < 0) {
        return 0;
    }
    if (value > limit) {
        return limit;
    }
    return value;
}

static uint8_t configured_marker_count(void)
{
    if (g_control_tuning.marker_active_count < 1U) {
        return 1U;
    }
    if (g_control_tuning.marker_active_count > 8U) {
        return 8U;
    }
    return g_control_tuning.marker_active_count;
}

static uint8_t bit_count8(uint8_t value)
{
    uint8_t count = 0U;

    while (value != 0U) {
        count = (uint8_t) (count + (value & 1U));
        value >>= 1U;
    }
    return count;
}

static uint8_t start_marker_detected(const EightIR_State *ir)
{
    uint8_t different_bits;
    uint8_t active_difference;
    uint8_t tolerance;

    if (g_start_ir_valid == 0U) {
        return (ir->active_count >= configured_marker_count()) ? 1U : 0U;
    }
    if (g_start_ir_active_count >= 3U && ir->active_count < 3U) {
        return 0U;
    }

    different_bits = bit_count8((uint8_t) (ir->raw ^ g_start_ir_raw));
    active_difference = (ir->active_count >= g_start_ir_active_count) ?
        (uint8_t) (ir->active_count - g_start_ir_active_count) :
        (uint8_t) (g_start_ir_active_count - ir->active_count);
    tolerance = (g_start_ir_active_count >= 5U) ? 2U :
        (g_start_ir_active_count >= 3U) ? 1U : 0U;

    return (different_bits <= tolerance &&
            active_difference <= tolerance) ? 1U : 0U;
}

static uint8_t start_pattern_finish_allowed(void)
{
    /* A pattern containing at least three black sensors is marker-like. */
    if (g_start_ir_active_count >= 3U) {
        return 1U;
    }
    /* Ordinary one/two-sensor line patterns repeat, so require a full turn. */
    return (g_finish_yaw_armed != 0U &&
            fabsf(g_lap_yaw_accum_deg) >=
                START_PATTERN_YAW_GATE_DEG) ? 1U : 0U;
}

static uint8_t lap_yaw_is_usable(void)
{
    return (g_imu.valid != 0U && g_imu.stale == 0U &&
            g_imu.calibrated != 0U && isfinite(g_imu.yaw_deg)) ? 1U : 0U;
}

static void update_lap_yaw(void)
{
    float delta;

    if (lap_yaw_is_usable() == 0U) {
        return;
    }
    if (g_lap_yaw_tracking == 0U) {
        g_lap_yaw_last_deg = g_imu.yaw_deg;
        g_lap_yaw_tracking = 1U;
        return;
    }

    delta = g_imu.yaw_deg - g_lap_yaw_last_deg;
    if (delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }
    g_lap_yaw_last_deg = g_imu.yaw_deg;

    /* Reject a resumed/stale sample that cannot be a 20 ms vehicle turn. */
    if (!isfinite(delta) || fabsf(delta) > 30.0f) {
        return;
    }
    g_lap_yaw_accum_deg += delta;
    if (!isfinite(g_lap_yaw_accum_deg)) {
        g_lap_yaw_accum_deg = 0.0f;
        g_lap_yaw_tracking = 0U;
        g_finish_yaw_armed = 0U;
        return;
    }
    if (fabsf(g_lap_yaw_accum_deg) >= LAP_YAW_ARM_DEG) {
        g_finish_yaw_armed = 1U;
    }
}

static void enter_safe(const char *reason, uint32_t now_ms)
{
    g_finish_braking = 0U;
    Moto_SetSafetyPermit(0U);
    Moto_EmergencyStop();
    PID_Reset(&g_line_pid);
    PID_Reset(&g_speed_pid);
    PID_Reset(&g_yaw_pid);
    g_control.mode = CONTROL_SAFE;
    g_control.left_command = 0;
    g_control.right_command = 0;
    g_control.mode_enter_ms = now_ms;
    g_manual_left = 0;
    g_manual_right = 0;
    g_last_track_left = 0;
    g_last_track_right = 0;
    if (reason != NULL) {
        (void) snprintf(
            g_last_command, sizeof(g_last_command), "%s", reason);
    }
}

static const char *motion_lock_reason(void)
{
    Moto_State motor = Moto_GetState();

    if (motor.hardware_locked != 0U) {
        return "LOCK:HW";
    }
    return NULL;
}

static void enter_diagnostic(uint32_t now_ms)
{
    const char *lock_reason = motion_lock_reason();

    if (lock_reason != NULL) {
        enter_safe(lock_reason, now_ms);
        return;
    }
    Moto_SetSafetyPermit(1U);
    g_control.mode = CONTROL_DIAGNOSTIC;
    g_control.mode_enter_ms = now_ms;
    g_manual_left = 0;
    g_manual_right = 0;
    (void) snprintf(g_last_command, sizeof(g_last_command), "DIAG READY");
}

static void enter_auto(uint32_t now_ms)
{
    EightIR_State ir = EightIR_GetState();
    const char *lock_reason = motion_lock_reason();

    if (lock_reason != NULL) {
        enter_safe(lock_reason, now_ms);
        return;
    }
    if (ir.frame_fresh == 0U || ir.active_count == 0U) {
        enter_safe("AUTO:NO LINE", now_ms);
        return;
    }
    Moto_SetSafetyPermit(1U);
    PID_Reset(&g_line_pid);
    PID_Reset(&g_speed_pid);
    PID_Reset(&g_yaw_pid);
    g_control.mode = CONTROL_AUTO_TRACK;
    g_control.mode_enter_ms = now_ms;
    g_control.lap_count = 0U;
    g_line_lost_ms = 0U;
    g_marker_clear_ms = 0U;
    g_auto_elapsed_ms = 0U;
    g_start_marker_armed = 0U;
    g_finish_braking = 0U;
    g_finish_brake_start_ms = 0U;
    g_last_track_left = 0;
    g_last_track_right = 0;
    g_start_ir_raw = ir.raw;
    g_start_ir_active_count = ir.active_count;
    g_start_ir_valid = 1U;
    g_lap_yaw_accum_deg = 0.0f;
    g_finish_yaw_armed = 0U;
    if (lap_yaw_is_usable() != 0U) {
        g_lap_yaw_last_deg = g_imu.yaw_deg;
        g_lap_yaw_tracking = 1U;
    } else {
        g_lap_yaw_last_deg = 0.0f;
        g_lap_yaw_tracking = 0U;
    }
    (void) snprintf(g_last_command, sizeof(g_last_command),
        "AUTO S:%02X N:%u", g_start_ir_raw, g_start_ir_active_count);
}

static void begin_finish_brake(const char *label, uint32_t now_ms)
{
    g_control.lap_count = 1U;
    g_control.left_command = 0;
    g_control.right_command = 0;
    g_finish_braking = 1U;
    g_finish_brake_start_ms = now_ms;
    PID_Reset(&g_line_pid);
    PID_Reset(&g_speed_pid);
    PID_Reset(&g_yaw_pid);
    (void) snprintf(g_last_command, sizeof(g_last_command), "%s", label);
    Moto_ActiveBrake();
}

static void set_manual_motion(
    int16_t left, int16_t right, const char *label, uint32_t now_ms)
{
    if (g_control.mode != CONTROL_DIAGNOSTIC) {
        enter_diagnostic(now_ms);
    }
    if (g_control.mode != CONTROL_DIAGNOSTIC) {
        g_manual_left = 0;
        g_manual_right = 0;
        return;
    }
    g_manual_left = left;
    g_manual_right = right;
    g_control.mode_enter_ms = now_ms;
    (void) snprintf(g_last_command, sizeof(g_last_command), "%s", label);
}

static void refresh_manual_power(void)
{
    if (g_manual_left > 0) {
        g_manual_left = g_diag_power;
    } else if (g_manual_left < 0) {
        g_manual_left = (int16_t) -g_diag_power;
    }
    if (g_manual_right > 0) {
        g_manual_right = g_diag_power;
    } else if (g_manual_right < 0) {
        g_manual_right = (int16_t) -g_diag_power;
    }
}

static void handle_command(uint8_t byte, uint32_t now_ms)
{
    uint8_t send_ack = 1U;

    switch (byte) {
        case 'D':
        case 'd':
            enter_diagnostic(now_ms);
            break;
        case 'A':
        case 'a':
            enter_auto(now_ms);
            break;
        case 'S':
        case 's':
            enter_safe("BT STOP", now_ms);
            break;
        case 'F':
        case 'f':
            set_manual_motion(
                g_diag_power, g_diag_power, "FWD", now_ms);
            break;
        case 'B':
        case 'b':
            set_manual_motion((int16_t) -g_diag_power,
                (int16_t) -g_diag_power, "BACK", now_ms);
            break;
        case 'L':
        case 'l':
            set_manual_motion((int16_t) -g_diag_power,
                g_diag_power, "SPIN LEFT", now_ms);
            break;
        case 'R':
        case 'r':
            set_manual_motion(g_diag_power,
                (int16_t) -g_diag_power, "SPIN RIGHT", now_ms);
            break;
        case '+':
            if (g_diag_power < configured_pwm_limit() - 50) {
                g_diag_power += 50;
            } else {
                g_diag_power = configured_pwm_limit();
            }
            refresh_manual_power();
            (void) snprintf(g_last_command, sizeof(g_last_command),
                "POWER %d", g_diag_power);
            break;
        case '-':
            if (g_diag_power > 100) {
                g_diag_power -= 50;
            }
            refresh_manual_power();
            (void) snprintf(g_last_command, sizeof(g_last_command),
                "POWER %d", g_diag_power);
            break;
        case 'I':
        case 'i':
            EightIR_SetReversed(
                (EightIR_IsReversed() == 0U) ? 1U : 0U);
            (void) snprintf(g_last_command, sizeof(g_last_command),
                "IR REV %u", EightIR_IsReversed());
            break;
        case 'C':
        case 'c':
            EightIR_StartCalibrate();
            enter_safe("IR CAL", now_ms);
            break;
        case 'G':
        case 'g':
#if APP_ENABLE_IMU
            enter_safe("GYRO CAL", now_ms);
            ICM20948_StartCalibration();
            g_imu = ICM20948_GetLast();
#else
            (void) snprintf(g_last_command, sizeof(g_last_command), "IMU OFF");
#endif
            break;
        case '?':
            UartBT_WriteStr(
                "\r\nA:auto D:diag F/B/L/R S:stop +/-:power "
                "I:IR-reverse C:IR-cal G:gyro-cal\r\n");
            send_ack = 0U;
            break;
        default:
            send_ack = 0U;
            break;
    }
    if (send_ack != 0U) {
        UartBT_Printf("\r\nACK %c %s MODE:%s\r\n",
            (char) byte, g_last_command, mode_name(g_control.mode));
    }
}

static void service_bluetooth(uint32_t now_ms)
{
    uint8_t byte;
    while (UartBT_Read(&byte)) {
        handle_command(byte, now_ms);
    }
}

static void service_button(uint32_t now_ms)
{
    uint8_t raw =
        (DL_GPIO_readPins(KEY_PORT, KEY_START_PIN) == 0U) ? 0U : 1U;

    if (raw != g_button_last_raw) {
        g_button_last_raw = raw;
        g_button_change_ms = now_ms;
    }
    if ((uint32_t) (now_ms - g_button_change_ms) >= 30U &&
        raw != g_button_stable) {
        g_button_stable = raw;
        if (raw == 0U) {
            if (g_control.mode == CONTROL_AUTO_TRACK) {
                enter_safe("KEY STOP", now_ms);
            } else {
                enter_auto(now_ms);
            }
        }
    }
}

static void run_auto(uint32_t now_ms)
{
    EightIR_State ir = EightIR_GetState();
    Encoder_State encoder = Encoder_GetState();
    float correction;
    float yaw_assist = 0.0f;
    float desired_yaw_rate;
    float configured_base = (float) configured_pwm_value(
        g_control_tuning.auto_base_pwm_permille);
    float base;
    float ramp_ratio = 1.0f;
    float line_error;
    float effective_speed_target;
    float speed_feedback;
    uint8_t turning;
    int16_t output_limit = configured_pwm_limit();
    int16_t left;
    int16_t right;

    g_auto_elapsed_ms = (uint32_t) (now_ms - g_control.mode_enter_ms);
    update_lap_yaw();

    if (g_finish_braking != 0U) {
        uint32_t brake_elapsed_ms =
            (uint32_t) (now_ms - g_finish_brake_start_ms);

        if (brake_elapsed_ms >= FINISH_BRAKE_HOLD_MS) {
            enter_safe("LAP DONE", now_ms);
        } else {
            Moto_ActiveBrake();
        }
        return;
    }

    if (g_auto_elapsed_ms > g_control_tuning.auto_timeout_ms) {
        enter_safe("AUTO TIMEOUT", now_ms);
        return;
    }
    if (ir.frame_fresh == 0U) {
        enter_safe("IR STALE", now_ms);
        return;
    }

    if (ir.active_count == 0U) {
        uint32_t lost_elapsed_ms;

        if (g_line_lost_ms == 0U) {
            g_line_lost_ms = now_ms;
        }
        lost_elapsed_ms = (uint32_t) (now_ms - g_line_lost_ms);

        /* Brief gaps keep the last steering direction at reduced power. */
        if (g_last_track_left > 0 || g_last_track_right > 0) {
            left = (int16_t) ((int32_t) g_last_track_left *
                LOST_LINE_HOLD_PERCENT / 100);
            right = (int16_t) ((int32_t) g_last_track_right *
                LOST_LINE_HOLD_PERCENT / 100);
            g_control.left_command = left;
            g_control.right_command = right;
            Moto_SetLR(left, right);
        } else {
            Moto_EmergencyStop();
        }
        if (lost_elapsed_ms >= g_control_tuning.line_lost_stop_ms) {
            enter_safe("LINE LOST", now_ms);
        }
        return;
    }
    g_line_lost_ms = 0U;

    /*
     * A is a transverse black start/stop line.  Do not accept it until the
     * car has clearly left the starting marker; accept one frame on return.
     */
    if (g_start_marker_armed == 0U) {
        if (start_marker_detected(&ir) == 0U) {
            if (g_marker_clear_ms == 0U) {
                g_marker_clear_ms = now_ms;
            } else if ((uint32_t) (now_ms - g_marker_clear_ms) >=
                       g_control_tuning.marker_clear_ms) {
                g_start_marker_armed = 1U;
            }
        } else {
            g_marker_clear_ms = 0U;
        }
    } else if (g_auto_elapsed_ms >= g_control_tuning.marker_min_lap_ms &&
               start_pattern_finish_allowed() != 0U &&
               start_marker_detected(&ir) != 0U) {
        /* Stop when the saved pre-start sensor pattern returns. */
        begin_finish_brake("LINE BRAKE", now_ms);
        return;
    }

    if (g_start_marker_armed != 0U && g_finish_yaw_armed != 0U &&
        fabsf(g_lap_yaw_accum_deg) >= LAP_YAW_FALLBACK_DEG) {
        /* The gyro is a bounded fallback if the transverse marker is missed. */
        begin_finish_brake("GYRO BRAKE", now_ms);
        return;
    }

    /*
     * Start both wheels from zero for two seconds.  Scaling the steering at
     * the same time prevents the car from twisting away from line A before
     * it has acquired forward motion.
     */
    if (g_auto_elapsed_ms < AUTO_START_RAMP_MS) {
        ramp_ratio = (float) g_auto_elapsed_ms /
                     (float) AUTO_START_RAMP_MS;
    }

    /* A single one of the two centre detectors is still "centred". */
    line_error = (float) ir.position;
    if (fabsf(line_error) <= 1.0f) {
        line_error = 0.0f;
    }

    turning = (fabsf(line_error) >= 2.0f) ? 1U : 0U;
    if (g_imu.valid != 0U && g_imu.stale == 0U &&
        g_imu.calibrated != 0U && isfinite(g_imu.yaw_rate_dps) &&
        fabsf(g_imu.yaw_rate_dps) >= CURVE_YAW_RATE_DPS) {
        turning = 1U;
    }

    base = configured_base * ramp_ratio;
    if (turning != 0U) {
        /*
         * On this clockwise oval the encoded right wheel is the inner wheel
         * in both semicircles.  Closing the speed loop there would increase
         * the inner-wheel PWM and fight the requested turn, so curves use a
         * slightly lower feed-forward speed instead.
         */
        base = configured_base * (CURVE_BASE_PERCENT / 100.0f) * ramp_ratio;
        PID_Reset(&g_speed_pid);
    } else if (g_control.speed_target_mm_s > 0 && encoder.valid != 0U &&
               (uint32_t) (now_ms - encoder.sample_ms) <= 30U) {
        speed_feedback = fabsf((float) encoder.speed_mm_s);
        effective_speed_target =
            (float) g_control.speed_target_mm_s * ramp_ratio;
        base += PID_Update(&g_speed_pid,
            effective_speed_target - speed_feedback, 0.010f);
    } else {
        PID_Reset(&g_speed_pid);
    }

    if (g_finish_yaw_armed != 0U &&
        fabsf(g_lap_yaw_accum_deg) >= LAP_YAW_SLOW_DEG) {
        float yaw_progress = fabsf(g_lap_yaw_accum_deg);
        float finish_ratio =
            (yaw_progress - LAP_YAW_SLOW_DEG) /
            (LAP_YAW_NOMINAL_DEG - LAP_YAW_SLOW_DEG);
        float finish_percent;
        float finish_base_limit;

        if (finish_ratio > 1.0f) finish_ratio = 1.0f;
        if (finish_ratio < 0.0f) finish_ratio = 0.0f;
        finish_percent = CURVE_BASE_PERCENT -
            (CURVE_BASE_PERCENT - FINISH_BASE_PERCENT) * finish_ratio;
        finish_base_limit = configured_base * finish_percent / 100.0f;
        if (base > finish_base_limit) {
            base = finish_base_limit;
        }
        PID_Reset(&g_speed_pid);
    }

    correction =
        PID_Update(&g_line_pid, line_error, 0.010f);

    /*
     * The line position requests a turn rate. The filtered Z gyro closes a
     * second loop around that request, damping oscillation without relying
     * on the drifting absolute yaw angle.
     */
    if (fabsf(line_error) >= 2.0f &&
        g_imu.valid != 0U && g_imu.stale == 0U &&
        g_imu.calibrated != 0U && isfinite(g_imu.yaw_rate_dps)) {
        desired_yaw_rate =
            line_error * g_control_tuning.yaw_rate_per_position;
        yaw_assist = PID_Update(&g_yaw_pid,
            desired_yaw_rate - g_imu.yaw_rate_dps, 0.010f);
    } else {
        PID_Reset(&g_yaw_pid);
    }
    correction += yaw_assist;
    correction *= ramp_ratio;
    if (correction > (float) output_limit) {
        correction = (float) output_limit;
    }
    if (correction < (float) -output_limit) {
        correction = (float) -output_limit;
    }

    left = (int16_t) (base + correction);
    right = (int16_t) (base - correction);
    if (left > output_limit) left = output_limit;
    if (left < 0) left = 0;
    if (right > output_limit) right = output_limit;
    if (right < 0) right = 0;

    g_control.left_command = left;
    g_control.right_command = right;
    g_last_track_left = left;
    g_last_track_right = right;
    Moto_SetLR(left, right);
}

static void run_diagnostic(uint32_t now_ms)
{
    if ((uint32_t) (now_ms - g_control.mode_enter_ms) >
        g_control_tuning.diagnostic_timeout_ms) {
        enter_safe("DIAG TIMEOUT", now_ms);
        return;
    }
    g_control.left_command = g_manual_left;
    g_control.right_command = g_manual_right;
    Moto_SetLR(g_manual_left, g_manual_right);
}

static void draw_oled(void)
{
    EightIR_State ir = EightIR_GetState();
    Moto_State motor = Moto_GetState();
    Encoder_State encoder = Encoder_GetState();
    char bits[12];
    char track[12];
    char line[22];
    uint8_t channel;
    int32_t yaw_tenths;
    uint32_t yaw_magnitude;
    uint32_t lap_yaw_magnitude;

    bits[0] = 'S';
    bits[1] = 'T';
    bits[2] = ':';
    track[0] = 'L';
    track[1] = 'N';
    track[2] = ':';
    for (channel = 0U; channel < 8U; channel++) {
        bits[3U + channel] = (ir.channels[channel] != 0U) ? '1' : '0';
        track[3U + channel] =
            (ir.channels[channel] == 0U) ? '#' : '.';
    }
    bits[11] = '\0';
    track[11] = '\0';

    SSD1306_Clear();
    SSD1306_ShowString(0U, 0U, "CH:12345678");
    SSD1306_ShowString(1U, 0U, bits);
    SSD1306_ShowString(2U, 0U, track);
    if (g_imu.valid == 0U || g_imu.stale != 0U) {
        (void) snprintf(line, sizeof(line), "YAW:IMU ERR P:%d",
            ir.position);
    } else if (g_imu.calibrated == 0U) {
        (void) snprintf(line, sizeof(line), "YAW:CAL %u/100",
            g_imu.calibration_samples);
    } else {
        yaw_tenths = (int32_t) (g_imu.yaw_deg * 10.0f);
        yaw_magnitude = (yaw_tenths < 0) ?
            (uint32_t) -yaw_tenths : (uint32_t) yaw_tenths;
        (void) snprintf(line, sizeof(line), "YAW:%c%lu.%lu P:%d N:%u",
            (yaw_tenths < 0) ? '-' : '+',
            (unsigned long) (yaw_magnitude / 10U),
            (unsigned long) (yaw_magnitude % 10U),
            ir.position, ir.active_count);
    }
    SSD1306_ShowString(3U, 0U, line);
    (void) snprintf(line, sizeof(line), "KEY:%s", g_last_command);
    SSD1306_ShowString(4U, 0U, line);
    (void) snprintf(line, sizeof(line), "M:%s E:%u L:%d R:%d",
        mode_name(g_control.mode),
        motor.standby_enabled,
        motor.left_permille / 10, motor.right_permille / 10);
    SSD1306_ShowString(5U, 0U, line);
    (void) snprintf(line, sizeof(line), "ENC:%ld S:%d V:%u",
        (long) encoder.total, encoder.speed_mm_s, encoder.valid);
    SSD1306_ShowString(6U, 0U, line);
    lap_yaw_magnitude = (uint32_t) fabsf(g_lap_yaw_accum_deg);
    (void) snprintf(line, sizeof(line), "T:%lu.%02lu Y:%lu%c L:%u",
        (unsigned long) (g_auto_elapsed_ms / 1000U),
        (unsigned long) ((g_auto_elapsed_ms % 1000U) / 10U),
        (unsigned long) lap_yaw_magnitude,
        (g_finish_yaw_armed != 0U) ? '*' : '-',
        g_control.lap_count);
    SSD1306_ShowString(7U, 0U, line);
    (void) SSD1306_Update();
}

void Control_Init(uint32_t now_ms)
{
    memset(&g_control, 0, sizeof(g_control));
    memset(&g_imu, 0, sizeof(g_imu));
    PID_Init(&g_line_pid, g_control_tuning.line_kp,
        g_control_tuning.line_ki, g_control_tuning.line_kd,
        g_control_tuning.line_integral_limit,
        g_control_tuning.line_output_limit,
        g_control_tuning.line_separation_error);
    PID_Init(&g_speed_pid, g_control_tuning.speed_kp,
        g_control_tuning.speed_ki, g_control_tuning.speed_kd,
        g_control_tuning.speed_integral_limit,
        g_control_tuning.speed_output_limit,
        g_control_tuning.speed_separation_error);
    PID_Init(&g_yaw_pid, g_control_tuning.yaw_kp,
        g_control_tuning.yaw_ki, g_control_tuning.yaw_kd,
        g_control_tuning.yaw_integral_limit,
        g_control_tuning.yaw_output_limit,
        g_control_tuning.yaw_separation_error);
    g_control.mode = CONTROL_SAFE;
    g_control.speed_target_mm_s = g_control_tuning.speed_target_mm_s;
    g_control.hardware_locked = (APP_MOTOR_HW_READY == 0U) ? 1U : 0U;
    g_control.mode_enter_ms = now_ms;
    g_diag_power = configured_pwm_value(
        g_control_tuning.diagnostic_pwm_permille);
    g_last_control_ms = now_ms;
    g_last_oled_ms = now_ms;
    g_finish_braking = 0U;
    g_finish_brake_start_ms = 0U;
    g_lap_yaw_accum_deg = 0.0f;
    g_lap_yaw_last_deg = 0.0f;
    g_lap_yaw_tracking = 0U;
    g_finish_yaw_armed = 0U;
    g_last_track_left = 0;
    g_last_track_right = 0;
    g_start_ir_raw = 0xFFU;
    g_start_ir_active_count = 0U;
    g_start_ir_valid = 0U;
    g_last_oled_recovery_ms = now_ms;
    g_last_imu_ms = now_ms;
    g_imu_period_ms = IMU_PERIOD_MS;
    g_button_last_raw = 1U;
    g_button_stable = 1U;
    g_button_change_ms = now_ms;
    (void) snprintf(g_last_command, sizeof(g_last_command), "BOOT SAFE");
    Moto_SetSafetyPermit(0U);
}

void Control_Service(uint32_t now_ms)
{
#if APP_ENABLE_BLUETOOTH
    service_bluetooth(now_ms);
#endif
    service_button(now_ms);

#if APP_ENABLE_IMU
    if ((uint32_t) (now_ms - g_last_imu_ms) >= g_imu_period_ms) {
        g_last_imu_ms = now_ms;
        if (ICM20948_Read(&g_imu, now_ms) != 0U) {
            g_imu_period_ms = IMU_PERIOD_MS;
        } else {
            g_imu_period_ms = 500U;
        }
    }
#endif

    if ((uint32_t) (now_ms - g_last_control_ms) >=
        APP_CONTROL_PERIOD_MS) {
        g_last_control_ms = now_ms;
        Encoder_Sample(now_ms);
        if (g_control.mode == CONTROL_AUTO_TRACK) {
            run_auto(now_ms);
        } else if (g_control.mode == CONTROL_DIAGNOSTIC) {
            run_diagnostic(now_ms);
        } else {
            Moto_EmergencyStop();
        }
    }

#if APP_ENABLE_OLED
    if ((uint32_t) (now_ms - g_last_oled_ms) >= OLED_PERIOD_MS) {
        g_last_oled_ms = now_ms;
        draw_oled();
    }
    if (SSD1306_IsOnline() == 0U &&
        (uint32_t) (now_ms - g_last_oled_recovery_ms) >= 1000U) {
        g_last_oled_recovery_ms = now_ms;
        (void) SSD1306_TryRecover();
    }
#endif
}

Control_State Control_GetState(void)
{
    return g_control;
}

void Control_RequestSafe(void)
{
    enter_safe("SAFE REQUEST", g_last_control_ms);
}
