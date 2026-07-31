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

#define DIAGNOSTIC_TIMEOUT_MS (30000UL)
#define AUTO_TIMEOUT_MS       (35000UL)
#define OLED_PERIOD_MS        (125UL)
#define IMU_PERIOD_MS         (20UL)
#define YAW_RATE_PER_POSITION (5.0f)
#define START_MARKER_MIN_LAP_MS  (5000UL)
#define START_MARKER_CLEAR_MS    (80UL)
#define LINE_FILTER_ALPHA        (0.25f)
#define LINE_CENTER_DEADBAND     (1.25f)
#define STRAIGHT_YAW_RATE_DPS    (10.0f)

static Control_State g_control;
static PID_Controller g_line_pid;
static PID_Controller g_speed_pid;
static PID_Controller g_yaw_pid;
static IMU_Data g_imu;
static int16_t g_diag_power;
static int16_t g_manual_left;
static int16_t g_manual_right;
static uint32_t g_last_control_ms;
static uint32_t g_last_oled_ms;
static uint32_t g_last_oled_recovery_ms;
static uint32_t g_last_imu_ms;
static uint32_t g_imu_period_ms;
static uint32_t g_line_lost_ms;
static uint32_t g_marker_clear_ms;
static uint32_t g_auto_elapsed_ms;
static float g_filtered_line_position;
static uint8_t g_start_marker_armed;
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

static uint8_t start_marker_detected(const EightIR_State *ir)
{
    /*
     * The A marker makes the six middle detectors see black.  Count all
     * active detectors here so UART channel order or IR reversal cannot
     * hide the marker; a normal 1.8 cm guide line activates far fewer.
     */
    return (ir->active_count >= 5U) ? 1U : 0U;
}

static void enter_safe(const char *reason, uint32_t now_ms)
{
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
    g_filtered_line_position = 0.0f;
    g_start_marker_armed = 0U;
    (void) snprintf(g_last_command, sizeof(g_last_command), "AUTO START");
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
            if (g_diag_power < APP_PWM_MAX_PERMILLE - 50) {
                g_diag_power += 50;
            } else {
                g_diag_power = APP_PWM_MAX_PERMILLE;
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
    float line_error;
    float base = (float) APP_AUTO_BASE_PWM_PERMILLE;
    float speed_feedback;
    int16_t left;
    int16_t right;

    g_auto_elapsed_ms = (uint32_t) (now_ms - g_control.mode_enter_ms);

    if (g_auto_elapsed_ms > AUTO_TIMEOUT_MS) {
        enter_safe("AUTO TIMEOUT", now_ms);
        return;
    }
    if (ir.frame_fresh == 0U) {
        enter_safe("IR STALE", now_ms);
        return;
    }

    if (ir.active_count == 0U) {
        if (g_line_lost_ms == 0U) {
            g_line_lost_ms = now_ms;
        }
        Moto_EmergencyStop();
        if ((uint32_t) (now_ms - g_line_lost_ms) >= 50U) {
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
                       START_MARKER_CLEAR_MS) {
                g_start_marker_armed = 1U;
            }
        } else {
            g_marker_clear_ms = 0U;
        }
    } else if (g_auto_elapsed_ms >= START_MARKER_MIN_LAP_MS &&
               start_marker_detected(&ir) != 0U) {
        /* One valid UART frame is enough; the narrow marker passes quickly. */
        g_control.lap_count = 1U;
        enter_safe("LAP DONE", now_ms);
        return;
    }

    g_filtered_line_position += LINE_FILTER_ALPHA *
        ((float) ir.position - g_filtered_line_position);
    line_error = g_filtered_line_position;
    if (fabsf(line_error) < LINE_CENTER_DEADBAND) {
        line_error = 0.0f;
    }

    if (line_error == 0.0f && ir.active_count <= 3U &&
        (g_imu.valid == 0U || g_imu.stale != 0U ||
         fabsf(g_imu.yaw_rate_dps) < STRAIGHT_YAW_RATE_DPS)) {
        base = (float) APP_AUTO_STRAIGHT_PWM_PERMILLE;
    }

    if (g_control.speed_target_mm_s > 0 && encoder.valid != 0U &&
        (uint32_t) (now_ms - encoder.sample_ms) <= 30U) {
        speed_feedback = fabsf((float) encoder.speed_mm_s);
        base += PID_Update(&g_speed_pid,
            (float) g_control.speed_target_mm_s - speed_feedback, 0.010f);
    } else {
        PID_Reset(&g_speed_pid);
    }

    correction = PID_Update(&g_line_pid, line_error, 0.010f);

    /*
     * The line position requests a turn rate. The filtered Z gyro closes a
     * second loop around that request, damping oscillation without relying
     * on the drifting absolute yaw angle.
     */
    if (line_error != 0.0f && g_imu.valid != 0U && g_imu.stale == 0U &&
        g_imu.calibrated != 0U && isfinite(g_imu.yaw_rate_dps)) {
        desired_yaw_rate = line_error * YAW_RATE_PER_POSITION;
        yaw_assist = PID_Update(&g_yaw_pid,
            desired_yaw_rate - g_imu.yaw_rate_dps, 0.010f);
    } else {
        PID_Reset(&g_yaw_pid);
    }
    correction += yaw_assist;
    if (correction > (float) APP_PWM_MAX_PERMILLE) {
        correction = (float) APP_PWM_MAX_PERMILLE;
    }
    if (correction < (float) -APP_PWM_MAX_PERMILLE) {
        correction = (float) -APP_PWM_MAX_PERMILLE;
    }

    left = (int16_t) (base + correction);
    right = (int16_t) (base - correction);
    if (left > APP_PWM_MAX_PERMILLE) left = APP_PWM_MAX_PERMILLE;
    if (left < 0) left = 0;
    if (right > APP_PWM_MAX_PERMILLE) right = APP_PWM_MAX_PERMILLE;
    if (right < 0) right = 0;

    g_control.left_command = left;
    g_control.right_command = right;
    Moto_SetLR(left, right);
}

static void run_diagnostic(uint32_t now_ms)
{
    if ((uint32_t) (now_ms - g_control.mode_enter_ms) >
        DIAGNOSTIC_TIMEOUT_MS) {
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
    (void) snprintf(line, sizeof(line), "T:%lu.%02lu A:%u L:%u",
        (unsigned long) (g_auto_elapsed_ms / 1000U),
        (unsigned long) ((g_auto_elapsed_ms % 1000U) / 10U),
        g_start_marker_armed,
        g_control.lap_count);
    SSD1306_ShowString(7U, 0U, line);
    (void) SSD1306_Update();
}

void Control_Init(uint32_t now_ms)
{
    memset(&g_control, 0, sizeof(g_control));
    memset(&g_imu, 0, sizeof(g_imu));
    PID_Init(&g_line_pid, 7.0f, 0.2f, 0.04f,
        20.0f, 110.0f, 4.0f);
    PID_Init(&g_speed_pid, 0.45f, 0.40f, 0.0f,
        150.0f, 140.0f, 200.0f);
    PID_Init(&g_yaw_pid, 0.8f, 0.02f, 0.0f,
        30.0f, 30.0f, 60.0f);
    g_control.mode = CONTROL_SAFE;
    g_control.speed_target_mm_s = APP_AUTO_TARGET_SPEED_MM_S;
    g_control.hardware_locked = (APP_MOTOR_HW_READY == 0U) ? 1U : 0U;
    g_control.mode_enter_ms = now_ms;
    g_diag_power = APP_DIAG_PWM_PERMILLE;
    g_last_control_ms = now_ms;
    g_last_oled_ms = now_ms;
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
