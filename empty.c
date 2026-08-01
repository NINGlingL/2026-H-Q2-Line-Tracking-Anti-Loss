/*
 * 车载平衡滚球运动控制系统 - 第三问
 *
 * MaixCAM Pro -> UART0:
 *   P:<signed_position_cm>,V:<signed_velocity_cm_s>\n, 115200 8N1, 20 Hz
 *   按本工程 SysConfig 接线：Maix TX -> MSPM0 PB1(RX)，Maix RX -> PB0(TX)。
 *   最新视觉程序已把 12.5 cm 刻度定义为中心 O：有效位置为 -12.5~+10.5 cm；
 *   V 也已投影到标定轴并带正负号。MCU 直接使用 P，并用连续 P 差分速度与
 *   视觉 V 做一致性融合，避免单帧速度噪声直接进入 PID。
 *
 * 第三问轨迹:
 *   O(0 cm) -> +5 cm -> -5 cm，并保持在 -5 cm；总时间不得超过 5 s。
 *
 * 平衡位置设置:
 *   实机最终标定位置为距丝杆最低端 62.0 mm。由于无绝对位置传感器，上电时
 *   机构必须仍在该标定点；固件将当前位置作为平衡基准并立即保持。
 *   PB21 只用于开始第三问 PID，不再修改步进电机零点。
 */

#include "ti_msp_dl_config.h"
#include "ssd1306.h"
#include "tmc2208.h"
#include <math.h>
#include <string.h>

/* ==================== 已知接口与机械参数 ==================== */
#define UART_BAUDRATE_BPS             115200U
#define VISION_PERIOD_MS              50U
#define VISION_HOLD_MS                150U
#define VISION_TIMEOUT_MS             700U
#define WAIT_VISION_TIMEOUT_MS        10000U
#define OLED_REFRESH_MS               100U
#define OLED_PAGE_PERIOD_MS           2000U
#define BUTTON_DEBOUNCE_MS             30U
#define WAIT_BUTTON_TIMEOUT_MS         300000U

#define BALL_CENTER_CM                0.0f
#define BALL_STEP_CM                  5.0f
#define BALL_POSITION_MIN_CM         -12.5f
#define BALL_POSITION_MAX_CM          10.5f
#define BALL_SPEED_LIMIT_CM_S         150.0f
#define BALL_JUMP_BASE_CM             2.0f
#define BALL_JUMP_SPEED_FACTOR        1.8f
#define VELOCITY_FILTER_ALPHA          0.35f
#define VISION_VELOCITY_SCALE          0.30f
#define VISION_VELOCITY_BLEND          0.25f

#define ARRIVAL_TOL_CM                0.60f
#define ARRIVAL_SPEED_CM_S            0.50f
#define ARRIVAL_DWELL_MS              150U
#define OSC_SWITCH_POSITION_CM         4.50f
#define PLUS_TIMEOUT_MS               3500U
#define TASK_TIMEOUT_MS               5000U
#define FINAL_HOLD_TIMEOUT_MS         60000U

#define BALANCE_HEIGHT_MM             62.0f
#define ACTUATOR_REL_MIN_MM          (-10.0f)
#define ACTUATOR_REL_MAX_MM            10.0f

/*
 * PID 输出为控制端相对水平位置的位移量。
 * 实机复测：+4800 步时小球到达负坐标 -4.08 cm，因此正步数会让球
 * 向负坐标运动；正位置误差必须映射为负电机位移。
 */
#define MOTOR_POSITIVE_LIFTS_END      1
#if MOTOR_POSITIVE_LIFTS_END
#define CONTROL_TO_MOTOR_SIGN        (-1.0f)
#else
#define CONTROL_TO_MOTOR_SIGN         (1.0f)
#endif

/*
 * 25 cm 摆杆、小钢球纯滚动的线性近似为：
 *   x'' ~= (5/7)g*(u/250 mm) ~= 2.8*u  cm/s^2（u 的单位为 mm）。
 * Kp=1.8 对应自然频率约 2.25 rad/s，Kd=1.4 对应阻尼比约 0.87；
 * Ki 仅用于抵消水管微小静态坡度。实机应根据 OLED 的 B/V/E/O 微调。
 */
#define PID_KP_MM_PER_CM              2.40f
#define PID_KI_MM_PER_CM_S            0.00f
#define PID_KD_MM_PER_CM_S            0.65f
#define PID_INTEGRAL_ZONE_CM          2.50f
#define PID_INTEGRAL_LIMIT_CM_S       8.00f
#define PID_OUTPUT_LIMIT_MM           8.00f
#define PID_OUTPUT_SLEW_MM_S          60.0f

#define STEPPER_MAX_SPEED_SPS        12000.0f
#define STEPPER_ACCEL_SPS2            80000.0f

/* 临时硬件隔离测试：绕过视觉/PID，在软件零点附近自动往返 ±3 mm。 */
#define MOTOR_SELF_TEST_MODE           1U
#define MOTOR_SELF_TEST_STEPS        1600U
#define MOTOR_SELF_TEST_PULSE_CYCLES (CPUCLK_FREQ / 2000U)

#if UART_0_BAUD_RATE != UART_BAUDRATE_BPS
#error "UART baud rate must match MaixCAM protocol (115200 bps)"
#endif

/* ==================== 毫秒时基 ==================== */
static volatile uint32_t g_ms = 0U;

void SysTick_Handler(void)
{
    g_ms++;
}

static uint32_t now_ms(void)
{
    return g_ms;
}

static void delay_ms(uint32_t ms)
{
    uint32_t cycles = (CPUCLK_FREQ / 3000U) * ms;
    while (cycles-- > 0U) {
        __NOP();
    }
}

static float absf_local(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float clampf_local(float value, float lower, float upper)
{
    if (value > upper) {
        value = upper;
    }
    if (value < lower) {
        value = lower;
    }
    return value;
}

static bool finitef_local(float value)
{
    return (isnan(value) == 0) && (isinf(value) == 0);
}

/* ==================== UART0 环形缓冲 ==================== */
#define RX_BUFFER_SIZE 128U
static char rx_buffer[RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0U;
static volatile uint16_t rx_tail = 0U;
static volatile uint32_t g_uart_rx_bytes = 0U;
static volatile uint16_t g_uart_overruns = 0U;

void UART0_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(UART_0_INST) == DL_UART_IIDX_RX) {
        while (DL_UART_isRXFIFOEmpty(UART_0_INST) == false) {
            char ch = (char)DL_UART_receiveData(UART_0_INST);
            uint16_t next = (uint16_t)((rx_head + 1U) % RX_BUFFER_SIZE);
            if (g_uart_rx_bytes < 99999U) {
                g_uart_rx_bytes++;
            }
            if (next != rx_tail) {
                rx_buffer[rx_head] = ch;
                rx_head = next;
            } else if (g_uart_overruns < 999U) {
                g_uart_overruns++;
            }
        }
    }
}

/* ==================== 视觉帧解析与合理性检查 ==================== */
#define LINE_BUF_SIZE 40U
static char line_buf[LINE_BUF_SIZE];
static uint8_t line_len = 0U;
static bool line_overflow = false;

static float g_ball_cm = BALL_CENTER_CM;
static float g_ball_vel_cm_s = 0.0f;          /* 融合后的有符号轴向速度 */
static float g_vision_vel_cm_s = 0.0f;        /* 视觉发送的有符号缩放速度 */
static bool g_ball_valid = false;
static bool g_ball_new = false;
static uint32_t g_last_data_ms = 0U;
static uint32_t g_last_sample_ms = 0U;
static uint32_t g_accepted_frames = 0U;
static uint16_t g_rejected_frames = 0U;
static uint32_t g_uart_lines = 0U;

/* PB21 对地按下，SysConfig 已配置内部上拉。上电按住不会触发，必须先松开再按。 */
static uint8_t g_button_last_raw = 1U;
static uint8_t g_button_stable = 1U;
static uint32_t g_button_change_ms = 0U;

/* 保留为全局符号，供 MCP 探针在线确认 OLED 刷新是否持续、总线是否健康。 */
volatile uint32_t g_oled_refresh_count = 0U;
volatile uint8_t g_oled_healthy_snapshot = 0U;

static bool parse_float_token(const char *text, float *value, const char **end)
{
    const char *p = text;
    float sign = 1.0f;
    float result = 0.0f;
    float fraction = 0.1f;
    uint8_t digits = 0U;

    if (*p == '+') {
        p++;
    } else if (*p == '-') {
        sign = -1.0f;
        p++;
    }

    while ((*p >= '0') && (*p <= '9')) {
        result = result * 10.0f + (float)(*p - '0');
        digits++;
        p++;
    }
    if (*p == '.') {
        p++;
        while ((*p >= '0') && (*p <= '9')) {
            result += (float)(*p - '0') * fraction;
            fraction *= 0.1f;
            digits++;
            p++;
        }
    }

    if (digits == 0U) {
        return false;
    }

    result *= sign;
    if (!finitef_local(result)) {
        return false;
    }
    *value = result;
    *end = p;
    return true;
}

static bool vision_sample_is_plausible(float position_cm,
                                       float reported_velocity_cm_s,
                                       uint32_t sample_ms)
{
    if ((position_cm < BALL_POSITION_MIN_CM) ||
        (position_cm > BALL_POSITION_MAX_CM) ||
        (absf_local(reported_velocity_cm_s) > BALL_SPEED_LIMIT_CM_S)) {
        return false;
    }

    if (g_ball_valid) {
        uint32_t dt_ms = sample_ms - g_last_sample_ms;
        if (dt_ms <= VISION_TIMEOUT_MS) {
            float dt_s = clampf_local((float)dt_ms * 0.001f, 0.01f, 0.25f);
            float reported_physical_cm_s =
                absf_local(reported_velocity_cm_s) / VISION_VELOCITY_SCALE;
            float jump_speed_cm_s = absf_local(g_ball_vel_cm_s);
            if (reported_physical_cm_s > jump_speed_cm_s) {
                jump_speed_cm_s = reported_physical_cm_s;
            }
            float allowed_jump = BALL_JUMP_BASE_CM +
                                 jump_speed_cm_s * dt_s *
                                 BALL_JUMP_SPEED_FACTOR;
            if (absf_local(position_cm - g_ball_cm) > allowed_jump) {
                return false;
            }
        }
    }
    return true;
}

static bool parse_vision_line(const char *line, float *position_cm, float *velocity_cm_s)
{
    const char *comma;
    const char *position_end;
    const char *velocity_end;

    if (strncmp(line, "P:", 2U) != 0) {
        return false;
    }
    comma = strstr(line + 2, ",V:");
    if (comma == NULL) {
        return false;
    }
    if (!parse_float_token(line + 2, position_cm, &position_end) ||
        (position_end != comma)) {
        return false;
    }
    if (!parse_float_token(comma + 3, velocity_cm_s, &velocity_end) ||
        (*velocity_end != '\0')) {
        return false;
    }
    return true;
}

static void accept_vision_line(const char *line)
{
    float position_cm;
    float reported_velocity_cm_s;
    float signed_velocity_cm_s = 0.0f;
    uint32_t sample_ms = now_ms();

    if (parse_vision_line(line, &position_cm, &reported_velocity_cm_s) &&
        vision_sample_is_plausible(position_cm, reported_velocity_cm_s, sample_ms)) {
        if (g_ball_valid) {
            uint32_t dt_ms = sample_ms - g_last_sample_ms;
            if ((dt_ms >= 10U) && (dt_ms <= VISION_TIMEOUT_MS)) {
                float dt_s = (float)dt_ms * 0.001f;
                float raw_velocity_cm_s =
                    (position_cm - g_ball_cm) / dt_s;
                float vision_velocity_cm_s =
                    reported_velocity_cm_s / VISION_VELOCITY_SCALE;
                float measured_velocity_cm_s;

                raw_velocity_cm_s = clampf_local(raw_velocity_cm_s,
                                                  -BALL_SPEED_LIMIT_CM_S,
                                                  BALL_SPEED_LIMIT_CM_S);
                vision_velocity_cm_s = clampf_local(vision_velocity_cm_s,
                                                     -BALL_SPEED_LIMIT_CM_S,
                                                     BALL_SPEED_LIMIT_CM_S);
                measured_velocity_cm_s =
                    (1.0f - VISION_VELOCITY_BLEND) * raw_velocity_cm_s +
                    VISION_VELOCITY_BLEND * vision_velocity_cm_s;
                signed_velocity_cm_s = g_ball_vel_cm_s +
                    VELOCITY_FILTER_ALPHA *
                    (measured_velocity_cm_s - g_ball_vel_cm_s);
            }
        }

        if (!finitef_local(position_cm) ||
            !finitef_local(signed_velocity_cm_s)) {
            if (g_rejected_frames < 65535U) {
                g_rejected_frames++;
            }
            return;
        }

        g_ball_cm = position_cm;
        g_ball_vel_cm_s = signed_velocity_cm_s;
        g_vision_vel_cm_s = reported_velocity_cm_s;
        g_last_sample_ms = sample_ms;
        g_last_data_ms = sample_ms;
        g_ball_valid = true;
        g_ball_new = true;
        if (g_accepted_frames < 99999U) {
            g_accepted_frames++;
        }
    } else {
        if (g_rejected_frames < 65535U) {
            g_rejected_frames++;
        }
    }
}

static void process_uart(void)
{
    while (rx_head != rx_tail) {
        char ch = rx_buffer[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % RX_BUFFER_SIZE);

        if ((ch == '\n') || (ch == '\r')) {
            if ((line_len > 0U) && (g_uart_lines < 99999U)) {
                g_uart_lines++;
            }
            if ((line_len > 0U) && !line_overflow) {
                line_buf[line_len] = '\0';
                accept_vision_line(line_buf);
            }
            line_len = 0U;
            line_overflow = false;
        } else if (!line_overflow) {
            if (line_len < (LINE_BUF_SIZE - 1U)) {
                line_buf[line_len++] = ch;
            } else {
                line_overflow = true;
            }
        }
    }
}

static bool vision_lost(void)
{
    return (!g_ball_valid) || ((now_ms() - g_last_data_ms) > VISION_TIMEOUT_MS);
}

/* ==================== PID ==================== */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float last_output_mm;
} BallPid;

static BallPid g_pid = {
    PID_KP_MM_PER_CM,
    PID_KI_MM_PER_CM_S,
    PID_KD_MM_PER_CM_S,
    0.0f,
    0.0f
};

static float g_target_cm = BALL_CENTER_CM;
static float g_error_cm = 0.0f;
static float g_control_mm = 0.0f;

static void pid_reset(void)
{
    g_pid.integral = 0.0f;
    g_pid.last_output_mm = 0.0f;
}

static bool pid_update(float error_cm, float ball_velocity_cm_s, float dt_s, float *output_mm)
{
    float derivative = -ball_velocity_cm_s;
    float unsaturated;
    float limited;
    float max_delta;

    /* 积分分离：大误差期间不累加，避免折返时积分冲击。 */
    if (absf_local(error_cm) <= PID_INTEGRAL_ZONE_CM) {
        g_pid.integral += error_cm * dt_s;
        g_pid.integral = clampf_local(g_pid.integral,
                                      -PID_INTEGRAL_LIMIT_CM_S,
                                      PID_INTEGRAL_LIMIT_CM_S);
    }

    unsaturated = g_pid.kp * error_cm +
                  g_pid.ki * g_pid.integral +
                  g_pid.kd * derivative;
    if (!finitef_local(unsaturated)) {
        pid_reset();
        *output_mm = 0.0f;
        return false;
    }

    limited = clampf_local(unsaturated, -PID_OUTPUT_LIMIT_MM, PID_OUTPUT_LIMIT_MM);
    max_delta = PID_OUTPUT_SLEW_MM_S * dt_s;
    limited = clampf_local(limited,
                           g_pid.last_output_mm - max_delta,
                           g_pid.last_output_mm + max_delta);
    limited = clampf_local(limited, -PID_OUTPUT_LIMIT_MM, PID_OUTPUT_LIMIT_MM);
    if (!finitef_local(limited)) {
        pid_reset();
        *output_mm = 0.0f;
        return false;
    }

    g_pid.last_output_mm = limited;
    *output_mm = limited;
    return true;
}

/* ==================== 第三问状态机 ==================== */
typedef enum {
    PHASE_READY = 0,
    PHASE_WAIT_VISION,
    PHASE_GO_PLUS,
    PHASE_GO_MINUS,
    PHASE_HOLD_MINUS,
    PHASE_SAFE
} ControlPhase;

typedef enum {
    FAULT_NONE = 0,
    FAULT_BUTTON_TIMEOUT,
    FAULT_WAIT_VISION_TIMEOUT,
    FAULT_VISION_TIMEOUT,
    FAULT_PLUS_TIMEOUT,
    FAULT_TASK_TIMEOUT,
    FAULT_FINAL_HOLD_TIMEOUT,
    FAULT_PID_NUMERIC
} FaultCode;

static ControlPhase g_phase = PHASE_READY;
static FaultCode g_fault = FAULT_NONE;
static uint32_t g_phase_start_ms = 0U;
static uint32_t g_task_start_ms = 0U;
static uint32_t g_arrival_start_ms = 0U;
static uint32_t g_task_finish_ms = 0U;
static uint32_t g_last_control_ms = 0U;

static void enter_safe(FaultCode fault)
{
    tmc2208_disable();
    pid_reset();
    g_control_mm = 0.0f;
    g_fault = fault;
    g_phase = PHASE_SAFE;
    g_phase_start_ms = now_ms();
}

static void request_pid_start(uint32_t time_ms)
{
    /*
     * 电机已在上电阶段接管 62 mm 平衡位。PB21 只清除控制器历史量并
     * 请求开始第三问；若视觉尚未就绪，则保持水平并等待有效帧。
     */
    tmc2208_stop();
    tmc2208_move_to(0);
    pid_reset();
    g_target_cm = BALL_CENTER_CM;
    g_error_cm = 0.0f;
    g_control_mm = 0.0f;
    g_phase = PHASE_WAIT_VISION;
    g_phase_start_ms = time_ms;
}

static void service_start_button(uint32_t time_ms)
{
    uint8_t raw = (DL_GPIO_readPins(KEY_PORT, KEY_START_PIN) == 0U) ? 0U : 1U;

    if (raw != g_button_last_raw) {
        g_button_last_raw = raw;
        g_button_change_ms = time_ms;
    }
    if (((time_ms - g_button_change_ms) >= BUTTON_DEBOUNCE_MS) &&
        (raw != g_button_stable)) {
        g_button_stable = raw;
        if ((raw == 0U) && (g_phase == PHASE_READY)) {
            request_pid_start(time_ms);
        }
    }
}

static bool target_is_settled(uint32_t time_ms)
{
    bool inside = (absf_local(g_target_cm - g_ball_cm) <= ARRIVAL_TOL_CM) &&
                  (absf_local(g_ball_vel_cm_s) <= ARRIVAL_SPEED_CM_S);

    if (!inside) {
        g_arrival_start_ms = 0U;
        return false;
    }
    if (g_arrival_start_ms == 0U) {
        g_arrival_start_ms = time_ms;
        return false;
    }
    return (time_ms - g_arrival_start_ms) >= ARRIVAL_DWELL_MS;
}

static bool trajectory_update(uint32_t time_ms)
{
    switch (g_phase) {
        case PHASE_READY:
            if ((time_ms - g_phase_start_ms) > WAIT_BUTTON_TIMEOUT_MS) {
                enter_safe(FAULT_BUTTON_TIMEOUT);
            }
            break;

        case PHASE_WAIT_VISION:
            if (!vision_lost()) {
                g_phase = PHASE_GO_PLUS;
                g_target_cm = BALL_CENTER_CM + BALL_STEP_CM;
                g_phase_start_ms = time_ms;
                g_task_start_ms = time_ms;
                g_arrival_start_ms = 0U;
                g_last_control_ms = time_ms - VISION_PERIOD_MS;
                pid_reset();
            } else if ((time_ms - g_phase_start_ms) > WAIT_VISION_TIMEOUT_MS) {
                enter_safe(FAULT_WAIT_VISION_TIMEOUT);
                return false;
            }
            break;

        case PHASE_GO_PLUS:
            g_target_cm = BALL_CENTER_CM + BALL_STEP_CM;
            if (g_ball_cm >= OSC_SWITCH_POSITION_CM) {
                g_phase = PHASE_GO_MINUS;
                g_target_cm = BALL_CENTER_CM - BALL_STEP_CM;
                g_phase_start_ms = time_ms;
                g_arrival_start_ms = 0U;
                pid_reset();
            } else if ((time_ms - g_phase_start_ms) > PLUS_TIMEOUT_MS) {
                enter_safe(FAULT_PLUS_TIMEOUT);
                return false;
            }
            break;

        case PHASE_GO_MINUS:
            g_target_cm = BALL_CENTER_CM - BALL_STEP_CM;
            if (g_ball_cm <= -OSC_SWITCH_POSITION_CM) {
                g_phase = PHASE_GO_PLUS;
                g_target_cm = BALL_CENTER_CM + BALL_STEP_CM;
                g_phase_start_ms = time_ms;
                g_arrival_start_ms = 0U;
                pid_reset();
            } else if ((time_ms - g_phase_start_ms) > PLUS_TIMEOUT_MS) {
                enter_safe(FAULT_TASK_TIMEOUT);
                return false;
            }
            break;

        case PHASE_HOLD_MINUS:
            g_target_cm = BALL_CENTER_CM - BALL_STEP_CM;
            if ((time_ms - g_phase_start_ms) > FINAL_HOLD_TIMEOUT_MS) {
                enter_safe(FAULT_FINAL_HOLD_TIMEOUT);
                return false;
            }
            break;

        case PHASE_SAFE:
        default:
            return false;
    }
    return (g_phase == PHASE_GO_PLUS) ||
           (g_phase == PHASE_GO_MINUS) ||
           (g_phase == PHASE_HOLD_MINUS);
}

static void control_step(void)
{
    uint32_t time_ms = now_ms();
    float dt_s;
    float motor_relative_mm;
    float steps_float;
    int32_t target_steps;

    if (!trajectory_update(time_ms)) {
        return;
    }

    dt_s = clampf_local((float)(time_ms - g_last_control_ms) * 0.001f, 0.01f, 0.15f);
    g_last_control_ms = time_ms;
    g_error_cm = g_target_cm - g_ball_cm;

    if (!pid_update(g_error_cm, g_ball_vel_cm_s, dt_s, &g_control_mm)) {
        enter_safe(FAULT_PID_NUMERIC);
        return;
    }

    motor_relative_mm = CONTROL_TO_MOTOR_SIGN * g_control_mm;
    motor_relative_mm = clampf_local(motor_relative_mm,
                                     -PID_OUTPUT_LIMIT_MM,
                                     PID_OUTPUT_LIMIT_MM);
    motor_relative_mm = clampf_local(motor_relative_mm,
                                     ACTUATOR_REL_MIN_MM,
                                     ACTUATOR_REL_MAX_MM);

    steps_float = motor_relative_mm * TMC_STEPS_PER_MM;
    steps_float = clampf_local(steps_float,
                               -PID_OUTPUT_LIMIT_MM * TMC_STEPS_PER_MM,
                               PID_OUTPUT_LIMIT_MM * TMC_STEPS_PER_MM);
    target_steps = (steps_float >= 0.0f) ?
                   (int32_t)(steps_float + 0.5f) :
                   (int32_t)(steps_float - 0.5f);
    tmc2208_move_to(target_steps);
}

static void control_supervise(void)
{
    uint32_t time_ms = now_ms();
    bool active = (g_phase == PHASE_GO_PLUS) ||
                  (g_phase == PHASE_GO_MINUS) ||
                  (g_phase == PHASE_HOLD_MINUS);

    if (active) {
        if (vision_lost()) {
            enter_safe(FAULT_VISION_TIMEOUT);
            return;
        }
        if ((time_ms - g_last_data_ms) > VISION_HOLD_MS) {
            /* 短时漏检先停止 STEP 并保持当前位置，恢复帧到达后自动续控。 */
            tmc2208_stop();
        }
    }

    if ((g_phase == PHASE_READY) ||
        (g_phase == PHASE_WAIT_VISION)) {
        (void)trajectory_update(time_ms);
    } else if ((g_phase == PHASE_GO_PLUS) &&
               ((time_ms - g_phase_start_ms) > PLUS_TIMEOUT_MS)) {
        enter_safe(FAULT_PLUS_TIMEOUT);
    } else if ((g_phase == PHASE_GO_MINUS) &&
               ((time_ms - g_phase_start_ms) > PLUS_TIMEOUT_MS)) {
        enter_safe(FAULT_TASK_TIMEOUT);
    } else if ((g_phase == PHASE_HOLD_MINUS) &&
               ((time_ms - g_phase_start_ms) > FINAL_HOLD_TIMEOUT_MS)) {
        enter_safe(FAULT_FINAL_HOLD_TIMEOUT);
    }
}

/* ==================== OLED ==================== */
static void fmt_snum(float value, int decimals, int max_integer_digits, char *buffer)
{
    char sign = '+';
    int scale = 1;
    int integer_part;
    int fraction_part;
    int cap = 1;
    int digits[4];
    int count = 0;
    int position = 0;

    if (value < 0.0f) {
        sign = '-';
        value = -value;
    }
    for (int i = 0; i < decimals; i++) {
        scale *= 10;
    }
    for (int i = 0; i < max_integer_digits; i++) {
        cap *= 10;
    }

    integer_part = (int)value;
    fraction_part = (int)((value - (float)integer_part) * (float)scale + 0.5f);
    if (fraction_part >= scale) {
        integer_part++;
        fraction_part -= scale;
    }
    if (integer_part >= cap) {
        integer_part = cap - 1;
    }

    do {
        digits[count++] = integer_part % 10;
        integer_part /= 10;
    } while ((integer_part > 0) && (count < 4));

    buffer[position++] = sign;
    while (count > 0) {
        buffer[position++] = (char)('0' + digits[--count]);
    }
    if (decimals > 0) {
        int divisor = scale / 10;
        buffer[position++] = '.';
        for (int i = 0; i < decimals; i++) {
            buffer[position++] = (char)('0' + (fraction_part / divisor) % 10);
            divisor /= 10;
        }
    }
    buffer[position] = '\0';
}

static void fmt_steps(int32_t value, char *buffer)
{
    char sign = '+';
    if (value < 0) {
        sign = '-';
        value = -value;
    }
    if (value > 99999) {
        value = 99999;
    }
    buffer[0] = sign;
    buffer[1] = (char)('0' + (value / 10000) % 10);
    buffer[2] = (char)('0' + (value / 1000) % 10);
    buffer[3] = (char)('0' + (value / 100) % 10);
    buffer[4] = (char)('0' + (value / 10) % 10);
    buffer[5] = (char)('0' + value % 10);
    buffer[6] = '\0';
}

static void fmt_unum(float value, int decimals, int max_integer_digits, char *buffer)
{
    char signed_text[16];

    if (value < 0.0f) {
        value = -value;
    }
    fmt_snum(value, decimals, max_integer_digits, signed_text);
    strcpy(buffer, &signed_text[1]);
}

static const char *phase_text(void)
{
    switch (g_phase) {
        case PHASE_READY:       return "READY";
        case PHASE_WAIT_VISION: return "WAIT";
        case PHASE_GO_PLUS:     return "GO+5";
        case PHASE_GO_MINUS:    return "GO-5";
        case PHASE_HOLD_MINUS:  return "HOLD";
        case PHASE_SAFE:
        default:                return "SAFE";
    }
}

static const char *uart_status_text(uint32_t time_ms)
{
    if (!g_ball_valid) {
        if (g_uart_rx_bytes == 0U) {
            return "WAIT";
        }
        if (g_rejected_frames != 0U) {
            return "BAD";
        }
        return "DATA";
    }
    if ((time_ms - g_last_data_ms) > VISION_TIMEOUT_MS) {
        return "OLD";
    }
    return "OK";
}

static const char *motor_status_text(void)
{
    if (!tmc2208_is_enabled()) {
        return "OFF";
    }
    return tmc2208_is_moving() ? "MOVE" : "IDLE";
}

static void oled_show_ready(uint32_t time_ms)
{
    char text[16];
    uint32_t age_ms = time_ms - g_last_data_ms;

    SSD1306_ShowString(0, 0, "UART:");
    SSD1306_ShowString(0, 30, uart_status_text(time_ms));
    SSD1306_ShowString(0, 60, "A:");
    if (g_ball_valid) {
        if (age_ms > 999U) {
            age_ms = 999U;
        }
        SSD1306_ShowNum(0, 72, (int32_t)age_ms, 3);
    } else {
        SSD1306_ShowString(0, 72, "---");
    }
    SSD1306_ShowString(0, 90, "ms");

    SSD1306_ShowString(1, 0, "CURRENT POS IS ZERO");
    SSD1306_ShowString(2, 0, "M:");
    SSD1306_ShowString(2, 12, motor_status_text());
    SSD1306_ShowString(2, 48, "Ph:READY");
    SSD1306_ShowString(3, 0, "PB21 START +-5 OSC");
    SSD1306_ShowString(4, 0, "By:");
    SSD1306_ShowNum(4, 18, (int32_t)g_uart_rx_bytes, 5);
    SSD1306_ShowString(4, 60, "Ln:");
    SSD1306_ShowNum(4, 78, (int32_t)g_uart_lines, 5);
    SSD1306_ShowString(5, 0, "A:");
    SSD1306_ShowNum(5, 12, (int32_t)g_accepted_frames, 5);
    SSD1306_ShowString(5, 48, "R:");
    SSD1306_ShowNum(5, 60, (int32_t)g_rejected_frames, 3);
    SSD1306_ShowString(5, 84, "O:");
    SSD1306_ShowNum(5, 96, (int32_t)g_uart_overruns, 2);
    SSD1306_ShowString(6, 0, "P:");
    fmt_snum(g_ball_cm, 2, 2, text);
    SSD1306_ShowString(6, 12, text);
    SSD1306_ShowString(6, 60, "V:");
    fmt_snum(g_ball_vel_cm_s, 1, 3, text);
    SSD1306_ShowString(6, 72, text);
    SSD1306_ShowString(7, 0, "MODE:CONTINUOUS OSC");
}

static void oled_show_status(uint32_t time_ms)
{
    char text[16];
    uint32_t age_ms = time_ms - g_last_data_ms;

    SSD1306_ShowString(0, 8, "OSC PID STATUS 1/2");

    SSD1306_ShowString(1, 0, "UART:");
    SSD1306_ShowString(1, 30, uart_status_text(time_ms));
    SSD1306_ShowString(1, 60, "A:");
    if (g_ball_valid) {
        if (age_ms > 999U) {
            age_ms = 999U;
        }
        SSD1306_ShowNum(1, 72, (int32_t)age_ms, 3);
    } else {
        SSD1306_ShowString(1, 72, "---");
    }
    SSD1306_ShowString(1, 90, "ms");

    SSD1306_ShowString(2, 0, "M:");
    SSD1306_ShowString(2, 12, motor_status_text());
    SSD1306_ShowString(2, 42, "Ph:");
    SSD1306_ShowString(2, 60, phase_text());

    SSD1306_ShowString(3, 0, "B:");
    fmt_snum(g_ball_cm, 2, 2, text);
    SSD1306_ShowString(3, 12, text);
    SSD1306_ShowString(3, 60, "T:");
    fmt_snum(g_target_cm, 2, 2, text);
    SSD1306_ShowString(3, 72, text);

    SSD1306_ShowString(4, 0, "V:");
    fmt_snum(g_ball_vel_cm_s, 1, 3, text);
    SSD1306_ShowString(4, 12, text);
    SSD1306_ShowString(4, 54, "E:");
    fmt_snum(g_error_cm, 2, 2, text);
    SSD1306_ShowString(4, 66, text);

    SSD1306_ShowString(5, 0, "O:");
    fmt_snum(g_control_mm, 2, 1, text);
    SSD1306_ShowString(5, 12, text);
    SSD1306_ShowString(5, 48, "St:");
    fmt_steps(tmc2208_get_position(), text);
    SSD1306_ShowString(5, 66, text);

    SSD1306_ShowString(6, 0, "Rx:");
    SSD1306_ShowNum(6, 18, (int32_t)g_accepted_frames, 5);
    SSD1306_ShowString(6, 54, "Rj:");
    SSD1306_ShowNum(6, 72, (int32_t)g_rejected_frames, 3);

    if (g_phase == PHASE_SAFE) {
        SSD1306_ShowString(7, 0, "FAULT:");
        SSD1306_ShowNum(7, 36, (int32_t)g_fault, 2);
        SSD1306_ShowString(7, 60, "MOTOR OFF");
    } else {
        SSD1306_ShowString(7, 0, "T:");
        if (g_phase == PHASE_HOLD_MINUS) {
            fmt_unum((float)g_task_finish_ms * 0.001f, 2, 1, text);
        } else if (g_phase == PHASE_WAIT_VISION) {
            fmt_unum(0.0f, 2, 1, text);
        } else {
            fmt_unum((float)(time_ms - g_task_start_ms) * 0.001f, 2, 1, text);
        }
        SSD1306_ShowString(7, 12, text);
        SSD1306_ShowString(7, 48, " Sp:");
        fmt_unum(tmc2208_get_current_speed(), 0, 4, text);
        SSD1306_ShowString(7, 72, text);
    }
}

static void oled_show_parameters(void)
{
    char text[16];

    SSD1306_ShowString(0, 2, "PID/MOTOR PARAM 2/2");

    SSD1306_ShowString(1, 0, "Kp:");
    fmt_unum(g_pid.kp, 2, 1, text);
    SSD1306_ShowString(1, 18, text);
    SSD1306_ShowString(1, 60, "Ki:");
    fmt_unum(g_pid.ki, 3, 1, text);
    SSD1306_ShowString(1, 78, text);

    SSD1306_ShowString(2, 0, "Kd:");
    fmt_unum(g_pid.kd, 2, 1, text);
    SSD1306_ShowString(2, 18, text);
    SSD1306_ShowString(2, 60, "Tol:");
    fmt_unum(ARRIVAL_TOL_CM, 2, 1, text);
    SSD1306_ShowString(2, 84, text);

    SSD1306_ShowString(3, 0, "Out:");
    fmt_unum(PID_OUTPUT_LIMIT_MM, 2, 1, text);
    SSD1306_ShowString(3, 24, text);
    SSD1306_ShowString(3, 54, "mm Slw:");
    fmt_unum(PID_OUTPUT_SLEW_MM_S, 1, 2, text);
    SSD1306_ShowString(3, 96, text);

    SSD1306_ShowString(4, 0, "Spd:");
    SSD1306_ShowNum(4, 24, (int32_t)STEPPER_MAX_SPEED_SPS, 4);
    SSD1306_ShowString(4, 54, "Acc:");
    SSD1306_ShowNum(4, 78, (int32_t)STEPPER_ACCEL_SPS2, 5);

    SSD1306_ShowString(5, 0, "Hold:");
    SSD1306_ShowNum(5, 30, (int32_t)VISION_HOLD_MS, 3);
    SSD1306_ShowString(5, 54, "Lost:");
    SSD1306_ShowNum(5, 84, (int32_t)VISION_TIMEOUT_MS, 3);
    SSD1306_ShowString(5, 102, "ms");

    SSD1306_ShowString(6, 0, "Vc:");
    fmt_snum(g_vision_vel_cm_s, 1, 3, text);
    SSD1306_ShowString(6, 18, text);
    SSD1306_ShowString(6, 60, "Lim:+-10mm");

    SSD1306_ShowString(7, 0, "B:");
    SSD1306_ShowNum(7, 12, (int32_t)g_uart_rx_bytes, 5);
    SSD1306_ShowString(7, 54, "L:");
    SSD1306_ShowNum(7, 66, (int32_t)g_uart_lines, 5);
}

static void oled_update(uint32_t time_ms)
{
    uint32_t page = (time_ms / OLED_PAGE_PERIOD_MS) & 1U;

    if (g_phase == PHASE_READY) {
        SSD1306_Clear();
        oled_show_ready(time_ms);
        SSD1306_Update();
        return;
    }

    /* 故障页必须持续可见，避免参数轮播遮住故障码和电机关闭状态。 */
    if (g_phase == PHASE_SAFE) {
        page = 0U;
    }

    SSD1306_Clear();
    if (page == 0U) {
        oled_show_status(time_ms);
    } else {
        oled_show_parameters();
    }
    SSD1306_Update();
}

/* ==================== 主函数 ==================== */
int main(void)
{
    uint32_t last_ui_ms;
    uint32_t motor_test_steps = 0U;
    int8_t motor_test_direction = 1;

    SYSCFG_DL_init();
    SysTick_Config(CPUCLK_FREQ / 1000U);
    g_phase_start_ms = now_ms();
    g_button_last_raw =
        (DL_GPIO_readPins(KEY_PORT, KEY_START_PIN) == 0U) ? 0U : 1U;
    g_button_stable = g_button_last_raw;
    g_button_change_ms = g_phase_start_ms;

    /*
     * 安全上电顺序：EN 禁用 -> STEP 停止 -> 将当前已标定机械位置接管为
     * 62 mm 平衡基准（相对坐标 0）-> 使能保持 -> 等待 PB21 启动 PID。
     * 无绝对位置传感器，因此上电前不得手动改变已标定机械位置。
     */
    tmc2208_init();
    tmc2208_set_max_speed(STEPPER_MAX_SPEED_SPS);
    tmc2208_set_accel(STEPPER_ACCEL_SPS2);
    tmc2208_set_limits_mm(ACTUATOR_REL_MIN_MM, ACTUATOR_REL_MAX_MM);
    tmc2208_set_current_position(0);

    delay_ms(50U);
    tmc2208_enable();
    tmc2208_move_to(0);
    if (MOTOR_SELF_TEST_MODE != 0U) {
        /* 彻底绕过 TIMA0，把 PA8 改成普通 GPIO 手动产生 STEP 脉冲。 */
        DL_TimerA_stopCounter(STEP_INST);
        DL_TimerA_disableInterrupt(STEP_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
        DL_GPIO_initDigitalOutput(GPIO_STEP_C0_IOMUX);
        DL_GPIO_clearPins(GPIO_STEP_C0_PORT, GPIO_STEP_C0_PIN);
        DL_GPIO_enableOutput(GPIO_STEP_C0_PORT, GPIO_STEP_C0_PIN);
        DL_GPIO_setPins(TMC2208_PORT, TMC2208_DIR_PIN);
    }
    if (MOTOR_SELF_TEST_MODE == 0U) {
        SSD1306_Init();
        oled_update(now_ms());
        g_oled_refresh_count++;
        g_oled_healthy_snapshot = SSD1306_IsHealthy() ? 1U : 0U;
    }

    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    last_ui_ms = now_ms();

    while (1) {
        uint32_t time_ms;

        time_ms = now_ms();
        process_uart();
        if (MOTOR_SELF_TEST_MODE != 0U) {
            DL_GPIO_setPins(GPIO_STEP_C0_PORT, GPIO_STEP_C0_PIN);
            delay_cycles(MOTOR_SELF_TEST_PULSE_CYCLES);
            DL_GPIO_clearPins(GPIO_STEP_C0_PORT, GPIO_STEP_C0_PIN);
            delay_cycles(MOTOR_SELF_TEST_PULSE_CYCLES);
            motor_test_steps++;
            if (motor_test_steps >= MOTOR_SELF_TEST_STEPS) {
                motor_test_steps = 0U;
                motor_test_direction = (int8_t)-motor_test_direction;
                if (motor_test_direction > 0) {
                    DL_GPIO_setPins(TMC2208_PORT, TMC2208_DIR_PIN);
                } else {
                    DL_GPIO_clearPins(TMC2208_PORT, TMC2208_DIR_PIN);
                }
            }
        } else {
            service_start_button(time_ms);
            if (g_ball_new) {
                g_ball_new = false;
                control_step();
            }
            control_supervise();
        }

        time_ms = now_ms();
        if ((MOTOR_SELF_TEST_MODE == 0U) &&
            ((time_ms - last_ui_ms) >= OLED_REFRESH_MS)) {
            last_ui_ms = time_ms;
            oled_update(time_ms);
            g_oled_refresh_count++;
            g_oled_healthy_snapshot = SSD1306_IsHealthy() ? 1U : 0U;
        }

        /* 独立 LFCLK 看门狗只允许在主循环末尾喂；任何 ISR 都不喂狗。 */
        DL_WWDT_restart(WWDT0_INST);
    }
}
