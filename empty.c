/*
 * 车载平衡滚球运动控制系统 - 第三问
 *
 * MaixCAM Pro -> UART0:
 *   P:<position_cm>,V:<signed_velocity_cm_s>\n, 115200 8N1, 20 Hz
 *   按本工程 SysConfig 接线：Maix TX -> MSPM0 PB1(RX)，Maix RX -> PB0(TX)。
 *
 * 第三问轨迹:
 *   O(0 cm) -> +5 cm -> -5 cm，并保持在 -5 cm；总时间不得超过 5 s。
 *
 * 平衡位置设置:
 *   本次单功能测试从 67 mm 位置开始。
 *   按下 PB21 后再向下移动 5 mm，到达 62 mm 并保持。
 */

#include "ti_msp_dl_config.h"
#include "ssd1306.h"
#include "tmc2208.h"
#include <math.h>
#include <string.h>

/* ==================== 已知接口与机械参数 ==================== */
#define UART_BAUDRATE_BPS             115200U
#define VISION_PERIOD_MS              50U
#define VISION_TIMEOUT_MS             250U
#define WAIT_VISION_TIMEOUT_MS        10000U
#define OLED_REFRESH_MS               100U
#define OLED_PAGE_PERIOD_MS           2000U
#define BUTTON_DEBOUNCE_MS             30U
#define WAIT_BUTTON_TIMEOUT_MS         300000U
#define BALANCE_MOVE_TIMEOUT_MS         5000U

#define BALL_CENTER_CM                0.0f
#define BALL_STEP_CM                  5.0f
#define BALL_POSITION_MIN_CM         -13.5f
#define BALL_POSITION_MAX_CM          13.5f
#define BALL_SPEED_LIMIT_CM_S         150.0f
#define BALL_JUMP_BASE_CM             2.0f
#define BALL_JUMP_SPEED_FACTOR        1.8f

#define ARRIVAL_TOL_CM                0.60f
#define ARRIVAL_SPEED_CM_S            0.50f
#define ARRIVAL_DWELL_MS              150U
#define PLUS_TIMEOUT_MS               2200U
#define TASK_TIMEOUT_MS               5000U
#define FINAL_HOLD_TIMEOUT_MS         60000U

#define BALANCE_HEIGHT_MM             77.0f
#define TEST_START_HEIGHT_MM          67.0f
#define TEST_DESCENT_MM                5.0f
#define TEST_TARGET_HEIGHT_MM         (TEST_START_HEIGHT_MM - TEST_DESCENT_MM)
#define ACTUATOR_MIN_MM                0.0f
#define ACTUATOR_MAX_MM              100.0f
#define ACTUATOR_TRAVEL_MM           (ACTUATOR_MAX_MM - ACTUATOR_MIN_MM)

/*
 * PID 输出为控制端相对水平位置的位移量。
 * 图 2 中铰链在左、控制端在右：控制端抬高会使球向负坐标滚动。
 * 若实机“正步数”实际让控制端下降，把此值改为 +1。
 */
#define MOTOR_POSITIVE_LIFTS_END      1
#if MOTOR_POSITIVE_LIFTS_END
#define CONTROL_TO_MOTOR_SIGN        (-1.0f)
#else
#define CONTROL_TO_MOTOR_SIGN         (1.0f)
#endif

/*
 * 首次上机参数。视觉端发出的速度带 VELOCITY_SCALE=0.3，因此 Kd 已按该比例
 * 补偿；实机仍应先悬空/低幅验证方向，再根据轨迹记录微调。
 */
#define PID_KP_MM_PER_CM              1.20f
#define PID_KI_MM_PER_CM_S            0.04f
#define PID_KD_MM_PER_CM_S            2.60f
#define PID_INTEGRAL_ZONE_CM          2.50f
#define PID_INTEGRAL_LIMIT_CM_S       8.00f
#define PID_OUTPUT_LIMIT_MM           6.00f
#define PID_OUTPUT_SLEW_MM_S          18.0f

#define STEPPER_MAX_SPEED_SPS         5000.0f
#define STEPPER_ACCEL_SPS2            24000.0f

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

void UART0_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(UART_0_INST) == DL_UART_IIDX_RX) {
        while (DL_UART_isRXFIFOEmpty(UART_0_INST) == false) {
            char ch = (char)DL_UART_receiveData(UART_0_INST);
            uint16_t next = (uint16_t)((rx_head + 1U) % RX_BUFFER_SIZE);
            if (next != rx_tail) {
                rx_buffer[rx_head] = ch;
                rx_head = next;
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
static float g_ball_vel_cm_s = 0.0f;
static bool g_ball_valid = false;
static bool g_ball_new = false;
static uint32_t g_last_data_ms = 0U;
static uint32_t g_last_sample_ms = 0U;
static uint32_t g_accepted_frames = 0U;
static uint16_t g_rejected_frames = 0U;

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

static bool vision_sample_is_plausible(float position_cm, float velocity_cm_s, uint32_t sample_ms)
{
    if ((position_cm < BALL_POSITION_MIN_CM) ||
        (position_cm > BALL_POSITION_MAX_CM) ||
        (absf_local(velocity_cm_s) > BALL_SPEED_LIMIT_CM_S)) {
        return false;
    }

    if (g_ball_valid) {
        uint32_t dt_ms = sample_ms - g_last_sample_ms;
        float dt_s = clampf_local((float)dt_ms * 0.001f, 0.01f, 0.25f);
        float allowed_jump = BALL_JUMP_BASE_CM +
                             absf_local(g_ball_vel_cm_s) * dt_s * BALL_JUMP_SPEED_FACTOR;
        if (absf_local(position_cm - g_ball_cm) > allowed_jump) {
            return false;
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
    float velocity_cm_s;
    uint32_t sample_ms = now_ms();

    if (parse_vision_line(line, &position_cm, &velocity_cm_s) &&
        vision_sample_is_plausible(position_cm, velocity_cm_s, sample_ms)) {
        g_ball_cm = position_cm;
        g_ball_vel_cm_s = velocity_cm_s;
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
    PHASE_WAIT_BALANCE = 0,
    PHASE_TEST_MOVE,
    PHASE_TEST_DONE,
    PHASE_WAIT_VISION,
    PHASE_GO_PLUS,
    PHASE_GO_MINUS,
    PHASE_HOLD_MINUS,
    PHASE_SAFE
} ControlPhase;

typedef enum {
    FAULT_NONE = 0,
    FAULT_BUTTON_TIMEOUT,
    FAULT_BALANCE_MOVE_TIMEOUT,
    FAULT_WAIT_VISION_TIMEOUT,
    FAULT_VISION_TIMEOUT,
    FAULT_PLUS_TIMEOUT,
    FAULT_TASK_TIMEOUT,
    FAULT_FINAL_HOLD_TIMEOUT,
    FAULT_PID_NUMERIC
} FaultCode;

static ControlPhase g_phase = PHASE_WAIT_BALANCE;
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

static void start_down_move(uint32_t time_ms)
{
    int32_t start_steps =
        (int32_t)(TEST_START_HEIGHT_MM * TMC_STEPS_PER_MM + 0.5f);
    int32_t target_steps =
        (int32_t)(TEST_TARGET_HEIGHT_MM * TMC_STEPS_PER_MM + 0.5f);

    /*
     * 当前位置由用户确认在 67 mm。PB21 只在初始等待状态响应一次，
     * 目标从 53600 微步降到 49600 微步，即再向下移动 5 mm。
     */
    tmc2208_set_current_position(start_steps);
    tmc2208_enable();
    tmc2208_move_to(target_steps);
    g_phase = PHASE_TEST_MOVE;
    g_phase_start_ms = time_ms;
}

static void service_balance_button(uint32_t time_ms)
{
    uint8_t raw = (DL_GPIO_readPins(KEY_PORT, KEY_START_PIN) == 0U) ? 0U : 1U;

    if (raw != g_button_last_raw) {
        g_button_last_raw = raw;
        g_button_change_ms = time_ms;
    }
    if (((time_ms - g_button_change_ms) >= BUTTON_DEBOUNCE_MS) &&
        (raw != g_button_stable)) {
        g_button_stable = raw;
        if ((raw == 0U) && (g_phase == PHASE_WAIT_BALANCE)) {
            start_down_move(time_ms);
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
        case PHASE_WAIT_BALANCE:
            if ((time_ms - g_phase_start_ms) > WAIT_BUTTON_TIMEOUT_MS) {
                enter_safe(FAULT_BUTTON_TIMEOUT);
            }
            break;

        case PHASE_TEST_MOVE:
            if (!tmc2208_is_moving() &&
                (tmc2208_get_position() ==
                 (int32_t)(TEST_TARGET_HEIGHT_MM * TMC_STEPS_PER_MM + 0.5f))) {
                /* 单功能验证阶段：到位后保持使能，不自动进入视觉 PID。 */
                g_phase = PHASE_TEST_DONE;
                g_phase_start_ms = time_ms;
            } else if ((time_ms - g_phase_start_ms) > BALANCE_MOVE_TIMEOUT_MS) {
                enter_safe(FAULT_BALANCE_MOVE_TIMEOUT);
            }
            break;

        case PHASE_TEST_DONE:
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
            if (target_is_settled(time_ms)) {
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
            if (target_is_settled(time_ms)) {
                g_phase = PHASE_HOLD_MINUS;
                g_phase_start_ms = time_ms;
                g_task_finish_ms = time_ms - g_task_start_ms;
                g_arrival_start_ms = 0U;
            } else if ((time_ms - g_task_start_ms) > TASK_TIMEOUT_MS) {
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
    float motor_absolute_mm;
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
    motor_absolute_mm = clampf_local(BALANCE_HEIGHT_MM + motor_relative_mm,
                                     ACTUATOR_MIN_MM,
                                     ACTUATOR_MAX_MM);

    steps_float = motor_absolute_mm * TMC_STEPS_PER_MM;
    steps_float = clampf_local(steps_float,
                               ACTUATOR_MIN_MM * TMC_STEPS_PER_MM,
                               ACTUATOR_MAX_MM * TMC_STEPS_PER_MM);
    target_steps = (int32_t)(steps_float + 0.5f);
    tmc2208_move_to(target_steps);
}

static void control_supervise(void)
{
    uint32_t time_ms = now_ms();

    if (((g_phase == PHASE_GO_PLUS) ||
         (g_phase == PHASE_GO_MINUS) ||
         (g_phase == PHASE_HOLD_MINUS)) && vision_lost()) {
        enter_safe(FAULT_VISION_TIMEOUT);
        return;
    }

    if ((g_phase == PHASE_WAIT_BALANCE) ||
        (g_phase == PHASE_TEST_MOVE) ||
        (g_phase == PHASE_WAIT_VISION)) {
        (void)trajectory_update(time_ms);
    } else if ((g_phase == PHASE_GO_PLUS) &&
               ((time_ms - g_phase_start_ms) > PLUS_TIMEOUT_MS)) {
        enter_safe(FAULT_PLUS_TIMEOUT);
    } else if ((g_phase == PHASE_GO_MINUS) &&
               ((time_ms - g_task_start_ms) > TASK_TIMEOUT_MS)) {
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
        case PHASE_WAIT_BALANCE: return "SET";
        case PHASE_TEST_MOVE:   return "DOWN";
        case PHASE_TEST_DONE:   return "DONE";
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
        return (g_rejected_frames == 0U) ? "WAIT" : "BAD";
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

static void oled_show_balance_setup(uint32_t time_ms)
{
    char text[16];
    uint32_t age_ms = time_ms - g_last_data_ms;
    float from_motor_mm =
        (float)tmc2208_get_position() / TMC_STEPS_PER_MM;

    from_motor_mm = clampf_local(from_motor_mm, 0.0f, ACTUATOR_TRAVEL_MM);
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

    if (g_phase == PHASE_WAIT_BALANCE) {
        SSD1306_ShowString(1, 0, "DOWN TEST 5.0MM");
        SSD1306_ShowString(2, 0, "KEY:PB21 M:OFF");
        SSD1306_ShowString(3, 0, "PRESS PB21 TO DOWN");
        SSD1306_ShowString(4, 0, "Rx:");
        SSD1306_ShowNum(4, 18, (int32_t)g_accepted_frames, 5);
        SSD1306_ShowString(4, 54, "Rj:");
        SSD1306_ShowNum(4, 72, (int32_t)g_rejected_frames, 3);
        SSD1306_ShowString(5, 0, "P:");
        fmt_snum(g_ball_cm, 2, 2, text);
        SSD1306_ShowString(5, 12, text);
        SSD1306_ShowString(5, 54, "V:");
        fmt_snum(g_ball_vel_cm_s, 1, 3, text);
        SSD1306_ShowString(5, 66, text);
        SSD1306_ShowString(6, 0, "START:67 TARGET:62");
        SSD1306_ShowString(7, 0, "VISION PID:DISABLED");
    } else if (g_phase == PHASE_TEST_MOVE) {
        SSD1306_ShowString(1, 0, "LOWERING TO 62.0MM");
        SSD1306_ShowString(2, 0, "MOTOR:");
        SSD1306_ShowString(2, 36, motor_status_text());
        SSD1306_ShowString(3, 0, "POS:");
        fmt_unum(from_motor_mm, 1, 2, text);
        SSD1306_ShowString(3, 24, text);
        SSD1306_ShowString(3, 54, "mm");
        SSD1306_ShowString(4, 0, "TARGET:62.0mm");
        SSD1306_ShowString(5, 0, "Rx:");
        SSD1306_ShowNum(5, 18, (int32_t)g_accepted_frames, 5);
        SSD1306_ShowString(5, 54, "Rj:");
        SSD1306_ShowNum(5, 72, (int32_t)g_rejected_frames, 3);
        SSD1306_ShowString(6, 0, "WAIT UNTIL READY");
        SSD1306_ShowString(7, 0, "DO NOT MOVE PIPE");
    } else {
        SSD1306_ShowString(1, 0, "DOWN TEST COMPLETE");
        SSD1306_ShowString(2, 0, "MOTOR:IDLE HOLD:ON");
        SSD1306_ShowString(3, 0, "POS:62.0mm");
        SSD1306_ShowString(4, 0, "MOVED DOWN:5.0mm");
        SSD1306_ShowString(5, 0, "PB21:LOCKED");
        SSD1306_ShowString(6, 0, "VISION PID:DISABLED");
        SSD1306_ShowString(7, 0, "TEST STEP COMPLETE");
    }
}

static void oled_show_status(uint32_t time_ms)
{
    char text[16];
    uint32_t age_ms = time_ms - g_last_data_ms;

    SSD1306_ShowString(0, 8, "TASK3 STATUS 1/2");

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

    SSD1306_ShowString(5, 0, "Lost:");
    SSD1306_ShowNum(5, 30, (int32_t)VISION_TIMEOUT_MS, 3);
    SSD1306_ShowString(5, 54, " Dwl:");
    SSD1306_ShowNum(5, 84, (int32_t)ARRIVAL_DWELL_MS, 3);
    SSD1306_ShowString(5, 102, "ms");

    SSD1306_ShowString(6, 0, "Nut:");
    fmt_unum(BALANCE_HEIGHT_MM, 1, 2, text);
    SSD1306_ShowString(6, 24, text);
    SSD1306_ShowString(6, 54, "mm 800st/mm");

    SSD1306_ShowString(7, 0, "KEY:PB21 U:PB0/1");
}

static void oled_update(uint32_t time_ms)
{
    uint32_t page = (time_ms / OLED_PAGE_PERIOD_MS) & 1U;

    if ((g_phase == PHASE_WAIT_BALANCE) ||
        (g_phase == PHASE_TEST_MOVE) ||
        (g_phase == PHASE_TEST_DONE)) {
        SSD1306_Clear();
        oled_show_balance_setup(time_ms);
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

    SYSCFG_DL_init();
    SysTick_Config(CPUCLK_FREQ / 1000U);
    g_phase_start_ms = now_ms();
    g_button_last_raw =
        (DL_GPIO_readPins(KEY_PORT, KEY_START_PIN) == 0U) ? 0U : 1U;
    g_button_stable = g_button_last_raw;
    g_button_change_ms = g_phase_start_ms;

    /*
     * 安全上电顺序：EN 禁用 -> STEP 停止 -> 把当前位置记为 67 mm
     * -> 等待 PB21 -> 再向下降到 62 mm 并保持。本阶段不进入视觉 PID。
     */
    tmc2208_init();
    tmc2208_set_max_speed(STEPPER_MAX_SPEED_SPS);
    tmc2208_set_accel(STEPPER_ACCEL_SPS2);
    tmc2208_set_limits_mm(ACTUATOR_MIN_MM, ACTUATOR_MAX_MM);
    tmc2208_set_current_position(
        (int32_t)(TEST_START_HEIGHT_MM * TMC_STEPS_PER_MM + 0.5f));

    delay_ms(50U);
    SSD1306_Init();
    oled_update(now_ms());
    g_oled_refresh_count++;
    g_oled_healthy_snapshot = SSD1306_IsHealthy() ? 1U : 0U;

    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    last_ui_ms = now_ms();

    while (1) {
        uint32_t time_ms;

        time_ms = now_ms();
        service_balance_button(time_ms);
        process_uart();
        if (g_ball_new) {
            g_ball_new = false;
            control_step();
        }
        control_supervise();

        time_ms = now_ms();
        if ((time_ms - last_ui_ms) >= OLED_REFRESH_MS) {
            last_ui_ms = time_ms;
            oled_update(time_ms);
            g_oled_refresh_count++;
            g_oled_healthy_snapshot = SSD1306_IsHealthy() ? 1U : 0U;
        }

        /* 独立 LFCLK 看门狗只允许在主循环末尾喂；任何 ISR 都不喂狗。 */
        DL_WWDT_restart(WWDT0_INST);
    }
}
