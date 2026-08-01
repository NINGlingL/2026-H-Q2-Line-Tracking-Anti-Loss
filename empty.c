  /*
 * 车载平衡滚球运动控制系统 — 第三问: 摆杆控球 PID
 *
 * 功能:
 *   1. 串口接收视觉数据 (P:位置cm, V:速度cm/s, 20Hz)
 *   2. 球位置 PD(PID) -> 杆端抬升量(mm) -> 步进电机目标位置
 *   3. 任务3轨迹: O -> O+5 -> O-5, 误差<=1cm, 总时间<=5s
 *   4. OLED 显示: 球位/目标/误差/速度/抬升量/步数/阶段
 *
 * 机械约定 (请按实际标定核对):
 *   - 视觉 0cm = 电机端, 25cm = 铰链端
 *   - 抬升电机端 -> 球滚向 +cm 方向
 *   - 水管水平 (丝杆螺母距电机 6.5cm) = 步进位置 0; 上电前把水管调到水平
 *   - 800 步/mm (1/8 细分, 2mm 丝杆)
 */

#include "ti_msp_dl_config.h"
#include "ssd1306.h"
#include "tmc2208.h"
#include <string.h>

/* ==================== 控制配置 (按实测调整) ==================== */
#define BALL_CENTER_CM    12.5f     /* 摆杆中心 O 在视觉坐标的 cm 值 */
#define BALL_STEP_CM      5.0f      /* 任务3 ±5cm */
#define BALL_HOLD_TOL_CM  0.5f      /* 到达目标判定容差 */
#define SEG_TIMEOUT_MS    2500      /* 每段超时强制切换 (防卡死) */

/* PID (输出 = 杆端抬升量 mm) */
#define PID_KP            3.0f      /* mm / cm 位置误差 */
#define PID_KD            1.8f      /* mm / (cm/s) 速度阻尼 */
#define PID_KI            0.3f      /* mm / (cm*s) 积分 */
#define LIFT_LIMIT_MM     12.0f     /* 抬升量输出限幅 ±mm */

/* 电机 */
#define LIFT_SIGN         1         /* 步数增=抬升=球滚向+cm; 方向反则改 -1 */
#define STEPPER_MAX_SPEED 10000.0f  /* 步/s */
#define STEPPER_ACCEL     60000.0f  /* 步/s^2 */

/* 测试模式 */
#define TEST_LOOP         1         /* 1=循环 O±5; 0=单次 O->+5->-5 */

/* 数据超时 */
#define DATA_TIMEOUT_MS   500

/* ==================== 毫秒时基 (SysTick) ==================== */
static volatile uint32_t g_ms = 0;

void SysTick_Handler(void) { g_ms++; }
static inline uint32_t now_ms(void) { return g_ms; }

static void delay_ms(uint32_t ms)
{
    uint32_t cycles = CPUCLK_FREQ / 1000 * ms / 3;
    while (cycles--) { __NOP(); }
}

/* ==================== UART0 环形缓冲 + 中断 ==================== */
#define RX_BUFFER_SIZE 128
static char              rx_buffer[RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

void UART0_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(UART_0_INST) & DL_UART_IIDX_RX) {
        while (DL_UART_isRXFIFOEmpty(UART_0_INST) == false) {
            char ch = DL_UART_receiveData(UART_0_INST);
            uint16_t next = (rx_head + 1) % RX_BUFFER_SIZE;
            if (next != rx_tail) {
                rx_buffer[rx_head] = ch;
                rx_head = next;
            }
        }
    }
}

/* ==================== 视觉数据 ==================== */
#define LINE_BUF_SIZE 32
static char    line_buf[LINE_BUF_SIZE];
static uint8_t line_len = 0;

static float    g_ball_cm    = BALL_CENTER_CM;   /* 球位置 cm */
static float    g_ball_vel   = 0.0f;             /* 球速度 cm/s */
static bool     g_ball_valid = false;            /* 收到过数据 */
static uint32_t g_last_data_ms = 0;
static uint8_t  g_ball_new   = 0;                /* 新帧标志 */

/* 手写浮点解析, 返回 1 成功 */
static int parse_float(const char *s, float *out)
{
    float sign = 1.0f;
    const char *p = s;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '+')      p++;
    else if (*p == '-') { sign = -1.0f; p++; }

    float value = 0.0f;
    int   digits = 0;
    while (*p >= '0' && *p <= '9') {
        value = value * 10.0f + (float)(*p - '0');
        digits++;
        p++;
    }
    if (*p == '.') {
        p++;
        float frac = 0.1f;
        while (*p >= '0' && *p <= '9') {
            value += (float)(*p - '0') * frac;
            frac  *= 0.1f;
            digits++;
            p++;
        }
    }
    if (digits == 0) return 0;
    *out = sign * value;
    return 1;
}

/* 逐字符组行, 完整行时解析 "P:xx.xx,V:yy.yy" */
static void process_uart(void)
{
    while (rx_head != rx_tail) {
        char ch = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;

        if (ch == '\n' || ch == '\r') {
            if (line_len > 0) {
                line_buf[line_len] = '\0';

                float p = 0.0f, v = 0.0f;
                const char *pp = strstr(line_buf, "P:");
                const char *vp = strstr(line_buf, "V:");
                if (pp != NULL && vp != NULL &&
                    parse_float(pp + 2, &p) && parse_float(vp + 2, &v)) {
                    g_ball_cm     = p;
                    g_ball_vel    = v;
                    g_ball_valid  = true;
                    g_last_data_ms = now_ms();
                    g_ball_new    = 1;
                }
                line_len = 0;
            }
        } else if (line_len < LINE_BUF_SIZE - 1) {
            line_buf[line_len++] = ch;
        }
    }
}

static int ball_lost(void)
{
    return (!g_ball_valid) || (now_ms() - g_last_data_ms > DATA_TIMEOUT_MS);
}

/* ==================== PID ==================== */
static float g_pid_kp = PID_KP, g_pid_ki = PID_KI, g_pid_kd = PID_KD;
static float g_integral = 0.0f;
static float g_lift_mm  = 0.0f;   /* PID 输出: 杆端抬升量 mm */
static float g_err_cm   = 0.0f;
static float g_target_cm = BALL_CENTER_CM;
static float g_last_target_cm = -9999.0f;

static float pid_update(float err, float ball_vel, float dt)
{
    float derr = -ball_vel;                    /* 目标为阶跃常量: d(err)/dt = -球速 */

    g_integral += err * dt;
    float i_max = LIFT_LIMIT_MM / (g_pid_ki + 0.0001f);   /* 积分限幅 (抗饱和) */
    if (g_integral >  i_max) g_integral =  i_max;
    if (g_integral < -i_max) g_integral = -i_max;

    float out = g_pid_kp * err + g_pid_kd * derr + g_pid_ki * g_integral;
    if (out >  LIFT_LIMIT_MM) out =  LIFT_LIMIT_MM;
    if (out < -LIFT_LIMIT_MM) out = -LIFT_LIMIT_MM;

    return out;
}

/* ==================== 任务3轨迹状态机 ==================== */
typedef enum {
    PHASE_GO_PLUS  = 1,   /* 前往 O+5 */
    PHASE_GO_MINUS = 2,   /* 前往 O-5 */
    PHASE_DONE     = 3,
} Phase;

static Phase    g_phase = PHASE_GO_PLUS;
static uint32_t g_phase_start_ms = 0;
static uint32_t g_total_start_ms = 0;
static int      g_ctrl_started = 0;

static void trajectory_update(float ball_cm, uint32_t t)
{
    if (g_phase == PHASE_GO_PLUS) {
        g_target_cm = BALL_CENTER_CM + BALL_STEP_CM;
        if ((ball_cm - g_target_cm < 0.0f ? -(ball_cm - g_target_cm) : ball_cm - g_target_cm) < BALL_HOLD_TOL_CM ||
            (t - g_phase_start_ms > SEG_TIMEOUT_MS)) {
            g_phase = PHASE_GO_MINUS;
            g_phase_start_ms = t;
        }
    } else if (g_phase == PHASE_GO_MINUS) {
        g_target_cm = BALL_CENTER_CM - BALL_STEP_CM;
        if ((ball_cm - g_target_cm < 0.0f ? -(ball_cm - g_target_cm) : ball_cm - g_target_cm) < BALL_HOLD_TOL_CM ||
            (t - g_phase_start_ms > SEG_TIMEOUT_MS)) {
#if TEST_LOOP
            g_phase = PHASE_GO_PLUS;
#else
            g_phase = PHASE_DONE;
#endif
            g_phase_start_ms = t;
        }
    } else {
        g_target_cm = BALL_CENTER_CM - BALL_STEP_CM;   /* 停在 -5 */
    }
}

/* ==================== 控制步 (每收到一帧新视觉数据调用) ==================== */
static uint32_t g_last_ctrl_ms = 0;

static void control_step(void)
{
    uint32_t t = now_ms();

    if (!g_ctrl_started) {
        g_ctrl_started  = 1;
        g_phase_start_ms = t;
        g_total_start_ms = t;
    }

    float dt = (float)(t - g_last_ctrl_ms) * 0.001f;
    if (dt < 0.01f) dt = 0.01f;
    if (dt > 0.25f) dt = 0.25f;
    g_last_ctrl_ms = t;

    /* 轨迹推进 */
    trajectory_update(g_ball_cm, t);

    /* 目标变化 -> 清积分 (防换段冲量) */
    if (g_target_cm != g_last_target_cm) {
        g_integral = 0.0f;
        g_last_target_cm = g_target_cm;
    }

    /* PID */
    g_err_cm  = g_target_cm - g_ball_cm;
    g_lift_mm = pid_update(g_err_cm, g_ball_vel, dt);

    /* 抬升量 -> 步数 (LIFT_SIGN 处理方向) */
    int32_t steps = (int32_t)(LIFT_SIGN * g_lift_mm * TMC_STEPS_PER_MM);
    tmc2208_move_to(steps);
}

/* ==================== 显示 ==================== */
/* 紧凑带符号浮点: "+12.34" / "+10.2" */
static void fmt_snum(float v, int dec, int max_int, char *buf)
{
    char sign = '+';
    if (v < 0.0f) { sign = '-'; v = -v; }

    int scale = 1;
    for (int i = 0; i < dec; i++) scale *= 10;
    int iv = (int)v;
    int fr = (int)((v - (float)iv) * (float)scale + 0.5f);
    if (fr >= scale) { iv++; fr -= scale; }

    int cap = 1;
    for (int i = 0; i < max_int; i++) cap *= 10;
    if (iv >= cap) iv = cap - 1;

    char id[4];
    int  n = 0;
    do { id[n++] = (char)('0' + iv % 10); iv /= 10; } while (iv > 0);

    buf[0] = sign;
    int pos = 1;
    while (n > 0) buf[pos++] = id[--n];
    if (dec > 0) {
        buf[pos++] = '.';
        int d = scale / 10;
        for (int i = 0; i < dec; i++) { buf[pos++] = (char)('0' + (fr / d) % 10); d /= 10; }
    }
    buf[pos] = '\0';
}

/* 步数 -> "±04000" */
static void fmt_steps(int32_t v, char *buf)
{
    char sign = '+';
    if (v < 0) { sign = '-'; v = -v; }
    if (v > 99999) v = 99999;
    buf[0] = sign;
    buf[1] = (char)('0' + (v / 10000) % 10);
    buf[2] = (char)('0' + (v / 1000)  % 10);
    buf[3] = (char)('0' + (v / 100)   % 10);
    buf[4] = (char)('0' + (v / 10)    % 10);
    buf[5] = (char)('0' + v % 10);
    buf[6] = '\0';
}

static void oled_update(void)
{
    char s[16];

    SSD1306_Clear();
    SSD1306_ShowString(0, 20, "BALL CONTROL");

    if (ball_lost()) {
        SSD1306_ShowString(1, 0, "VISION LOST");
        SSD1306_ShowString(2, 0, "Hold position");
        SSD1306_ShowString(3, 0, "Check camera");
        SSD1306_Update();
        return;
    }

    /* 球位置 */
    SSD1306_ShowString(1, 0, "Ball:");
    fmt_snum(g_ball_cm, 2, 2, s);
    SSD1306_ShowString(1, 30, s);
    SSD1306_ShowString(1, 84, "cm");

    /* 目标 */
    SSD1306_ShowString(2, 0, "Tgt:");
    fmt_snum(g_target_cm, 2, 2, s);
    SSD1306_ShowString(2, 30, s);
    SSD1306_ShowString(2, 84, "cm");

    /* 误差 */
    SSD1306_ShowString(3, 0, "Err:");
    fmt_snum(g_err_cm, 2, 2, s);
    SSD1306_ShowString(3, 30, s);

    /* 速度 */
    SSD1306_ShowString(4, 0, "Vel:");
    fmt_snum(g_ball_vel, 1, 2, s);
    SSD1306_ShowString(4, 30, s);
    SSD1306_ShowString(4, 66, "cm/s");

    /* 抬升量 */
    SSD1306_ShowString(5, 0, "Lift:");
    fmt_snum(g_lift_mm, 1, 2, s);
    SSD1306_ShowString(5, 30, s);
    SSD1306_ShowString(5, 66, "mm");

    /* 步数 */
    SSD1306_ShowString(6, 0, "Stp:");
    fmt_steps(tmc2208_get_position(), s);
    SSD1306_ShowString(6, 30, s);

    /* 阶段 + 已用时间 */
    SSD1306_ShowString(7, 0, "Phs:");
    if (g_phase == PHASE_GO_PLUS)      SSD1306_ShowString(7, 24, "+5");
    else if (g_phase == PHASE_GO_MINUS) SSD1306_ShowString(7, 24, "-5");
    else                                SSD1306_ShowString(7, 24, "OK ");
    SSD1306_ShowString(7, 42, "T:");
    fmt_snum((float)(now_ms() - g_total_start_ms) * 0.001f, 1, 1, s);
    SSD1306_ShowString(7, 60, s);
    SSD1306_ShowString(7, 84, "s");

    SSD1306_Update();
}

/* ==================== 主函数 ==================== */
int main(void)
{
    SYSCFG_DL_init();
    SysTick_Config(CPUCLK_FREQ / 1000);

    /* 电机: 上电前把水管调到水平 (螺母 6.5cm 高), 当前位置 = 0 */
    tmc2208_init();
    tmc2208_set_max_speed(STEPPER_MAX_SPEED);
    tmc2208_set_accel(STEPPER_ACCEL);
    tmc2208_set_current_position(0);
    tmc2208_enable();

    /* OLED */
    delay_ms(50);
    SSD1306_Init();
    SSD1306_Clear();
    SSD1306_ShowString(0, 16, "BALL CONTROL");
    SSD1306_ShowString(2, 10, "PID Init...");
    SSD1306_ShowString(4, 10, "Waiting vision");
    SSD1306_Update();

    /* UART */
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    uint32_t last_ui = 0;

    while (1) {
        /* 1. 解析串口 */
        process_uart();

        /* 2. 新视觉帧 -> 控制 */
        if (g_ball_new) {
            g_ball_new = 0;
            control_step();
        }

        /* 3. OLED */
        uint32_t t = now_ms();
        if (t - last_ui >= 100) {
            last_ui = t;
            oled_update();
        }
    }
}
