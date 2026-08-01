/*
 * TMC2208 步进电机驱动实现 (MSPM0G3507)
 *
 * STEP 由 TIMA0 PWM 输出到 PA8, 每个 PWM 周期 = 一个步进脉冲。
 * TIMA0 的 zero 事件中断每个周期触发一次, 在其中:
 *   1. 计数步数 / 判断到达目标
 *   2. 按梯形曲线更新下一步的速度 (改 LOAD/CC 寄存器)
 *
 * 定时器时钟 4MHz (BUSCLK 32MHz / 8)。
 */

#include "tmc2208.h"
#include "ti_msp_dl_config.h"

/* ==================== 定时器参数 ==================== */
#define STEP_CLK_HZ        STEP_INST_CLK_FREQ /* 32 MHz BUSCLK / 8 = 4 MHz */
#define HARD_MAX_SPEED_SPS 12000.0f
#define HARD_MAX_ACCEL_SPS2 80000.0f
/* 实机确认：DIR 高电平时，移动端从丝杆最低点向上运动。 */
#define TMC_UP_DIRECTION_PIN_LEVEL 1
#define HARD_MIN_LIMIT_STEPS ((int32_t)(TMC_HARD_MIN_MM * TMC_STEPS_PER_MM))
#define HARD_MAX_LIMIT_STEPS ((int32_t)(TMC_HARD_MAX_MM * TMC_STEPS_PER_MM))

/* ==================== 内部状态 ==================== */
static volatile int32_t  s_position  = 0;      /* 当前绝对位置 (步) */
static volatile int32_t  s_target    = 0;      /* 目标位置 (步) */
static volatile int32_t  s_remaining = 0;      /* 剩余步数 (正数) */
static volatile int8_t   s_dir       = 1;      /* 当前方向 +1 / -1 */
static volatile bool     s_moving    = false;
static volatile bool     s_enabled   = false;

/* ==================== 速度 / 加减速 (步/s, 步/s^2) ==================== */
static volatile float s_max_speed = 5000.0f;
static volatile float s_accel     = 24000.0f;
static const float    s_min_speed = 200.0f;    /* 起步 / 低速兜底 */
static volatile float s_cur_speed = 0.0f;      /* 当前速度 */
static volatile bool  s_decel     = false;     /* 已进入减速段 */

/* 软限位使用最低点起算的绝对坐标，默认覆盖 0~100 mm。 */
static int32_t s_min_limit = HARD_MIN_LIMIT_STEPS;
static int32_t s_max_limit = HARD_MAX_LIMIT_STEPS;

/* ==================== 内部函数 ==================== */

/* 设置步进间隔 (定时器 tick 数), LOAD=interval-1, CC=LOAD/2 -> 50% 占空比 */
static void set_step_interval(uint32_t interval)
{
    uint32_t load;

    /* 寄存器写入前同函数硬限幅：TIMA0 LOAD 为 16 位，且至少留 3 tick。 */
    if (interval > 65536U) interval = 65536U;
    if (interval < 3U)     interval = 3U;
    load = interval - 1U;

    DL_TimerA_setLoadValue(STEP_INST, load);
    DL_TimerA_setCaptureCompareValue(STEP_INST, load / 2U, DL_TIMER_CC_0_INDEX);
}

/* 开始按指定间隔发脉冲 */
static void start_stepping(uint32_t interval)
{
    set_step_interval(interval);
    DL_TimerA_clearInterruptStatus(STEP_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_enableInterrupt(STEP_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_startCounter(STEP_INST);
    s_moving = true;
}

/* 停止发脉冲 */
static void stop_stepping(void)
{
    DL_TimerA_stopCounter(STEP_INST);
    DL_TimerA_disableInterrupt(STEP_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    s_moving = false;
}

static void set_direction_pin(int8_t direction)
{
    if (direction > 0) {
#if TMC_UP_DIRECTION_PIN_LEVEL
        DL_GPIO_setPins(TMC2208_PORT, TMC2208_DIR_PIN);
#else
        DL_GPIO_clearPins(TMC2208_PORT, TMC2208_DIR_PIN);
#endif
    } else {
#if TMC_UP_DIRECTION_PIN_LEVEL
        DL_GPIO_clearPins(TMC2208_PORT, TMC2208_DIR_PIN);
#else
        DL_GPIO_setPins(TMC2208_PORT, TMC2208_DIR_PIN);
#endif
    }
}

/* ==================== API ==================== */

void tmc2208_init(void)
{
    /* DIR/EN 已由 SysConfig 配置为输出; 这里设成安全默认: 禁用驱动 */
    DL_GPIO_setPins(TMC2208_PORT, TMC2208_EN_PIN);    /* EN 高 = 禁用 */
    set_direction_pin(1);                            /* 正方向：从最低点向上 */

    stop_stepping();
    s_enabled  = false;
    s_position = 0;
    s_target   = 0;
    s_remaining= 0;
    s_dir      = 1;
    s_cur_speed= 0.0f;
    s_decel    = false;

    NVIC_EnableIRQ(STEP_INST_INT_IRQN);
}

void tmc2208_set_max_speed(float sps)
{
    if (sps < 100.0f)  sps = 100.0f;
    if (sps > HARD_MAX_SPEED_SPS) sps = HARD_MAX_SPEED_SPS;
    s_max_speed = sps;
}

void tmc2208_set_accel(float sps2)
{
    if (sps2 < 10.0f) sps2 = 10.0f;
    if (sps2 > HARD_MAX_ACCEL_SPS2) sps2 = HARD_MAX_ACCEL_SPS2;
    s_accel = sps2;
}

void tmc2208_set_limits_mm(float min_mm, float max_mm)
{
    int32_t min_steps;
    int32_t max_steps;

    if (min_mm < TMC_HARD_MIN_MM) min_mm = TMC_HARD_MIN_MM;
    if (max_mm > TMC_HARD_MAX_MM) max_mm = TMC_HARD_MAX_MM;
    if (min_mm >= max_mm) {
        min_mm = TMC_HARD_MIN_MM;
        max_mm = TMC_HARD_MAX_MM;
    }

    min_steps = (int32_t)(min_mm * TMC_STEPS_PER_MM);
    max_steps = (int32_t)(max_mm * TMC_STEPS_PER_MM);
    if (min_steps < HARD_MIN_LIMIT_STEPS) min_steps = HARD_MIN_LIMIT_STEPS;
    if (max_steps > HARD_MAX_LIMIT_STEPS) max_steps = HARD_MAX_LIMIT_STEPS;
    s_min_limit = min_steps;
    s_max_limit = max_steps;
}

void tmc2208_set_current_position(int32_t steps)
{
    stop_stepping();
    s_position = steps;
    s_target   = steps;
    s_remaining= 0;
    s_cur_speed= 0.0f;
    s_decel    = false;
}

void tmc2208_enable(void)
{
    stop_stepping();
    DL_GPIO_clearPins(TMC2208_PORT, TMC2208_EN_PIN);   /* EN 低 = 使能 */
    s_enabled = true;
}

void tmc2208_disable(void)
{
    stop_stepping();
    DL_GPIO_setPins(TMC2208_PORT, TMC2208_EN_PIN);     /* EN 高 = 禁用 */
    s_enabled = false;
}

bool tmc2208_is_enabled(void)
{
    return s_enabled;
}

void tmc2208_move_to(int32_t target)
{
    /*
     * 执行器入口硬限幅：先限制到 100 mm 物理行程，再限制到应用软限位，
     * 全部在启动 STEP 寄存器之前完成。
     */
    if (target < HARD_MIN_LIMIT_STEPS) target = HARD_MIN_LIMIT_STEPS;
    if (target > HARD_MAX_LIMIT_STEPS) target = HARD_MAX_LIMIT_STEPS;
    if (target < s_min_limit) target = s_min_limit;
    if (target > s_max_limit) target = s_max_limit;

    if (!s_enabled) {
        s_target = s_position;
        s_remaining = 0;
        return;
    }

    if (s_moving) {
        /* 移动中重新定位 (球控每 50ms 更新目标):
         * 同方向 -> 只更新目标/剩余步数, 保持当前速度与加减速状态, 平滑追踪
         * 反方向 -> 先停, 再反向启动
         */
        if (target == s_position) {
            /* 目标=当前位置: 立即停住 */
            stop_stepping();
            s_target = s_position;
            return;
        }

        int8_t new_dir = (target > s_position) ? 1 : -1;
        if (new_dir == s_dir) {
            s_target    = target;
            s_remaining = (s_dir > 0) ? (target - s_position) : (s_position - target);
            if (s_remaining <= 0) {
                stop_stepping();
                s_target = s_position;
                return;
            }
            s_decel = false;   /* 距离变远, 允许继续加速 */
            return;
        }

        /* 反向: 停住再从当前点反向 */
        stop_stepping();
        s_target    = target;
        s_dir       = new_dir;
        s_remaining = (s_dir > 0) ? (target - s_position) : (s_position - target);
        s_decel     = false;
        s_cur_speed = s_min_speed;

        set_direction_pin(s_dir);
        start_stepping((uint32_t)((float)STEP_CLK_HZ / s_min_speed));
        return;
    }

    /* 空闲: 从当前点启动新移动 */
    if (target == s_position) {
        s_target = target;
        return;
    }

    s_target    = target;
    s_dir       = (target > s_position) ? 1 : -1;
    s_remaining = (s_dir > 0) ? (target - s_position) : (s_position - target);
    s_decel     = false;
    s_cur_speed = s_min_speed;

    /* 先定方向, 再开始发脉冲 */
    set_direction_pin(s_dir);

    uint32_t interval = (uint32_t)((float)STEP_CLK_HZ / s_min_speed);
    start_stepping(interval);
}

void tmc2208_move_steps(int32_t steps)
{
    int64_t target = (int64_t)s_position + (int64_t)steps;
    if (target < (int64_t)HARD_MIN_LIMIT_STEPS) target = HARD_MIN_LIMIT_STEPS;
    if (target > (int64_t)HARD_MAX_LIMIT_STEPS) target = HARD_MAX_LIMIT_STEPS;
    tmc2208_move_to((int32_t)target);
}

void tmc2208_stop(void)
{
    s_target    = s_position;
    s_remaining = 0;
    s_cur_speed = 0.0f;
    stop_stepping();
}

bool tmc2208_is_moving(void)
{
    return s_moving;
}

int32_t tmc2208_get_position(void) { return s_position; }
int32_t tmc2208_get_target(void)   { return s_target; }

int8_t tmc2208_get_direction(void)
{
    return s_dir;
}

float tmc2208_get_current_speed(void)
{
    return s_cur_speed;
}

/* ==================== 步进 ISR ====================
 * 每个 PWM 周期 (= 一个步进脉冲) 触发一次。
 * 本函数执行时, 当前脉冲已经输出, 位置已前进一步。
 */
void tmc2208_step_isr(void)
{
    s_position += s_dir;
    s_remaining--;

    /* 到达目标 */
    if (s_remaining <= 0) {
        s_target    = s_position;
        s_cur_speed = 0.0f;
        s_decel     = false;
        stop_stepping();
        return;
    }

    /* ---- 梯形速度规划 (Euler 离散) ---- */
    float v    = s_cur_speed;
    float stop_dist = (v * v) / (2.0f * s_accel);   /* 当前速度刹停所需步数 */
    float dt   = 1.0f / v;                          /* 本步耗时 (s) */

    if (!s_decel && ((float)s_remaining > stop_dist)) {
        /* 加速段 (或达到上限后匀速) */
        v += s_accel * dt;
        if (v > s_max_speed) v = s_max_speed;
    } else {
        /* 减速段: 一旦进入不再回加速, 保证平滑刹停 */
        s_decel = true;
        v -= s_accel * dt;
        if (v < s_min_speed) v = s_min_speed;
    }
    s_cur_speed = v;

    /* 设置下一步的间隔 */
    uint32_t interval = (uint32_t)((float)STEP_CLK_HZ / v);
    set_step_interval(interval);
}

/* TIMA0 中断服务函数 (zero 事件 = 一个周期完成) */
void TIMA0_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(STEP_INST) == DL_TIMERA_IIDX_ZERO) {
        DL_TimerA_clearInterruptStatus(STEP_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
        tmc2208_step_isr();
    }
}
