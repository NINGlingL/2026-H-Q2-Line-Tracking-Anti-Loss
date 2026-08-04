/**
 *  TB6612 电机驱动库 实现
 */
#include "moto.h"

/* ========== 内部常量 ========== */
#define PWM_PERIOD  1000U

/* ========== 当前状态 ========== */
static char    g_moto_dir   = 'S';
static uint8_t g_moto_speed = 0;

/*
 *  PWM 极性说明:
 *  SysConfig 生成 TIMG0 的 CC_OCTL = INIT_VAL_LOW
 *  输出: LOW(0..CC) → HIGH(CC..period)
 *  所以 HIGH 占空比 = (period - CC) / period
 *  要用 CC = period - (pct * period / 100) 来补偿
 */

/* ========== 初始化 ========== */

void Moto_Init(void)
{
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, PWM_PERIOD, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, PWM_PERIOD, DL_TIMER_CC_1_INDEX);
    DL_GPIO_clearPins(GPIOB, AIN2_PIN_3_PIN | AIN1_PIN_2_PIN |
                              BIN2_PIN_0_PIN | BN1_PIN_1_PIN);
    g_moto_dir   = 'S';
    g_moto_speed = 0;
}

/* ========== 辅助: 百分比 → CC 寄存器值 ========== */

static uint32_t pct_to_cc(uint8_t pct)
{
    if (pct > 100) pct = 100;
    return PWM_PERIOD - ((uint32_t)pct * PWM_PERIOD / 100U);
}

/* ========== 辅助: 设置 PWM ========== */

static void pwm_set(uint8_t pct)
{
    uint32_t cc = pct_to_cc(pct);
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, cc, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, cc, DL_TIMER_CC_1_INDEX);
}

/* ========== 基本控制 ========== */

void Moto_Forward(uint8_t pct)
{
    DL_GPIO_clearPins(GPIOB, AIN2_PIN_3_PIN | AIN1_PIN_2_PIN |
                              BIN2_PIN_0_PIN | BN1_PIN_1_PIN);
    DL_GPIO_setPins(GPIOB, AIN1_PIN_2_PIN | BN1_PIN_1_PIN);
    pwm_set(pct);
    g_moto_dir   = 'F';
    g_moto_speed = (pct > 100) ? 100 : pct;
}

void Moto_Backward(uint8_t pct)
{
    DL_GPIO_clearPins(GPIOB, AIN2_PIN_3_PIN | AIN1_PIN_2_PIN |
                              BIN2_PIN_0_PIN | BN1_PIN_1_PIN);
    DL_GPIO_setPins(GPIOB, AIN2_PIN_3_PIN | BIN2_PIN_0_PIN);
    pwm_set(pct);
    g_moto_dir   = 'B';
    g_moto_speed = (pct > 100) ? 100 : pct;
}

void Moto_Left(uint8_t pct)
{
    DL_GPIO_clearPins(GPIOB, AIN2_PIN_3_PIN | AIN1_PIN_2_PIN |
                              BIN2_PIN_0_PIN | BN1_PIN_1_PIN);
    DL_GPIO_setPins(GPIOB, BN1_PIN_1_PIN);
    pwm_set(pct);
    g_moto_dir   = 'L';
    g_moto_speed = (pct > 100) ? 100 : pct;
}

void Moto_Right(uint8_t pct)
{
    DL_GPIO_clearPins(GPIOB, AIN2_PIN_3_PIN | AIN1_PIN_2_PIN |
                              BIN2_PIN_0_PIN | BN1_PIN_1_PIN);
    DL_GPIO_setPins(GPIOB, AIN1_PIN_2_PIN);
    pwm_set(pct);
    g_moto_dir   = 'R';
    g_moto_speed = (pct > 100) ? 100 : pct;
}

void Moto_Stop(void)
{
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, PWM_PERIOD, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, PWM_PERIOD, DL_TIMER_CC_1_INDEX);
    DL_GPIO_clearPins(GPIOB, AIN2_PIN_3_PIN | AIN1_PIN_2_PIN |
                              BIN2_PIN_0_PIN | BN1_PIN_1_PIN);
    g_moto_dir   = 'S';
    g_moto_speed = 0;
}

void Moto_SetSpeed(uint8_t pct)
{
    pwm_set(pct);
    g_moto_speed = (pct > 100) ? 100 : pct;
}

uint8_t Moto_SpeedUp(uint8_t delta)
{
    uint16_t ns = g_moto_speed + delta;
    if (ns > 100) ns = 100;
    pwm_set((uint8_t)ns);
    g_moto_speed = (uint8_t)ns;
    return (uint8_t)ns;
}

uint8_t Moto_SpeedDown(uint8_t delta)
{
    int16_t ns = (int16_t)g_moto_speed - (int16_t)delta;
    if (ns < 0) ns = 0;
    pwm_set((uint8_t)ns);
    g_moto_speed = (uint8_t)ns;
    if (ns == 0) Moto_Stop();
    return (uint8_t)ns;
}

/* ========== 状态查询 ========== */

char    Moto_GetDirection(void) { return g_moto_dir; }
uint8_t Moto_GetSpeed(void)     { return g_moto_speed; }

uint32_t Moto_GetCC0(void)
{
    return DL_TimerG_getCaptureCompareValue(PWM_0_INST, DL_TIMER_CC_0_INDEX);
}

uint32_t Moto_GetCC1(void)
{
    return DL_TimerG_getCaptureCompareValue(PWM_0_INST, DL_TIMER_CC_1_INDEX);
}

uint32_t Moto_GetGPIO(void)
{
    return DL_GPIO_readPins(GPIOB, AIN2_PIN_3_PIN | AIN1_PIN_2_PIN |
                                    BIN2_PIN_0_PIN | BN1_PIN_1_PIN);
}

/* ========== 开机自检 ========== */

void Moto_SelfTest(Moto_SelfTestCallback cb)
{
    if (cb) cb(1, 80, 1);
    Moto_Forward(80);
    delay_cycles(CPUCLK_FREQ * 1);

    if (cb) cb(2, 50, 2);
    Moto_SetSpeed(50);
    delay_cycles(CPUCLK_FREQ * 2);

    if (cb) cb(3, 20, 2);
    Moto_SetSpeed(20);
    delay_cycles(CPUCLK_FREQ * 2);

    Moto_Stop();
    if (cb) cb(0, 0, 0);
}
