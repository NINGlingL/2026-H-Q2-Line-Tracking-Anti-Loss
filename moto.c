#include "moto.h"
#include "app_config.h"
#include "ti_msp_dl_config.h"

#define PWM_PERIOD_COUNTS (200U)

static Moto_State g_motor;

static uint32_t command_to_compare(int16_t command)
{
    uint32_t magnitude;

    if (command < 0) {
        magnitude = (uint32_t) (-command);
    } else {
        magnitude = (uint32_t) command;
    }
    return PWM_PERIOD_COUNTS -
           (magnitude * PWM_PERIOD_COUNTS / 1000U);
}

static void set_right_direction(int16_t command)
{
    DL_GPIO_clearPins(MOTO_PORT, MOTO_AIN1_PIN | MOTO_AIN2_PIN);
    if (command > 0) {
        DL_GPIO_setPins(MOTO_PORT, MOTO_AIN2_PIN);
    } else if (command < 0) {
        DL_GPIO_setPins(MOTO_PORT, MOTO_AIN1_PIN);
    }
}

static void set_left_direction(int16_t command)
{
    DL_GPIO_clearPins(MOTO_PORT, MOTO_BIN1_PIN | MOTO_BIN2_PIN);
    if (command > 0) {
        DL_GPIO_setPins(MOTO_PORT, MOTO_BIN2_PIN);
    } else if (command < 0) {
        DL_GPIO_setPins(MOTO_PORT, MOTO_BIN1_PIN);
    }
}

void Moto_Init(void)
{
    g_motor.left_permille = 0;
    g_motor.right_permille = 0;
    g_motor.safety_permit = 0U;
    g_motor.standby_enabled = 0U;
    g_motor.hardware_locked = (APP_MOTOR_HW_READY == 0U) ? 1U : 0U;
    Moto_EmergencyStop();
}

void Moto_SetSafetyPermit(uint8_t permit)
{
    if ((permit == 0U) || (APP_MOTOR_HW_READY == 0U)) {
        g_motor.safety_permit = 0U;
        Moto_EmergencyStop();
        return;
    }
    g_motor.safety_permit = 1U;
}

void Moto_SetLR(int16_t left_permille, int16_t right_permille)
{
    uint32_t left_compare;
    uint32_t right_compare;

    /*
     * P0 actuator hard limits are deliberately inline in the function that
     * writes the physical registers. They do not rely on caller validation.
     */
    if (left_permille > 700) {
        left_permille = 700;
    }
    if (left_permille < -700) {
        left_permille = -700;
    }
    if (right_permille > 700) {
        right_permille = 700;
    }
    if (right_permille < -700) {
        right_permille = -700;
    }

    if ((g_motor.safety_permit == 0U) ||
        (APP_MOTOR_HW_READY == 0U)) {
        Moto_EmergencyStop();
        return;
    }

    /*
     * Remove bridge drive before changing direction to prevent shoot-through
     * during a sign reversal.
     */
    DL_TimerA_setCaptureCompareValue(
        PWM_0_INST, PWM_PERIOD_COUNTS, DL_TIMER_CC_0_INDEX);
    DL_TimerA_setCaptureCompareValue(
        PWM_0_INST, PWM_PERIOD_COUNTS, DL_TIMER_CC_1_INDEX);

    set_right_direction(right_permille);
    set_left_direction(left_permille);

    right_compare = command_to_compare(right_permille);
    left_compare = command_to_compare(left_permille);

    if (right_compare > PWM_PERIOD_COUNTS) {
        right_compare = PWM_PERIOD_COUNTS;
    }
    if (left_compare > PWM_PERIOD_COUNTS) {
        left_compare = PWM_PERIOD_COUNTS;
    }

    DL_GPIO_setPins(STBY_PORT, STBY_STBY3_PIN);
    g_motor.standby_enabled = 1U;
    DL_TimerA_setCaptureCompareValue(
        PWM_0_INST, right_compare, DL_TIMER_CC_0_INDEX);
    DL_TimerA_setCaptureCompareValue(
        PWM_0_INST, left_compare, DL_TIMER_CC_1_INDEX);

    g_motor.left_permille = left_permille;
    g_motor.right_permille = right_permille;
}

void Moto_Stop(void)
{
    Moto_EmergencyStop();
}

void Moto_EmergencyStop(void)
{
    /* Minimal safe sequence: PWM zero, STBY low, direction pins low. */
    DL_TimerA_setCaptureCompareValue(
        PWM_0_INST, PWM_PERIOD_COUNTS, DL_TIMER_CC_0_INDEX);
    DL_TimerA_setCaptureCompareValue(
        PWM_0_INST, PWM_PERIOD_COUNTS, DL_TIMER_CC_1_INDEX);
    DL_GPIO_clearPins(STBY_PORT, STBY_STBY3_PIN);
    DL_GPIO_clearPins(MOTO_PORT,
        MOTO_AIN1_PIN | MOTO_AIN2_PIN | MOTO_BIN1_PIN | MOTO_BIN2_PIN);
    g_motor.left_permille = 0;
    g_motor.right_permille = 0;
    g_motor.standby_enabled = 0U;
}

Moto_State Moto_GetState(void)
{
    return g_motor;
}
