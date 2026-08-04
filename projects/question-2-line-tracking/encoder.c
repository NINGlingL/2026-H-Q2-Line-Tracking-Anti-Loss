#include "encoder.h"
#include "app_config.h"
#include "ti_msp_dl_config.h"
#include <limits.h>

#define ENCODER_MAX_SAMPLE_MS     (50UL)
#define ENCODER_DELTA_MARGIN      (8UL)
#define ENCODER_SIGNAL_HOLD_MS    (500UL)

static Encoder_State g_encoder;
static uint16_t g_previous_count;
static uint32_t g_last_pulse_ms;

void Encoder_Init(void)
{
    g_encoder.total = 0;
    g_encoder.delta = 0;
    g_encoder.speed_mm_s = 0;
    g_encoder.valid = 0U;
    g_encoder.sample_ms = 0U;
    g_last_pulse_ms = 0U;

    /*
     * Hall encoder outputs are commonly open-drain. Keep the SysConfig QEI
     * mux while explicitly enabling the MCU pull-ups on PB10/PB11.
     */
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_QEI_R_PHA_IOMUX, GPIO_QEI_R_PHA_IOMUX_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_QEI_R_PHB_IOMUX, GPIO_QEI_R_PHB_IOMUX_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    /* QEI inputs are explicitly inputs before the counter is started. */
    DL_TimerG_stopCounter(QEI_R_INST);
    DL_TimerG_setCCPDirection(
        QEI_R_INST, DL_TIMER_CC0_INPUT | DL_TIMER_CC1_INPUT);
    DL_TimerG_setTimerCount(QEI_R_INST, 0U);
    g_previous_count = 0U;
    DL_TimerG_startCounter(QEI_R_INST);
}

void Encoder_Sample(uint32_t now_ms)
{
    uint32_t elapsed_ms =
        (uint32_t) (now_ms - g_encoder.sample_ms);
    uint32_t max_delta;
    uint16_t current_count;
    int64_t speed;
    int32_t delta;

    current_count =
        (uint16_t) DL_TimerG_getTimerCount(QEI_R_INST);
    /* Signed 16-bit subtraction handles counter wrap in either direction. */
    delta = (int32_t) (int16_t) (current_count - g_previous_count);
    g_previous_count = current_count;

    if (g_encoder.sample_ms == 0U) {
        elapsed_ms = APP_CONTROL_PERIOD_MS;
    }
    max_delta =
        (APP_ENCODER_COUNTS_PER_REV * APP_MOTOR_MAX_VALID_RPM *
         elapsed_ms) / 60000UL + ENCODER_DELTA_MARGIN;

    if (elapsed_ms == 0U || elapsed_ms > ENCODER_MAX_SAMPLE_MS ||
        delta > (int32_t) max_delta ||
        delta < -(int32_t) max_delta) {
        g_encoder.delta = 0;
        g_encoder.speed_mm_s = 0;
        g_encoder.valid = 0U;
    } else {
        g_encoder.delta = (int16_t) delta;
        speed =
            (int64_t) delta * (int64_t) APP_WHEEL_CIRCUMFERENCE_UM /
            ((int64_t) APP_ENCODER_COUNTS_PER_REV *
             (int64_t) elapsed_ms);
        if (speed > INT16_MAX) {
            speed = INT16_MAX;
        } else if (speed < INT16_MIN) {
            speed = INT16_MIN;
        }
        g_encoder.speed_mm_s = (int16_t) speed;

        if (delta > 0 && g_encoder.total > INT32_MAX - delta) {
            g_encoder.total = INT32_MAX;
        } else if (delta < 0 &&
                   g_encoder.total < INT32_MIN - delta) {
            g_encoder.total = INT32_MIN;
        } else {
            g_encoder.total += delta;
        }
        if (delta != 0) {
            g_last_pulse_ms = now_ms;
        }
        g_encoder.valid =
            (g_last_pulse_ms != 0U &&
             (uint32_t) (now_ms - g_last_pulse_ms) <=
                ENCODER_SIGNAL_HOLD_MS) ? 1U : 0U;
    }
    g_encoder.sample_ms = now_ms;
}

Encoder_State Encoder_GetState(void)
{
    return g_encoder;
}
