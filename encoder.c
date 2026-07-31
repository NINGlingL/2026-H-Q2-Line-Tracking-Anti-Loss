#include "encoder.h"
#include "app_config.h"
#include "ti_msp_dl_config.h"
#include <limits.h>

#define ENCODER_CENTER_COUNT      (32768U)
#define ENCODER_MAX_SAMPLE_MS     (50UL)
#define ENCODER_DELTA_MARGIN      (8UL)

static Encoder_State g_encoder;

void Encoder_Init(void)
{
    g_encoder.total = 0;
    g_encoder.delta = 0;
    g_encoder.speed_mm_s = 0;
    g_encoder.valid = 1U;
    g_encoder.sample_ms = 0U;
    DL_TimerG_setTimerCount(QEI_R_INST, ENCODER_CENTER_COUNT);
    DL_TimerG_startCounter(QEI_R_INST);
}

void Encoder_Sample(uint32_t now_ms)
{
    uint32_t elapsed_ms =
        (uint32_t) (now_ms - g_encoder.sample_ms);
    uint32_t max_delta;
    int64_t speed;
    int32_t delta =
        (int32_t) DL_TimerG_getTimerCount(QEI_R_INST) -
        (int32_t) ENCODER_CENTER_COUNT;

    DL_TimerG_setTimerCount(QEI_R_INST, ENCODER_CENTER_COUNT);

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
        g_encoder.valid = 1U;
    }
    g_encoder.sample_ms = now_ms;
}

Encoder_State Encoder_GetState(void)
{
    return g_encoder;
}
