#include "encoder.h"
#include "ti_msp_dl_config.h"

#define ENCODER_CENTER_COUNT      (32768U)
#define ENCODER_MAX_TICK_DELTA    (4096)

static Encoder_State g_encoder;

void Encoder_Init(void)
{
    g_encoder.total = 0;
    g_encoder.delta = 0;
    g_encoder.valid = 1U;
    g_encoder.sample_ms = 0U;
    DL_TimerG_setTimerCount(QEI_R_INST, ENCODER_CENTER_COUNT);
    DL_TimerG_startCounter(QEI_R_INST);
}

void Encoder_Sample(uint32_t now_ms)
{
    int32_t delta =
        (int32_t) DL_TimerG_getTimerCount(QEI_R_INST) -
        (int32_t) ENCODER_CENTER_COUNT;

    DL_TimerG_setTimerCount(QEI_R_INST, ENCODER_CENTER_COUNT);

    if (delta > ENCODER_MAX_TICK_DELTA ||
        delta < -ENCODER_MAX_TICK_DELTA) {
        g_encoder.delta = 0;
        g_encoder.valid = 0U;
    } else {
        g_encoder.delta = (int16_t) delta;
        g_encoder.total += delta;
        g_encoder.valid = 1U;
    }
    g_encoder.sample_ms = now_ms;
}

Encoder_State Encoder_GetState(void)
{
    return g_encoder;
}
