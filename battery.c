#include "battery.h"
#include "app_config.h"
#include "ti_msp_dl_config.h"

static Battery_State g_battery;
static uint32_t g_last_sample_ms;
static uint8_t g_low_count;

void Battery_Init(void)
{
    g_battery.raw = 0U;
    g_battery.millivolts = 0U;
    g_battery.configured =
        ((APP_MOTOR_HW_READY != 0U) && (APP_BATTERY_LOW_MV != 0UL)) ? 1U : 0U;
    g_battery.valid = 0U;
    g_battery.low_latched = 0U;
    g_last_sample_ms = 0U;
    g_low_count = 0U;
}

void Battery_Service(uint32_t now_ms)
{
    uint32_t scaled_mv;
    uint32_t timeout;

    if ((uint32_t) (now_ms - g_last_sample_ms) < 100U) {
        return;
    }
    g_last_sample_ms = now_ms;

    DL_ADC12_clearInterruptStatus(
        BAT_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_startConversion(BAT_ADC_INST);
    timeout = 320000U; /* 10 ms at the configured 32 MHz SYSOSC. */
    while ((DL_ADC12_getRawInterruptStatus(
                BAT_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) == 0U) &&
           (timeout > 0U)) {
        timeout--;
    }
    if (timeout == 0U) {
        g_battery.valid = 0U;
        DL_ADC12_stopConversion(BAT_ADC_INST);
        DL_ADC12_enableConversions(BAT_ADC_INST);
        return;
    }

    g_battery.raw =
        (uint16_t) DL_ADC12_getMemResult(BAT_ADC_INST, BAT_ADC_ADCMEM_0);
    DL_ADC12_clearInterruptStatus(
        BAT_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableConversions(BAT_ADC_INST);

    scaled_mv = ((uint32_t) g_battery.raw * 3300UL *
                    (APP_BATTERY_DIV_TOP_OHM + APP_BATTERY_DIV_BOTTOM_OHM)) /
                (4095UL * APP_BATTERY_DIV_BOTTOM_OHM);
    g_battery.millivolts = scaled_mv;
    g_battery.valid = (g_battery.raw > 8U && g_battery.raw < 4088U) ? 1U : 0U;

    if (g_battery.configured == 0U || g_battery.valid == 0U) {
        return;
    }

    if (g_battery.millivolts < APP_BATTERY_LOW_MV) {
        if (g_low_count < 3U) {
            g_low_count++;
        }
        if (g_low_count >= 3U) {
            g_battery.low_latched = 1U;
        }
    } else {
        g_low_count = 0U;
    }
}

Battery_State Battery_GetState(void)
{
    return g_battery;
}

uint8_t Battery_IsSafe(void)
{
    if (g_battery.configured == 0U || g_battery.valid == 0U) {
        return 0U;
    }
    return (g_battery.low_latched == 0U) ? 1U : 0U;
}
