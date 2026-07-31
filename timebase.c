#include "timebase.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t g_millis;

void Timebase_Init(void)
{
    g_millis = 0U;
    (void) SysTick_Config(CPUCLK_FREQ / 1000U);
}

uint32_t Timebase_Millis(void)
{
    return g_millis;
}

uint8_t Timebase_Elapsed(uint32_t now, uint32_t then, uint32_t interval_ms)
{
    return ((uint32_t) (now - then) >= interval_ms) ? 1U : 0U;
}

void SysTick_Handler(void)
{
    g_millis++;
}
