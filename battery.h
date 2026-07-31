#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>

typedef struct {
    uint16_t raw;
    uint32_t millivolts;
    uint8_t configured;
    uint8_t valid;
    uint8_t low_latched;
} Battery_State;

void Battery_Init(void);
void Battery_Service(uint32_t now_ms);
Battery_State Battery_GetState(void);
uint8_t Battery_IsSafe(void);

#endif
