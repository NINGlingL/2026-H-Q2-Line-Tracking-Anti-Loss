#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

void Timebase_Init(void);
uint32_t Timebase_Millis(void);
uint8_t Timebase_Elapsed(uint32_t now, uint32_t then, uint32_t interval_ms);

#endif
