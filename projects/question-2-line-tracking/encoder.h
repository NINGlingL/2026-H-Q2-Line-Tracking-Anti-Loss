#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef struct {
    int32_t total;
    int16_t delta;
    int16_t speed_mm_s;
    uint8_t valid;
    uint32_t sample_ms;
} Encoder_State;

void Encoder_Init(void);
void Encoder_Sample(uint32_t now_ms);
Encoder_State Encoder_GetState(void);

#endif
