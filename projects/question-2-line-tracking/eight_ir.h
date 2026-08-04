#ifndef EIGHT_IR_H
#define EIGHT_IR_H

#include <stdint.h>

typedef struct {
    uint8_t channels[8];      /* 0 = black line, 1 = white background */
    uint8_t raw;
    uint8_t active_count;
    int8_t position;          /* -7 left to +7 right */
    uint8_t frame_valid;
    uint8_t frame_fresh;
    uint8_t warming_up;
    uint32_t last_frame_ms;
    uint32_t good_frames;
    uint32_t bad_frames;
    uint32_t overflow_bytes;
} EightIR_State;

void EightIR_Init(uint32_t now_ms);
void EightIR_Service(uint32_t now_ms);
void EightIR_SetReversed(uint8_t reversed);
uint8_t EightIR_IsReversed(void);
EightIR_State EightIR_GetState(void);
void EightIR_StartCalibrate(void);

#endif
