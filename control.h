#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

typedef enum {
    CONTROL_SAFE = 0,
    CONTROL_DIAGNOSTIC,
    CONTROL_AUTO_TRACK,
    CONTROL_FAULT
} Control_Mode;

typedef struct {
    Control_Mode mode;
    int16_t left_command;
    int16_t right_command;
    int16_t encoder_target;
    uint8_t lap_count;
    uint8_t hardware_locked;
    uint32_t mode_enter_ms;
} Control_State;

void Control_Init(uint32_t now_ms);
void Control_Service(uint32_t now_ms);
Control_State Control_GetState(void);
void Control_RequestSafe(void);

#endif
