#ifndef MOTO_H
#define MOTO_H

#include <stdint.h>

typedef struct {
    int16_t left_permille;
    int16_t right_permille;
    uint8_t safety_permit;
    uint8_t standby_enabled;
    uint8_t hardware_locked;
} Moto_State;

void Moto_Init(void);
void Moto_SetSafetyPermit(uint8_t permit);
void Moto_SetLR(int16_t left_permille, int16_t right_permille);
void Moto_ActiveBrake(void);
void Moto_Stop(void);
void Moto_EmergencyStop(void);
Moto_State Moto_GetState(void);

#endif
