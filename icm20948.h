#ifndef ICM20948_H
#define ICM20948_H

#include <stdint.h>

typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float temperature_c;
    uint8_t valid;
    uint8_t stale;
    uint8_t who_am_i;
    uint32_t timestamp_ms;
    uint32_t failures;
} IMU_Data;

uint8_t ICM20948_Init(void);
uint8_t ICM20948_Read(IMU_Data *data, uint32_t now_ms);
IMU_Data ICM20948_GetLast(void);

#endif
