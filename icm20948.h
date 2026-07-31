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
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;
    uint8_t valid;
    uint8_t stale;
    uint8_t who_am_i;
    uint8_t i2c_address;
    uint8_t calibrated;
    uint16_t calibration_samples;
    uint32_t timestamp_ms;
    uint32_t failures;
} IMU_Data;

uint8_t ICM20948_Init(void);
uint8_t ICM20948_Read(IMU_Data *data, uint32_t now_ms);
void ICM20948_StartCalibration(void);
IMU_Data ICM20948_GetLast(void);

#endif
