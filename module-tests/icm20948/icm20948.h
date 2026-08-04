/**
 * ICM-20948 9轴IMU 驱动 (I2C, 地址 0x68)
 * 与 SSD1306 共用 I2C_0_INST (PA0=SDA PA1=SCL)
 */
#ifndef __ICM20948_H__
#define __ICM20948_H__
#include <stdint.h>

typedef struct { float ax,ay,az, gx,gy,gz, temp; } imu_t;

int     ICM20948_Init(void);       /* 返回 0=成功 */
uint8_t ICM20948_WhoAmI(void);     /* 应返回 0xEA */
void    ICM20948_Read(imu_t *d);   /* 读加速度+角速度 */
#endif
