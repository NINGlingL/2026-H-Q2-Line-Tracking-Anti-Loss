/**
 * ICM20948 9轴IMU 驱动 (I2C)
 * 与 SSD1306 共用 I2C0 总线 (0x68 vs 0x3C)
 */
#ifndef __ICM20948_H__
#define __ICM20948_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ICM20948_I2C_ADDR  0x68    /* AD0=GND */
#define ICM20948_WHO_AM_I  0xEA    /* 芯片ID */

/* ========== 数据结构 ========== */

typedef struct {
    float ax, ay, az;       /* 加速度 (m/s^2) */
    float gx, gy, gz;       /* 角速度 (°/s) */
    float temp_c;           /* 温度 (°C) */
} IMU_Data;

/* ========== 初始化 ========== */

/** 初始化 ICM20948 (唤醒 + 配置量程), 返回 true=成功 */
bool ICM20948_Init(void);

/** 读取 WHO_AM_I, 返回 0xEA 表示通信正常 */
uint8_t ICM20948_WhoAmI(void);

/* ========== 数据读取 ========== */

/** 读取加速度和角速度 (一次 I2C burst read) */
void ICM20948_Read(IMU_Data *imu);

/** 仅读加速度 */
void ICM20948_ReadAccel(IMU_Data *imu);

/** 仅读角速度 */
void ICM20948_ReadGyro(IMU_Data *imu);

#ifdef __cplusplus
}
#endif

#endif
