/**
 * ICM20948 9轴IMU 驱动实现 (I2C)
 */
#include "icm20948.h"

/* ICM20948 寄存器 (Bank 0, 上电默认) */
#define REG_WHO_AM_I     0x00
#define REG_USER_CTRL    0x03
#define REG_PWR_MGMT_1   0x06
#define REG_ACCEL_XOUT_H 0x2D
#define REG_GYRO_XOUT_H  0x33
#define REG_TEMP_OUT_H   0x39
#define REG_ACCEL_CONFIG 0x14
#define REG_GYRO_CONFIG  0x15
#define REG_BANK_SEL     0x7F

/* I2C 总线超时 */
#define I2C_TO  500000U

/* OLED 共用 I2C, OLED_INST 已在 ti_msp_dl_config.h 定义 */
#define IMU_I2C  OLED_INST

/* ========== 底层 I2C ========== */

static bool i2c_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    uint32_t to;

    DL_I2C_fillControllerTXFIFO(IMU_I2C, buf, 2);
    to = I2C_TO;
    while (!(DL_I2C_getControllerStatus(IMU_I2C) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--to == 0) goto i2c_err;
    }
    DL_I2C_startControllerTransfer(IMU_I2C, ICM20948_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    to = I2C_TO;
    while (DL_I2C_getControllerStatus(IMU_I2C) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--to == 0) goto i2c_err;
    }
    to = I2C_TO;
    while (!(DL_I2C_getControllerStatus(IMU_I2C) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--to == 0) goto i2c_err;
    }
    DL_I2C_flushControllerTXFIFO(IMU_I2C);
    return true;

i2c_err:
    DL_I2C_resetControllerTransfer(IMU_I2C);
    DL_I2C_flushControllerTXFIFO(IMU_I2C);
    return false;
}

static bool i2c_read(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint32_t to;

    /* 先写寄存器地址 */
    DL_I2C_fillControllerTXFIFO(IMU_I2C, &reg, 1);
    to = I2C_TO;
    while (!(DL_I2C_getControllerStatus(IMU_I2C) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--to == 0) goto i2c_err;
    }
    DL_I2C_startControllerTransfer(IMU_I2C, ICM20948_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    to = I2C_TO;
    while (DL_I2C_getControllerStatus(IMU_I2C) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--to == 0) goto i2c_err;
    }
    to = I2C_TO;
    while (!(DL_I2C_getControllerStatus(IMU_I2C) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--to == 0) goto i2c_err;
    }
    DL_I2C_flushControllerTXFIFO(IMU_I2C);

    /* 再读数据 */
    DL_I2C_startControllerTransfer(IMU_I2C, ICM20948_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);
    to = I2C_TO;
    while (DL_I2C_getControllerStatus(IMU_I2C) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--to == 0) goto i2c_err;
    }

    for (uint8_t i = 0; i < len; i++) {
        to = I2C_TO;
        while (DL_I2C_isRXFIFOEmpty(IMU_I2C)) {
            if (--to == 0) goto i2c_err;
        }
        data[i] = DL_I2C_receiveControllerData(IMU_I2C);
    }

    DL_I2C_flushControllerRXFIFO(IMU_I2C);
    DL_I2C_flushControllerTXFIFO(IMU_I2C);
    return true;

i2c_err:
    DL_I2C_resetControllerTransfer(IMU_I2C);
    DL_I2C_flushControllerTXFIFO(IMU_I2C);
    DL_I2C_flushControllerRXFIFO(IMU_I2C);
    return false;
}

/* ========== 初始化 ========== */

uint8_t ICM20948_WhoAmI(void)
{
    uint8_t who = 0;
    if (!i2c_read(REG_WHO_AM_I, &who, 1)) return 0;
    return who;
}

bool ICM20948_Init(void)
{
    uint8_t who = ICM20948_WhoAmI();
    if (who != ICM20948_WHO_AM_I) return false;

    /* 唤醒: CLKSEL=1 (Auto select), 清除 SLEEP */
    i2c_write(REG_PWR_MGMT_1, 0x01);
    delay_cycles(CPUCLK_FREQ / 100);  /* ~10ms 等待稳定 */

    /* 加速度计: ±4g */
    i2c_write(REG_ACCEL_CONFIG, 0x02 << 1);

    /* 陀螺仪: ±500°/s */
    i2c_write(REG_GYRO_CONFIG, 0x01 << 1);

    return true;
}

/* ========== 数据读取 ========== */

static inline float accel_to_ms2(int16_t raw)
{
    /* ±4g → LSB = 4/32768 g/LSB, 1g = 9.80665 m/s^2 */
    return (float)raw * 4.0f * 9.80665f / 32768.0f;
}

static inline float gyro_to_dps(int16_t raw)
{
    /* ±500°/s → LSB = 500/32768 °/s/LSB */
    return (float)raw * 500.0f / 32768.0f;
}

void ICM20948_Read(IMU_Data *imu)
{
    uint8_t buf[12];
    int16_t raw[6];

    if (!i2c_read(REG_ACCEL_XOUT_H, buf, 12)) {
        /* 读取失败, 清零 */
        imu->ax = imu->ay = imu->az = 0;
        imu->gx = imu->gy = imu->gz = 0;
        imu->temp_c = 0;
        return;
    }

    raw[0] = (int16_t)((buf[0] << 8) | buf[1]);   /* AX */
    raw[1] = (int16_t)((buf[2] << 8) | buf[3]);   /* AY */
    raw[2] = (int16_t)((buf[4] << 8) | buf[5]);   /* AZ */
    raw[3] = (int16_t)((buf[6] << 8) | buf[7]);   /* GX */
    raw[4] = (int16_t)((buf[8] << 8) | buf[9]);   /* GY */
    raw[5] = (int16_t)((buf[10] << 8) | buf[11]); /* GZ */

    imu->ax = accel_to_ms2(raw[0]);
    imu->ay = accel_to_ms2(raw[1]);
    imu->az = accel_to_ms2(raw[2]);
    imu->gx = gyro_to_dps(raw[3]);
    imu->gy = gyro_to_dps(raw[4]);
    imu->gz = gyro_to_dps(raw[5]);
}

void ICM20948_ReadAccel(IMU_Data *imu)
{
    uint8_t buf[6];
    int16_t raw[3];

    if (!i2c_read(REG_ACCEL_XOUT_H, buf, 6)) {
        imu->ax = imu->ay = imu->az = 0;
        return;
    }

    raw[0] = (int16_t)((buf[0] << 8) | buf[1]);
    raw[1] = (int16_t)((buf[2] << 8) | buf[3]);
    raw[2] = (int16_t)((buf[4] << 8) | buf[5]);

    imu->ax = accel_to_ms2(raw[0]);
    imu->ay = accel_to_ms2(raw[1]);
    imu->az = accel_to_ms2(raw[2]);
}

void ICM20948_ReadGyro(IMU_Data *imu)
{
    uint8_t buf[6];
    int16_t raw[3];

    if (!i2c_read(REG_GYRO_XOUT_H, buf, 6)) {
        imu->gx = imu->gy = imu->gz = 0;
        return;
    }

    raw[0] = (int16_t)((buf[0] << 8) | buf[1]);
    raw[1] = (int16_t)((buf[2] << 8) | buf[3]);
    raw[2] = (int16_t)((buf[4] << 8) | buf[5]);

    imu->gx = gyro_to_dps(raw[0]);
    imu->gy = gyro_to_dps(raw[1]);
    imu->gz = gyro_to_dps(raw[2]);
}
