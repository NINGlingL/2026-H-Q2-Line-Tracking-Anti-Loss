/**
 * ICM-20948 9轴IMU 驱动实现 (I2C)
 */
#include "icm20948.h"
#include "ti_msp_dl_config.h"

#define ADDR  0x68
#define TO    200000UL

/* ── I2C 写寄存器 ── */
static void _w(uint8_t reg, uint8_t val)
{
    uint8_t d[2] = {reg, val};
    volatile uint32_t t = TO;
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE))
        if (--t == 0) return;
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, d, 2);
    DL_I2C_startControllerTransfer(I2C_0_INST, ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    t = TO; while (!(DL_I2C_getControllerStatus(I2C_0_INST)&DL_I2C_CONTROLLER_STATUS_BUSY_BUS)) if(--t==0)return;
    t = TO; while (!(DL_I2C_getControllerStatus(I2C_0_INST)&DL_I2C_CONTROLLER_STATUS_IDLE)) if(--t==0)return;
}

/* ── I2C 读寄存器 ── */
static uint8_t _r(uint8_t reg);     /* 前向声明 */
static void _rb(uint8_t reg, uint8_t *buf, uint8_t len);

/* ── 被 empty.c 调用的测试函数 ── */
uint8_t _icm_test_read(uint8_t reg) { return _r(reg); }
void _icm_test_read12(uint8_t reg, uint8_t *buf, uint8_t len) { _rb(reg, buf, len); }

static uint8_t _r(uint8_t reg)
{
    volatile uint32_t t;
    /* 等空闲 */
    t = TO; while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) if(--t==0)return 0;
    /* 写寄存器地址 (TX) */
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1);
    t = TO; while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) if(--t==0)return 0;
    DL_I2C_startControllerTransfer(I2C_0_INST, ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    t = TO; while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)) if(--t==0) break;
    t = TO; while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) if(--t==0) break;
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    /* 读数据 (RX) */
    t = TO; while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) if(--t==0)return 0;
    DL_I2C_startControllerTransfer(I2C_0_INST, ADDR, DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    t = TO; while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)) if(--t==0) break;
    t = TO; while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) if(--t==0) break;
    uint8_t val = DL_I2C_receiveControllerData(I2C_0_INST);
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);
    return val;
}

/* ── I2C 突发读 ── */
static void _rb(uint8_t reg, uint8_t *buf, uint8_t len)
{
    volatile uint32_t t;
    t = TO; while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) if(--t==0) return;
    /* TX: 写寄存器地址 */
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1);
    DL_I2C_startControllerTransfer(I2C_0_INST, ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    t = TO; while (!(DL_I2C_getControllerStatus(I2C_0_INST)&DL_I2C_CONTROLLER_STATUS_BUSY_BUS)) if(--t==0)return;
    t = TO; while (!(DL_I2C_getControllerStatus(I2C_0_INST)&DL_I2C_CONTROLLER_STATUS_IDLE)) if(--t==0)return;
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    /* RX: 读 len 字节 */
    t = TO; while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) if(--t==0) return;
    DL_I2C_startControllerTransfer(I2C_0_INST, ADDR, DL_I2C_CONTROLLER_DIRECTION_RX, len);
    t = TO; while (!(DL_I2C_getControllerStatus(I2C_0_INST)&DL_I2C_CONTROLLER_STATUS_BUSY_BUS)) if(--t==0)return;
    t = TO; while (!(DL_I2C_getControllerStatus(I2C_0_INST)&DL_I2C_CONTROLLER_STATUS_IDLE)) if(--t==0)return;
    for (uint8_t i = 0; i < len; i++) buf[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);
}

/* ── Bank 切换 ── */
static void _bank(uint8_t b) { _w(0x7F, b); }

#define B0 0x00
#define B2 0x20
#define PWR_MGMT_1  0x06
#define PWR_MGMT_2  0x07
#define ACCEL_XOUT  0x2D
#define GYRO_XOUT   0x33
#define ACCEL_CONFIG 0x14
#define GYRO_CONFIG1 0x01
#define GYRO_SMPLRT  0x00
#define ACCEL_SMPLRT 0x10

uint8_t ICM20948_WhoAmI(void) { return _r(0x00); }

int ICM20948_Init(void)
{
    if (ICM20948_WhoAmI() != 0xEA) return -1;

    /* 复位 */
    _w(PWR_MGMT_1, 0x80);
    for (volatile uint32_t d = 0; d < 3200000; d++) __NOP();  /* ~100ms */

    /* 唤醒 + 自动时钟 */
    _w(PWR_MGMT_1, 0x01);
    for (volatile uint32_t d = 0; d < 1600000; d++) __NOP();  /* ~50ms */
    _w(PWR_MGMT_2, 0x00);

    /* 陀螺 ±2000dps DLPF ~41Hz */
    _bank(B2);
    _w(GYRO_CONFIG1, 0x18);
    _w(GYRO_SMPLRT, 0x00);

    /* 加速度 ±2g DLPF ~41Hz */
    _w(ACCEL_CONFIG, 0x00);
    _w(ACCEL_SMPLRT, 0x00);

    _bank(B0);
    for (volatile uint32_t d = 0; d < 1600000; d++) __NOP();  /* ~50ms */
    return 0;
}

void ICM20948_Read(imu_t *d)
{
    uint8_t buf[6];
    int16_t raw;

    /* 加速度 6 字节 (0x2D~0x32) */
    _rb(ACCEL_XOUT, buf, 6);
    raw = (int16_t)((buf[0]<<8)|buf[1]); d->ax = raw / 16384.0f;
    raw = (int16_t)((buf[2]<<8)|buf[3]); d->ay = raw / 16384.0f;
    raw = (int16_t)((buf[4]<<8)|buf[5]); d->az = raw / 16384.0f;

    /* 陀螺 6 字节 (0x33~0x38) */
    _rb(GYRO_XOUT, buf, 6);
    raw = (int16_t)((buf[0]<<8)|buf[1]); d->gx = raw / 16.4f;
    raw = (int16_t)((buf[2]<<8)|buf[3]); d->gy = raw / 16.4f;
    raw = (int16_t)((buf[4]<<8)|buf[5]); d->gz = raw / 16.4f;

    d->temp = 21.0f;
}
