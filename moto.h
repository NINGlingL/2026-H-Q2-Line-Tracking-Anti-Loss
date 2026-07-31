/**
 *  TB6612 电机驱动库 (MSPM0G3507 + PWM + GPIO)
 *
 *  硬件连接:
 *    PWM:  TIMG0  CCP0=PA12(PWMA)  CCP1=PA13(PWMB)
 *    GPIO: PB16=AIN1  PB15=AIN2  (Motor A)
 *          PB2=BIN1   PB3=BIN2   (Motor B)
 *          STBY 接 3.3V (使能)
 *
 *  TB6612 真值表 (与 L298N 相同):
 *    Forward:  AIN1=1 AIN2=0  BIN1=1 BIN2=0
 *    Backward: AIN1=0 AIN2=1  BIN1=0 BIN2=1
 *    Left:     BIN1=1 BIN2=0  (右轮转, 左轮停)
 *    Right:    AIN1=1 AIN2=0  (左轮转, 右轮停)
 *    Stop:     全部清零
 *
 *  使用前必须调用 Moto_Init() 初始化
 */
#ifndef __MOTO_H__
#define __MOTO_H__

#include "ti_msp_dl_config.h"

/* ========== 初始化 ========== */

/** 初始化电机 GPIO 和 PWM (占空比归零, 引脚清零) */
void Moto_Init(void);

/* ========== 基本控制 ========== */

/** 前进, pct: 0~100 占空比 */
void Moto_Forward(uint8_t pct);

/** 后退, pct: 0~100 占空比 */
void Moto_Backward(uint8_t pct);

/** 左转 (右轮正转, 左轮停), pct: 0~100 */
void Moto_Left(uint8_t pct);

/** 右转 (左轮正转, 右轮停), pct: 0~100 */
void Moto_Right(uint8_t pct);

/** 停止 (PWM归零, 所有方向引脚清零) */
void Moto_Stop(void);

/** 保持当前方向, 仅修改速度, pct: 0~100 */
void Moto_SetSpeed(uint8_t pct);

/** 加速 delta%, 返回新速度 */
uint8_t Moto_SpeedUp(uint8_t delta);

/** 减速 delta%, 返回新速度 (减到0自动停止) */
uint8_t Moto_SpeedDown(uint8_t delta);

/* ========== 状态查询 ========== */

/** 获取当前方向字符: 'F' 'B' 'L' 'R' 'S' */
char Moto_GetDirection(void);

/** 获取当前速度 0~100 */
uint8_t Moto_GetSpeed(void);

/** 获取当前 PWM CC0 值 */
uint32_t Moto_GetCC0(void);

/** 获取当前 PWM CC1 值 */
uint32_t Moto_GetCC1(void);

/** 获取 GPIOB 方向引脚状态 */
uint32_t Moto_GetGPIO(void);

/* ========== 开机自检 ========== */

/**
 * 开机自检序列:
 *   80% 前进 1s → 50% 前进 2s → 20% 前进 2s → 停止
 *  callback: 每段开始时回调, 可传 NULL
 *   参数 (phase, speed, duration_sec)
 */
typedef void (*Moto_SelfTestCallback)(uint8_t phase, uint8_t speed, uint8_t sec);

void Moto_SelfTest(Moto_SelfTestCallback cb);

#endif
