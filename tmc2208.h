/*
 * TMC2208 步进电机驱动 (MSPM0G3507)
 *
 * 硬件:
 *   STEP : PA8  (TIMA0_CCP0, PWM 输出, 4MHz 定时器时钟)
 *   DIR  : PA13 (GPIO)
 *   EN   : PA15 (GPIO, 低有效)
 *   MS1/MS2 接 GND -> 1/8 细分
 *
 * 电机 28HS30-0604A-013 (丝杆型):
 *   步距角 1.8° (200 步/转), 丝杆导程 2mm
 *   1/8 细分 -> 1600 步/转 -> 800 步/mm
 *
 * 运动控制: 梯形加减速, 由 TIMA0 zero 事件中断每步更新速度
 */

#ifndef TMC2208_H
#define TMC2208_H

#include <stdint.h>
#include <stdbool.h>

/* ============ 机械参数 (按实际结构校准) ============ */
#define TMC_MOTOR_STEPS_PER_REV   200.0f   /* 1.8° 步距角: 200 步/转 */
#define TMC_MICROSTEPS            8        /* MS1=MS2=GND -> 1/8 细分 */
#define TMC_LEAD_MM               2.0f     /* 丝杆导程 mm */
#define TMC_STEPS_PER_REV         ((uint32_t)(TMC_MOTOR_STEPS_PER_REV * TMC_MICROSTEPS))  /* 1600 */
#define TMC_STEPS_PER_MM          ((float)TMC_STEPS_PER_REV / TMC_LEAD_MM)                 /* 800 */

/* ============ 运动参数 ============ */
void  tmc2208_set_max_speed(float steps_per_s);   /* 最大步进速度 (步/s) */
void  tmc2208_set_accel(float steps_per_s2);      /* 加速度 (步/s^2) */
void  tmc2208_set_current_position(int32_t steps);/* 把当前位置标定为某值 (清零用) */

/* ============ 基本控制 ============ */
void  tmc2208_init(void);          /* 初始化引脚/定时器, 默认禁用 */
void  tmc2208_enable(void);        /* EN 拉低: 使能驱动 */
void  tmc2208_disable(void);       /* EN 拉高 + 停脉冲: 禁用 */
bool  tmc2208_is_enabled(void);

/* ============ 运动 ============ */
void  tmc2208_move_to(int32_t target);   /* 绝对定位 (带加减速) */
void  tmc2208_move_steps(int32_t steps); /* 相对移动 */
void  tmc2208_stop(void);                /* 立即停, 停在当前位置 */
bool  tmc2208_is_moving(void);

/* ============ 状态查询 ============ */
int32_t tmc2208_get_position(void);       /* 当前绝对位置 (步) */
int32_t tmc2208_get_target(void);         /* 目标位置 (步) */
int8_t  tmc2208_get_direction(void);      /* 当前方向 +1 / -1 */
float   tmc2208_get_current_speed(void);  /* 当前速度 (步/s) */

/* ============ 中断 ============ */
void tmc2208_step_isr(void);   /* 由 TIMA0 zero 事件 ISR 调用 */

#endif /* TMC2208_H */
