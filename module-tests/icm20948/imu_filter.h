/**
 * IMU 姿态解算 — Mahony 互补滤波器
 * 陀螺仪积分 + 加速度计修正 → 四元数 → 欧拉角
 */
#ifndef __IMU_FILTER_H__
#define __IMU_FILTER_H__
#include <stdint.h>

typedef struct { float w,x,y,z; } Quat;
typedef struct { float roll,pitch,yaw; } Euler;

void  IMU_Filter_Init(float dt, float kp, float ki);
void  IMU_Filter_Update(float gx, float gy, float gz,
                        float ax, float ay, float az);
void  IMU_Filter_GetQuat(Quat *q);
void  IMU_Filter_GetEuler(Euler *e);   /* roll/pitch/yaw (deg) */
#endif
