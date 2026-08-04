/**
 * IMU 姿态解算 — Mahony 互补滤波器 (6-DOF: 陀螺 + 加速度计)
 *
 * 参考: Mahony et al., "Nonlinear Complementary Filters on SO(3)"
 * 陀螺积分 → 四元数 → 加速度重力方向修正漂移
 */
#include "imu_filter.h"
#include <math.h>

static float dt, kp, ki;
static float q0=1,q1=0,q2=0,q3=0;  /* 四元数 */
static float ix=0,iy=0,iz=0;        /* 积分项 */

void IMU_Filter_Init(float _dt, float _kp, float _ki)
{
    dt=_dt; kp=_kp; ki=_ki;
    q0=1; q1=0; q2=0; q3=0;
    ix=iy=iz=0;
}

void IMU_Filter_Update(float gx, float gy, float gz,
                       float ax, float ay, float az)
{
    float norm, vx,vy,vz, ex,ey,ez;

    /* 1. 归一化加速度 */
    norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) return;
    norm = 1.0f / norm;
    ax *= norm; ay *= norm; az *= norm;

    /* 2. 重力方向 (四元数推算) */
    vx = 2.0f*(q1*q3 - q0*q2);
    vy = 2.0f*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    /* 3. 误差 = 测量 叉乘 推算 */
    ex = ay*vz - az*vy;
    ey = az*vx - ax*vz;
    ez = ax*vy - ay*vx;

    /* 4. 积分误差 */
    ix += ex * ki * dt;
    iy += ey * ki * dt;
    iz += ez * ki * dt;

    /* 5. PI 修正陀螺 */
    gx += kp*ex + ix;
    gy += kp*ey + iy;
    gz += kp*ez + iz;

    /* 6. 四元数积分 (一阶) */
    float ha = 0.5f * dt;
    float qa = q0, qb = q1, qc = q2, qd = q3;
    q0 += (-qb*gx - qc*gy - qd*gz) * ha;
    q1 += ( qa*gx + qd*gy - qc*gz) * ha;
    q2 += ( qd*gx + qa*gy + qb*gz) * ha;
    q3 += (-qc*gx + qb*gy + qa*gz) * ha;

    /* 7. 归一化四元数 */
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm < 1e-6f) return;
    norm = 1.0f / norm;
    q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;
}

void IMU_Filter_GetQuat(Quat *q) { q->w=q0; q->x=q1; q->y=q2; q->z=q3; }

void IMU_Filter_GetEuler(Euler *e)
{
    /* 四元数 → 欧拉角 (ZYX 顺规, rad→deg) */
    float r = 57.29578f;  /* 180/PI */

    float sinr = 2.0f*(q0*q1 + q2*q3);
    float cosr = 1.0f - 2.0f*(q1*q1 + q2*q2);
    e->roll = atan2f(sinr, cosr) * r;

    float sinp = 2.0f*(q0*q2 - q3*q1);
    if (fabsf(sinp) >= 1.0f)
        e->pitch = (sinp > 0 ? 1.5708f : -1.5708f) * r;
    else
        e->pitch = asinf(sinp) * r;

    float siny = 2.0f*(q0*q3 + q1*q2);
    float cosy = 1.0f - 2.0f*(q2*q2 + q3*q3);
    e->yaw = atan2f(siny, cosy) * r;
}
