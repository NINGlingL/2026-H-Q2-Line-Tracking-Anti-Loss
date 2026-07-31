/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "battery.h"
#include "app_config.h"
#include "control.h"
#include "eight_ir.h"
#include "encoder.h"
#include "icm20948.h"
#include "moto.h"
#include "ssd1306.h"
#include "timebase.h"
#include "ti_msp_dl_config.h"
#include "uart_bt.h"

/* ======================== 用户调车参数区 ======================== */
const Control_Tuning g_control_tuning = {
    .pwm_limit_permille       = 300,   /* 6880a79最高输出：300=30%，moto.c也有30%硬限幅 */
    .auto_base_pwm_permille   = 220,   /* 6880a79自动循迹固定基础速度：220=22% */
    .diagnostic_pwm_permille  = 200,   /* 手动诊断模式电机输出：200=20% */
    .speed_target_mm_s        = 120,   /* 编码器有效时目标速度：120mm/s；无效时固定使用基础PWM */
    .line_kp                  = 8.0f,  /* 6880a79八路循迹PID比例P */
    .line_ki                  = 0.3f,  /* 6880a79八路循迹PID积分I */
    .line_kd                  = 0.12f, /* 6880a79八路循迹PID微分D */
    .line_integral_limit      = 20.0f, /* 循迹PID积分限幅 */
    .line_output_limit        = 70.0f, /* 循迹PID输出限幅 */
    .line_separation_error    = 4.0f,  /* 循迹PID积分分离误差 */
    .speed_kp                 = 0.45f, /* 编码器速度PID比例P */
    .speed_ki                 = 0.40f, /* 编码器速度PID积分I */
    .speed_kd                 = 0.0f,  /* 编码器速度PID微分D */
    .speed_integral_limit     = 150.0f,/* 编码器速度PID积分限幅 */
    .speed_output_limit       = 140.0f,/* 编码器速度PID输出限幅 */
    .speed_separation_error   = 200.0f,/* 编码器速度PID积分分离误差 */
    .yaw_kp                   = 0.8f,  /* 陀螺仪角速度辅助比例P */
    .yaw_ki                   = 0.02f, /* 陀螺仪角速度辅助积分I */
    .yaw_kd                   = 0.0f,  /* 陀螺仪角速度辅助微分D */
    .yaw_integral_limit       = 30.0f, /* 陀螺仪辅助PID积分限幅 */
    .yaw_output_limit         = 30.0f, /* 陀螺仪辅助PID输出限幅 */
    .yaw_separation_error     = 60.0f, /* 陀螺仪辅助PID积分分离误差 */
    .yaw_rate_per_position    = 5.0f,  /* 每单位循迹偏差对应的目标转向角速度 */
    .marker_active_count      = 5U,    /* 停车线判定：至少5路同时检测黑线，兼容中间六路 */
    .marker_clear_ms          = 80U,   /* 离开起点启停线的确认时间：单位ms */
    .marker_min_lap_ms        = 5000U, /* 启动后至少运行多久才允许识别终点：单位ms */
    .line_lost_stop_ms        = 50U,   /* 连续丢线多久后停车：单位ms */
    .auto_timeout_ms          = 35000U,/* 自动循迹最长运行时间：超时立即停车 */
    .diagnostic_timeout_ms    = 30000U /* 手动诊断模式最长运行时间：单位ms */
};
/* ====================== 用户调车参数区结束 ====================== */

/*
 * Keep the Keil load segment aligned for the MSPM0 64-bit flash writer.
 * The scatter file places this inert word after the application image.
 */
__attribute__((used)) static const uint64_t gMSPM0FlashWritePad =
    UINT64_C(0xFFFFFFFFFFFFFFFF);

int main(void)
{
    uint32_t now_ms;

    SYSCFG_DL_init();
    Moto_Init();
    Timebase_Init();
#if APP_ENABLE_BATTERY_ADC
    Battery_Init();
#endif
    Encoder_Init();
#if APP_ENABLE_BLUETOOTH
    UartBT_Init();
#endif
    EightIR_Init(Timebase_Millis());
#if APP_ENABLE_IMU
    (void) ICM20948_Init();
#endif
#if APP_ENABLE_OLED
    (void) SSD1306_Init();
#endif
    Control_Init(Timebase_Millis());

    while (1) {
        now_ms = Timebase_Millis();
        EightIR_Service(now_ms);
#if APP_ENABLE_BATTERY_ADC
        Battery_Service(now_ms);
#endif
        Control_Service(now_ms);

        /* The independent watchdog is fed only at the main-loop tail. */
        DL_WWDT_restart(WWDT0_INST);
        __WFE();
    }
}
