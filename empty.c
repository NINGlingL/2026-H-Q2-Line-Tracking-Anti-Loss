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
    .pwm_limit_permille       = 700,   /* 软件最高输出：700=70%，不要超过moto.c的70%硬限幅 */
    .start_pwm_permille       = 220,   /* 起步输出：220=22%，觉得起步仍猛就减小，电机不动则增大 */
    .start_ramp_ms            = 2000U, /* 软启动时间：用多少毫秒从起步输出平滑升到正常输出 */
    .curve_pwm_permille       = 450,   /* 弯道基础速度：450=45% */
    .straight_pwm_permille    = 550,   /* 直线基础速度：550=55% */
    .diagnostic_pwm_permille  = 200,   /* 手动诊断模式电机输出：200=20% */
    .speed_target_mm_s        = 300,   /* 编码器有效时的目标速度：单位mm/s */
    .line_kp                  = 7.0f,  /* 循迹比例P：越大转向越强，过大会左右摇摆 */
    .line_ki                  = 0.2f,  /* 循迹积分I：修正长期偏差，过大会累积振荡 */
    .line_kd                  = 0.04f, /* 循迹微分D：抑制快速偏转，过大会放大传感器跳变 */
    .line_filter_alpha        = 0.25f, /* 循迹滤波：0~1，越小越平稳但转向响应越慢 */
    .line_center_deadband     = 1.25f, /* 直线中心死区：增大可减轻摇摆，过大会降低修线灵敏度 */
    .yaw_kp                   = 0.8f,  /* 陀螺仪角速度辅助比例P */
    .yaw_ki                   = 0.02f, /* 陀螺仪角速度辅助积分I */
    .yaw_rate_per_position    = 5.0f,  /* 每单位循迹偏差对应的目标转向角速度 */
    .straight_yaw_rate_dps    = 10.0f, /* 小于该角速度才判定为直线：单位度/秒 */
    .marker_active_count      = 5U,    /* 启停线判定：同时检测黑线的最少传感器数量 */
    .marker_clear_ms          = 80U,   /* 离开起点启停线的确认时间：单位ms */
    .marker_min_lap_ms        = 5000U, /* 启动后至少运行多久才允许识别终点：单位ms */
    .line_lost_stop_ms        = 50U,   /* 连续丢线多久后停车：单位ms */
    .auto_timeout_ms          = 35000U /* 自动循迹最长运行时间：超时立即停车 */
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
