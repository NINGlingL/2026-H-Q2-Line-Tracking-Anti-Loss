/**
 * ICM-20948 陀螺仪 OLED 显示 — Mahony 姿态解算
 */
#include "ti_msp_dl_config.h"
#include "ssd1306.h"
#include "icm20948.h"
#include "imu_filter.h"
#include <stdio.h>

static void dly(uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms * 4000; i++) __NOP();
}

int main(void)
{
    imu_t imu;
    Euler e;
    Quat  q;
    char  b[22];
    uint32_t n = 0;

    SYSCFG_DL_init();
    SSD1306_Init();

    if (ICM20948_Init() != 0) {
        SSD1306_Clear();
        SSD1306_ShowString(2, 0, "ICM20948 NOT FOUND");
        SSD1306_ShowString(4, 0, "SDA=PA0 SCL=PA1 AD0=GND");
        SSD1306_Update();
        while (1) __NOP();
    }

    /* Mahony 滤波器: dt=10ms, Kp=2.0, Ki=0.1 */
    IMU_Filter_Init(0.01f, 2.0f, 0.1f);

    while (1) {
        ICM20948_Read(&imu);

        /* 陀螺: dps→rad/s,  用滤波器更新 */
        float gx = imu.gx * 0.0174533f;
        float gy = imu.gy * 0.0174533f;
        float gz = imu.gz * 0.0174533f;
        IMU_Filter_Update(gx, gy, gz, imu.ax, imu.ay, imu.az);

        IMU_Filter_GetEuler(&e);
        IMU_Filter_GetQuat(&q);
        n++;

        SSD1306_Clear();

        /* [0] 欧拉角 */
        snprintf(b, sizeof(b), "R%+5.0f P%+5.0f Y%+5.0f",
                 (double)e.roll, (double)e.pitch, (double)e.yaw);
        SSD1306_ShowString(0, 0, b);

        /* [1][2] 陀螺 dps */
        SSD1306_ShowString(1, 0, "GX"); SSD1306_WriteFloat(1, 18, imu.gx, 1);
        SSD1306_ShowString(1, 60, "GY"); SSD1306_WriteFloat(1, 78, imu.gy, 1);
        SSD1306_ShowString(2, 0, "GZ"); SSD1306_WriteFloat(2, 18, imu.gz, 1);

        /* [3][4] 加速度 g */
        SSD1306_ShowString(3, 0, "AX"); SSD1306_WriteFloat(3, 18, imu.ax, 2);
        SSD1306_ShowString(3, 60, "AY"); SSD1306_WriteFloat(3, 78, imu.ay, 2);
        SSD1306_ShowString(4, 0, "AZ"); SSD1306_WriteFloat(4, 18, imu.az, 2);

        /* [5] 四元数 */
        snprintf(b, sizeof(b), "Q %.2f %.2f %.2f %.2f",
                 (double)q.w, (double)q.x, (double)q.y, (double)q.z);
        SSD1306_ShowString(5, 0, b);

        /* [6][7] 状态 */
        SSD1306_ShowString(6, 0, "Mahony  Kp=2.0 Ki=0.1");
        snprintf(b, sizeof(b), "#%lu", (unsigned long)n);
        SSD1306_ShowString(7, 0, b);

        SSD1306_Update();
        dly(10);  /* ~100Hz */
    }
}
