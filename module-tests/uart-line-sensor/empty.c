 /**
 * 八路巡线红外传感器 + OLED 显示
 * I2C0 PA0=SDA PA1=SCL → SSD1306 OLED (0x3C)
 * UART3 PA14=TX PA13=RX → 八路传感器 (115200)
 */
#include "ti_msp_dl_config.h"
#include "ssd1306.h"
#include "eight_ir.h"
#include <stdio.h>

static void dly(uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms * 4000; i++) __NOP();
}

static void dly_poll(uint32_t ms)
{
    while (ms--) {
        EightIR_Poll();
        for (volatile uint32_t i = 0; i < 4000; i++) __NOP();
    }
}

int main(void)
{
    EightIR_State ir;
    EightIR_Stats stats;
    char b[22];
    uint32_t n = 0;

    SYSCFG_DL_init();
    SSD1306_Init();
    EightIR_Init();

    SSD1306_Clear();
    SSD1306_ShowString(2, 12, "8ch IR Sensor");
    SSD1306_ShowString(4, 18, "Init OK...");
    SSD1306_Update();
    dly_poll(1000);

    while (1) {
        EightIR_GetState(&ir);
        EightIR_GetStats(&stats);
        n++;

        /* 未收到有效数字帧时，周期性重发数字量模式命令 */
        if (!ir.frame_ready && (n % 20U) == 0U) EightIR_StartDigital();

        SSD1306_Clear();

        snprintf(b, sizeof(b), "8IR #%lu %s", (unsigned long)n,
                 ir.frame_ready ? "OK" : "WAIT");
        SSD1306_ShowString(0, 0, b);

        SSD1306_ShowString(1, 0, "1 2 3 4 5 6 7 8");

        snprintf(b, sizeof(b), "%c %c %c %c %c %c %c %c",
            ir.ch[0] ? 'O':'X', ir.ch[1] ? 'O':'X', ir.ch[2] ? 'O':'X', ir.ch[3] ? 'O':'X',
            ir.ch[4] ? 'O':'X', ir.ch[5] ? 'O':'X', ir.ch[6] ? 'O':'X', ir.ch[7] ? 'O':'X');
        SSD1306_ShowString(2, 0, b);

        snprintf(b, sizeof(b), "Raw:0x%02X %u%u%u%u%u%u%u%u", ir.raw,
            ir.ch[0], ir.ch[1], ir.ch[2], ir.ch[3], ir.ch[4], ir.ch[5], ir.ch[6], ir.ch[7]);
        SSD1306_ShowString(3, 0, b);

        snprintf(b, sizeof(b), "Pos:%+d cnt:%u", (int)ir.position, ir.active_count);
        SSD1306_ShowString(4, 0, b);

        snprintf(b, sizeof(b), "D:%lu bad:%lu", (unsigned long)stats.digital_frames,
            (unsigned long)stats.bad_frames);
        SSD1306_ShowString(5, 0, b);

        snprintf(b, sizeof(b), "RX:%lu ov:%lu", (unsigned long)stats.rx_bytes,
            (unsigned long)stats.overflow_bytes);
        SSD1306_ShowString(6, 0, b);

        SSD1306_ShowString(7, 0, "PA14T PA13R PA0/1");

        SSD1306_Update();
        dly_poll(100);
    }
}
