#include "ti_msp_dl_config.h"
#include "ssd1306.h"

int main(void)
{
    SYSCFG_DL_init();

    /* OLED 初始化 */
    SSD1306_Init();

    /* 第一行 */
    SSD1306_ShowString(0, 0, "  I LOVE YOU!");
    /* 心的位置需要特殊处理，可以画一个小图案或者留空 */

    /* 第三行 */
    SSD1306_ShowString(3, 0, "  From MSPM0");
    /* 第五行 */
    SSD1306_ShowString(5, 0, "  G3507 & OLED");

    /* 画一个简单的心形 */
    /* 左半边 */
    SSD1306_DrawPixel(56, 10, 1); SSD1306_DrawPixel(57, 9, 1);
    SSD1306_DrawPixel(58, 8, 1);  SSD1306_DrawPixel(59, 8, 1);
    SSD1306_DrawPixel(56, 11, 1); SSD1306_DrawPixel(57, 10, 1);
    SSD1306_DrawPixel(58, 9, 1);  SSD1306_DrawPixel(59, 9, 1);
    SSD1306_DrawPixel(56, 12, 1); SSD1306_DrawPixel(57, 11, 1);
    SSD1306_DrawPixel(58, 10, 1); SSD1306_DrawPixel(59, 10, 1);
    /* 右半边 */
    SSD1306_DrawPixel(64, 10, 1); SSD1306_DrawPixel(65, 9, 1);
    SSD1306_DrawPixel(66, 8, 1);  SSD1306_DrawPixel(67, 8, 1);
    SSD1306_DrawPixel(64, 11, 1); SSD1306_DrawPixel(65, 10, 1);
    SSD1306_DrawPixel(66, 9, 1);  SSD1306_DrawPixel(67, 9, 1);
    SSD1306_DrawPixel(64, 12, 1); SSD1306_DrawPixel(65, 11, 1);
    SSD1306_DrawPixel(66, 10, 1); SSD1306_DrawPixel(67, 10, 1);
    /* 底部尖角 */
    SSD1306_DrawPixel(60, 13, 1); SSD1306_DrawPixel(61, 13, 1);
    SSD1306_DrawPixel(62, 13, 1); SSD1306_DrawPixel(63, 13, 1);
    SSD1306_DrawPixel(61, 14, 1); SSD1306_DrawPixel(62, 14, 1);
    SSD1306_DrawPixel(62, 15, 1);

    SSD1306_Update();

    while (1);
}
