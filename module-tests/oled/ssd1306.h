/**
 * SSD1306 0.96寸 OLED 驱动库 (I2C)
 * 适用芯片: MSPM0G3507
 * 分辨率: 128x64
 * I2C地址: 0x3C
 */

#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "ti_msp_dl_config.h"
#include <string.h>

/* ========== SSD1306 地址 & 尺寸 ========== */
#define SSD1306_I2C_ADDR    0x3C
#define SSD1306_WIDTH       128
#define SSD1306_HEIGHT      64
#define SSD1306_PAGES       8       // 64/8 = 8 页

/* ========== 控制字节 ========== */
#define SSD1306_CMD_SINGLE  0x00    // 下一个字节是命令
#define SSD1306_CMD_STREAM  0x00    // 后续都是命令
#define SSD1306_DATA_STREAM 0x40    // 后续都是数据

/* ========== 基本命令 ========== */
#define SSD1306_DISPLAY_OFF     0xAE
#define SSD1306_DISPLAY_ON      0xAF
#define SSD1306_SET_CONTRAST    0x81
#define SSD1306_DISPLAY_ALL_ON  0xA5

/* ========== API ========== */

/** 初始化 OLED，必须在 SYSCFG_DL_init() 之后调用 */
void SSD1306_Init(void);

/** 清屏 */
void SSD1306_Clear(void);

/** 全屏填充 */
void SSD1306_Fill(uint8_t data);

/** 刷新显示（写入整屏缓冲区） */
void SSD1306_Update(void);

/** 设置光标位置 (页 0~7, 列 0~127) */
void SSD1306_SetCursor(uint8_t page, uint8_t col);

/** 在当前位置显示一个 6x8 英文字符 */
void SSD1306_ShowChar(uint8_t page, uint8_t col, char ch);

/** 显示字符串，自动换行 */
void SSD1306_ShowString(uint8_t page, uint8_t col, const char *str);

/** 画一个像素点 */
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);

/** 画水平线 */
void SSD1306_DrawHLine(uint8_t x, uint8_t y, uint8_t len, uint8_t color);

/** 显示数字 */
void SSD1306_ShowNum(uint8_t page, uint8_t col, int32_t num, uint8_t len);

#endif
