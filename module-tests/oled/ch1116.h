/**
 * CH1116 0.96寸 OLED 驱动库 (I2C)
 * 适用芯片: MSPM0G3507
 * 分辨率: 128x64
 * I2C地址: 0x3C (7-bit) / 0x78 (8-bit)
 *
 * 与 SSD1306 的主要差异:
 *   - CH1116 内部 RAM 为 132x64, 需设置列偏移(2)居中
 *   - DC-DC 控制命令 (0xAD) 是 CH1116 特有的
 *   - 初始化序列略有不同
 */

#ifndef __CH1116_H__
#define __CH1116_H__

#include "ti_msp_dl_config.h"
#include <string.h>

/* ========== CH1116 地址 & 尺寸 ========== */
#define CH1116_I2C_ADDR     0x3C    /* 7-bit 地址, 对应模块丝印 0x78 */
#define CH1116_WIDTH        128
#define CH1116_HEIGHT       64
#define CH1116_PAGES        8       /* 64/8 = 8 页 */
#define CH1116_COL_OFFSET   2       /* CH1116 内部132列, 偏移2居中显示128列 */

/* ========== 控制字节 ========== */
#define CH1116_CMD_SINGLE   0x00    /* 下一个字节是命令 */
#define CH1116_CMD_STREAM   0x00    /* 后续都是命令 */
#define CH1116_DATA_STREAM  0x40    /* 后续都是数据 */

/* ========== 基本命令 ========== */
#define CH1116_DISPLAY_OFF      0xAE
#define CH1116_DISPLAY_ON       0xAF
#define CH1116_SET_CONTRAST     0x81
#define CH1116_DISPLAY_ALL_ON   0xA5
#define CH1116_NORMAL_DISPLAY   0xA6
#define CH1116_INVERSE_DISPLAY  0xA7

/* ========== API ========== */

/** 初始化 OLED，必须在 SYSCFG_DL_init() 之后调用 */
void CH1116_Init(void);

/** 清屏 */
void CH1116_Clear(void);

/** 全屏填充 */
void CH1116_Fill(uint8_t data);

/** 刷新显示（写入整屏缓冲区） */
void CH1116_Update(void);

/** 设置光标位置 (页 0~7, 列 0~127) */
void CH1116_SetCursor(uint8_t page, uint8_t col);

/** 在当前位置显示一个 6x8 英文字符 */
void CH1116_ShowChar(uint8_t page, uint8_t col, char ch);

/** 显示字符串，自动换行 */
void CH1116_ShowString(uint8_t page, uint8_t col, const char *str);

/** 画一个像素点 */
void CH1116_DrawPixel(uint8_t x, uint8_t y, uint8_t color);

/** 画水平线 */
void CH1116_DrawHLine(uint8_t x, uint8_t y, uint8_t len, uint8_t color);

/** 显示数字 */
void CH1116_ShowNum(uint8_t page, uint8_t col, int32_t num, uint8_t len);

/** 显示无符号数字 */
void CH1116_ShowUNum(uint8_t page, uint8_t col, uint32_t num, uint8_t len);

/** 显示浮点数 (1位小数) */
void CH1116_ShowFloat(uint8_t page, uint8_t col, float num, uint8_t len);

/** 显示汉字 (16x16, 需外部字库) */
void CH1116_ShowChinese(uint8_t page, uint8_t col, const uint8_t *hz_data);

/** 显示 16x16 点阵汉字字符串 (需字库数组) */
void CH1116_ShowHZString(uint8_t page, uint8_t col, const uint8_t *str,
                          const uint8_t (*font)[32], uint8_t font_start);

#endif
