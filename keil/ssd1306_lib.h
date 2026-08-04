/**
 * SSD1306 0.96瀵?OLED 椹卞姩搴?(I2C)
 * 閫傜敤鑺墖: MSPM0G3507
 * 鍒嗚鲸鐜? 128x64
 * I2C鍦板潃: 0x3C
 */

#ifndef __SSD1306_LIB_H__
#define __SSD1306_LIB_H__

#include "../ti_msp_dl_config.h"
#include <string.h>

/* ========== SSD1306 鍦板潃 & 灏哄 ========== */
#define SSD1306_I2C_ADDR    0x3C
#define SSD1306_WIDTH       128
#define SSD1306_HEIGHT      64
#define SSD1306_PAGES       8       // 64/8 = 8 椤?
/* ========== 鎺у埗瀛楄妭 ========== */
#define SSD1306_CMD_SINGLE  0x00    // 涓嬩竴涓瓧鑺傛槸鍛戒护
#define SSD1306_CMD_STREAM  0x00    // 鍚庣画閮芥槸鍛戒护
#define SSD1306_DATA_STREAM 0x40    // 鍚庣画閮芥槸鏁版嵁

/* ========== 鍩烘湰鍛戒护 ========== */
#define SSD1306_DISPLAY_OFF     0xAE
#define SSD1306_DISPLAY_ON      0xAF
#define SSD1306_SET_CONTRAST    0x81
#define SSD1306_DISPLAY_ALL_ON  0xA5

/* ========== API ========== */

/** 鍒濆鍖?OLED锛屽繀椤诲湪 SYSCFG_DL_init() 涔嬪悗璋冪敤 */
void SSD1306_Init(void);

/** 娓呭睆 */
void SSD1306_Clear(void);

/** 鍏ㄥ睆濉厖 */
void SSD1306_Fill(uint8_t data);

/** 鍒锋柊鏄剧ず锛堝啓鍏ユ暣灞忕紦鍐插尯锛?*/
void SSD1306_Update(void);

/** 璁剧疆鍏夋爣浣嶇疆 (椤?0~7, 鍒?0~127) */
void SSD1306_SetCursor(uint8_t page, uint8_t col);

/** 鍦ㄥ綋鍓嶄綅缃樉绀轰竴涓?6x8 鑻辨枃瀛楃 */
void SSD1306_ShowChar(uint8_t page, uint8_t col, char ch);

/** 鏄剧ず瀛楃涓诧紝鑷姩鎹㈣ */
void SSD1306_ShowString(uint8_t page, uint8_t col, const char *str);

/** 鐢讳竴涓儚绱犵偣 */
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);

/** 鐢绘按骞崇嚎 */
void SSD1306_DrawHLine(uint8_t x, uint8_t y, uint8_t len, uint8_t color);

/** 鏄剧ず鏁板瓧 */
void SSD1306_ShowNum(uint8_t page, uint8_t col, int32_t num, uint8_t len);

#endif

