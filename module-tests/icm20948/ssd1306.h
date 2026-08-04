/**
 * SSD1306 128x64 OLED 驱动 (I2C)
 * I2C_0_INST PA0=SDA PA1=SCL 地址 0x3C
 */
#ifndef __SSD1306_H__
#define __SSD1306_H__
#include <stdint.h>

#define SSD1306_I2C_ADDR  0x3C
#define SSD1306_WIDTH     128
#define SSD1306_HEIGHT    64
#define SSD1306_PAGES     8

void SSD1306_Init(void);
void SSD1306_Clear(void);
void SSD1306_Update(void);
void SSD1306_ShowString(uint8_t page, uint8_t col, const char *str);
void SSD1306_WriteFloat(uint8_t page, uint8_t col, float val, uint8_t dec);
#endif
