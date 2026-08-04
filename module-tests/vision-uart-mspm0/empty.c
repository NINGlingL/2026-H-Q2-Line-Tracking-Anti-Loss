/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

/**
 * MSPM0G3507 串口接收并显示到 OLED
 * 功能: 串口接收任意字符并显示在 OLED 上
 * 配置: UART0 (PA10/PA11, 115200bps), I2C0 (PA0/PA1)
 */

#include "ti_msp_dl_config.h"
#include "ssd1306.h"
#include <string.h>

/* ========== 全局变量 ========== */
#define RX_BUFFER_SIZE      256
static char rx_buffer[RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static char display_buffer[128];
static uint8_t display_line = 0;
static uint8_t display_col = 0;

/* ========== 辅助函数 ========== */

/**
 * 简单延时函数 (毫秒)
 */
void delay_ms(uint32_t ms)
{
    uint32_t cycles = CPUCLK_FREQ / 1000 * ms / 3;
    while (cycles--) {
        __NOP();
    }
}

/**
 * 检查接收缓冲区是否有数据
 */
bool is_data_available(void)
{
    return (rx_head != rx_tail);
}

/**
 * 从接收缓冲区读取一个字符
 */
char get_char(void)
{
    if (rx_head == rx_tail) {
        return 0;
    }

    char ch = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    return ch;
}

/**
 * 清空显示缓冲区
 */
void clear_display_buffer(void)
{
    memset(display_buffer, 0, sizeof(display_buffer));
    display_line = 0;
    display_col = 0;
}

/**
 * 添加字符到显示缓冲区
 */
void add_char_to_display(char ch)
{
    // 处理回车换行
    if (ch == '\n' || ch == '\r') {
        display_col = 0;
        display_line++;
        if (display_line >= 8) {
            // 屏幕满了，滚动显示
            display_line = 7;
            // 简单实现：清屏重新开始
            clear_display_buffer();
        }
        return;
    }

    // 处理退格
    if (ch == '\b' || ch == 0x7F) {
        if (display_col > 0) {
            display_col--;
        }
        return;
    }

    // 处理可显示字符
    if (ch >= 0x20 && ch <= 0x7E) {
        // 检查是否需要换行 (每行最多21个字符, 6像素宽字体)
        if (display_col >= 21) {
            display_col = 0;
            display_line++;
            if (display_line >= 8) {
                display_line = 7;
                clear_display_buffer();
            }
        }

        // 显示字符
        SSD1306_ShowChar(display_line, display_col * 6, ch);
        display_col++;
    }
}

/**
 * 显示启动画面
 */
void display_startup_screen(void)
{
    SSD1306_Clear();
    SSD1306_ShowString(2, 20, "MSPM0G3507");
    SSD1306_ShowString(3, 10, "UART Display");
    SSD1306_ShowString(5, 10, "115200 bps");
    SSD1306_ShowString(6, 10, "Ready...");
    SSD1306_Update();
}

/**
 * 显示欢迎信息
 */
void display_welcome(void)
{
    SSD1306_Clear();
    SSD1306_ShowString(0, 0, "UART->OLED");
    SSD1306_ShowString(1, 0, "Send data:");
    SSD1306_Update();
    display_line = 2;
    display_col = 0;
}

/* ========== 中断处理函数 ========== */

/**
 * UART0 中断服务函数
 */
void UART0_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(UART_0_INST) & DL_UART_IIDX_RX) {
        while (DL_UART_isRXFIFOEmpty(UART_0_INST) == false) {
            char ch = DL_UART_receiveData(UART_0_INST);

            // 存入环形缓冲区
            uint16_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;
            if (next_head != rx_tail) {
                rx_buffer[rx_head] = ch;
                rx_head = next_head;
            }
        }
    }
}

/* ========== 主函数 ========== */

int main(void)
{
    // 系统初始化
    SYSCFG_DL_init();

    // 延时等待电源稳定
    delay_ms(100);

    // 初始化 OLED
    SSD1306_Init();
    display_startup_screen();
    delay_ms(2000);

    // 使能 UART 中断
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    // 显示欢迎界面
    display_welcome();

    // 主循环
    while (1) {
        // 处理接收到的字符
        if (is_data_available()) {
            char ch = get_char();

            // 特殊命令处理
            if (ch == 0x0C) {  // Ctrl+L: 清屏
                SSD1306_Clear();
                display_welcome();
            } else {
                // 显示字符
                add_char_to_display(ch);
                SSD1306_Update();
            }
        }

        // 小延时
        for (volatile uint32_t i = 0; i < 100; i++);
    }
}
