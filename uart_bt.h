/**
 *  HC-05 蓝牙串口驱动库 (MSPM0G3507 + UART2)
 *
 *  硬件: PB16=RX  PB15=TX  9600bps 8N1
 *  使用中断接收 + 环形缓冲区
 */
#ifndef __UART_BT_H__
#define __UART_BT_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ========== 缓冲区大小 ========== */
#define UART_BT_BUF_SIZE  256

/* ========== 初始化 ========== */

/** 初始化蓝牙串口 (波特率 9600, 使能 RX 中断, NVIC) */
void UartBT_Init(void);

/* ========== 接收 ========== */

/** 缓冲区中有多少字节可读 */
uint16_t UartBT_Available(void);

/** 读取一个字节 (缓冲区空返回 0, 阻塞版用 UartBT_ReadBlocking) */
bool UartBT_Read(uint8_t *c);

/** 读取一行 (直到 '\n' 或 maxlen-1), 返回实际长度 */
uint16_t UartBT_ReadLine(char *buf, uint16_t maxlen);

/** 读取所有可用数据到 buf, 返回字节数 */
uint16_t UartBT_ReadAll(uint8_t *buf, uint16_t maxlen);

/** 查看第 n 个字节但不取出 */
bool UartBT_Peek(uint16_t idx, uint8_t *c);

/** 丢弃 n 个字节 */
void UartBT_Flush(uint16_t n);

/** 清空缓冲区 */
void UartBT_Clear(void);

/** 获取总接收字节数 */
uint32_t UartBT_GetRxCount(void);

/* ========== 发送 ========== */

/** 发送一个字节 */
void UartBT_Write(uint8_t c);

/** 发送字符串 */
void UartBT_WriteStr(const char *s);

/** 发送数据块 */
void UartBT_WriteBuf(const uint8_t *data, uint16_t len);

/** 格式化发送 (类似 printf, 最大 128 字节) */
void UartBT_Printf(const char *fmt, ...);

/* ========== 命令行 ========== */

/** 非阻塞检查是否有完整一行, 有则复制到 buf 返回 true */
bool UartBT_GetCommand(char *buf, uint16_t maxlen);

#endif
