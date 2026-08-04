/**
 *  HC-05 蓝牙串口驱动库 实现
 */
#include "uart_bt.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ========== 环形缓冲区 ========== */
static volatile uint8_t  g_ring[UART_BT_BUF_SIZE];
static volatile uint16_t g_wr = 0;    /* 写指针 (ISR 独占) */
static volatile uint16_t g_rd = 0;    /* 读指针 (主循环独占) */
static volatile uint32_t g_total = 0; /* 总接收字节数 */

/* ========== UART ISR ========== */

void BT_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(BT_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(BT_INST)) {
            uint8_t c = DL_UART_Main_receiveData(BT_INST);
            g_total++;
            uint16_t next = (g_wr + 1) % UART_BT_BUF_SIZE;
            if (next != g_rd) {
                g_ring[g_wr] = c;
                g_wr = next;
            }
        }
        break;
    default:
        break;
    }
}

/* ========== 初始化 ========== */

static void poll_rx_fifo(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(BT_INST)) {
        uint16_t next = (uint16_t) ((g_wr + 1U) % UART_BT_BUF_SIZE);
        uint8_t byte = DL_UART_Main_receiveData(BT_INST);

        g_total++;
        if (next != g_rd) {
            g_ring[g_wr] = byte;
            g_wr = next;
        }
    }
}

void UartBT_Init(void)
{
    g_wr = 0U;
    g_rd = 0U;
    g_total = 0U;
    DL_UART_Main_setRXFIFOThreshold(BT_INST,
        DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    /*
     * HC-05 uses one-byte commands. Polling the RX FIFO is non-blocking and
     * avoids depending on board-specific UART interrupt routing.
     */
    DL_UART_Main_disableInterrupt(BT_INST,
        DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_clearInterruptStatus(BT_INST, 0xFFU);
    NVIC_DisableIRQ(BT_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(BT_INST_INT_IRQN);
    while (!DL_UART_Main_isRXFIFOEmpty(BT_INST)) {
        (void) DL_UART_Main_receiveData(BT_INST);
    }
}

/* ========== 接收 ========== */

uint16_t UartBT_Available(void)
{
    poll_rx_fifo();
    if (g_wr >= g_rd)
        return g_wr - g_rd;
    else
        return UART_BT_BUF_SIZE - g_rd + g_wr;
}

bool UartBT_Read(uint8_t *c)
{
    poll_rx_fifo();
    if (g_rd == g_wr) return false;
    *c = g_ring[g_rd];
    g_rd = (g_rd + 1) % UART_BT_BUF_SIZE;
    return true;
}

uint16_t UartBT_ReadLine(char *buf, uint16_t maxlen)
{
    uint16_t i;
    for (i = 0; i < maxlen - 1; i++) {
        uint8_t c;
        if (!UartBT_Read(&c)) break;
        if (c == '\n') { buf[i] = '\0'; return i; }
        if (c == '\r') continue;
        buf[i] = (char)c;
    }
    buf[i] = '\0';
    return i;
}

uint16_t UartBT_ReadAll(uint8_t *buf, uint16_t maxlen)
{
    uint16_t i;
    for (i = 0; i < maxlen; i++) {
        if (!UartBT_Read(&buf[i])) break;
    }
    return i;
}

bool UartBT_Peek(uint16_t idx, uint8_t *c)
{
    if (idx >= UartBT_Available()) return false;
    *c = g_ring[(g_rd + idx) % UART_BT_BUF_SIZE];
    return true;
}

void UartBT_Flush(uint16_t n)
{
    uint16_t avail = UartBT_Available();
    if (n > avail) n = avail;
    g_rd = (g_rd + n) % UART_BT_BUF_SIZE;
}

void UartBT_Clear(void)
{
    g_rd = g_wr;
}

uint32_t UartBT_GetRxCount(void)
{
    return g_total;
}

/* ========== 发送 ========== */

void UartBT_Write(uint8_t c)
{
    DL_UART_Main_transmitDataBlocking(BT_INST, c);
}

void UartBT_WriteStr(const char *s)
{
    while (*s) UartBT_Write((uint8_t)*s++);
}

void UartBT_WriteBuf(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
        UartBT_Write(data[i]);
}

void UartBT_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list va;
    va_start(va, fmt);
    vsnprintf(buf, sizeof(buf), fmt, va);
    va_end(va);
    UartBT_WriteStr(buf);
}

/* ========== 命令行 ========== */

bool UartBT_GetCommand(char *buf, uint16_t maxlen)
{
    /* 检查缓冲区中是否有 '\n' 或 '\r' */
    uint16_t avail = UartBT_Available();
    for (uint16_t i = 0; i < avail; i++) {
        uint8_t c;
        UartBT_Peek(i, &c);
        if (c == '\n' || c == '\r') {
            /* 读到完整一行 */
            uint16_t len = 0;
            while (len < maxlen - 1 && UartBT_Available() > 0) {
                uint8_t ch;
                UartBT_Read(&ch);
                if (ch == '\n' || ch == '\r') {
                    if (len == 0) continue; /* 跳过空行开头 */
                    buf[len] = '\0';
                    /* 丢弃后续的 \r\n */
                    while (UartBT_Available() > 0) {
                        UartBT_Peek(0, &ch);
                        if (ch != '\n' && ch != '\r') break;
                        UartBT_Read(&ch);
                    }
                    return true;
                }
                buf[len++] = (char)ch;
            }
            buf[len] = '\0';
            return len > 0;
        }
    }
    return false;
}
