/**
 * 八路巡线红外传感器驱动 (MSPM0G3507, UART3 PA14=TX PA13=RX)
 *
 * 使用 UART RX 中断接收数据，避免 OLED 刷屏期间 UART FIFO 溢出。
 * 数字量协议: $D,x1:0,x2:1,x3:1,x4:1,x5:1,x6:1,x7:1,x8:1#
 */
#include "eight_ir.h"
#include <string.h>

#define EIGHT_IR_RING_SIZE 2048U
#define EIGHT_IR_FRAME_SIZE 128U

static volatile uint8_t g_rx_ring[EIGHT_IR_RING_SIZE];
static volatile uint16_t g_rx_head = 0;
static volatile uint16_t g_rx_tail_idx = 0;

static uint8_t g_buf[EIGHT_IR_FRAME_SIZE];
static uint8_t g_idx  = 0;
static uint8_t g_go   = 0;
static uint8_t g_val[8] = {1,1,1,1,1,1,1,1};
static uint8_t g_frame_ready = 0;
static uint8_t g_last_raw = 0xFF;
static uint8_t g_rx_tail_pos = 0;

static volatile EightIR_Stats g_stats = {
    .last_frame_type = '-',
    .rx_tail = "----------------",
};

static void _send(const char *s)
{
    while (*s) DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t)*s++);
}

static void _update_tail(uint8_t c)
{
    if (c == '\r') c = 'r';
    else if (c == '\n') c = 'n';
    else if (c < 0x20 || c > 0x7E) c = '.';

    g_stats.rx_tail[g_rx_tail_pos++] = (char)c;
    if (g_rx_tail_pos >= 16) g_rx_tail_pos = 0;
    g_stats.rx_tail[16] = '\0';
}

static uint8_t _parse_digital(uint8_t *v)
{
    uint8_t ok = 1;

    /* 标准格式: $D,x1:0,x2:1,...,x8:0# */
    for (int i = 0; i < 8; i++) {
        uint8_t *p = (uint8_t *) strstr((const char *) g_buf, i == 0 ? "x1:" :
            i == 1 ? "x2:" : i == 2 ? "x3:" : i == 3 ? "x4:" :
            i == 4 ? "x5:" : i == 5 ? "x6:" : i == 6 ? "x7:" : "x8:");
        if (p && (p[3] == '0' || p[3] == '1')) v[i] = p[3] - '0';
        else ok = 0;
    }
    if (ok) return 1;

    /* 兼容格式: $D,0,1,1,...# 或 $D,01111111# */
    uint8_t cnt = 0;
    uint8_t plain = 0;
    for (uint8_t i = 2; g_buf[i] && g_buf[i] != '#' && cnt < 8; i++) {
        if (g_buf[i] == ':' && (g_buf[i + 1] == '0' || g_buf[i + 1] == '1')) {
            v[cnt++] = g_buf[++i] - '0';
            plain = 0;
        } else if (g_buf[i] == ',') {
            plain = 1;
        } else if (plain && (g_buf[i] == '0' || g_buf[i] == '1')) {
            v[cnt++] = g_buf[i] - '0';
        } else if (g_buf[i] != ' ' && g_buf[i] != '\r' && g_buf[i] != '\n') {
            plain = 0;
        }
    }

    return cnt == 8;
}

static void _update_raw(void)
{
    g_last_raw = (g_val[0]<<7)|(g_val[1]<<6)|(g_val[2]<<5)|(g_val[3]<<4)|
                 (g_val[4]<<3)|(g_val[5]<<2)|(g_val[6]<<1)| g_val[7];
}

static void _feed(uint8_t c)
{
    if (c == '$') {
        g_go  = 1;
        g_idx = 0;
        g_buf[g_idx++] = c;
        return;
    }
    if (!g_go) return;

    if (g_idx >= sizeof(g_buf) - 1) {
        g_go = 0;
        g_idx = 0;
        g_stats.bad_frames++;
        return;
    }

    g_buf[g_idx++] = c;

    if (c == '#') {
        g_go = 0;
        g_buf[g_idx] = '\0';

        if (g_buf[1] == 'D') {
            uint8_t v[8];
            g_stats.digital_candidates++;
            if (_parse_digital(v)) {
                for (int i = 0; i < 8; i++) g_val[i] = v[i];
                _update_raw();
                g_frame_ready = 1;
                g_stats.digital_frames++;
            } else {
                g_stats.bad_frames++;
            }
            g_stats.last_frame_type = 'D';
        } else if (g_buf[1] == 'A') {
            g_stats.analog_frames++;
            g_stats.last_frame_type = 'A';
        } else {
            g_stats.bad_frames++;
            g_stats.last_frame_type = g_buf[1];
        }
    }
}

void EightIR_Init(void)
{
    memset((void *)g_rx_ring, 0, sizeof(g_rx_ring));
    memset((void *)&g_stats, 0, sizeof(g_stats));
    g_stats.last_frame_type = '-';
    for (int i = 0; i < 16; i++) g_stats.rx_tail[i] = '-';
    g_stats.rx_tail[16] = '\0';

    g_rx_head = 0;
    g_rx_tail_idx = 0;
    g_rx_tail_pos = 0;
    g_idx = 0;
    g_go = 0;
    g_frame_ready = 0;
    for (int i = 0; i < 8; i++) g_val[i] = 1;
    _update_raw();

    while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST))
        DL_UART_Main_receiveDataBlocking(UART_0_INST);

    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    EightIR_StartDigital();
}

void EightIR_Poll(void)
{
    while (g_rx_tail_idx != g_rx_head) {
        uint8_t c = g_rx_ring[g_rx_tail_idx++];
        if (g_rx_tail_idx >= EIGHT_IR_RING_SIZE) g_rx_tail_idx = 0;
        _update_tail(c);
        _feed(c);
    }
}

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) {
                uint16_t next = g_rx_head + 1U;
                if (next >= EIGHT_IR_RING_SIZE) next = 0;
                uint8_t c = DL_UART_Main_receiveDataBlocking(UART_0_INST);
                g_stats.rx_bytes++;
                if (next != g_rx_tail_idx) {
                    g_rx_ring[g_rx_head] = c;
                    g_rx_head = next;
                } else {
                    g_stats.overflow_bytes++;
                }
            }
            break;
        default:
            break;
    }
}

void EightIR_StartDigital(void) { _send("$0,0,1#"); }
void EightIR_StartAnalog(void) { _send("$0,1,0#"); }
void EightIR_StartDigitalAndAnalog(void) { _send("$0,1,1#"); }
void EightIR_StartCalibrate(void) { _send("$1,0,0#"); }
void EightIR_StopOutput(void) { _send("$0,0,0#"); }
void EightIR_SendCmd(const char *cmd) { _send(cmd); }

uint8_t EightIR_HasFrame(void)
{
    EightIR_Poll();
    return g_frame_ready;
}

uint8_t EightIR_GetByte(void)
{
    EightIR_Poll();
    return g_last_raw;
}

uint8_t EightIR_GetActiveCount(void)
{
    uint8_t count = 0;
    EightIR_Poll();
    for (int i = 0; i < 8; i++) if (g_val[i] == 0) count++;
    return count;
}

int8_t EightIR_GetPosition(void)
{
    static const int8_t weight[8] = {-7,-5,-3,-1,1,3,5,7};
    int16_t pos = 0;
    uint8_t count = 0;

    EightIR_Poll();
    for (int i = 0; i < 8; i++) {
        if (g_val[i] == 0) {
            pos += weight[i];
            count++;
        }
    }
    return count ? (int8_t)(pos / count) : 0;
}

void EightIR_GetState(EightIR_State *state)
{
    if (!state) return;

    EightIR_Poll();
    for (int i = 0; i < 8; i++) state->ch[i] = g_val[i];
    state->raw = g_last_raw;
    state->active_count = EightIR_GetActiveCount();
    state->position = EightIR_GetPosition();
    state->frame_ready = g_frame_ready;
}

void EightIR_GetStats(EightIR_Stats *stats)
{
    if (!stats) return;

    EightIR_Poll();
    *stats = g_stats;
}

void EightIR_Read(uint8_t *raw)
{
    if (!raw) return;
    *raw = EightIR_GetByte();
}

void EightIR_GetDigits(uint8_t *s1,uint8_t *s2,uint8_t *s3,uint8_t *s4,
                       uint8_t *s5,uint8_t *s6,uint8_t *s7,uint8_t *s8)
{
    EightIR_Poll();
    if (s1) *s1=g_val[0]; if (s2) *s2=g_val[1];
    if (s3) *s3=g_val[2]; if (s4) *s4=g_val[3];
    if (s5) *s5=g_val[4]; if (s6) *s6=g_val[5];
    if (s7) *s7=g_val[6]; if (s8) *s8=g_val[7];
}
