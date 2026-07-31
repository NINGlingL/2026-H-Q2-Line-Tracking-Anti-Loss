#include "eight_ir.h"
#include "app_config.h"
#include "ti_msp_dl_config.h"
#include <string.h>

#define IR_RING_SIZE  (512U)
#define IR_FRAME_SIZE (96U)

static volatile uint8_t g_ring[IR_RING_SIZE];
static volatile uint16_t g_write_index;
static volatile uint16_t g_read_index;
static volatile uint32_t g_overflow_bytes;
static uint8_t g_frame[IR_FRAME_SIZE];
static uint8_t g_frame_index;
static uint8_t g_in_frame;
static uint8_t g_stream_requested;
static uint8_t g_reversed;
static uint32_t g_start_ms;
static EightIR_State g_state;

static void uart_send(const char *text)
{
    while (*text != '\0') {
        DL_UART_Main_transmitDataBlocking(IR8_INST, (uint8_t) *text);
        text++;
    }
}

static uint8_t parse_channel(const char *frame, uint8_t channel,
                             uint8_t *value)
{
    char key[4];
    const char *found;

    key[0] = 'x';
    key[1] = (char) ('1' + channel);
    key[2] = ':';
    key[3] = '\0';
    found = strstr(frame, key);
    if (found == NULL || (found[3] != '0' && found[3] != '1')) {
        return 0U;
    }
    *value = (uint8_t) (found[3] - '0');
    return 1U;
}

static void accept_digital_frame(uint32_t now_ms)
{
    static const int8_t weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
    uint8_t parsed[8];
    uint8_t active = 0U;
    uint8_t raw = 0U;
    int16_t weighted_sum = 0;
    uint8_t i;

    for (i = 0U; i < 8U; i++) {
        if (parse_channel((const char *) g_frame, i, &parsed[i]) == 0U) {
            g_state.bad_frames++;
            return;
        }
    }

    for (i = 0U; i < 8U; i++) {
        uint8_t source = (g_reversed != 0U) ? (uint8_t) (7U - i) : i;
        g_state.channels[i] = parsed[source];
        raw = (uint8_t) ((raw << 1) | parsed[source]);
        if (parsed[source] == 0U) {
            active++;
            weighted_sum += weights[i];
        }
    }

    g_state.raw = raw;
    g_state.active_count = active;
    if (active > 0U) {
        g_state.position = (int8_t) (weighted_sum / (int16_t) active);
    }
    g_state.frame_valid = 1U;
    g_state.frame_fresh = 1U;
    g_state.warming_up = 0U;
    g_state.last_frame_ms = now_ms;
    g_state.good_frames++;
}

static void feed_byte(uint8_t byte, uint32_t now_ms)
{
    if (byte == '$') {
        g_in_frame = 1U;
        g_frame_index = 0U;
        g_frame[g_frame_index++] = byte;
        return;
    }
    if (g_in_frame == 0U) {
        return;
    }
    if (g_frame_index >= (IR_FRAME_SIZE - 1U)) {
        g_in_frame = 0U;
        g_frame_index = 0U;
        g_state.bad_frames++;
        return;
    }

    g_frame[g_frame_index++] = byte;
    if (byte == '#') {
        g_frame[g_frame_index] = '\0';
        g_in_frame = 0U;
        if (g_frame_index > 2U && g_frame[1] == 'D') {
            accept_digital_frame(now_ms);
        }
    }
}

void EightIR_Init(uint32_t now_ms)
{
    uint8_t i;

    memset((void *) g_ring, 0, sizeof(g_ring));
    memset(&g_state, 0, sizeof(g_state));
    g_write_index = 0U;
    g_read_index = 0U;
    g_overflow_bytes = 0U;
    g_frame_index = 0U;
    g_in_frame = 0U;
    g_stream_requested = 0U;
    g_reversed = APP_IR_REVERSED_DEFAULT;
    g_start_ms = now_ms;
    g_state.raw = 0xFFU;
    g_state.warming_up = 1U;
    for (i = 0U; i < 8U; i++) {
        g_state.channels[i] = 1U;
    }

    while (!DL_UART_Main_isRXFIFOEmpty(IR8_INST)) {
        (void) DL_UART_Main_receiveData(IR8_INST);
    }
    NVIC_ClearPendingIRQ(IR8_INST_INT_IRQN);
    NVIC_EnableIRQ(IR8_INST_INT_IRQN);
}

void EightIR_Service(uint32_t now_ms)
{
    if (g_stream_requested == 0U &&
        (uint32_t) (now_ms - g_start_ms) >= APP_IR_WARMUP_MS) {
        uart_send("$0,0,1#");
        g_stream_requested = 1U;
    }

    while (g_read_index != g_write_index) {
        uint8_t byte = g_ring[g_read_index];
        g_read_index++;
        if (g_read_index >= IR_RING_SIZE) {
            g_read_index = 0U;
        }
        feed_byte(byte, now_ms);
    }
    g_state.overflow_bytes = g_overflow_bytes;

    if (g_state.frame_valid != 0U &&
        (uint32_t) (now_ms - g_state.last_frame_ms) >
            APP_IR_FRAME_TIMEOUT_MS) {
        g_state.frame_fresh = 0U;
    }
}

void EightIR_SetReversed(uint8_t reversed)
{
    g_reversed = (reversed != 0U) ? 1U : 0U;
}

uint8_t EightIR_IsReversed(void)
{
    return g_reversed;
}

EightIR_State EightIR_GetState(void)
{
    return g_state;
}

void EightIR_StartCalibrate(void)
{
    uart_send("$1,0,0#");
}

void IR8_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(IR8_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(IR8_INST)) {
                uint16_t next = (uint16_t) (g_write_index + 1U);
                uint8_t byte;
                if (next >= IR_RING_SIZE) {
                    next = 0U;
                }
                byte = DL_UART_Main_receiveData(IR8_INST);
                if (next != g_read_index) {
                    g_ring[g_write_index] = byte;
                    g_write_index = next;
                } else {
                    g_overflow_bytes++;
                }
            }
            break;
        default:
            break;
    }
}
