/**
 * 八路巡线红外传感器驱动 (MSPM0G3507)
 *
 * 硬件连接:
 *   UART3 PA14 = TX -> 模块 RX
 *   UART3 PA13 = RX <- 模块 TX
 *   波特率 115200, 8N1
 *
 * 模块协议:
 *   $0,0,1#  启动数字量连续发送
 *   $D,x1:0,x2:1,x3:1,x4:1,x5:1,x6:1,x7:1,x8:1#
 *   0 = 检测到黑线, 1 = 白底
 */
#ifndef __EIGHT_IR_H__
#define __EIGHT_IR_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

typedef struct {
    uint8_t ch[8];          /* x1~x8: 0=黑线, 1=白底 */
    uint8_t raw;            /* bit7=x1 ... bit0=x8 */
    uint8_t active_count;   /* 检测到黑线的探头数量 */
    int8_t position;        /* -7..+7, 左负右正；无黑线时为0 */
    uint8_t frame_ready;    /* 已成功收到至少一帧数字量数据 */
} EightIR_State;

typedef struct {
    uint32_t rx_bytes;
    uint32_t digital_frames;
    uint32_t digital_candidates;
    uint32_t analog_frames;
    uint32_t bad_frames;
    uint32_t overflow_bytes;
    uint8_t last_frame_type;
    char rx_tail[17];       /* 最近16个可显示字符，调试用 */
} EightIR_Stats;

void EightIR_Init(void);
void EightIR_Poll(void);
void EightIR_StartDigital(void);
void EightIR_StartAnalog(void);
void EightIR_StartDigitalAndAnalog(void);
void EightIR_StartCalibrate(void);
void EightIR_StopOutput(void);
void EightIR_SendCmd(const char *cmd);

uint8_t EightIR_HasFrame(void);
uint8_t EightIR_GetByte(void);
uint8_t EightIR_GetActiveCount(void);
int8_t EightIR_GetPosition(void);
void EightIR_GetState(EightIR_State *state);
void EightIR_GetStats(EightIR_Stats *stats);

/* 兼容旧接口 */
void EightIR_Read(uint8_t *raw);
void EightIR_GetDigits(uint8_t *s1,uint8_t *s2,uint8_t *s3,uint8_t *s4,
                       uint8_t *s5,uint8_t *s6,uint8_t *s7,uint8_t *s8);

#endif
