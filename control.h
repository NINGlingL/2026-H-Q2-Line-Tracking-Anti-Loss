/**
 *  蓝牙指令控制器 (解析 HC-05 指令 → 控制电机)
 *
 *  合法指令: F/B/L/R = 方向  S = 停  -/+ = 加减速  0-9 = 档位
 *  使用前初始化 UartBT + Moto 库, 然后循环调用 Control_Poll()
 */
#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "ti_msp_dl_config.h"
#include "moto.h"
#include "uart_bt.h"
#include <stdbool.h>

/* ========== 初始化 ========== */

/** 初始化控制器状态 */
void Control_Init(void);

/* ========== 主循环轮询 ========== */

/**
 * 从蓝牙缓冲区读取并执行一条指令
 * @return true 有新指令被执行
 */
bool Control_Poll(void);

/* ========== 状态查询 ========== */

/** 最后执行的指令描述字符串 */
const char *Control_LastCmd(void);

/** 是否有新指令待刷新 */
bool Control_NeedRefresh(void);

/** 清除刷新标志 (OLED 刷新后调用) */
void Control_ClearRefresh(void);

#endif
