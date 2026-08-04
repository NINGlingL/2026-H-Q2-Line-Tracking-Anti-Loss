/**
 *  蓝牙指令控制器 实现
 */
#include "control.h"
#include <stdio.h>

static char     g_last[21] = "None";
static bool     g_refresh  = false;

/* ========== 字符过滤 ========== */

static bool is_cmd(uint8_t c)
{
    if (c == 'F' || c == 'f') return true;
    if (c == 'B' || c == 'b') return true;
    if (c == 'L' || c == 'l') return true;
    if (c == 'R' || c == 'r') return true;
    if (c == 'S' || c == 's') return true;
    if (c == '+' || c == '-' || c == '=' || c == '_') return true;
    if (c >= '0' && c <= '9') return true;
    return false;
}

static bool cmd_read(uint8_t *c)
{
    while (UartBT_Read(c)) {
        if (is_cmd(*c)) return true;
    }
    return false;
}

/* ========== 指令执行 ========== */

static void cmd_exec(char c)
{
    uint8_t sp = Moto_GetSpeed();

    switch (c) {
    /* ---- 方向 ---- */
    case 'F': case 'f':
        Moto_Forward(sp ? sp : 50);
        snprintf(g_last, sizeof(g_last), "FWD  %u%%", Moto_GetSpeed());
        break;
    case 'B': case 'b':
        Moto_Backward(sp ? sp : 50);
        snprintf(g_last, sizeof(g_last), "BACK %u%%", Moto_GetSpeed());
        break;
    case 'L': case 'l':
        Moto_Left(sp ? sp : 50);
        snprintf(g_last, sizeof(g_last), "LEFT %u%%", Moto_GetSpeed());
        break;
    case 'R': case 'r':
        Moto_Right(sp ? sp : 50);
        snprintf(g_last, sizeof(g_last), "RIGHT %u%%", Moto_GetSpeed());
        break;
    case 'S': case 's':
        Moto_Stop();
        snprintf(g_last, sizeof(g_last), "STOP");
        break;

    /* ---- 加减速 ---- */
    case '-': case '_':
        Moto_SpeedUp(10);
        snprintf(g_last, sizeof(g_last), "Spd+ %u%%", Moto_GetSpeed());
        break;
    case '+': case '=':
        Moto_SpeedDown(10);
        snprintf(g_last, sizeof(g_last), "Spd- %u%%", Moto_GetSpeed());
        break;

    /* ---- 档位: 只调速, 方向为S时才切Forward ---- */
    case '0':
        Moto_Stop();
        snprintf(g_last, sizeof(g_last), "Spd 0%% STOP");
        break;
    case '1': case '2': case '3':
    case '4': case '5': case '6':
    case '7': case '8': case '9':
    {
        uint8_t pct = (uint8_t)(c - '0') * 10;
        Moto_SetSpeed(pct);
        snprintf(g_last, sizeof(g_last), "Spd %u%%", pct);
        if (Moto_GetDirection() == 'S') Moto_Forward(pct);
        break;
    }

    default:
        snprintf(g_last, sizeof(g_last), "?:%c", c);
        break;
    }
}

/* ========== 对外接口 ========== */

void Control_Init(void)
{
    g_last[0] = '\0';
    g_refresh = false;
}

bool Control_Poll(void)
{
    uint8_t c;
    if (!cmd_read(&c)) return false;

    cmd_exec((char)c);
    g_refresh = true;
    return true;
}

const char *Control_LastCmd(void)    { return g_last; }
bool        Control_NeedRefresh(void) { return g_refresh; }

void Control_ClearRefresh(void)      { g_refresh = false; }
