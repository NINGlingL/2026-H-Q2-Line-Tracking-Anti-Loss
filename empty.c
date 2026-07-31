/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "battery.h"
#include "control.h"
#include "eight_ir.h"
#include "encoder.h"
#include "icm20948.h"
#include "moto.h"
#include "ssd1306.h"
#include "timebase.h"
#include "ti_msp_dl_config.h"
#include "uart_bt.h"

int main(void)
{
    uint32_t now_ms;

    SYSCFG_DL_init();
    Moto_Init();
    Timebase_Init();
    Battery_Init();
    Encoder_Init();
    UartBT_Init();
    EightIR_Init(Timebase_Millis());
#if APP_ENABLE_IMU
    (void) ICM20948_Init();
#endif
#if APP_ENABLE_OLED
    (void) SSD1306_Init();
#endif
    Control_Init(Timebase_Millis());

    while (1) {
        now_ms = Timebase_Millis();
        EightIR_Service(now_ms);
        Battery_Service(now_ms);
        Control_Service(now_ms);

        /* The independent watchdog is fed only at the main-loop tail. */
        DL_WWDT_restart(WWDT0_INST);
        __WFE();
    }
}
