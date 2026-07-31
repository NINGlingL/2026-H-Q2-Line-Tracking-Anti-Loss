/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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

/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gPWM_0Backup;
DL_TimerG_backupConfig gQEI_RBackup;
DL_UART_Main_backupConfig gIR8Backup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_PWM_0_init();
    SYSCFG_DL_QEI_R_init();
    SYSCFG_DL_OLED_init();
    SYSCFG_DL_IMU20948_init();
    SYSCFG_DL_IR8_init();
    SYSCFG_DL_BT_init();
    SYSCFG_DL_BAT_ADC_init();
    SYSCFG_DL_WWDT0_init();
    /* Ensure backup structures have no valid state */
	gPWM_0Backup.backupRdy 	= false;
	gQEI_RBackup.backupRdy 	= false;
	gIR8Backup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(PWM_0_INST, &gPWM_0Backup);
	retStatus &= DL_TimerG_saveConfiguration(QEI_R_INST, &gQEI_RBackup);
	retStatus &= DL_UART_Main_saveConfiguration(IR8_INST, &gIR8Backup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(PWM_0_INST, &gPWM_0Backup, false);
	retStatus &= DL_TimerG_restoreConfiguration(QEI_R_INST, &gQEI_RBackup, false);
	retStatus &= DL_UART_Main_restoreConfiguration(IR8_INST, &gIR8Backup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(PWM_0_INST);
    DL_TimerG_reset(QEI_R_INST);
    DL_I2C_reset(OLED_INST);
    DL_I2C_reset(IMU20948_INST);
    DL_UART_Main_reset(IR8_INST);
    DL_UART_Main_reset(BT_INST);
    DL_ADC12_reset(BAT_ADC_INST);
    DL_WWDT_reset(WWDT0_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(PWM_0_INST);
    DL_TimerG_enablePower(QEI_R_INST);
    DL_I2C_enablePower(OLED_INST);
    DL_I2C_enablePower(IMU20948_INST);
    DL_UART_Main_enablePower(IR8_INST);
    DL_UART_Main_enablePower(BT_INST);
    DL_ADC12_enablePower(BAT_ADC_INST);
    DL_WWDT_enablePower(WWDT0_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C0_IOMUX,GPIO_PWM_0_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_0_C0_PORT, GPIO_PWM_0_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C1_IOMUX,GPIO_PWM_0_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_0_C1_PORT, GPIO_PWM_0_C1_PIN);

    DL_GPIO_initPeripheralInputFunction(GPIO_QEI_R_PHA_IOMUX,GPIO_QEI_R_PHA_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_QEI_R_PHB_IOMUX,GPIO_QEI_R_PHB_IOMUX_FUNC);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_OLED_IOMUX_SDA,
        GPIO_OLED_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_OLED_IOMUX_SCL,
        GPIO_OLED_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_OLED_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_OLED_IOMUX_SCL);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_IMU20948_IOMUX_SDA,
        GPIO_IMU20948_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_IMU20948_IOMUX_SCL,
        GPIO_IMU20948_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_IMU20948_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_IMU20948_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_IR8_IOMUX_TX, GPIO_IR8_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_IR8_IOMUX_RX, GPIO_IR8_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_BT_IOMUX_TX, GPIO_BT_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_BT_IOMUX_RX, GPIO_BT_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutput(STBY_STBY3_IOMUX);

    DL_GPIO_initDigitalInputFeatures(KEY_START_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(MOTO_AIN1_IOMUX);

    DL_GPIO_initDigitalOutput(MOTO_AIN2_IOMUX);

    DL_GPIO_initDigitalOutput(MOTO_BIN1_IOMUX);

    DL_GPIO_initDigitalOutput(MOTO_BIN2_IOMUX);

    DL_GPIO_clearPins(MOTO_PORT, MOTO_AIN1_PIN |
		MOTO_AIN2_PIN |
		MOTO_BIN1_PIN |
		MOTO_BIN2_PIN);
    DL_GPIO_enableOutput(MOTO_PORT, MOTO_AIN1_PIN |
		MOTO_AIN2_PIN |
		MOTO_BIN1_PIN |
		MOTO_BIN2_PIN);
    DL_GPIO_clearPins(GPIOB, STBY_STBY3_PIN);
    DL_GPIO_enableOutput(GPIOB, STBY_STBY3_PIN);

}


SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    /* Set default configuration */
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);

}


/*
 * Timer clock configuration to be sourced by  / 8 (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   4000000 Hz = 4000000 Hz / (8 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gPWM_0ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gPWM_0Config = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 200,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_0_init(void) {

    DL_TimerA_setClockConfig(
        PWM_0_INST, (DL_TimerA_ClockConfig *) &gPWM_0ClockConfig);

    DL_TimerA_initPWMMode(
        PWM_0_INST, (DL_TimerA_PWMConfig *) &gPWM_0Config);

    DL_TimerA_setCaptureCompareOutCtl(PWM_0_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_0_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, 200, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(PWM_0_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_0_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, 200, DL_TIMER_CC_1_INDEX);

    DL_TimerA_enableClock(PWM_0_INST);


    
    DL_TimerA_setCCPDirection(PWM_0_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}


static const DL_TimerG_ClockConfig gQEI_RClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 0U
};


SYSCONFIG_WEAK void SYSCFG_DL_QEI_R_init(void) {

    DL_TimerG_setClockConfig(
        QEI_R_INST, (DL_TimerG_ClockConfig *) &gQEI_RClockConfig);

    DL_TimerG_configQEI(QEI_R_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);
    DL_TimerG_configQEI(QEI_R_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setLoadValue(QEI_R_INST, 65535);
    DL_TimerG_enableClock(QEI_R_INST);
}


static const DL_I2C_ClockConfig gOLEDClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_OLED_init(void) {

    DL_I2C_setClockConfig(OLED_INST,
        (DL_I2C_ClockConfig *) &gOLEDClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(OLED_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(OLED_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(OLED_INST);
    /* Set frequency to 100000 Hz*/
    DL_I2C_setTimerPeriod(OLED_INST, 31);
    DL_I2C_setControllerTXFIFOThreshold(OLED_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(OLED_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(OLED_INST);


    /* Enable module */
    DL_I2C_enableController(OLED_INST);


}
static const DL_I2C_ClockConfig gIMU20948ClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_IMU20948_init(void) {

    DL_I2C_setClockConfig(IMU20948_INST,
        (DL_I2C_ClockConfig *) &gIMU20948ClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(IMU20948_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(IMU20948_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(IMU20948_INST);
    /* Set frequency to 100000 Hz*/
    DL_I2C_setTimerPeriod(IMU20948_INST, 31);
    DL_I2C_setControllerTXFIFOThreshold(IMU20948_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(IMU20948_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(IMU20948_INST);


    /* Enable module */
    DL_I2C_enableController(IMU20948_INST);


}


static const DL_UART_Main_ClockConfig gIR8ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_8
};

static const DL_UART_Main_Config gIR8Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_IR8_init(void)
{
    DL_UART_Main_setClockConfig(IR8_INST, (DL_UART_Main_ClockConfig *) &gIR8ClockConfig);

    DL_UART_Main_init(IR8_INST, (DL_UART_Main_Config *) &gIR8Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115107.91
     */
    DL_UART_Main_setOversampling(IR8_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(IR8_INST, IR8_IBRD_4_MHZ_115200_BAUD, IR8_FBRD_4_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(IR8_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);


    DL_UART_Main_enable(IR8_INST);
}

static const DL_UART_Main_ClockConfig gBTClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_8
};

static const DL_UART_Main_Config gBTConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_BT_init(void)
{
    DL_UART_Main_setClockConfig(BT_INST, (DL_UART_Main_ClockConfig *) &gBTClockConfig);

    DL_UART_Main_init(BT_INST, (DL_UART_Main_Config *) &gBTConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 9600
     *  Actual baud rate: 9598.08
     */
    DL_UART_Main_setOversampling(BT_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(BT_INST, BT_IBRD_4_MHZ_9600_BAUD, BT_FBRD_4_MHZ_9600_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(BT_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);


    DL_UART_Main_enable(BT_INST);
}

/* BAT_ADC Initialization */
static const DL_ADC12_ClockConfig gBAT_ADCClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_ULPCLK,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_8,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_BAT_ADC_init(void)
{
    DL_ADC12_setClockConfig(BAT_ADC_INST, (DL_ADC12_ClockConfig *) &gBAT_ADCClockConfig);
    DL_ADC12_configConversionMem(BAT_ADC_INST, BAT_ADC_ADCMEM_0,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setPowerDownMode(BAT_ADC_INST,DL_ADC12_POWER_DOWN_MODE_MANUAL);
    DL_ADC12_setSampleTime0(BAT_ADC_INST,500);
    DL_ADC12_enableConversions(BAT_ADC_INST);
}

SYSCONFIG_WEAK void SYSCFG_DL_WWDT0_init(void)
{
    /*
     * Initialize WWDT0 in Watchdog mode with following settings
     *   Watchdog Source Clock = (LFCLK Freq) / (WWDT Clock Divider)
     *                         = 32768Hz / 4 = 8.19 kHz
     *   Watchdog Period       = (WWDT Clock Divider) ∗ (WWDT Period Count) / 32768Hz
     *                         = 4 * 2^12 / 32768Hz = 500.00 ms
     *   Window0 Closed Period = (WWDT Period) * (Window0 Closed Percent)
     *                         = 500.00 ms * 0% = 0.00 s
     *   Window1 Closed Period = (WWDT Period) * (Window1 Closed Percent)
     *                         = 500.00 ms * 0% = 0.00 s
     */
    DL_WWDT_initWatchdogMode(WWDT0_INST, DL_WWDT_CLOCK_DIVIDE_4,
        DL_WWDT_TIMER_PERIOD_12_BITS, DL_WWDT_RUN_IN_SLEEP,
        DL_WWDT_WINDOW_PERIOD_0, DL_WWDT_WINDOW_PERIOD_0);

    /* Set Window0 as active window */
    DL_WWDT_setActiveWindow(WWDT0_INST, DL_WWDT_WINDOW0);

}
