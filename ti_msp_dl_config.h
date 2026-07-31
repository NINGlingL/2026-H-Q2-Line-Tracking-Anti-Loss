/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA1
#define PWM_0_INST_IRQHandler                                   TIMA1_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                              4000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOB
#define GPIO_PWM_0_C0_PIN                                          DL_GPIO_PIN_0
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM12)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM12_PF_TIMA1_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOB
#define GPIO_PWM_0_C1_PIN                                          DL_GPIO_PIN_1
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM13)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM13_PF_TIMA1_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX




/* Defines for QEI_R */
#define QEI_R_INST                                                         TIMG8
#define QEI_R_INST_IRQHandler                                   TIMG8_IRQHandler
#define QEI_R_INST_INT_IRQN                                     (TIMG8_INT_IRQn)
/* Pin configuration defines for QEI_R PHA Pin */
#define GPIO_QEI_R_PHA_PORT                                                GPIOB
#define GPIO_QEI_R_PHA_PIN                                        DL_GPIO_PIN_10
#define GPIO_QEI_R_PHA_IOMUX                                     (IOMUX_PINCM27)
#define GPIO_QEI_R_PHA_IOMUX_FUNC                    IOMUX_PINCM27_PF_TIMG8_CCP0
/* Pin configuration defines for QEI_R PHB Pin */
#define GPIO_QEI_R_PHB_PORT                                                GPIOB
#define GPIO_QEI_R_PHB_PIN                                        DL_GPIO_PIN_11
#define GPIO_QEI_R_PHB_IOMUX                                     (IOMUX_PINCM28)
#define GPIO_QEI_R_PHB_IOMUX_FUNC                    IOMUX_PINCM28_PF_TIMG8_CCP1



/* Defines for OLED */
#define OLED_INST                                                           I2C0
#define OLED_INST_IRQHandler                                     I2C0_IRQHandler
#define OLED_INST_INT_IRQN                                         I2C0_INT_IRQn
#define OLED_BUS_SPEED_HZ                                                 400000
#define GPIO_OLED_SDA_PORT                                                 GPIOA
#define GPIO_OLED_SDA_PIN                                          DL_GPIO_PIN_0
#define GPIO_OLED_IOMUX_SDA                                       (IOMUX_PINCM1)
#define GPIO_OLED_IOMUX_SDA_FUNC                        IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_OLED_SCL_PORT                                                 GPIOA
#define GPIO_OLED_SCL_PIN                                          DL_GPIO_PIN_1
#define GPIO_OLED_IOMUX_SCL                                       (IOMUX_PINCM2)
#define GPIO_OLED_IOMUX_SCL_FUNC                        IOMUX_PINCM2_PF_I2C0_SCL

/* Defines for IMU20948 */
#define IMU20948_INST                                                       I2C1
#define IMU20948_INST_IRQHandler                                 I2C1_IRQHandler
#define IMU20948_INST_INT_IRQN                                     I2C1_INT_IRQn
#define IMU20948_BUS_SPEED_HZ                                             400000
#define GPIO_IMU20948_SDA_PORT                                             GPIOB
#define GPIO_IMU20948_SDA_PIN                                      DL_GPIO_PIN_3
#define GPIO_IMU20948_IOMUX_SDA                                  (IOMUX_PINCM16)
#define GPIO_IMU20948_IOMUX_SDA_FUNC                   IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_IMU20948_SCL_PORT                                             GPIOB
#define GPIO_IMU20948_SCL_PIN                                      DL_GPIO_PIN_2
#define GPIO_IMU20948_IOMUX_SCL                                  (IOMUX_PINCM15)
#define GPIO_IMU20948_IOMUX_SCL_FUNC                   IOMUX_PINCM15_PF_I2C1_SCL


/* Defines for IR8 */
#define IR8_INST                                                           UART3
#define IR8_INST_FREQUENCY                                               4000000
#define IR8_INST_IRQHandler                                     UART3_IRQHandler
#define IR8_INST_INT_IRQN                                         UART3_INT_IRQn
#define GPIO_IR8_RX_PORT                                                   GPIOB
#define GPIO_IR8_TX_PORT                                                   GPIOB
#define GPIO_IR8_RX_PIN                                           DL_GPIO_PIN_13
#define GPIO_IR8_TX_PIN                                           DL_GPIO_PIN_12
#define GPIO_IR8_IOMUX_RX                                        (IOMUX_PINCM30)
#define GPIO_IR8_IOMUX_TX                                        (IOMUX_PINCM29)
#define GPIO_IR8_IOMUX_RX_FUNC                         IOMUX_PINCM30_PF_UART3_RX
#define GPIO_IR8_IOMUX_TX_FUNC                         IOMUX_PINCM29_PF_UART3_TX
#define IR8_BAUD_RATE                                                   (115200)
#define IR8_IBRD_4_MHZ_115200_BAUD                                           (2)
#define IR8_FBRD_4_MHZ_115200_BAUD                                          (11)
/* Defines for BT */
#define BT_INST                                                            UART2
#define BT_INST_FREQUENCY                                                4000000
#define BT_INST_IRQHandler                                      UART2_IRQHandler
#define BT_INST_INT_IRQN                                          UART2_INT_IRQn
#define GPIO_BT_RX_PORT                                                    GPIOB
#define GPIO_BT_TX_PORT                                                    GPIOB
#define GPIO_BT_RX_PIN                                            DL_GPIO_PIN_16
#define GPIO_BT_TX_PIN                                            DL_GPIO_PIN_15
#define GPIO_BT_IOMUX_RX                                         (IOMUX_PINCM33)
#define GPIO_BT_IOMUX_TX                                         (IOMUX_PINCM32)
#define GPIO_BT_IOMUX_RX_FUNC                          IOMUX_PINCM33_PF_UART2_RX
#define GPIO_BT_IOMUX_TX_FUNC                          IOMUX_PINCM32_PF_UART2_TX
#define BT_BAUD_RATE                                                      (9600)
#define BT_IBRD_4_MHZ_9600_BAUD                                             (26)
#define BT_FBRD_4_MHZ_9600_BAUD                                              (3)





/* Defines for BAT_ADC */
#define BAT_ADC_INST                                                        ADC0
#define BAT_ADC_INST_IRQHandler                                  ADC0_IRQHandler
#define BAT_ADC_INST_INT_IRQN                                    (ADC0_INT_IRQn)
#define BAT_ADC_ADCMEM_0                                      DL_ADC12_MEM_IDX_0
#define BAT_ADC_ADCMEM_0_REF                     DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define BAT_ADC_ADCMEM_0_REF_VOLTAGE_V                                       3.3
#define GPIO_BAT_ADC_C0_PORT                                               GPIOA
#define GPIO_BAT_ADC_C0_PIN                                       DL_GPIO_PIN_27



/* Port definition for Pin Group STBY */
#define STBY_PORT                                                        (GPIOB)

/* Defines for STBY3: GPIOB.14 with pinCMx 31 on package pin 2 */
#define STBY_STBY3_PIN                                          (DL_GPIO_PIN_14)
#define STBY_STBY3_IOMUX                                         (IOMUX_PINCM31)
/* Port definition for Pin Group KEY */
#define KEY_PORT                                                         (GPIOB)

/* Defines for START: GPIOB.21 with pinCMx 49 on package pin 20 */
#define KEY_START_PIN                                           (DL_GPIO_PIN_21)
#define KEY_START_IOMUX                                          (IOMUX_PINCM49)
/* Port definition for Pin Group MOTO */
#define MOTO_PORT                                                        (GPIOA)

/* Defines for AIN1: GPIOA.14 with pinCMx 36 on package pin 7 */
#define MOTO_AIN1_PIN                                           (DL_GPIO_PIN_14)
#define MOTO_AIN1_IOMUX                                          (IOMUX_PINCM36)
/* Defines for AIN2: GPIOA.15 with pinCMx 37 on package pin 8 */
#define MOTO_AIN2_PIN                                           (DL_GPIO_PIN_15)
#define MOTO_AIN2_IOMUX                                          (IOMUX_PINCM37)
/* Defines for BIN1: GPIOA.12 with pinCMx 34 on package pin 5 */
#define MOTO_BIN1_PIN                                           (DL_GPIO_PIN_12)
#define MOTO_BIN1_IOMUX                                          (IOMUX_PINCM34)
/* Defines for BIN2: GPIOA.13 with pinCMx 35 on package pin 6 */
#define MOTO_BIN2_PIN                                           (DL_GPIO_PIN_13)
#define MOTO_BIN2_IOMUX                                          (IOMUX_PINCM35)


/* Defines for WWDT */
#define WWDT0_INST                                                       (WWDT0)
#define WWDT0_INT_IRQN                                          (WWDT0_INT_IRQn)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_QEI_R_init(void);
void SYSCFG_DL_OLED_init(void);
void SYSCFG_DL_IMU20948_init(void);
void SYSCFG_DL_IR8_init(void);
void SYSCFG_DL_BT_init(void);
void SYSCFG_DL_BAT_ADC_init(void);

void SYSCFG_DL_WWDT0_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
