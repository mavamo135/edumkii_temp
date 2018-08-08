/*
 * Copyright (c) 2015-2018, Texas Instruments Incorporated
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

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "BOOSTXL_EDUMKII.h"

#define Board_initGeneral           BOOSTXL_EDUMKII_initGeneral

/* ADCs */
#define Board_ADC_ACC_X             BOOSTXL_EDUMKII_ADC_ACC_X
#define Board_ADC_ACC_Y             BOOSTXL_EDUMKII_ADC_ACC_Y
#define Board_ADC_ACC_Z             BOOSTXL_EDUMKII_ADC_ACC_Z
#define Board_ADC_MIC               BOOSTXL_EDUMKII_ADC_MIC
#define Board_ADC_JOY_X             BOOSTXL_EDUMKII_ADC_JOY_X
#define Board_ADC_JOY_Y             BOOSTXL_EDUMKII_ADC_JOY_Y

/* GPIO */
#define Board_GPIO_LED_ON           BOOSTXL_EDUMKII_GPIO_LED_ON
#define Board_GPIO_LED_OFF          BOOSTXL_EDUMKII_GPIO_LED_OFF

/* GPIO Inputs */
#define Board_GPIO_BUTTON1_LP       MSP_EXP432P401R_GPIO_BUTTON1
#define Board_GPIO_BUTTON2_LP       MSP_EXP432P401R_GPIO_BUTTON2
#define Board_GPIO_BUTTON1_BP       BOOSTXL_EDUMKII_GPIO_BUTTON1
#define Board_GPIO_BUTTON2_BP       BOOSTXL_EDUMKII_GPIO_BUTTON2
#define Board_GPIO_JOY_SEL          BOOSTXL_EDUMKII_GPIO_JOY_SEL
#define Board_GPIO_OPT3001_INT      BOOSTXL_EDUMKII_GPIO_OPT3001_INT
#define Board_GPIO_TMP006_INT       BOOSTXL_EDUMKII_GPIO_TMP006_INT

/* GPIO Outputs */
#define Board_GPIO_LED_LP           MSP_EXP432P401R_GPIO_LED

 /* I2C */
#define Board_I2C                   BOOSTXL_EDUMKII_I2C

/* PWM */
#define Board_PWM_RGB_R_BP          BOOSTXL_EDUMKII_PWM_RGB_R
#define Board_PWM_RGB_G_BP          BOOSTXL_EDUMKII_PWM_RGB_G
#define Board_PWM_RGB_B_BP          BOOSTXL_EDUMKII_PWM_RGB_B
#define Board_PWM_RGB_R_LP          MSP_EXP432P401R_PWM_RGB_R
#define Board_PWM_RGB_G_LP          MSP_EXP432P401R_PWM_RGB_G
#define Board_PWM_RGB_B_LP          MSP_EXP432P401R_PWM_RGB_B
#define Board_PWM_BUZZ              BOOSTXL_EDUMKII_PWM_BUZZ

/* LCD SPI */
#define Board_SPI_LCD               BOOSTXL_EDUMKII_SPI_LCD
#define Board_GPIO_LCD_CS           BOOSTXL_EDUMKII_GPIO_LCD_CS
#define Board_GPIO_LCD_RS           BOOSTXL_EDUMKII_GPIO_LCD_RS
#define Board_GPIO_LCD_RST          BOOSTXL_EDUMKII_GPIO_LCD_RST

/* Timers */
#define Board_TIMER0                BOOSTXL_EDUMKII_TIMER_T32_0
#define Board_TIMER1                BOOSTXL_EDUMKII_TIMER_T32_1
#define Board_TIMER2                BOOSTXL_EDUMKII_TIMER_TA_0
#define Board_TIMER3                BOOSTXL_EDUMKII_TIMER_TA_1
#define Board_TIMER4                BOOSTXL_EDUMKII_TIMER_TA_2

/* UART */
#define Board_UART0                 BOOSTXL_EDUMKII_UART0

/* Watchdog */
#define Board_WATCHDOG0             BOOSTXL_EDUMKII_WATCHDOG

/* Board specific I2C addresses */
#define Board_TMP_ADDR              (0x40)    /* Temperature Sensor */
#define TMP006_OBJ_TEMP             (0x0001)  /* Object Temp Result Register */
#define Board_OPT_ADDR              (0x40)    /* Ambient Ligth Sensor */
#define OPT3001_RES_REG             (0x0000)  /* Result Register */

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
