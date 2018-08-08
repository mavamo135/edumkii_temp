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

/** ============================================================================
 *  @file       BOOSTXL_EDUMKII.h
 *
 *  @brief      BOOSTXL_EDUMKIIR Board Specific APIs
 *
 *  The BOOSTXL_EDUMKII header file should be included in an application as
 *  follows:
 *  @code
 *  #include <BOOSTXL_EDUMKII.h>
 *  @endcode
 *
 * Launchpad - Boosterpack Pin Configuration.
 *
 * NOTE: Temperature Sensor TMP006
 * NOTE: Ambient Light Sensor OPT3001
 *                           ____________________________________
 *                          |  J1                            J2  |
 *                          | ----                          ---- |
 *                  <- 3.3V | |  | -> A15                   |  | | GND  ->
 * JOYSTICK, HOR(X) <- P6.0 | |  | -> UCA2RXD      TA0.2 <- |  | | P2.5 -> Servo PWM
 *     UART, BP->LP <- P3.2 | |  | -> UCA2TXD               |  | | P3.0 -> NC
 *     UART, BP<-LP <- P3.3 | |  |                          |  | | P5.7 -> LCD, !RST
 * JOYSTICK, SELECT <- P4.1 | |  |                          |  | | RST  -> NC
 *       MICROPHONE <- P4.3 | |  | -> A10       UCB0SIMO <- |  | | P1.6 -> LCD, SDA/SPI MOSI
 *     LCD, SPI CLK <- P1.5 | |  | -> UCB0CLK   UCB0SOMI <- |  | | P1.7 -> NC
 * AMBIENT LIGHT INT<- P4.6 | |  |                          |  | | P5.0 -> LCD, SPI CS
 * AMBIENT LIGHT|I2C<- P6.5 | |  | -> UCB1SCL               |  | | P5.2 -> NC
 * TEMP SENSOR  |   <- P6.4 | |  | -> UCB1SDA               |  | | P3.6 -> TEMP SENSOR INT
 *                          | ----                          ---- |
 *                          |  J3         MSP432P401R        J4  |
 *                          | ----                          ---- |
 *                  <- +5V  | |  |                 TA0.4 <- |  | | P2.7 -> BUZZER OUT
 *                  <- GND  | |  |                 TA0.3 <- |  | | P2.6 -> RGB_LED, RED
 * ACCELEROMETER, X <- P6.1 | |  | -> A14          TA0.1 <- |  | | P2.4 -> RGB_LED, GRN
 * ACCELEROMETER, Y <- P4.0 | |  | -> A13          TA2.1 <- |  | | P5.6 -> RGB_LED, BLU
 * ACCELEROMETER, Z <- P4.2 | |  | -> A11                   |  | | P6.6 -> NC
 * JOYSTICK, VER(Y) <- P4.4 | |  | -> A9                    |  | | P6.7 -> NC
 *               NC <- P4.5 | |  |                          |  | | P2.3 -> GATOR HOLE, I/O
 *               NC <- P4.7 | |  |                          |  | | P5.1 -> BUTTON 1, I/O
 *               NC <- P5.4 | |  |                          |  | | P3.5 -> BUTTON 2, I/O
 *               NC <- P5.5 | |  |                          |  | | P3.7 -> LCD, RS PIN
 *                          | ----                          ---- |
 *                          |                                    |
 *                          |    |-> P1.1 -> On board BUTTON 1   |
 *                          |    |-> P1.4 -> On board BUTTON 2   |
 *                          |    |-> P1.0 -> On board LED        |
 *                          |    |        ________________       |
 *                          |    |----->|TA1.1->P2.0->RED|       |
 *                          |    |----->|TA1.2->P2.1->GRN|       |
 *                          |    |----->|TA1.3->P2.2->BLU|       |
 *                          |           |________________|       |
 *                          |            On board RGB_LED        |
 *                          |____________________________________|
 *
 *  ============================================================================
 */

#ifndef __BOOSTXL_EDUMKII_H
#define __BOOSTXL_EDUMKII_H

#ifdef __cplusplus
extern "C" {
#endif

/* LEDs on BOOSTXL_EDUMKII are active high. */
#define BOOSTXL_EDUMKII_GPIO_LED_OFF (0)
#define BOOSTXL_EDUMKII_GPIO_LED_ON  (1)

/*!
 *  @def    BOOSTXL_EDUMKII_ADCName
 *  @brief  Enum of ADC channels on the BOOSTXL_EDUMKII dev board
 */
typedef enum BOOSTXL_EDUMKII_ADCName {
    /* P4.0 -> ADC13 -> ACCELEROMETER,Y */
    BOOSTXL_EDUMKII_ADC_ACC_Y = 0,
    /* P4.2 -> ADC11 -> ACCELEROMETER,Z */
    BOOSTXL_EDUMKII_ADC_ACC_Z = 1,
    /* P4.3 -> ADC10 -> MICROPHONE */
    BOOSTXL_EDUMKII_ADC_MIC = 2,
    /* P4.4 -> ADC9 -> JOYSTICK,VER(Y) */
    BOOSTXL_EDUMKII_ADC_JOY_Y = 3,
    /* P6.0 -> ADC15 -> JOYSTICK,HOR(X) */
    BOOSTXL_EDUMKII_ADC_JOY_X = 4,
    /* P6.1 -> ADC14 -> ACCELEROMETER,X */
    BOOSTXL_EDUMKII_ADC_ACC_X = 5,

    BOOSTXL_EDUMKII_ADCCOUNT
} BOOSTXL_EDUMKII_ADCName;

/*!
 *  @def    BOOSTXL_EDUMKII_GPIOName
 *  @brief  Enum of GPIO names on the BOOSTXL_EDUMKII dev board
 */
typedef enum BOOSTXL_EDUMKII_GPIOName {
    /* P1.1 -> BUTTON1,MSP432 LAUNCHPAD */
    MSP_EXP432P401R_GPIO_BUTTON1 = 0,
    /* P1.4 -> BUTTON2,MSP432 LAUNCHPAD */
    MSP_EXP432P401R_GPIO_BUTTON2 = 1,
    /* P5.1 -> BUTTON1,EDUMKII BOOSTERPACK */
    BOOSTXL_EDUMKII_GPIO_BUTTON1 = 2,
    /* P3.5 -> BUTTON2,EDUMKII BOOSTERPACK */
    BOOSTXL_EDUMKII_GPIO_BUTTON2 = 3,
    /* P4.1 -> JOYSTICK,SELECT */
    BOOSTXL_EDUMKII_GPIO_JOY_SEL = 4,
    /* P4.6 -> OPT3001 INT */
    BOOSTXL_EDUMKII_GPIO_OPT3001_INT = 5,
    /* P3.6 -> TMP006 INT */
    BOOSTXL_EDUMKII_GPIO_TMP006_INT = 6,
    /* P1.0 -> LED,MSP432 LAUNCHPAD */
    MSP_EXP432P401R_GPIO_LED = 7,
    /* P5.0 -> LCD,SPI CS */
    BOOSTXL_EDUMKII_GPIO_LCD_CS = 8,
    /* P3.7 -> LCD,RS PIN */
    BOOSTXL_EDUMKII_GPIO_LCD_RS = 9,
    /* P5.7 -> LCD,!RST */
    BOOSTXL_EDUMKII_GPIO_LCD_RST = 10,

    BOOSTXL_EDUMKII_GPIOCOUNT
} BOOSTXL_EDUMKII_GPIOName;

/*!
 *  @def    BOOSTXL_EDUMKII_I2CName
 *  @brief  Enum of I2C names on the BOOSTXL_EDUMKII dev board
 */
typedef enum BOOSTXL_EDUMKII_I2CName {
    BOOSTXL_EDUMKII_I2C = 0,

    BOOSTXL_EDUMKII_I2CCOUNT
} BOOSTXL_EDUMKII_I2CName;

/*!
 *  @def    BOOSTXL_EDUMKII_PWMName
 *  @brief  Enum of PWM names on the BOOSTXL_EDUMKII dev board
 */
typedef enum BOOSTXL_EDUMKII_PWMName {
    /* P2.4 -> TA0.1 -> RGB_LED_GRN,EDUMKII BOOSTERPACK */
    BOOSTXL_EDUMKII_PWM_RGB_G = 0,
    /* P2.6 -> TA0.3 -> RGB_LED_RED,EDUMKII BOOSTERPACK */
    BOOSTXL_EDUMKII_PWM_RGB_R = 1,
    /* P2.7 -> TA0.4 -> BUZZER,EDUMKII BOOSTERPACK */
    BOOSTXL_EDUMKII_PWM_BUZZ = 2,
    /* P5.6 -> TA2.1 -> RGB_LED_BLU,EDUMKII BOOSTERPACK */
    BOOSTXL_EDUMKII_PWM_RGB_B = 3,
    /* P2.0 -> TA1.1 -> RGB_LED_RED,MSP432 LAUNCHPAD */
    MSP_EXP432P401R_PWM_RGB_R = 4,
    /* P2.1 -> TA1.2 -> RGB_LED_GRN,MSP432 LAUNCHPAD */
    MSP_EXP432P401R_PWM_RGB_G = 5,
    /* P2.2 -> TA1.3 -> RGB_LED_BLU,MSP432 LAUNCHPAD */
    MSP_EXP432P401R_PWM_RGB_B = 6,

    BOOSTXL_EDUMKII_PWMCOUNT
} BOOSTXL_EDUMKII_PWMName;

/*!
 *  @def    BOOSTXL_EDUMKII_SPIName
 *  @brief  Enum of SPI names on the BOOSTXL_EDUMKII dev board
 */
typedef enum BOOSTXL_EDUMKII_SPIName {
    BOOSTXL_EDUMKII_SPI_LCD = 0,

    BOOSTXL_EDUMKII_SPICOUNT
} BOOSTXL_EDUMKII_SPIName;

/*!
 *  @def    BOOSTXL_EDUMKII_TimerName
 *  @brief  Enum of Timer names on the BOOSTXL_EDUMKII dev board
 */
typedef enum BOOSTXL_EDUMKII_TimerName {
    BOOSTXL_EDUMKII_TIMER_T32_0 = 0,
    BOOSTXL_EDUMKII_TIMER_T32_1 = 1,
    BOOSTXL_EDUMKII_TIMER_TA_0 = 2,
    BOOSTXL_EDUMKII_TIMER_TA_2 = 3,
    BOOSTXL_EDUMKII_TIMER_TA_1 = 4,

    BOOSTXL_EDUMKII_TIMERCOUNT
} BOOSTXL_EDUMKII_TimerName;

/*!
 *  @def    BOOSTXL_EDUMKII_UARTName
 *  @brief  Enum of UART names on the BOOSTXL_EDUMKII dev board
 */
typedef enum BOOSTXL_EDUMKII_UARTName {
    BOOSTXL_EDUMKII_UART0 = 0,

    BOOSTXL_EDUMKII_UARTCOUNT
} BOOSTXL_EDUMKII_UARTName;

/*!
 *  @def    BOOSTXL_EDUMKII_WatchdogName
 *  @brief  Enum of Watchdog names on the BOOSTXL_EDUMKII dev board
 */
typedef enum BOOSTXL_EDUMKII_WatchdogName {
    BOOSTXL_EDUMKII_WATCHDOG = 0,

    BOOSTXL_EDUMKII_WATCHDOGCOUNT
} BOOSTXL_EDUMKII_WatchdogName;

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 */
extern void BOOSTXL_EDUMKII_initGeneral(void);


#ifdef __cplusplus
}
#endif

#endif /* __BOOSTXL_EDUMKII_H */
