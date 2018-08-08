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

/*
 *  ======== BOOSTXL_EDUMKII.c ========
 *  This file is responsible for setting up the board specific items for the
 *  MSP_EXP432P401R + BOOSTXL_EDUMKII board.
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
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef DeviceFamily_MSP432P401x
#define DeviceFamily_MSP432P401x
#endif

#include <ti/devices/DeviceFamily.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432.h>

#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/rom.h>
#include <ti/devices/msp432p4xx/driverlib/rom_map.h>
#include <ti/devices/msp432p4xx/driverlib/adc14.h>
#include <ti/devices/msp432p4xx/driverlib/dma.h>
#include <ti/devices/msp432p4xx/driverlib/gpio.h>
#include <ti/devices/msp432p4xx/driverlib/i2c.h>
#include <ti/devices/msp432p4xx/driverlib/interrupt.h>
#include <ti/devices/msp432p4xx/driverlib/pmap.h>
#include <ti/devices/msp432p4xx/driverlib/ref_a.h>
#include <ti/devices/msp432p4xx/driverlib/spi.h>
#include <ti/devices/msp432p4xx/driverlib/timer_a.h>
#include <ti/devices/msp432p4xx/driverlib/timer32.h>
#include <ti/devices/msp432p4xx/driverlib/uart.h>
#include <ti/devices/msp432p4xx/driverlib/wdt_a.h>

#include "BOOSTXL_EDUMKII.h"

/*
 *  =============================== ADC ===============================
 */
#include <ti/drivers/ADC.h>
#include <ti/drivers/adc/ADCMSP432.h>

/* ADC objects */
ADCMSP432_Object adcMSP432Objects[BOOSTXL_EDUMKII_ADCCOUNT];

/* ADC configuration structure */
const ADCMSP432_HWAttrsV1 adcMSP432HWAttrs[BOOSTXL_EDUMKII_ADCCOUNT] = {
    {
        /* P4.0 -> ADC13 -> ACCELEROMETER,Y */
        .adcPin = ADCMSP432_P4_0_A13,
        .refVoltage = REF_A_VREF2_5V,
        .resolution = ADC_12BIT,
    },
    {
        /* P4.2 -> ADC11 -> ACCELEROMETER,Z */
        .adcPin = ADCMSP432_P4_2_A11,
        .refVoltage = REF_A_VREF2_5V,
        .resolution = ADC_12BIT,
    },
    {
        /* P4.3 -> ADC10 -> MICROPHONE */
        .adcPin = ADCMSP432_P4_3_A10,
        .refVoltage = REF_A_VREF2_5V,
        .resolution = ADC_12BIT,
    },
    {
        /* P4.4 -> ADC9 -> JOYSTICK,VER(Y) */
        .adcPin = ADCMSP432_P4_4_A9,
        .refVoltage = REF_A_VREF2_5V,
        .resolution = ADC_12BIT,
    },
    {
        /* P6.0 -> ADC15 -> JOYSTICK,HOR(X) */
        .adcPin = ADCMSP432_P6_0_A15,
        .refVoltage = REF_A_VREF2_5V,
        .resolution = ADC_12BIT,
    },
    {
        /* P6.1 -> ADC14 -> ACCELEROMETER,X */
        .adcPin = ADCMSP432_P6_1_A14,
        .refVoltage = REF_A_VREF2_5V,
        .resolution = ADC_12BIT,
    },
};

const ADC_Config ADC_config[BOOSTXL_EDUMKII_ADCCOUNT] = {
    {
        .fxnTablePtr = &ADCMSP432_fxnTable,
        .object = &adcMSP432Objects[BOOSTXL_EDUMKII_ADC_ACC_Y],
        .hwAttrs = &adcMSP432HWAttrs[BOOSTXL_EDUMKII_ADC_ACC_Y]
    },
    {
        .fxnTablePtr = &ADCMSP432_fxnTable,
        .object = &adcMSP432Objects[BOOSTXL_EDUMKII_ADC_ACC_Z],
        .hwAttrs = &adcMSP432HWAttrs[BOOSTXL_EDUMKII_ADC_ACC_Z]
    },
    {
        .fxnTablePtr = &ADCMSP432_fxnTable,
        .object = &adcMSP432Objects[BOOSTXL_EDUMKII_ADC_MIC],
        .hwAttrs = &adcMSP432HWAttrs[BOOSTXL_EDUMKII_ADC_MIC]
    },
    {
        .fxnTablePtr = &ADCMSP432_fxnTable,
        .object = &adcMSP432Objects[BOOSTXL_EDUMKII_ADC_JOY_Y],
        .hwAttrs = &adcMSP432HWAttrs[BOOSTXL_EDUMKII_ADC_JOY_Y]
    },
    {
        .fxnTablePtr = &ADCMSP432_fxnTable,
        .object = &adcMSP432Objects[BOOSTXL_EDUMKII_ADC_JOY_X],
        .hwAttrs = &adcMSP432HWAttrs[BOOSTXL_EDUMKII_ADC_JOY_X]
    },
    {
        .fxnTablePtr = &ADCMSP432_fxnTable,
        .object = &adcMSP432Objects[BOOSTXL_EDUMKII_ADC_ACC_X],
        .hwAttrs = &adcMSP432HWAttrs[BOOSTXL_EDUMKII_ADC_ACC_X]
    },
};

const uint_least8_t ADC_count = BOOSTXL_EDUMKII_ADCCOUNT;

/*
 *  =============================== DMA ===============================
 */
#include <ti/drivers/dma/UDMAMSP432.h>

#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(dmaControlTable, 256)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=256
#elif defined(__GNUC__)
__attribute__ ((aligned (256)))
#endif
static DMA_ControlTable dmaControlTable[8];

/*
 *  ======== dmaErrorHwi ========
 *  This is the handler for the uDMA error interrupt.
 */
static void dmaErrorHwi(uintptr_t arg)
{
    int status = MAP_DMA_getErrorStatus();
    MAP_DMA_clearErrorStatus();

    /* Suppress unused variable warning */
    (void)status;

    while (1);
}

UDMAMSP432_Object udmaMSP432Object;

const UDMAMSP432_HWAttrs udmaMSP432HWAttrs = {
    .controlBaseAddr = (void *)dmaControlTable,
    .dmaErrorFxn = (UDMAMSP432_ErrorFxn)dmaErrorHwi,
    .intNum = INT_DMA_ERR,
    .intPriority = (~0)
};

const UDMAMSP432_Config UDMAMSP432_config = {
    .object = &udmaMSP432Object,
    .hwAttrs = &udmaMSP432HWAttrs
};

/*
 *  ============================= Display =============================
 */
#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>
#define MAXPRINTLEN 1024

DisplayUart_Object displayUartObject;

static char displayBuf[MAXPRINTLEN];

const DisplayUart_HWAttrs displayUartHWAttrs = {
    .uartIdx = BOOSTXL_EDUMKII_UART0,
    .baudRate = 115200,
    .mutexTimeout = (unsigned int)(-1),
    .strBuf = displayBuf,
    .strBufLen = MAXPRINTLEN
};

const Display_Config Display_config[] = {
    {
#  if defined(BOARD_DISPLAY_UART_USE_ANSI)
        .fxnTablePtr = &DisplayUartAnsi_fxnTable,
#  else /* Default to minimal UART with no cursor placement */
        .fxnTablePtr = &DisplayUartMin_fxnTable,
#  endif
        .object = &displayUartObject,
        .hwAttrs = &displayUartHWAttrs
    },
};

const uint_least8_t Display_count = sizeof(Display_config) / sizeof(Display_Config);

/*
 *  ======== BOOSTXL_EDUMKII_initGeneral ========
 */
void BOOSTXL_EDUMKII_initGeneral(void)
{
    Power_init();
}

/*
 *  =============================== GPIO ===============================
 */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOMSP432.h>

/*
 * Array of Pin configurations
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in BOOSTXL_EDUMKII.h
 * NOTE: Pins not used for interrupts should be placed at the end of the
 *       array.  Callback entries can be omitted from callbacks array to
 *       reduce memory usage.
 */
GPIO_PinConfig gpioPinConfigs[] = {
    /* Input pins */
    /*
     * NOTE: Specifying FALLING edge triggering for these buttons to ensure the
     * interrupts are signaled immediately.  See the description of the
     * PowerMSP432 driver's automatic pin parking feature for this rationale.
     */
    /* P1.1 -> BUTTON1,MSP432 LAUNCHPAD */
    GPIOMSP432_P1_1 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING,
    /* P1.4 -> BUTTON2,MSP432 LAUNCHPAD */
    GPIOMSP432_P1_4 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING,
    /* P5.1 -> BUTTON1,EDUMKII BOOSTERPACK */
    GPIOMSP432_P5_1 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING,
    /* P3.5 -> BUTTON2,EDUMKII BOOSTERPACK */
    GPIOMSP432_P3_5 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING,
    /* P4.1 -> JOYSTICK,SELECT */
    GPIOMSP432_P4_1 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING,
    /* P4.6 -> OPT3001 INT */
    GPIOMSP432_P4_6 | GPIO_CFG_IN_NOPULL | GPIO_CFG_IN_INT_FALLING,
    /* P3.6 -> TMP006 INT */
    GPIOMSP432_P3_6 | GPIO_CFG_IN_NOPULL | GPIO_CFG_IN_INT_FALLING,

    /* Output pins */
    /* P1.0 -> LED,MSP432 LAUNCHPAD */
    GPIOMSP432_P1_0 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_LOW | GPIO_CFG_OUT_LOW,
    /* P5.0 -> LCD,SPI CS */
    GPIOMSP432_P5_0 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_HIGH,
    /* P3.7 -> LCD,RS PIN */
    GPIOMSP432_P3_7 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_LOW | GPIO_CFG_OUT_LOW,
    /* P5.7 -> LCD,!RST */
    GPIOMSP432_P5_7 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_LOW | GPIO_CFG_OUT_HIGH,
};

/*
 * Array of callback function pointers
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in BOOSTXL_EDUMKII.h
 * NOTE: Pins not used for interrupts can be omitted from callbacks array to
 *       reduce memory usage (if placed at end of gpioPinConfigs array).
 */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    /* BUTTON1,MSP432 LAUNCHPAD */
    NULL,
    /* BUTTON2,MSP432 LAUNCHPAD */
    NULL,
    /* BUTTON1,EDUMKII BOOSTERPACK */
    NULL,
    /* BUTTON2,EDUMKII BOOSTERPACK */
    NULL,
    /* JOYSTICK,SELECT */
    NULL,
    /* OPT3001 INT */
    NULL,
    /* TMP006 INT */
    NULL
};

const GPIOMSP432_Config GPIOMSP432_config = {
    .pinConfigs = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .numberOfPinConfigs = sizeof(gpioPinConfigs)/sizeof(GPIO_PinConfig),
    .numberOfCallbacks = sizeof(gpioCallbackFunctions)/sizeof(GPIO_CallbackFxn),
    .intPriority = (~0)
};
/*
 *  =============================== I2C ===============================
 */
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CMSP432.h>

I2CMSP432_Object i2cMSP432Objects[BOOSTXL_EDUMKII_I2CCOUNT];

const I2CMSP432_HWAttrsV1 i2cMSP432HWAttrs[BOOSTXL_EDUMKII_I2CCOUNT] = {
    {
        .baseAddr = EUSCI_B1_BASE,
        .intNum = INT_EUSCIB1,
        .intPriority = (~0),
        .clockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK,
        .dataPin = I2CMSP432_P6_4_UCB1SDA,
        .clkPin = I2CMSP432_P6_5_UCB1SCL,
    },
};

const I2C_Config I2C_config[BOOSTXL_EDUMKII_I2CCOUNT] = {
    {
        .fxnTablePtr = &I2CMSP432_fxnTable,
        .object = &i2cMSP432Objects[BOOSTXL_EDUMKII_I2C],
        .hwAttrs = &i2cMSP432HWAttrs[BOOSTXL_EDUMKII_I2C]
    },
};

const uint_least8_t I2C_count = BOOSTXL_EDUMKII_I2CCOUNT;
/*
 *  =============================== Power ===============================
 */
const PowerMSP432_ConfigV1 PowerMSP432_config = {
    .policyInitFxn = &PowerMSP432_initPolicy,
    .policyFxn = &PowerMSP432_sleepPolicy,
    .initialPerfLevel = 2,
    .enablePolicy = true,
    .enablePerf = true,
    .enableParking = true
};
/*
 *  =============================== PWM ===============================
 */
#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTimerMSP432.h>

PWMTimerMSP432_Object pwmTimerMSP432Objects[BOOSTXL_EDUMKII_PWMCOUNT];

const PWMTimerMSP432_HWAttrsV2 pwmTimerMSP432HWAttrs[BOOSTXL_EDUMKII_PWMCOUNT] = {
    {
        /* P2.4 -> TA0.1 -> RGB_LED_GRN,EDUMKII BOOSTERPACK */
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .pwmPin = PWMTimerMSP432_P2_4_TA0CCR1A,
    },
    {
        /* P2.6 -> TA0.3 -> RGB_LED_RED,EDUMKII BOOSTERPACK */
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .pwmPin = PWMTimerMSP432_P2_6_TA0CCR3A,
    },
    {
        /* P2.7 -> TA0.4 -> BUZZER,EDUMKII BOOSTERPACK */
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .pwmPin = PWMTimerMSP432_P2_7_TA0CCR4A,
    },
    {
        /* P5.6 -> TA2.1 -> RGB_LED_BLU,EDUMKII BOOSTERPACK */
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .pwmPin = PWMTimerMSP432_P5_6_TA2CCR1A,
    },
    {
        /* P2.0 -> TA1.1 -> RGB_LED_RED,MSP432 LAUNCHPAD */
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .pwmPin = PWMTimerMSP432_P2_0_TA1CCR1A,
    },
    {
        /* P2.1 -> TA1.2 -> RGB_LED_GRN,MSP432 LAUNCHPAD */
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .pwmPin = PWMTimerMSP432_P2_1_TA1CCR2A,
    },
    {
        /* P2.2 -> TA1.3 -> RGB_LED_BLU,MSP432 LAUNCHPAD */
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .pwmPin = PWMTimerMSP432_P2_2_TA1CCR3A,
    },
};

const PWM_Config PWM_config[BOOSTXL_EDUMKII_PWMCOUNT] = {
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[BOOSTXL_EDUMKII_PWM_RGB_G],
        .hwAttrs = &pwmTimerMSP432HWAttrs[BOOSTXL_EDUMKII_PWM_RGB_G]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[BOOSTXL_EDUMKII_PWM_RGB_R],
        .hwAttrs = &pwmTimerMSP432HWAttrs[BOOSTXL_EDUMKII_PWM_RGB_R]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[BOOSTXL_EDUMKII_PWM_BUZZ],
        .hwAttrs = &pwmTimerMSP432HWAttrs[BOOSTXL_EDUMKII_PWM_BUZZ]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[BOOSTXL_EDUMKII_PWM_RGB_B],
        .hwAttrs = &pwmTimerMSP432HWAttrs[BOOSTXL_EDUMKII_PWM_RGB_B]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[MSP_EXP432P401R_PWM_RGB_R],
        .hwAttrs = &pwmTimerMSP432HWAttrs[MSP_EXP432P401R_PWM_RGB_R]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[MSP_EXP432P401R_PWM_RGB_G],
        .hwAttrs = &pwmTimerMSP432HWAttrs[MSP_EXP432P401R_PWM_RGB_G]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[MSP_EXP432P401R_PWM_RGB_B],
        .hwAttrs = &pwmTimerMSP432HWAttrs[MSP_EXP432P401R_PWM_RGB_B]
    },
};

const uint_least8_t PWM_count = BOOSTXL_EDUMKII_PWMCOUNT;

/*
 *  =============================== SPI ===============================
 */
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPIMSP432DMA.h>

/* SPI objects */
SPIMSP432DMA_Object spiMSP432DMAObjects[BOOSTXL_EDUMKII_SPICOUNT];

/* SPI configuration structure */
const SPIMSP432DMA_HWAttrsV1 spiMSP432DMAHWAttrs[BOOSTXL_EDUMKII_SPICOUNT] = {
    {
        .baseAddr = EUSCI_B0_BASE,
        .bitOrder = EUSCI_B_SPI_LSB_FIRST,
        .clockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK,
        .defaultTxBufValue = 0,
        .dmaIntNum = INT_DMA_INT1,
        .intPriority = (~0),
        .rxDMAChannelIndex = DMA_CH1_EUSCIB0RX0,
        .txDMAChannelIndex = DMA_CH0_EUSCIB0TX0,
        .clkPin = SPIMSP432DMA_P1_5_UCB0CLK,
        .simoPin = SPIMSP432DMA_P1_6_UCB0SIMO,
        .somiPin = SPIMSP432DMA_P1_7_UCB0SOMI,
        .stePin = NULL, /* SPIMSP432DMA_P1_4_UCB0STE */
        .pinMode = EUSCI_SPI_3PIN
    },
};

const SPI_Config SPI_config[BOOSTXL_EDUMKII_SPICOUNT] = {
    {
        .fxnTablePtr = &SPIMSP432DMA_fxnTable,
        .object = &spiMSP432DMAObjects[BOOSTXL_EDUMKII_SPI_LCD],
        .hwAttrs = &spiMSP432DMAHWAttrs[BOOSTXL_EDUMKII_SPI_LCD]
    },
};

const uint_least8_t SPI_count = BOOSTXL_EDUMKII_SPICOUNT;
/*
 *  =============================== Timer ===============================
 */
#include <ti/drivers/Timer.h>
#include <ti/drivers/timer/TimerMSP432.h>

TimerMSP432_Object timerMSP432Objects[BOOSTXL_EDUMKII_TIMERCOUNT];

const TimerMSP432_HWAttrs timerMSP432HWAttrs[BOOSTXL_EDUMKII_TIMERCOUNT] = {
    {
        .timerBaseAddress = TIMER32_0_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .intNum = INT_T32_INT1,
        .intPriority = ~0
    },
    {
        .timerBaseAddress = TIMER32_1_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .intNum = INT_T32_INT2,
        .intPriority = ~0
    },
    {
        .timerBaseAddress = TIMER_A0_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .intNum = INT_TA0_0,
        .intPriority = ~0
    },
    {
        .timerBaseAddress = TIMER_A2_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .intNum = INT_TA2_0,
        .intPriority = ~0
    },
    {
        .timerBaseAddress = TIMER_A1_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .intNum = INT_TA1_0,
        .intPriority = ~0
    },
};

const Timer_Config Timer_config[BOOSTXL_EDUMKII_TIMERCOUNT] = {
    {
        .fxnTablePtr = &TimerMSP432_Timer32_fxnTable,
        .object = &timerMSP432Objects[BOOSTXL_EDUMKII_TIMER_T32_0],
        .hwAttrs = &timerMSP432HWAttrs[BOOSTXL_EDUMKII_TIMER_T32_0]
    },
    {
        .fxnTablePtr = &TimerMSP432_Timer32_fxnTable,
        .object = &timerMSP432Objects[BOOSTXL_EDUMKII_TIMER_T32_1],
        .hwAttrs = &timerMSP432HWAttrs[BOOSTXL_EDUMKII_TIMER_T32_1]
    },
    {
        .fxnTablePtr = &TimerMSP432_Timer_A_fxnTable,
        .object = &timerMSP432Objects[BOOSTXL_EDUMKII_TIMER_TA_0],
        .hwAttrs = &timerMSP432HWAttrs[BOOSTXL_EDUMKII_TIMER_TA_0]
    },
    {
        .fxnTablePtr = &TimerMSP432_Timer_A_fxnTable,
        .object = &timerMSP432Objects[BOOSTXL_EDUMKII_TIMER_TA_2],
        .hwAttrs = &timerMSP432HWAttrs[BOOSTXL_EDUMKII_TIMER_TA_2]
    },
    {
        .fxnTablePtr = &TimerMSP432_Timer_A_fxnTable,
        .object = &timerMSP432Objects[BOOSTXL_EDUMKII_TIMER_TA_1],
        .hwAttrs = &timerMSP432HWAttrs[BOOSTXL_EDUMKII_TIMER_TA_1]
    },
};

const uint_least8_t Timer_count = BOOSTXL_EDUMKII_TIMERCOUNT;

/*
 *  =============================== UART ===============================
 */
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTMSP432.h>

UARTMSP432_Object uartMSP432Objects[BOOSTXL_EDUMKII_UARTCOUNT];
unsigned char uartMSP432RingBuffer[BOOSTXL_EDUMKII_UARTCOUNT][32];

/*
 * The baudrate dividers were determined by using the MSP432 baudrate
 * calculator
 * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
const UARTMSP432_BaudrateConfig uartMSP432Baudrates[] = {
    /* {baudrate, input clock, prescalar, UCBRFx, UCBRSx, oversampling} */
    {
        .outputBaudrate = 115200,
        .inputClockFreq = 12000000,
        .prescalar = 6,
        .hwRegUCBRFx = 8,
        .hwRegUCBRSx = 32,
        .oversampling = 1
    },
    {115200, 6000000,   3,  4,   2, 1},
    {115200, 3000000,   1, 10,   0, 1},
    {9600,   12000000, 78,  2,   0, 1},
    {9600,   6000000,  39,  1,   0, 1},
    {9600,   3000000,  19,  8,  85, 1},
    {9600,   32768,     3,  0, 146, 0}
};

const UARTMSP432_HWAttrsV1 uartMSP432HWAttrs[BOOSTXL_EDUMKII_UARTCOUNT] = {
    {
        .baseAddr = EUSCI_A0_BASE,
        .intNum = INT_EUSCIA0,
        .intPriority = (~0),
        .clockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        .bitOrder = EUSCI_A_UART_LSB_FIRST,
        .numBaudrateEntries = sizeof(uartMSP432Baudrates) /
            sizeof(UARTMSP432_BaudrateConfig),
        .baudrateLUT = uartMSP432Baudrates,
        .ringBufPtr  = uartMSP432RingBuffer[BOOSTXL_EDUMKII_UART0],
        .ringBufSize = sizeof(uartMSP432RingBuffer[BOOSTXL_EDUMKII_UART0]),
        .rxPin = UARTMSP432_P1_2_UCA0RXD,
        .txPin = UARTMSP432_P1_3_UCA0TXD,
    },
};

const UART_Config UART_config[BOOSTXL_EDUMKII_UARTCOUNT] = {
    {
        .fxnTablePtr = &UARTMSP432_fxnTable,
        .object = &uartMSP432Objects[BOOSTXL_EDUMKII_UART0],
        .hwAttrs = &uartMSP432HWAttrs[BOOSTXL_EDUMKII_UART0]
    }
};

const uint_least8_t UART_count = BOOSTXL_EDUMKII_UARTCOUNT;

/*
 *  =============================== Watchdog ===============================
 */
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogMSP432.h>

WatchdogMSP432_Object watchdogMSP432Objects[BOOSTXL_EDUMKII_WATCHDOGCOUNT];

const WatchdogMSP432_HWAttrs watchdogMSP432HWAttrs[BOOSTXL_EDUMKII_WATCHDOGCOUNT] = {
    {
        .baseAddr = WDT_A_BASE,
        .intNum = INT_WDT_A,
        .intPriority = (~0),
        .clockSource = WDT_A_CLOCKSOURCE_SMCLK,
        .clockDivider = WDT_A_CLOCKDIVIDER_8192K
    },
};

const Watchdog_Config Watchdog_config[BOOSTXL_EDUMKII_WATCHDOGCOUNT] = {
    {
        .fxnTablePtr = &WatchdogMSP432_fxnTable,
        .object = &watchdogMSP432Objects[BOOSTXL_EDUMKII_WATCHDOG],
        .hwAttrs = &watchdogMSP432HWAttrs[BOOSTXL_EDUMKII_WATCHDOG]
    },
};

const uint_least8_t Watchdog_count = BOOSTXL_EDUMKII_WATCHDOGCOUNT;
