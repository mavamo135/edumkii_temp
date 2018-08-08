/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
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
 *  ======== gpiomain.c ========
 */

/* For usleep() */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* POSIX Header files */
#include <pthread.h>

/* RTOS header files */
#include <ti/sysbios/BIOS.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>
#include <ti/display/Display.h>

/* Board Header file */
#include "Board.h"

/* Boosterpack EDUMKII BSP */
#include "BSP/hal_lcd.h"
#include "BSP/lcd.h"
#include <ti/grlib/grlib.h>

/***** GLOBAL VARIABLES *****/
extern uint_fast16_t TEMP;
extern uint_fast16_t TARGET;
extern char STATE[];
extern uint_fast16_t STATEFLAG;
extern pthread_mutex_t hold;
extern uint_fast16_t ADDTARGET,SUBTARGET;

//
//  ======== compareTempTarget ========
//  Compare current temperature with target temperature & update STATE (Cooling, Heating, Off) global variable.
//
void compareTempTarget(unsigned int temp, unsigned int target)
{
    if(TARGET < TEMP){
        strcpy(STATE, "COOLING");
        STATEFLAG = 0;
    }else if(TARGET > TEMP){
        strcpy(STATE, "HEATING");
        STATEFLAG = 2;
    }else{
        strcpy(STATE, "  OFF  ");
        STATEFLAG = 1;
    }
}

//
//  ======== gpioButtonFxn0 ========
//  Callback function for the GPIO interrupt on Board_GPIO_BUTTON0.
//
void gpioButtonFxn0(uint8_t index)
{
    // Lower TARGET
    SUBTARGET++;
}

//
//  ======== gpioButtonFxn1 ========
//  Callback function for the GPIO interrupt on Board_GPIO_BUTTON1.
//
void gpioButtonFxn1(uint8_t index)
{
    // Raise TARGET
    ADDTARGET++;
}

void updateTarget()
{
    TARGET += ADDTARGET - SUBTARGET;
    ADDTARGET =0;
    SUBTARGET =0;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Graphic library context */
    Graphics_Context  g_sContext;

    /* configure & open Display driver */
    Display_Handle    myDisplay;
    Display_Params    params;
    Display_Params_init(&params);
    myDisplay = Display_open(Display_Type_UART, &params);

    /* Call driver init functions */
    GPIO_init();
    // I2C_init();
    // SDSPI_init();
    // SPI_init();
    // UART_init();
    // Watchdog_init();

    // install Button callback left switch
    GPIO_setCallback(Board_GPIO_BUTTON1_BP, gpioButtonFxn0);
    // Enable interrupts
    GPIO_enableInt(Board_GPIO_BUTTON1_BP);

    // install Button callback right switch
    GPIO_setCallback(Board_GPIO_BUTTON2_BP, gpioButtonFxn1);
    // Enable interrupts
    GPIO_enableInt(Board_GPIO_BUTTON2_BP);

    /* Initializes display*/
    Crystalfontz128x128_Init();

    /* Set default screen orientation*/
    Crystalfontz128x128_SetOrientation(0);

    /* Initializes graphics context*/
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128, &g_sCrystalfontz128x128_funcs);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);
    Graphics_drawStringCentered(&g_sContext,
                                (int8_t *)"State:",
                                6,
                                64,
                                30,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(&g_sContext,
                                (int8_t *)"Temperature:",
                                12,
                                64,
                                60,
                                OPAQUE_TEXT);

    while (1) {
        usleep(250000); // Only update Serial terminal every 0.25s

        pthread_mutex_lock(&hold);

        updateTarget();
        compareTempTarget(TEMP,TARGET);
        Display_printf(myDisplay, 0, 0, "TEMP: %d / TARGET: %d / STATE: %s", TEMP, TARGET, STATE);

        /* Display state */
        Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)STATE,
                                    AUTO_STRING_LENGTH,
                                    64,
                                    40,
                                    OPAQUE_TEXT);

        /* Display temperature */
        char string[10];
        sprintf(string, "%d", TEMP);
        Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)string,
                                    5,
                                    55,
                                    70,
                                    OPAQUE_TEXT);
        sprintf(string, "C");
        Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)string,
                                    5,
                                    80,
                                    70,
                                    OPAQUE_TEXT);
        pthread_mutex_unlock(&hold);
  }
}
