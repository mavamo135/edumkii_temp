/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
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
 *  ======== pwm.c ========
 */

/* For usleep() */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>

/* POSIX Header files */
#include <pthread.h>

/* RTOS header files */
#include <ti/sysbios/BIOS.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/display/Display.h>
#include <ti/drivers/PWM.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>

/* Board Header file */
#include "Board.h"

/***** GLOBAL VARIABLES *****/
extern uint_fast16_t TEMP;
extern uint_fast16_t TARGET;
extern uint_fast16_t STATEFLAG;
extern char STATE[];
extern pthread_mutex_t hold;

uint32_t getBrightness(){
    uint_fast16_t delta = abs(TEMP - TARGET);
    if(delta > 80){
        delta = 80;
    }
    delta = (PWM_DUTY_FRACTION_MAX/80) * delta;
    return delta;
}
/*
 *  ======== mainThread ========
 */
void *pwmThread(void *arg0)
{
    /* Period and duty in microseconds */
    uint16_t   pwmPeriod = 3000;


    /* Sleep time in microseconds */
    uint32_t   time = 50000;
    PWM_Handle pwm1;
    PWM_Handle pwm2;
    PWM_Params params;

    /* Call driver init functions. */
    PWM_init();

    PWM_Params_init(&params);
    params.dutyUnits = PWM_DUTY_FRACTION;
    params.dutyValue = 0;
    params.periodUnits = PWM_PERIOD_US;
    params.periodValue = pwmPeriod;

    // open & start pwm 1 (Red)
    pwm1 = PWM_open(Board_PWM_RGB_R_BP, &params);
    if (pwm1 == NULL) {
        /* Board_PWM0 did not open */
        while (1);
    }
    PWM_start(pwm1);

    // open & start pwm 2 (Blue)
    pwm2 = PWM_open(Board_PWM_RGB_B_BP, &params);

    if (pwm2 == NULL) {
        /* Board_PWM0 did not open */
        while (1);
    }
    PWM_start(pwm2);


    /* Loop forever incrementing the PWM duty */
    while (1) {
        pthread_mutex_lock(&hold);

        if(STATEFLAG == 0){                // Cooling
            PWM_setDuty(pwm2, getBrightness());
            PWM_setDuty(pwm1, 0);
        }else if(STATEFLAG == 2){          // Heating
            PWM_setDuty(pwm1, getBrightness());
            PWM_setDuty(pwm2, 0);
        }else {                            // OFF
            PWM_setDuty(pwm1, 0);
            PWM_setDuty(pwm2, 0);
        }

        pthread_mutex_unlock(&hold);
        usleep(time);
    }
}
