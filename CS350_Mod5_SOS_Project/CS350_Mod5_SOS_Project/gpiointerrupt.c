/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
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
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include <ti/drivers/Timer.h>
#include <stdbool.h>

typedef enum { State_SOS, State_OK} State;
typedef enum {Message_SOS, Message_OK} Message;

volatile State currState = State_SOS;
volatile Message currMessage = Message_SOS;
volatile bool messageComplete = true;
volatile bool changeMessage = false;
volatile int ticks = 0;


void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    // Check if message is complete,
    if (messageComplete) {
        // If changeMessage flag is raised,
        if (changeMessage) {
            // If current Message is SOS, update State and Message to OK and reset changeMessage
            if (currMessage == Message_SOS) {
                currMessage = Message_OK;
                currState = State_OK;
                changeMessage = false;
            }
        }
        // If changeMessage flag is not raised, reset State and Message to SOS
        else {
            currMessage = Message_SOS;
            currState = State_SOS;
        }
        // Reset messageComplete to false
        messageComplete = false;
    }
    switch (currState) {
    // Switch case for SOS, uses count of ticks to determine whether it needs to turn on or off a particular light
    case State_SOS:
        if (ticks == 0 || ticks == 2 || ticks == 4) {
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        }
        if (ticks == 1 || ticks == 3) {
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        }
        if (ticks >= 5 && ticks <= 7) {
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        }
        if (ticks >= 8 && ticks <= 10){
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
        }
        if (ticks == 11 || ticks == 15) {
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
        }
        if (ticks >= 12 && ticks <= 14) {
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
        }
        if (ticks >= 16 && ticks <= 18) {
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
        }
        if (ticks >= 19 && ticks <= 21) {
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
        }
        if (ticks == 22 || ticks == 24 || ticks == 26) {
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        }
        if (ticks == 23 || ticks == 25) {
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        }
        if (ticks >= 27 && ticks <= 32) {
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        }
        // If final tick, change messageComplete flag to True and reset tick counter
        if (ticks == 33) {
            messageComplete = true;
            ticks = -1;
        }
        ++ticks;
        break;

    // Switch case for OK, uses count of ticks to determine whether it needs to turn on or off a particular light
    case State_OK:
        if (ticks >= 0 && ticks <= 2) {
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
        }
        if (ticks == 3 || ticks == 7) {
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
        }
        if (ticks >= 4 && ticks <= 6) {
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
        }
        if (ticks >= 8 && ticks <= 10) {
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
        }
        if (ticks >= 11 && ticks <= 13) {
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
        }
        if (ticks >= 14 && ticks <= 16) {
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
        }
        if (ticks == 17 || ticks == 19) {
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        }
        if (ticks == 18) {
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        }
        if (ticks >= 20 && ticks <= 22) {
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
        }
        if (ticks >= 23 && ticks <= 28) {
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
        }
        // If final tick, change messageComplete flag to true and reset ticks
        if (ticks == 29) {
            messageComplete = true;
            ticks = -1;
        }
        // Increment ticks through each iteration
        ++ticks;
        break;
    }
}

void initTimer(void)
{
    Timer_Handle timer0;
    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);
    params.period = 500000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {

        }
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {

        }
    }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    changeMessage = true;
    /* Toggle an LED */
    //GPIO_toggle(CONFIG_GPIO_LED_0);
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Toggle an LED */
    //GPIO_toggle(CONFIG_GPIO_LED_1);
    changeMessage = true;

}



/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    //GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }
    initTimer();

    return (NULL);
}
