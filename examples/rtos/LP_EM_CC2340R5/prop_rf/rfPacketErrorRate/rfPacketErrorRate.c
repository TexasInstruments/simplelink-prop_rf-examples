/*
 * Copyright (c) 2013-2022, Texas Instruments Incorporated
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

/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

/* Board Header Files */
#include "ti_drivers_config.h"

/* TI Drivers */
#include <ti/display/Display.h>
#include <ti/drivers/GPIO.h>

/* Application specific Header files */
#include "menu.h"

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/cpu.h)

/* Application specific Header files */
#include "menu.h"

extern void menu_runTask();

/*
Interrupt handler for BTN1 and BTN2.
*/
void buttonCallbackFunction(uint_least8_t index)
{
    /* Simple debounce logic, only toggle if the button is still pushed (low) */
    CPUDelay((uint32_t)((48000000/3)*0.050f));
    if (!GPIO_read(index))
    {
        if (index == CONFIG_GPIO_BTN1)
        {
            menu_notifyButtonPressed(Button_Select);
        }
        else
        {
            menu_notifyButtonPressed(Button_Navigate);
        }
    }
}

/***** Function definitions *****/

void *mainThread(void *arg0)
{
    /* Initialize UART and SPI for the display driver */
    Display_init();

    GPIO_setConfig(CONFIG_GPIO_BTN1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BTN2, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BTN1, buttonCallbackFunction);
    GPIO_setCallback(CONFIG_GPIO_BTN2, buttonCallbackFunction);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BTN1);
    GPIO_enableInt(CONFIG_GPIO_BTN2);

    /* Initialize menu */
    menu_init();
    menu_runTask();

    return NULL;
}
