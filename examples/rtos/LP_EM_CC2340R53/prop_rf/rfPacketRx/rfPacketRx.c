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

/* TI Drivers */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/rcl/RCL.h>
#include <ti/drivers/rcl/RCL_Scheduler.h>
#include <ti/drivers/rcl/commands/generic.h>

/* SysConfig Generated */
#include "ti_drivers_config.h"
#include "ti_radio_config.h"

/***** Defines *****/
/* Packet RX Configuration */
#define MAX_LENGTH              (30U) // Max packet length
#define NUM_DATA_ENTRIES        (2U)  // Number of data entries

/* RCL buffer length */
#define BUFF_STRUCT_LENGTH      (2048U)

/* Indicates if FS is off */
#define FS_OFF                  (1U)  // 0: On, 1: Off

/* Indicates if RX packet is not stored */
#define DISCARD_RX_PACKET       (0U)  // 0: Store received packets in rxBuffers
                                      // 1: Discard packets

/* RF Frequency (Hz) to program */
#define FREQUENCY               (2440000000U)

/* Number of packets to initialize for multi buffer */
#define NUM_OF_PACKETS          (1U)

/***** Variable Declarations *****/
/* RCL Commands */
extern RCL_CmdGenericRx    rclPacketRxCmdGenericRx;       // RX command
RCL_StatsGeneric    stats;              // Statistic command

/* RCL Client used to open RCL */
static RCL_Client  rclClient;

RCL_MultiBuffer *multiBuffer;

/* Counters for RCL event callback */
volatile uint32_t gCmdDone = 0;         // Command done
volatile uint32_t gPktRcvd = 0;         // Packet received

/* Buffer used to store RCL packet */
uint32_t buffer[NUM_DATA_ENTRIES][BUFF_STRUCT_LENGTH/4];

/***** Callback Functions *****/
void defaultCallback(RCL_Command *cmd, LRF_Events lrfEvents, RCL_Events rclEvents)
{
    if (rclEvents.lastCmdDone)
    {
        gCmdDone += 1;
    }
    if (rclEvents.rxEntryAvail)
    {
        gPktRcvd += 1;
        GPIO_toggle(CONFIG_GPIO_RLED);

        /* Clear multi buffer after receive finishes for next receive */
        RCL_MultiBuffer_clear(multiBuffer);
    }
}

/***** Function definitions *****/
void *mainThread(void *arg0)
{
    /* Initialize and open RCL */
    RCL_init();

    RCL_Handle rclHandle = RCL_open(&rclClient, &LRF_config);

    /* Set RF frequency */
    rclPacketRxCmdGenericRx.rfFrequency = FREQUENCY;

    /* Start command as soon as possible */
    rclPacketRxCmdGenericRx.common.scheduling = RCL_Schedule_Now;
    rclPacketRxCmdGenericRx.common.status = RCL_CommandStatus_Idle;

    rclPacketRxCmdGenericRx.config.fsOff = FS_OFF;                        // Turn off FS
    rclPacketRxCmdGenericRx.config.discardRxPackets = DISCARD_RX_PACKET;  // Store received packet

    /* Callback triggers on last command done or packet received */
    rclPacketRxCmdGenericRx.common.runtime.callback = defaultCallback;
    rclPacketRxCmdGenericRx.common.runtime.rclCallbackMask.value = RCL_EventLastCmdDone.value |
                                                 RCL_EventRxEntryAvail.value;

    /* Maximum packet length */
    rclPacketRxCmdGenericRx.maxPktLen = MAX_LENGTH;

    /* Set command to run forever until completion */
    rclPacketRxCmdGenericRx.common.timing.relGracefulStopTime = 0;

    /*  Go back to sync search after receiving */
    rclPacketRxCmdGenericRx.config.repeated = 1;

    /* Setup generic status command */
    stats = RCL_StatsGeneric_DefaultRuntime();

    /* Set RX command statistics structure */
    rclPacketRxCmdGenericRx.stats = &stats;
    rclPacketRxCmdGenericRx.stats->config.activeUpdate = 1;

    GPIO_setConfig(CONFIG_GPIO_RLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_OFF);

    /* Initialize multi-buffer to allow RCL to store RX packet */
    for(int i = 0; i < NUM_OF_PACKETS; i++)
    {
        multiBuffer = (RCL_MultiBuffer *) buffer[i];
        RCL_MultiBuffer_init(multiBuffer, BUFF_STRUCT_LENGTH);
        RCL_MultiBuffer_put(&rclPacketRxCmdGenericRx.rxBuffers, multiBuffer);
    }

    /* Submit command */
    RCL_Command_submit(rclHandle, &rclPacketRxCmdGenericRx);

    /* Pend on command completion */
    RCL_Command_pend(&rclPacketRxCmdGenericRx);

    return NULL;
}
