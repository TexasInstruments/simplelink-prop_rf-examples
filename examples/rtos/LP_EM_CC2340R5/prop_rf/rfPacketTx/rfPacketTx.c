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

/* TI Drivers */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/rcl/RCL.h>
#include <ti/drivers/rcl/RCL_Scheduler.h>
#include <ti/drivers/rcl/commands/generic.h>
#include "ti_drivers_config.h"

#if defined(USE_250KBPS_MSK)
#include <setup/rcl_settings_msk_250_kbps.h>
#elif defined(USE_500KBPS_MSK)
#include <setup/rcl_settings_msk_500_kbps.h>
#else
#include <setup/rcl_settings_ble_generic.h>
#endif

/***** Defines *****/
/* Packet TX Configuration */
#define MAX_LENGTH              (30U) // Max packet length
#define NUM_DATA_ENTRIES        (2U)  // Number of data entries
#define NUM_PAD_BYTES           (3U)  // Number of pad bytes

/* Header length */
#if defined(USE_500KBPS_MSK) || defined(FIXED_LENGTH_SETUP) // 500KBPS is always set up for fixed length packets
#define HDR_LEN                 (0U)
#else
#if defined(USE_250KBPS_MSK) // 250KBPS with variable length enabled
#define HDR_LEN                 (1U)
#else // 1 Mbps with variable length enabled
#define HDR_LEN                 (2U)
#endif
#endif

#define PACKET_INTERVAL     500000  /* Set packet interval to 500000us or 500ms */

/* Indicates if FS is off */
#define FS_OFF                  (1U)  // 0: On, 1: Off

#if defined(USE_250KBPS_MSK) || defined(USE_500KBPS_MSK)
#define FREQUENCY               (2433000000U)
#else
#define FREQUENCY               (2440000000U)
#endif

#define TX_POWER (5U)


/***** Variable declarations *****/
/* RCL Commands */
RCL_CmdGenericTx   txCmd;               // TX command

/* RCL Client used to open RCL */
static RCL_Client  rclClient;

/* TX packet buffer */
uint32_t packet[NUM_DATA_ENTRIES][3 + ((MAX_LENGTH + 10)/ 4)];

/* Counters for RCL event callback */
volatile uint32_t gCmdDone = 0;     // Command done

/***** Callback Functions *****/
void defaultCallback(RCL_Command *cmd, LRF_Events lrfEvents, RCL_Events rclEvents)
{
    if (rclEvents.lastCmdDone)
    {
        gCmdDone += 1;

        GPIO_toggle(CONFIG_GPIO_GLED);
    }
}

/***** Function definitions *****/

void *mainThread(void *arg0)
{
    /* Initialize and open RCL */
    RCL_init();

#if defined(USE_250KBPS_MSK)
    RCL_Handle rclHandle = RCL_open(&rclClient, &LRF_configMsk250Kbps);
#elif defined(USE_500KBPS_MSK)
    RCL_Handle rclHandle = RCL_open(&rclClient, &LRF_configMsk500Kbps);
#else
    RCL_Handle rclHandle = RCL_open(&rclClient, &LRF_configBle);
#endif

    /* Setup generic transmit command */
    txCmd = RCL_CmdGenericTx_DefaultRuntime();

    /* Set RF frequency */
    txCmd.rfFrequency = FREQUENCY;
#if !(defined(USE_250KBPS_MSK) || defined(USE_500KBPS_MSK))
    txCmd.common.phyFeatures = RCL_PHY_FEATURE_SUB_PHY_1_MBPS_BLE;
#endif

    /* Start command as soon as possible */
    txCmd.common.scheduling = RCL_Schedule_Now;
    txCmd.common.status = RCL_CommandStatus_Idle;

    txCmd.config.fsOff = FS_OFF; // Turn off FS
	txCmd.txPower.dBm = TX_POWER;

    /* Callback triggers on last command done */
    txCmd.common.runtime.callback = defaultCallback;
    txCmd.common.runtime.rclCallbackMask.value = RCL_EventLastCmdDone.value;

    /* Set RCL TX buffer packet to be packet buffer */
    RCL_Buffer_TxBuffer *txPacket = (RCL_Buffer_TxBuffer *)&packet;

    GPIO_setConfig(CONFIG_GPIO_GLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_OFF);

    while(1)
    {
        /* Create packet with random payload */
        uint8_t *txData;
        txData = RCL_TxBuffer_init(txPacket, NUM_PAD_BYTES, HDR_LEN, MAX_LENGTH);
#if !(defined(USE_500KBPS_MSK) || defined(FIXED_LENGTH_SETUP))
#if defined(USE_250KBPS_MSK)
        txData[0] = MAX_LENGTH;
#else
        txData[0] = 0;
        txData[1] = MAX_LENGTH;
#endif
#endif
        for (int i = HDR_LEN; i < MAX_LENGTH; i++)
        {
            txData[i] = rand();
        }

        /* Set packet to transmit */
        RCL_TxBuffer_put(&txCmd.txBuffers, txPacket);

        txCmd.common.status = RCL_CommandStatus_Idle;

        /* Submit command */
        RCL_Command_submit(rclHandle, &txCmd);

        /* Pend on command completion */
        RCL_Command_pend(&txCmd);

        usleep(PACKET_INTERVAL);
    }
}
