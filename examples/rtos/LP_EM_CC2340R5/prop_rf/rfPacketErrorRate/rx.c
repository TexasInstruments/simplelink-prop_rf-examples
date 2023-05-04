/*
 * Copyright (c) 2016-2019, Texas Instruments Incorporated
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
/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#if defined(__IAR_SYSTEMS_ICC__)
#include <intrinsics.h>
#endif
/* Board Header files */
#include "ti_drivers_config.h"

/* TI Drivers */
#include <ti/drivers/rcl/RCL.h>
#include <ti/drivers/rcl/RCL_Scheduler.h>
#include <ti/drivers/rcl/commands/generic.h>
#include <setup/rcl_settings_msk_250_kbps.h>
#include <setup/rcl_settings_msk_500_kbps.h>
#include <setup/rcl_settings_ble_generic.h>


/* Application specific Header files */
#include "menu.h"
#include "config.h"

/***** Defines *****/
#define RX_DURATION RCL_SCHEDULER_SYSTIM_MS(1000)

/* Packet RX Configuration */
#define NUM_DATA_ENTRIES        (2U)  // Number of data entries

/* RCL buffer length */
#define BUFF_STRUCT_LENGTH      (1000U)

/* Indicates if FS is off */
#define FS_OFF                  (1U)  // 0: On, 1: Off

/* Indicates if RX packet is not stored */
#define DISCARD_RX_PACKET       (0U)  // 0: Store received packets in rxBuffers
                                      // 1: Discard packets

/* Multiplier to convert MHz to Hz */
#define FREQUENCY_MHZ_TO_HZ     (1000000U)

/* Number of packets to initialize for multi buffer */
#define NUM_OF_PACKETS          (1U)

/***** Variable Declarations *****/
/* RCL Commands */
RCL_CmdGenericRx    rxCmd;              // RX command
RCL_StatsGeneric    stats;              // Statistic command

/* RCL Client used to open RCL */
static RCL_Client  rxRclClient;

RCL_MultiBuffer *multiBuffer;

/* Counters for RCL event callback */
volatile uint32_t gRxCmdDone = 0;         // Command done
volatile uint32_t gPktRcvd = 0;         // Packet received

/* Boolean to determine if a packet was received */
static uint8_t packetReceived = false;


static volatile bool     bFirstPacket = true, bSecondPacket = true;
static volatile bool     bPacketsLost = false;
static volatile uint16_t nRxPkts = 0, nMissPkts = 0, nExpPkts = 0;

static rx_metrics rxMetrics = {
    .packetsReceived = 0,
    .packetsMissed   = 0,
    .packetsExpected = 0,
    .nRxTimeouts      = 0,
    .nPktsPerTimeout = 0,
    .rssi            = 0,
    .crcOK           = 0,
    .throughput      = 0
};

/* Buffer used to store RCL packet */
uint32_t buffer[NUM_DATA_ENTRIES][BUFF_STRUCT_LENGTH/4];

/***** Callback Functions *****/
void rxCallback(RCL_Command *cmd, LRF_Events lrfEvents, RCL_Events rclEvents)
{
    if (rclEvents.lastCmdDone)
    {
        gRxCmdDone += 1;
    }
    if (rclEvents.rxEntryAvail)
    {
        gPktRcvd += 1;

        /* Update boolean indicating packet received to update menu */
        packetReceived = true;

        /* Increment number of packets received */
        nRxPkts++;

        /* Clear multi buffer after receive finishes for next receive */
        RCL_MultiBuffer_clear(multiBuffer);
    }
}

/* Reset all the volatile variables */
static void rx_resetVariables(void)
{
    nRxPkts = 0U;
    gPktRcvd = 0U;
    gRxCmdDone = 0U;
    packetReceived = false;
}

/* Runs the receiving part of the test application and returns a result */
TestResult rx_runRxTest(const ApplicationConfig* config)
{
    TestResult testResult;
    RCL_Handle rclHandle;

    /* Initialize RCL */
    RCL_init();

    /* Setup generic receive command */
    rxCmd = RCL_CmdGenericRx_DefaultRuntime();

    /* Open RCL */
    if(config->rfSetup == RCL_Generic_BLE_1M)
    {
        rclHandle = RCL_open(&rxRclClient, &LRF_configBle);
        rxCmd.common.phyFeatures = RCL_PHY_FEATURE_SUB_PHY_1_MBPS_BLE;
    }
    else if(config->rfSetup == RCL_Generic_250K_MSK)
    {
        rclHandle = RCL_open(&rxRclClient, &LRF_configMsk250Kbps);
    }
    else
    {
        rclHandle = RCL_open(&rxRclClient, &LRF_configMsk500Kbps);
    }


    /* Set RF frequency */
    rxCmd.rfFrequency = config->frequencyTable[config->frequency].frequency * FREQUENCY_MHZ_TO_HZ;

    /* Start command as soon as possible */
    rxCmd.common.scheduling = RCL_Schedule_Now;
    rxCmd.common.status = RCL_CommandStatus_Idle;

    rxCmd.config.fsOff = FS_OFF;                        // Turn off FS
    rxCmd.config.discardRxPackets = DISCARD_RX_PACKET;  // Store received packet

    /* Callback triggers on last command done or packet received */
    rxCmd.common.runtime.callback = rxCallback;
    rxCmd.common.runtime.rclCallbackMask.value = RCL_EventLastCmdDone.value |
                                                 RCL_EventRxEntryAvail.value;

    /* Maximum packet length */
    rxCmd.maxPktLen = config->payloadLength;

    /* Set command to run until end of defined graceful timeout duration */
    rxCmd.common.timing.relGracefulStopTime = RX_DURATION;

    /* End after receiving one packet */
    rxCmd.config.repeated = 0U;

    /* Setup generic status command */
    stats = RCL_StatsGeneric_DefaultRuntime();

    /* Set RX command statistics structure */
    rxCmd.stats = &stats;

    /* Initialize multi-buffer to allow RCL to store RX packet */
    for(int i = 0; i < 1U; i++)
    {
        multiBuffer = (RCL_MultiBuffer *) buffer[i];
        RCL_MultiBuffer_init(multiBuffer, BUFF_STRUCT_LENGTH);
        RCL_MultiBuffer_put(&rxCmd.rxBuffers, multiBuffer);
    }

    while(1)
    {
        /* Submit command */
        RCL_Command_submit(rclHandle, &rxCmd);

        /* Pend on command completion */
        RCL_Command_pend(&rxCmd);

        if(packetReceived || bPacketsLost)
        {
            rxMetrics.packetsReceived = nRxPkts;
            rxMetrics.packetsMissed   = config->packetCount - nRxPkts;
            rxMetrics.packetsExpected = config->packetCount;
            rxMetrics.rssi            = rxCmd.stats->lastRssi;
            rxMetrics.crcOK           = rxCmd.stats->nRxOk;
            menu_updateRxScreen(&rxMetrics);
        }

        /* If user presses button or max packet is reached, return back to menu */
        if(menu_isButtonPressed() || config->packetCount <= nRxPkts)
        {
            /* Close RCL handle */
            RCL_close(rclHandle);

            if((config->packetCount >= nRxPkts))
            {
                rxMetrics.packetsReceived = nRxPkts;
                rxMetrics.packetsMissed   = rxCmd.stats->nRxNok;
                rxMetrics.packetsExpected = config->packetCount;
                rxMetrics.rssi            = rxCmd.stats->lastRssi;
                rxMetrics.crcOK           = rxCmd.stats->nRxOk;
                menu_updateRxScreen(&rxMetrics);

                testResult = TestResult_Finished;
            }
            else
            {
                testResult = TestResult_Aborted;
            }

            /* Reset all variables */
            rx_resetVariables();

            return(testResult);
        }

        // Set RCL status to Idle to indicate next submit
        rxCmd.common.status = RCL_CommandStatus_Idle;
    }

}

