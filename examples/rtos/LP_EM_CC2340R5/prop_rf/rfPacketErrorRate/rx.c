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
#include <setup/rcl_settings_msk_250_kbps_fec.h>
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

#define NFRACBITS               9   // 32 - ceil(log2(4000000)) - 1
#define LOG2_100MILLION         27  // ceil(log2(10^8)=26.575424759098897) = 27

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

uint32_t nFracBits        = 0;
uint32_t nBitsDeltaTimeUs = 0;
uint32_t  startTime = 0, endTime = 0;
uint32_t  currTimerVal = 0, rxTimeoutVal = 0;
uint32_t deltaTimeUs = 0;
uint32_t deltaTimePacket = 0;
uint32_t deltaTimePacketUs = 0;
uint32_t pktIntervalEstUs  = 0;
uint32_t nBits = 0;
uint32_t throughputI = 0, throughputQ = 0;

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

         if(bFirstPacket)
        {
            startTime = rxCmd.stats->lastTimestamp;
            /* Lock out this read after the first packet */
            bFirstPacket = false;
        }
        else
        {
            endTime = rxCmd.stats->lastTimestamp;
            /* Calculate the delta between two consecutive packets */
            deltaTimePacket   = endTime - startTime;
            deltaTimePacketUs = deltaTimePacket/(RCL_SCHEDULER_SYSTIM_US(1));

            /* Set current packet time stamp as the start time for the next
            * delta calculation
            */
            startTime    = endTime;
            deltaTimeUs += deltaTimePacketUs;
#if defined(__TI_COMPILER_VERSION__)
            nFracBits   = _norm(deltaTimeUs);
#elif defined(__IAR_SYSTEMS_ICC__)
            nFracBits   = __CLZ(deltaTimeUs);
#elif defined(__GNUC__)
            nFracBits   = __builtin_clz(deltaTimeUs);
#else
#error This compiler is not supported.
#endif
            nBitsDeltaTimeUs = 32UL - nFracBits;

            if(nBitsDeltaTimeUs >= LOG2_100MILLION)
            {
                // deltaTimeUs is two orders of magnitude larger that 10^6
                // Throughput_I = (N_bits * 2^nFracBits)/(delT_us/10^6))
                //              = Throughput_I / 2^nFracBits
                // Shift N_bits up to occupy the MSbs and then divide by
                // (delT_us/10^6) which is at least 6 bits wide
                //   log2(1e8)-log2(1e6) = 6.643856189774724
                throughputI = (nBits << nFracBits)/ (deltaTimeUs / 1000000UL);
                throughputQ = throughputI & ((1 << nFracBits) - 1);
                throughputI = throughputI >> nFracBits;
            }
            else
            {
                // deltaTimeUs is smaller or comparable to 10^8
                // Throughput_I = (N_bits * 2^NFRAC)/(delT_us/10^6)
                //              = (N_bits)* (round(10^6*2^NFRAC)/delT_us)
                //              = (N_bits)* ((10^6*2^NFRAC + delT_us/2)/delT_us)
                //              = Throughput_I / 2^nFracBits
                throughputI = nBits * (((1000000UL << NFRACBITS) + (deltaTimeUs >> 1))/deltaTimeUs);
                throughputQ = throughputI & ((1 << NFRACBITS) - 1);
                throughputI = throughputI >> NFRACBITS;
            }
            rxMetrics.throughput = throughputI;
        }

        /* Read out received packet */
        RCL_Buffer_DataEntry *rxPkt = RCL_MultiBuffer_RxEntry_get(&(rxCmd.rxBuffers), NULL);

        nBits += (rxPkt->length << 3);

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
    bFirstPacket = true;

    nBits = 0;
    startTime = endTime = 0;
    deltaTimeUs = deltaTimePacket = deltaTimePacketUs = 0;
    throughputI = throughputQ = 0;
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
        rclHandle = RCL_open(&rxRclClient, &LRF_configMsk250KbpsFec);
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

        rxMetrics.packetsExpected = config->packetCount;
        if(packetReceived || bPacketsLost)
        {
            rxMetrics.packetsReceived = nRxPkts;
            rxMetrics.packetsMissed   = config->packetCount - nRxPkts;
            rxMetrics.rssi            = rxCmd.stats->lastRssi;
            rxMetrics.crcOK           = rxCmd.stats->nRxOk;
            rxMetrics.throughput = throughputI;
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
                rxMetrics.rssi            = rxCmd.stats->lastRssi;
                rxMetrics.crcOK           = rxCmd.stats->nRxOk;
                rxMetrics.throughput      = throughputI;
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

