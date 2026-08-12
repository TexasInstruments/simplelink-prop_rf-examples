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
/* Standard C Libraries */
#include <stdlib.h>
#include <unistd.h>

/* TI Drivers */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/rcl/RCL.h>
#include <ti/drivers/rcl/RCL_Scheduler.h>
#include <ti/drivers/rcl/commands/generic.h>


/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>

/* Board Header files */
#include "ti_drivers_config.h"
#include "ti_radio_config.h"

/* Application specific Header files */
#include "menu.h"
#include "config.h"

/***** Defines *****/
/* Packet TX Configuration */
#define MAX_LENGTH              (255U) // Max packet length
#define NUM_DATA_ENTRIES        (2U)  // Number of data entries
#define NUM_PAD_BYTES           (3U)  // Number of pad bytes

/* RF Frequency (Hz) to program */
#define FREQUENCY               (2440000000U)

/* Indicates if FS is off */
#define FS_OFF                  (1U)  // 0: On, 1: Off

/* Multiplier to convert MHz to Hz */
#define FREQUENCY_MHZ_TO_HZ     (1000000U)

/***** Variable declarations *****/
/* RCL Commands */
RCL_CmdGenericTx   *rclPacketTxCmdGenericTx;               // TX command
extern RCL_CmdGenericTx rclPacketTxCmdGenericTx_msk_250_kbps_0;
extern RCL_CmdGenericTx rclPacketTxCmdGenericTx_ble_gen_2;
extern RCL_CmdGenericTx rclPacketTxCmdGenericTx_msk_250_kbps_fec_1;

/* RCL Client used to open RCL */
static RCL_Client  txRclClient;

/* TX packet buffer */
uint32_t packet[NUM_DATA_ENTRIES][3 + ((MAX_LENGTH + 10)/ 4)];

/* Counters for RCL event callback */
volatile uint32_t gTxCmdDone = 0;     // Command done

#define MAX_PAYLOAD_LENGTH      254 // Maximum length of the packet to send (Even due to HS requirement)
#define MAX_BLE_PAYLOAD_LENGTH  30  // Maximum length of the BLE4/5 packet to send
#define DATA_ENTRY_HEADER_SIZE  8   // Constant header size of a Generic Data Entry
#define NUM_APPENDED_BYTES      0

#define EXTENDED_HEADER_LENGTH  9
#define BLE_BASE_FREQUENCY      2300 // When programming the channel in the BLE TX command it is the
                                     // offset from 2300 MHz

#define MAX_BLE_PWR_LEVEL_DBM         5
#define MAX_2_4_GHZ_PWR_LEVEL_DBM     5
#define MAX_SUB1_PWR_LEVEL_DBM        13
#define MAX_SUB1_BOOST_PWR_LEVEL_DBM  14

#define ABORT_GRACEFUL          1   // Option for the RF cancel command
#define ABORT_ABRUPT            0   // Option for the RF cancel command

/* Inter-packet intervals for each phy mode in ms*/
#define PKT_INTERVAL_MS_2GFSK   60
#define PKT_INTERVAL_MS_CUSTOM  60
#define PKT_INTERVAL_MS_SLR     80
#define PKT_INTERVAL_MS_LRM     500
#define PKT_INTERVAL_MS_OOK     100
#define PKT_INTERVAL_MS_HSM     50
#define PKT_INTERVAL_MS_BLE     100

#define RF_TX20_ENABLED         0xFFFF // Tx power setting when high PA is in use
#define CENTER_FREQ_EU          0x0364 // Center Frequency 868 MHz
#define CENTER_FREQ_US          0x0393 // Center Frequency 915 MHz

/* IEEE 802.15.4g Header Configuration
 * _S indicates the shift for a given bit field
 * _M indicates the mask required to isolate a given bit field
 */
#define IEEE_HDR_LEN_S          0U
#define IEEE_HDR_LEN_M          0x00FFU
#define IEEE_HDR_CRC_S          12U
#define IEEE_HDR_CRC_M          0x1000U
#define IEEE_HDR_WHTNG_S        11U
#define IEEE_HDR_WHTNG_M        0x0800U
#define IEEE_HDR_CRC_2BYTE      1U
#define IEEE_HDR_CRC_4BYTE      0U
#define IEEE_HDR_WHTNG_EN       1U
#define IEEE_HDR_WHTNG_DIS      0U

#define IEEE_HDR_CREATE(crc, whitening, length) {            \
    (crc << IEEE_HDR_CRC_S | whitening << IEEE_HDR_WHTNG_S | \
    ((length << IEEE_HDR_LEN_S) & IEEE_HDR_LEN_M))           \
}

///***** Variable declarations *****/
static volatile uint32_t nTxPkts = 0;

static volatile bool bPacketTxDone = false;

/*
This interval is dependent on data rate and packet length, and might need to be changed
if any of these parameter changes
*/
uint32_t packetInterval;

static tx_metrics txMetrics = {
    .transmitPowerDbm = 0,
    .dataRateBps      = 0,
    .packetIntervalMs = 0
};

/***** Callback Functions *****/
void txCallback(RCL_Command *cmd, LRF_Events lrfEvents, RCL_Events rclEvents)
{
    if (rclEvents.lastCmdDone)
    {
        gTxCmdDone += 1;

        /* Increment packet count everytime TX is complete */
        nTxPkts ++;
    }
}

/* Reset all the volatile variables */
static void tx_resetVariables(void)
{
    nTxPkts = 0U;
    gTxCmdDone = 0U;
    bPacketTxDone = false;
}

/* Runs the transmitting part of the test application and returns a result. */
TestResult tx_runTxTest(const ApplicationConfig* config)
{
    uint8_t hdrLen = 0;
    RCL_Handle rclHandle;
    uint8_t pktLen = config->payloadLength;

    /* Initialize RCL */
    RCL_init();

    /* Open RCL */
    if(config->rfSetup == RCL_Generic_BLE_1M)
    {
        rclPacketTxCmdGenericTx = &rclPacketTxCmdGenericTx_ble_gen_2;
        rclHandle = RCL_open(&txRclClient, &LRF_config_ble_gen_2);
    }
    else if(config->rfSetup == RCL_Generic_250K_MSK)
    {
        rclPacketTxCmdGenericTx = &rclPacketTxCmdGenericTx_msk_250_kbps_0;
        rclHandle = RCL_open(&txRclClient, &LRF_config_msk_250_kbps_0);
    }
    else
    {
        rclPacketTxCmdGenericTx = &rclPacketTxCmdGenericTx_msk_250_kbps_fec_1;
        rclHandle = RCL_open(&txRclClient, &LRF_config_msk_250_kbps_fec_1);
    }

    /* Set RF frequency */
    rclPacketTxCmdGenericTx->rfFrequency = config->frequencyTable[config->frequency].frequency * FREQUENCY_MHZ_TO_HZ;

    /* Start command as soon as possible */
    rclPacketTxCmdGenericTx->common.scheduling = RCL_Schedule_Now;
    rclPacketTxCmdGenericTx->common.status = RCL_CommandStatus_Idle;

    rclPacketTxCmdGenericTx->config.fsOff = FS_OFF; // Turn off FS

    /* Callback triggers on last command done */
    rclPacketTxCmdGenericTx->common.runtime.callback = txCallback;
    rclPacketTxCmdGenericTx->common.runtime.rclCallbackMask.value = RCL_EventLastCmdDone.value;
    rclPacketTxCmdGenericTx->txPower = LRF_TxPower_Use_Max;

    /* Set RCL TX buffer packet to be packet buffer */
    RCL_Buffer_TxBuffer *txPacket = (RCL_Buffer_TxBuffer *)&packet;

    /* Determine the data rate in bits per seconds */
    txMetrics.dataRateBps = config_dataRateTable_Lut[config->rfSetup];
    if(txMetrics.dataRateBps > 0)
    {
        // Dummy logic to not optimize out config_dataRateTable_Lut
        txMetrics.dataRateBps = txMetrics.dataRateBps;
    }

    uint16_t i;
    for (i = 0U; i < config->packetCount; i++)
    {
        /* If user pressed a button */
        if (menu_isButtonPressed())
        {
            if (RCL_CommandStatus_Idle != rclPacketTxCmdGenericTx->common.status)
            {
                /* Stop command as soon as possible if active*/
                RCL_Command_stop(rclHandle, RCL_StopType_Hard);

                /* Pend on command completion */
                RCL_Command_pend(rclPacketTxCmdGenericTx);
            }

            /* Close RCL */
            RCL_close(rclHandle);

            /* Final update TX screen */
            menu_updateTxScreen(nTxPkts);

            /* Reset all variables */
            tx_resetVariables();

            return (TestResult_Aborted);
        }
        else if (config->packetCount != nTxPkts)
        {
            /* Create packet with random payload */
            uint8_t *txData;
#if (!defined(FIXED_LENGTH_SETUP))
            if((config->rfSetup == RCL_Generic_250K_MSK) || (config->rfSetup == RCL_Generic_250_MSK_FEC))
            {
                hdrLen = 1;
                txData = RCL_TxBuffer_init(txPacket, NUM_PAD_BYTES, hdrLen, pktLen);
                txData[0] = pktLen;
            }
            else if(config->rfSetup == RCL_Generic_BLE_1M)
            {
                hdrLen = 2;
                txData = RCL_TxBuffer_init(txPacket, NUM_PAD_BYTES, hdrLen, pktLen);
                txData[0] = 0;
                txData[1] = pktLen;
            }
            else
            {
				// If you added a new PHY. make sure the header setup is correct
                while(1);
            }
#else
            txData = RCL_TxBuffer_init(txPacket, NUM_PAD_BYTES, hdrLen, pktLen);
#endif

            for (int i = hdrLen; i < pktLen; i++)
            {
                txData[i] = rand();
            }

            /* Set packet to transmit */
            RCL_TxBuffer_put(&(rclPacketTxCmdGenericTx->txBuffers), txPacket);

            rclPacketTxCmdGenericTx->common.status = RCL_CommandStatus_Idle;

            /* Submit command */
            RCL_Command_submit(rclHandle, rclPacketTxCmdGenericTx);

            /* Pend on command completion */
            RCL_Command_pend(rclPacketTxCmdGenericTx);

            usleep(10000);
        }
    }

    /* Close RCL */
    RCL_close(rclHandle);

    /* Final update TX screen */
    menu_updateTxScreen(nTxPkts);

    /* Reset all variables */
    tx_resetVariables();

    return (TestResult_Finished);
}
