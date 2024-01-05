/******************************************************************************
 Group: CMCU LPRF
 Target Device: cc23xx

 ******************************************************************************
 
 Copyright (c) 2023, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>
#include <string.h>
#include <limits.h>

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
#define NUM_PAD_BYTES           (3U)  // Number of pad bytes

/* The index of the length in the TX packet and the length of the header */
#define LEN_INDEX (RCL_REGISTER_FIELD_PBE_GENERIC_RAM_LENCFG_LENPOS/CHAR_BIT)
#define HDR_LEN   (LEN_INDEX + (RCL_REGISTER_FIELD_PBE_GENERIC_RAM_LENCFG_NUMLENBITS/CHAR_BIT))

/* Set Transmit (echo) delay to 100ms */
#define TX_DELAY             (4*100000U)

/* RCL buffer length */
#define BUFF_STRUCT_LENGTH      (2048U)

/* Indicates if FS is off */
#define FS_OFF                  (1U)

/* Indicates if RX packet is not stored */
#define DISCARD_RX_PACKET       (0U)  // 0: Store received packets in rxBuffers
                                      // 1: Discard packets
#define FREQUENCY               (2440000000U) /* RF Frequency (Hz) to program */

/* Number of packets to initialize for multi buffer */
#define NUM_OF_PACKETS          (1U)

/***** Variable Declarations *****/
/* RCL Commands */
extern RCL_CmdGenericTx rclPacketTxCmdGenericTx;
extern RCL_CmdGenericRx rclPacketRxCmdGenericRx;
RCL_StatsGeneric    stats;              // Statistic command

/* RCL Client used to open RCL */
static RCL_Client  rclClient;

/*create a Txbuffer from tx packet and a multibuffer from the rx packet*/
RCL_Buffer_TxBuffer *txBuffer;
RCL_MultiBuffer *multiBuffer;

/* Counters for RCL event callback */
volatile uint32_t txCounter = 0;
volatile uint32_t rxCounter = 0;

/* TX and Rx packets */
uint32_t txPacket[NUM_DATA_ENTRIES][3 + ((MAX_LENGTH + 10)/ 4)];
uint32_t rxPacket[NUM_DATA_ENTRIES][BUFF_STRUCT_LENGTH/4];

/***** Callback Functions *****/
void echoCallback(RCL_Command *cmd, LRF_Events lrfEvents, RCL_Events rclEvents)
{

    if (rclEvents.lastCmdDone && cmd == (RCL_Command*)&rclPacketTxCmdGenericTx)
    {
        /* success with Tx command */
        txCounter += 1;
        GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_OFF);
        GPIO_toggle(CONFIG_GPIO_RLED);
    }
    if (rclEvents.rxEntryAvail)
    {
        /* successful Rx packet */
        rxCounter += 1;
        GPIO_toggle(CONFIG_GPIO_RLED);
        GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_OFF);

        /* Copy Rx payload to transmit */
        txBuffer = (RCL_Buffer_TxBuffer *)&txPacket;
        uint8_t *txData;
        txData = RCL_TxBuffer_init(txBuffer, NUM_PAD_BYTES, HDR_LEN, MAX_LENGTH);
        RCL_Buffer_DataEntry *pDataEntry = RCL_MultiBuffer_RxEntry_get((List_List *) &multiBuffer, NULL);
        memcpy(txData - pDataEntry->pad0, pDataEntry->data, MAX_LENGTH);

        /* Clear Multi Buffer*/
        RCL_MultiBuffer_clear(multiBuffer);
    }
    if(!rclEvents.rxEntryAvail && !rclEvents.lastCmdDone) // any uncaught event
    {
        // Error Condition: set LED1, clear LED2
        GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_ON);
        GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_OFF);
    }
}

/***** Function definitions *****/
void *mainThread(void *arg0)
{
    /* Initialize and open RCL */
    RCL_init();

    RCL_Handle rclHandle = RCL_open(&rclClient, &LRF_config);

    /* Configure and write to LEDs */
    GPIO_setConfig(CONFIG_GPIO_RLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_OFF);

    GPIO_setConfig(CONFIG_GPIO_GLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_OFF);

    /* Set RF frequency for Rx command */
    rclPacketRxCmdGenericRx.rfFrequency = FREQUENCY;

    /* Start command as soon as possible */
    rclPacketRxCmdGenericRx.common.scheduling = RCL_Schedule_Now;
    rclPacketRxCmdGenericRx.common.status = RCL_CommandStatus_Idle;

    /* Turn off FS */
    rclPacketRxCmdGenericRx.config.fsOff = FS_OFF;

    /* Set Tx Power */
    rclPacketRxCmdGenericRx.config.discardRxPackets = DISCARD_RX_PACKET;

    /* Callback triggers on Rx command done or packet received */
    rclPacketRxCmdGenericRx.common.runtime.callback = echoCallback;
    rclPacketRxCmdGenericRx.common.runtime.rclCallbackMask.value = RCL_EventLastCmdDone.value |  RCL_EventRxEntryAvail.value;

    /* Maximum packet length */
    rclPacketRxCmdGenericRx.maxPktLen = MAX_LENGTH;

    /* end after receiving one packet */
    rclPacketRxCmdGenericRx.config.repeated = 0;

    /* Setup generic status command */
    stats = RCL_StatsGeneric_DefaultRuntime();

    /* Set RX command statistics structure */
    rclPacketRxCmdGenericRx.stats = &stats;
    rclPacketRxCmdGenericRx.stats->config.activeUpdate = 1;

    /* Set RF frequency for Tx command */
    rclPacketTxCmdGenericTx.rfFrequency = FREQUENCY;

    /* Start Tx command as soon as possible */
    rclPacketTxCmdGenericTx.common.scheduling = RCL_Schedule_AbsTime;
    rclPacketTxCmdGenericTx.common.status = RCL_CommandStatus_Idle;

    /* Callback triggers on Tx command done */
    rclPacketTxCmdGenericTx.common.runtime.callback = echoCallback;
    rclPacketTxCmdGenericTx.common.runtime.rclCallbackMask.value = RCL_EventLastCmdDone.value;

    while(1)
    {
        /* Initialize multi-buffer to allow RCL to store RX packet */
        for(int i = 0; i < NUM_OF_PACKETS; i++)
        {
            multiBuffer = (RCL_MultiBuffer *) rxPacket[i];
            RCL_MultiBuffer_init(multiBuffer, BUFF_STRUCT_LENGTH);
            RCL_MultiBuffer_put(&rclPacketRxCmdGenericRx.rxBuffers, multiBuffer);
        }

        /* Submit Rx command */
        RCL_Command_submit(rclHandle, &rclPacketRxCmdGenericRx);

        /* Pend on Rx command completion */
        RCL_Command_pend(&rclPacketRxCmdGenericRx);

        /* Set Tx packet to transmit */
        RCL_TxBuffer_put(&rclPacketTxCmdGenericTx.txBuffers, txBuffer);
        rclPacketTxCmdGenericTx.common.timing.absStartTime = RCL_Scheduler_getCurrentTime() + TX_DELAY;

        /* Submit Tx command */
        RCL_Command_submit(rclHandle, &rclPacketTxCmdGenericTx);

        /* Pend on Tx command completion */
        RCL_Command_pend(&rclPacketTxCmdGenericTx);
    }
}
