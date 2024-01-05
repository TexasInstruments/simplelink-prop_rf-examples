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
#include <unistd.h>
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
/* Packet TX Configuration */
#define MAX_LENGTH              (30U) // Max packet length
#define NUM_DATA_ENTRIES        (2U)  // Number of data entries
#define NUM_PAD_BYTES           (3U)  // Number of pad bytes

/* The index of the length in the TX packet and the length of the header */
#define LEN_INDEX (RCL_REGISTER_FIELD_PBE_GENERIC_RAM_LENCFG_LENPOS/CHAR_BIT)
#define HDR_LEN   (LEN_INDEX + (RCL_REGISTER_FIELD_PBE_GENERIC_RAM_LENCFG_NUMLENBITS/CHAR_BIT))

/* RCL buffer length */
#define BUFF_STRUCT_LENGTH      (2048U)

/* Set Receive timeout to 500ms */
#define RX_TIMEOUT          (4*500000U)

/* Set packet interval to 1000000us or 1000ms */
#define PACKET_INTERVAL     (4*1000000U)

/* Indicates if FS is off */
#define FS_OFF                  (1U)
#define FREQUENCY               (2440000000U)

/* Set Tx Power */
#define TX_POWER (5U)

/* Number of packets to initialize for multi buffer */
#define NUM_OF_PACKETS          (1U)


/***** Variable declarations *****/
/* RCL Commands */
extern RCL_CmdGenericTx rclPacketTxCmdGenericTx;
extern RCL_CmdGenericRx rclPacketRxCmdGenericRx;
RCL_StatsGeneric   stats;              // Statistic command

/* RCL Client used to open RCL */
static RCL_Client  rclClient;

/*create a Txbuffer from tx packet and a multibuffer from the rx packet*/
RCL_Buffer_TxBuffer *txBuffer;
RCL_MultiBuffer *multiBuffer;

/* TX and Rx packets */
uint32_t txPacket[NUM_DATA_ENTRIES][3 + ((MAX_LENGTH + 10)/ 4)];
uint32_t rxPacket[NUM_DATA_ENTRIES][BUFF_STRUCT_LENGTH/4];

/* Indicates if RX packet is not stored */
#define DISCARD_RX_PACKET       (0U)

/* Counters*/
volatile uint32_t sentDone = 0;
volatile uint32_t recPacket = 0;
volatile uint32_t rxTimeout = 0;
volatile uint32_t same = 0;
volatile uint32_t different = 0;

/* Payloads */
uint8_t *txData;

/***** Callback Functions *****/
void echoCallback(RCL_Command *cmd, LRF_Events lrfEvents, RCL_Events rclEvents)
{
    if(rclEvents.lastCmdDone && cmd == (RCL_Command*)&rclPacketTxCmdGenericTx)
    {
        /* success with Tx command */
        sentDone += 1;
        GPIO_toggle(CONFIG_GPIO_GLED);
        GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_OFF);
    }
    if(rclEvents.rxEntryAvail)
    {
        /* successful Rx packet */
        recPacket += 1;

        /* Check the packet against what was transmitted */
        uint8_t status = memcmp(txData - ((RCL_Buffer_DataEntry *)multiBuffer->data)->pad0, ((RCL_Buffer_DataEntry *)multiBuffer->data)->data, MAX_LENGTH);
        if (!status)
        {
            /* Toggle LED1, clear LED2 to indicate RX */
            same += 1;
            GPIO_toggle(CONFIG_GPIO_GLED);
            GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_OFF);
        }
        else
        {
            /* Error Condition: set both LEDs */
            different += 1;
            GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_ON);
            GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_ON);
        }
        RCL_MultiBuffer_clear(multiBuffer);
    }
    else if(rclEvents.lastCmdDone && cmd == (RCL_Command*)&rclPacketRxCmdGenericRx)
    {
        /* Rx Timeout */
        rxTimeout += 1;
        GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_OFF);
        GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_ON);
    }
    if(!rclEvents.rxEntryAvail && !rclEvents.lastCmdDone)
    {
        /* Error Condition: set LED1, clear LED2 */
        GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_ON);
        GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_ON);
    }
}


void *mainThread(void *arg0)
{
    /* Initialize and open RCL */
    RCL_init();

    RCL_Handle rclHandle = RCL_open(&rclClient, &LRF_config);

    /* Configure and write to LEDs */
    GPIO_setConfig(CONFIG_GPIO_GLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_OFF);

    GPIO_setConfig(CONFIG_GPIO_RLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_OFF);

    /* Set RF frequency for Tx command */
    rclPacketTxCmdGenericTx.rfFrequency = FREQUENCY;

    /* Turn off FS */
    rclPacketTxCmdGenericTx.config.fsOff = FS_OFF;

    /* Set Tx Power */
    rclPacketTxCmdGenericTx.txPower.dBm = TX_POWER;

    /* Start Tx command at given time */
    rclPacketTxCmdGenericTx.common.scheduling = RCL_Schedule_AbsTime;
    rclPacketTxCmdGenericTx.common.status = RCL_CommandStatus_Idle;

    /* Callback triggers on last command done */
    rclPacketTxCmdGenericTx.common.runtime.callback = echoCallback;
    rclPacketTxCmdGenericTx.common.runtime.rclCallbackMask.value = RCL_EventLastCmdDone.value;

    /* declare Tx buffer */
    txBuffer = (RCL_Buffer_TxBuffer *)&txPacket;

    /* Set RF frequency for Rx command */
    rclPacketRxCmdGenericRx.rfFrequency = FREQUENCY;

    /* Store Rx packet */
    rclPacketRxCmdGenericRx.config.discardRxPackets = DISCARD_RX_PACKET;

    /* Callback triggers on last command done */
    rclPacketRxCmdGenericRx.common.runtime.callback = echoCallback;
    rclPacketRxCmdGenericRx.common.runtime.rclCallbackMask.value = RCL_EventLastCmdDone.value | RCL_EventRxEntryAvail.value;

    /* Start Rx command as soon as possible */
    rclPacketRxCmdGenericRx.common.scheduling = RCL_Schedule_Now;
    rclPacketRxCmdGenericRx.common.status = RCL_CommandStatus_Idle;

    /* Stop the command after %RX_TIMEOUT ms */
    rclPacketRxCmdGenericRx.common.timing.relGracefulStopTime = RX_TIMEOUT;

    /* Maximum packet length */
    rclPacketRxCmdGenericRx.maxPktLen = MAX_LENGTH;

    /* end after receiving one packet */
    rclPacketRxCmdGenericRx.config.repeated = 0;

    /* Setup generic status command */
    stats = RCL_StatsGeneric_DefaultRuntime();

    /* Set RX command statistics structure */
    rclPacketRxCmdGenericRx.stats = &stats;
    rclPacketRxCmdGenericRx.stats->config.activeUpdate = 1;

    /* Get current time*/
    uint32_t currentTime = RCL_Scheduler_getCurrentTime();

    while(1)
    {
        /* Create packet with random payload */
        txData = RCL_TxBuffer_init(txBuffer, NUM_PAD_BYTES, HDR_LEN, MAX_LENGTH);

        /* Zero out data in header before the length field */
        for (int s = 0; s < LEN_INDEX; s++)
        {
            txData[s] = 0U;
        }

        /* Set the packet length */
        txData[LEN_INDEX] = MAX_LENGTH;

        /* Generate a random payload */
        for (int i = HDR_LEN; i < MAX_LENGTH; i++)
        {
            txData[i] = rand();
        }

        /* Schedule command to start %PACKET_INTERVAL ms later */
        currentTime += PACKET_INTERVAL;
        rclPacketTxCmdGenericTx.common.timing.absStartTime = currentTime;

        /* Set Tx packet to transmit */
        RCL_TxBuffer_put(&rclPacketTxCmdGenericTx.txBuffers, txBuffer);

        /* Submit Tx command */
        RCL_Command_submit(rclHandle, &rclPacketTxCmdGenericTx);

        /* Pend on Tx command completion */
        RCL_Command_pend(&rclPacketTxCmdGenericTx);

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
    }
}
