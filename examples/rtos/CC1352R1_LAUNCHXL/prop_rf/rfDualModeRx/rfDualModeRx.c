/*
 * Copyright (c) 2019, Texas Instruments Incorporated
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
/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/GPIO.h>

/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_ble_mailbox.h)

/* Board Header files */
#include "ti_drivers_config.h"

/* Application Header files */
#include "RFQueue.h"

#include <ti_radio_config.h>

/***** Defines *****/
/*
 * BLE PACKET
 * ________________________________________________________________________________________________________________________
 *|                                 |               |                                 |          |           |             |
 *| PDU HEADER FIELD             2B | ADV ADDR      | PACKET DATA                 27B | CRC   3B | RSSI   1B | STATUS   2B |
 *| ADV TYPE   1B | PACKET LEN   1B |  - BLE4    6B | SERIAL NUMBER   2B | DATA   25B |          |           |             |
 *|                                 |  - BLE5   10B |                                 |          |           |             |
 *|_________________________________|_______________|_________________________________|__________|___________|_____________|
 *
 * PROPRIETARY PACKET
 * _____________________________________________________________
 *|                |                                 |          |
 *| LEN FIELD   1B | PACKET DATA                 27B | CRC   2B |
 *|                | SERIAL NUMBER   2B | DATA   25B |          |
 *|________________|_________________________________|__________|
 *
 */

/* Packet TX Configuration */
#define BLE5_ADV_ADDR_LENGTH             10
#define BLE4_ADV_ADDR_LENGTH             6

#define ADV_NONCONN_IND                  2 //Packet type for BLE4
#define AUX_ADV_IND                      7 //Packet type for BLE5

#define PROP_LEN_FIELD_LENGTH            1
#define BLE_PDU_HEADER_FIELD_LENGTH      2

/* Buffer needs to be large enough for either packet */
#define PACKET_DATA_LENGTH               27
#define PAYLOAD_LENGTH                   (PACKET_DATA_LENGTH + BLE5_ADV_ADDR_LENGTH) //Create buffer that can accept BLE4 or BLE5 packet

#define NUM_DATA_ENTRIES                 2 // NOTE: Only two data entries supported at the moment
#define NUM_APPENDED_BYTES               8 // BLE PDU Header (2 Bytes), CRC (3 Bytes), RSSI (1 Byte), Status (2 Byte)

/***** Variable declarations *****/
static RF_Object rfBleObject;
static RF_Handle rfBleHandle;

static RF_Object rfPropObject;
static RF_Handle rfPropHandle;

/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is aligned to a 4 byte boundary
 * (requirement from the RF core)
 */
#if defined(__TI_COMPILER_VERSION__)
    #pragma DATA_ALIGN (rxDataEntryBuffer, 4);
        static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                 PAYLOAD_LENGTH,
                                                                 NUM_APPENDED_BYTES)];
#elif defined(__IAR_SYSTEMS_ICC__)
    #pragma data_alignment = 4
        static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                 PAYLOAD_LENGTH,
                                                                 NUM_APPENDED_BYTES)];
#elif defined(__GNUC__)
        static uint8_t rxDataEntryBuffer [RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
            PAYLOAD_LENGTH, NUM_APPENDED_BYTES)] __attribute__ ((aligned (4)));
#else
    #error This compiler is not supported.
#endif

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t packetType;
static uint8_t packetLength;
static uint8_t* packetDataPointer;

static uint8_t propPacket[PACKET_DATA_LENGTH];
static uint8_t blePacket[PACKET_DATA_LENGTH];

/***** Function definitions *****/
void rxDoneCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e){
    if (e & RF_EventRxEntryDone)
    {
        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();
        if(h == rfPropHandle)
        {
            /* Handle the packet data, located at &currentDataEntry->data:
             * - Length is the first byte with the current configuration
             * - Data starts from the second byte */
            packetLength      = *(uint8_t*)(&currentDataEntry->data);
            packetDataPointer = (uint8_t*)(&currentDataEntry->data + PROP_LEN_FIELD_LENGTH);

            /* Toggle LED2, clear LED1 to indicate Prop RX */
            memcpy(propPacket, packetDataPointer, packetLength);
            GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_ON);
            GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_OFF);
        }
        else
        {
            /* Handle the packet data, located at &currentDataEntry->data:
             * - First byte of the PDU will indicate packet type (BLE4 or BLE5)
             * - Total length (including the length of the address) is the second byte of the PDU header
             * - Data starts after the PDU header and ble address field */
            packetType        = (*(uint8_t*)(&currentDataEntry->data));

            if(packetType == ADV_NONCONN_IND) // BLE4 packet received
            {
                packetLength      = (*(uint8_t*)(&currentDataEntry->data + BLE_PDU_HEADER_FIELD_LENGTH - 1)) - BLE4_ADV_ADDR_LENGTH;
                packetDataPointer = (uint8_t*)(&currentDataEntry->data + BLE4_ADV_ADDR_LENGTH + BLE_PDU_HEADER_FIELD_LENGTH);
            }
            else // BLE5 packet received
            {
                packetLength      = (*(uint8_t*)(&currentDataEntry->data + BLE_PDU_HEADER_FIELD_LENGTH - 1)) - BLE5_ADV_ADDR_LENGTH;
                packetDataPointer = (uint8_t*)(&currentDataEntry->data + BLE5_ADV_ADDR_LENGTH + BLE_PDU_HEADER_FIELD_LENGTH);
            }


            memcpy(blePacket, packetDataPointer, packetLength);
            int16_t status = memcmp(blePacket, propPacket, packetLength);
            if(status == 0)
            {
                /* Toggle LED1, clear LED2 to indicate ble packet received matches */
                GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_OFF);
                GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_ON);
            }
            else
            {
                /* Set both LEDs to indicate ble packet received does not match */
                GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_ON);
                GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_ON);
            }
        }
        RFQueue_nextEntry();
    }
}

void *mainThread(void *arg0){
    uint16_t cmdStatus;
    RF_EventMask terminationReason;

    GPIO_setConfig(CONFIG_GPIO_RLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_GLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_OFF);

   /* Data queue should be large enough for either packet */
   if( RFQueue_defineQueue(&dataQueue,
                               rxDataEntryBuffer,
                               sizeof(rxDataEntryBuffer),
                               NUM_DATA_ENTRIES,
                               PAYLOAD_LENGTH + NUM_APPENDED_BYTES))
   {
       /* Failed to allocate space for all data entries */
       GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_ON);
       GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_ON);
       while(1);
   }

   /* Initialize multimode scheduling params
    * - Params shared between rf drivers since commands are synchronous
    * - Ignore end time
    * - Priority should not affect reception
    */
   RF_ScheduleCmdParams schParams;
   schParams.endTime = 0;
   // TODO: Need to replace with updated parameters from RFLIB-107
   //schParams.priority = RF_PriorityNormal;
   schParams.allowDelay = RF_AllowDelayAny;
   schParams.activityInfo = 0;
   schParams.duration = 0;
   schParams.endType = RF_EndNotSpecified;
   schParams.startTime = 0;
   schParams.startType = RF_StartNotSpecified;

   /* Initialize Prop RF Driver */
   RF_Params rfPropParams;
   RF_Params_init(&rfPropParams);

   /* Set mode for multiple clients */
   RF_prop.rfMode = RF_MODE_MULTIPLE;

   RF_cmdPropRx.pQueue = &dataQueue;
   RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;  /* Discard ignored packets from Rx queue */
   RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;   /* Discard packets with CRC error from Rx queue */
   RF_cmdPropRx.maxPktLen = PACKET_DATA_LENGTH;        /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
   RF_cmdPropRx.pktConf.bRepeatOk = 0;
   RF_cmdPropRx.pktConf.bRepeatNok = 0;

   /* Request access to the prop radio and
    * - Radio is not powered on by RF_open
    * - RF_cmdFs will power on the radio to cache the frequency settings
    */
#if defined(DeviceFamily_CC26X0R2)
   rfPropHandle = RF_open(&rfPropObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfPropParams);
#else
   rfPropHandle = RF_open(&rfPropObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfPropParams);
#endif// DeviceFamily_CC26X0R2
   (void)RF_scheduleCmd(rfPropHandle, (RF_Op*)&RF_cmdFs, &schParams, NULL, 0);

   /* Initialize BLE RF Driver */
   RF_Params rfBleParams;
   RF_Params_init(&rfBleParams);

   /* Set mode for multiple clients */
   RF_modeBle.rfMode = RF_MODE_MULTIPLE;

   RF_ble_cmdBleGenericRx.pParams->pRxQ = &dataQueue;
   RF_ble_cmdBleGenericRx.pParams->bRepeat = 0;
   RF_ble_cmdBleGenericRx.pParams->rxConfig.bAutoFlushCrcErr = 1;

   /* Request access to the bleradio and
    * - RF_ble_cmdFs does not need to run unless no channel is specified (0xFF)
    * - Channel 17 (0x8C) is used by default
    */
   rfBleHandle = RF_open(&rfBleObject, &RF_modeBle, (RF_RadioSetup*)&RF_ble_cmdRadioSetup, &rfBleParams);

   while(1)
   {
       terminationReason = RF_runScheduleCmd(rfPropHandle, (RF_Op*)&RF_cmdPropRx,
                                                 &schParams, &rxDoneCallback,
                                                 RF_EventRxEntryDone);

       if(terminationReason & RF_EventCmdPreempted)
       {
           // It is possible for a scheduled command to be preempted by another
           // higher priority command. In this case the RF driver will either
           // cancel/abort/stop the preempted command and return the appropriate
           // event flag. Additionally, the command preempted event flag is also set.

       }

       // Mask off the RF_EventCmdPreempted bit to allow further processing
       // in the switch-case block
       switch (terminationReason & ~(RF_EventCmdPreempted))
       {
           case RF_EventLastCmdDone:
               // A stand-alone radio operation command or the last radio
               // operation command in a chain finished.
               break;
           case RF_EventCmdCancelled:
               // Command cancelled before it was started; it can be caused
               // by RF_cancelCmd() or RF_flushCmd().
               break;
           case RF_EventCmdAborted:
               // Abrupt command termination caused by RF_cancelCmd() or
               // RF_flushCmd().
               break;
           case RF_EventCmdStopped:
               // Graceful command termination caused by RF_cancelCmd() or
               // RF_flushCmd().
               break;
           default:
               // Uncaught error event
               while(1);
       }

       cmdStatus = ((volatile RF_Op*)&RF_cmdPropRx)->status;

       switch(cmdStatus)
       {
           case PROP_DONE_OK:
               // Packet received with CRC OK
               break;
           case PROP_DONE_RXERR:
               // Packet received with CRC error
               break;
           case PROP_DONE_RXTIMEOUT:
               // Observed end trigger while in sync search
               break;
           case PROP_DONE_BREAK:
               // Observed end trigger while receiving packet when the command is
               // configured with endType set to 1
               break;
           case PROP_DONE_ENDED:
               // Received packet after having observed the end trigger; if the
               // command is configured with endType set to 0, the end trigger
               // will not terminate an ongoing reception
               break;
           case PROP_DONE_STOPPED:
               // received CMD_STOP after command started and, if sync found,
               // packet is received
               break;
           case PROP_DONE_ABORT:
               // Received CMD_ABORT after command started
               break;
           case PROP_ERROR_RXBUF:
               // No RX buffer large enough for the received data available at
               // the start of a packet
               break;
           case PROP_ERROR_RXFULL:
               // Out of RX buffer space during reception in a partial read
               break;
           case PROP_ERROR_PAR:
               // Observed illegal parameter
               break;
           case PROP_ERROR_NO_SETUP:
               // Command sent without setting up the radio in a supported
               // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
               break;
           case PROP_ERROR_NO_FS:
               // Command sent without the synthesizer being programmed
               break;
           case PROP_ERROR_RXOVF:
               // RX overflow observed during operation
               break;
           default:
               // Uncaught error event - these could come from the
               // pool of states defined in rf_mailbox.h
               while(1);
       }

      /* Switch from PROP -> BLE client */
      terminationReason = RF_runScheduleCmd(rfBleHandle, (RF_Op*)&RF_ble_cmdBleGenericRx,
                                     &schParams, &rxDoneCallback,
                                     RF_EventRxEntryDone);

      if(terminationReason & RF_EventCmdPreempted)
      {
          // It is possible for a scheduled command to be preempted by another
          // higher priority command. In this case the RF driver will either
          // cancel/abort/stop the preempted command and return the appropriate
          // event flag. Additionally, the command preempted event flag is also set.

      }

      // Mask off the RF_EventCmdPreempted bit to allow further processing
      // in the switch-case block
      switch (terminationReason & ~(RF_EventCmdPreempted))
      {
          case RF_EventLastCmdDone:
              // A stand-alone radio operation command or the last radio
              // operation command in a chain finished.
              break;
          case RF_EventCmdCancelled:
              // Command cancelled before it was started; it can be caused
              // by RF_cancelCmd() or RF_flushCmd().
              break;
          case RF_EventCmdAborted:
              // Abrupt command termination caused by RF_cancelCmd() or
              // RF_flushCmd().
              break;
          case RF_EventCmdStopped:
              // Graceful command termination caused by RF_cancelCmd() or
              // RF_flushCmd().
              break;
          default:
              // Uncaught error event
              while(1);
      }

      cmdStatus = RF_ble_cmdBleGenericRx.status;

      switch(cmdStatus)
      {
          case BLE_DONE_OK:
              // Packet transmitted successfully
              break;
          case BLE_DONE_STOPPED:
              // received CMD_STOP while transmitting packet and finished
              // transmitting packet
              break;
          case BLE_DONE_ABORT:
              // Received CMD_ABORT while transmitting packet
              break;
          case BLE_ERROR_PAR:
              // Observed illegal parameter
              break;
          case BLE_ERROR_NO_SETUP:
              // Command sent without setting up the radio in a supported
              // mode using CMD_BLE_RADIO_SETUP or CMD_RADIO_SETUP
              break;
          case BLE_ERROR_NO_FS:
              // Command sent without the synthesizer being programmed
              break;
          case BLE_ERROR_TXUNF:
              // TX underflow observed during operation
              break;
          default:
              // Uncaught error event - these could come from the
              // pool of states defined in rf_mailbox.h
              while(1);
      }
   }
}
