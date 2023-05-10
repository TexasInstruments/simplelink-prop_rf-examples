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
/* Standard C Libraries */
#include <stdlib.h>

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/GPIO.h>

/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_ble_mailbox.h)

/* Board Header files */
#include "ti_drivers_config.h"
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
#if defined(DeviceFamily_CC13X2) || defined(DeviceFamily_CC13X2X7) || defined(DeviceFamily_CC13X4)
#define EXTENDED_HEADER_LENGTH            9
#endif

/* Packet TX Configuration */
#define PACKET_DATA_LENGTH                    27 /* Packet for BLE5 cannot exceed 37 with header (10) */
#define PACKET_INTERVAL      RF_convertMsToRatTicks(500) /* Set packet interval to 500ms */

/***** Variable declarations *****/
static RF_Object rfBleObject;
static RF_Handle rfBleHandle;

static RF_Object rfPropObject;
static RF_Handle rfPropHandle;

static uint8_t packet[PACKET_DATA_LENGTH];
static uint16_t seqNumber;

/* Packet TX struct for BLE5 */
#if defined(DeviceFamily_CC13X2) || defined(DeviceFamily_CC13X2X7) || defined(DeviceFamily_CC13X4)
rfc_ble5ExtAdvEntry_t ble5ExtAdvPacket;
#endif

/***** Function definitions *****/
static void txDoneCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventLastCmdDone)
    {
        /* Successful TX */
        if (h == rfPropHandle)
        {
            /* Toggle LED2, clear LED1 to indicate Prop TX */
            GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_ON);
            GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_OFF);

        }
        else
        {
            /* Toggle LED1, clear LED2 to indicate Ble TX */
            GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_ON);
        }
    }
    else
    {
        /* Error Condition: set both LEDs */
        GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_ON);
        GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_ON);
    }
}

void *mainThread(void *arg0)
{
    uint32_t curtime;
    uint32_t cmdStatus;
    RF_EventMask terminationReason;

    GPIO_setConfig(CONFIG_GPIO_RLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_GLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_OFF);

    /* Initialize multimode scheduling params
     * - Params shared between rf drivers since commands are synchronous
     * - Ignore end time
     * - Priority should not affect transmission
     */
    RF_ScheduleCmdParams schParams;
    schParams.endTime = 0;
    // TODO: Need to replace with updated parameters from RFLIB-107
    // schParams.priority = RF_PriorityNormal;
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

    RF_cmdPropTx.pktLen = PACKET_DATA_LENGTH;
    RF_cmdPropTx.pPkt = packet;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_ABSTIME;
    RF_cmdPropTx.startTrigger.pastTrig = 1;
    RF_cmdPropTx.startTime = 0;

    /* Request access to the prop radio and
     * - Radio is not powered on by RF_open
     * - RF_cmdFs will power on the radio to cache the frequency settings
     */
    rfPropHandle = RF_open(&rfPropObject, &RF_prop,
                           (RF_RadioSetup*) &RF_cmdPropRadioDivSetup,
                           &rfPropParams);
    (void)RF_scheduleCmd(rfPropHandle, (RF_Op*) &RF_cmdFs, &schParams, NULL, 0);

    /* Initialize BLE RF Driver */
    RF_Params rfBleParams;
    RF_Params_init(&rfBleParams);

    /* Set mode for multiple clients */
    RF_modeBle.rfMode = RF_MODE_MULTIPLE;

    /* Request access to the ble radio */
#if defined(DeviceFamily_CC13X2) || defined(DeviceFamily_CC13X2X7) || defined(DeviceFamily_CC13X4)
    RF_ble_cmdBle5AdvAux.pParams->pAdvPkt = (uint8_t *)&ble5ExtAdvPacket;
    ble5ExtAdvPacket.extHdrInfo.length = EXTENDED_HEADER_LENGTH;
    ble5ExtAdvPacket.advDataLen = PACKET_DATA_LENGTH;
    ble5ExtAdvPacket.pAdvData = packet;
    RF_ble_cmdBle5AdvAux.startTrigger.triggerType = TRIG_ABSTIME;
    RF_ble_cmdBle5AdvAux.startTrigger.pastTrig = 1;
    RF_ble_cmdBle5AdvAux.startTime = 0;
    RF_ble_cmdBle5AdvAux.txPower = 0;
#else
    RF_ble_cmdBleAdvNc.pParams->advLen = PACKET_DATA_LENGTH;
    RF_ble_cmdBleAdvNc.pParams->pAdvData = packet;
    RF_ble_cmdBleAdvNc.startTrigger.triggerType = TRIG_ABSTIME;
    RF_ble_cmdBleAdvNc.startTrigger.pastTrig = 1;
    RF_ble_cmdBleAdvNc.startTime = 0;
#endif

    /* Request access to the bleradio and
     * - RF_ble_cmdFs does not need to run unless no channel is specified (0xFF)
     * - Channel 17 (0x8C) is used by default
     */
    rfBleHandle = RF_open(&rfBleObject, &RF_modeBle,
                          (RF_RadioSetup*)&RF_ble_cmdRadioSetup, &rfBleParams);

    /* Get current time */
    curtime = RF_getCurrentTime();

    while (1)
    {
        /* Create packet with incrementing sequence number and random payload */
        packet[0] = (uint8_t) (seqNumber >> 8);
        packet[1] = (uint8_t) (seqNumber++);
        uint8_t i;
        for (i = 2; i < PACKET_DATA_LENGTH; i++)
        {
            packet[i] = rand();
        }

        /* Set absolute TX time to utilize automatic power management */
        curtime += PACKET_INTERVAL;

        /* Transmit prop packet */
        RF_cmdPropTx.startTime = curtime;

        terminationReason = RF_runScheduleCmd(rfPropHandle, (RF_Op*) &RF_cmdPropTx,
                                      &schParams, txDoneCallback, 0);

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
                while (1);
        }

        cmdStatus = ((volatile RF_Op*) &RF_cmdPropTx)->status;

        switch (cmdStatus)
        {
            case PROP_DONE_OK:
                // Packet transmitted successfully
                break;
            case PROP_DONE_STOPPED:
                // received CMD_STOP while transmitting packet and finished
                // transmitting packet
                break;
            case PROP_DONE_ABORT:
                // Received CMD_ABORT while transmitting packet
                break;
            case PROP_ERROR_PAR:
                // Observed illegal parameter
                break;
            case PROP_ERROR_NO_SETUP:
                // Command sent without setting up the radio in a supported
                // mode using CMD_BLE_RADIO_SETUP or CMD_RADIO_SETUP
                break;
            case PROP_ERROR_NO_FS:
                // Command sent without the synthesizer being programmed
                break;
            case PROP_ERROR_TXUNF:
                // TX underflow observed during operation
                break;
            default:
                // Uncaught error event - these could come from the
                // pool of states defined in rf_mailbox.h
                while (1);
        }

        /* Transmit BLE packet */
        /* Set absolute TX time to utilize automatic power management */
        curtime += PACKET_INTERVAL;

        /* Switch from PROP -> BLE client */
#if (defined DeviceFamily_CC13X2) || defined(DeviceFamily_CC13X2X7) || defined(DeviceFamily_CC13X4)
        RF_ble_cmdBle5AdvAux.startTime = curtime;
        terminationReason = RF_runScheduleCmd(rfBleHandle, (RF_Op*)&RF_ble_cmdBle5AdvAux,
                                              &schParams, txDoneCallback, 0);
#else
        RF_ble_cmdBleAdvNc.startTime = curtime;
        terminationReason = RF_runScheduleCmd(rfBleHandle, (RF_Op*)&RF_ble_cmdBleAdvNc,
                                              &schParams, txDoneCallback, 0);
#endif

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
                while (1);
        }

#if (defined DeviceFamily_CC13X2) || defined(DeviceFamily_CC13X2X7) || defined(DeviceFamily_CC13X4)
        cmdStatus = RF_ble_cmdBle5AdvAux.status;
#else
        cmdStatus = RF_ble_cmdBleAdvNc.status;
#endif

        switch (cmdStatus)
        {
            case BLE_DONE_OK:
                // Packet transmitted successfully
                break;
            case BLE_DONE_STOPPED:
                // received CMD_STOP while transmitting packet and finished
                // transmitting packetoi
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
                while (1);
        }
    }
}
