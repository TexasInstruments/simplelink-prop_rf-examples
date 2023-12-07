/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
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

#include <rfClient.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */ 
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/display/Display.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/GPIO.h>
#include <ti/display/DisplayExt.h>

/* Board Header files */
#include "ti_drivers_config.h"
#include <ti_radio_config.h>

/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)
#include DeviceFamily_constructPath(driverlib/cpu.h)

/* Application Header files */
#include "oad/native_oad/oad_client.h"
#include "oad/native_oad/oad_storage.h"
#include "oad/native_oad/oad_protocol.h"
#include "oad/native_oad/oad_image_header_app.h"
#include "radio/radio.h"

/* This is on-chip exclusive */
#ifdef OAD_ONCHIP
#include "clientStorage.h"
#endif

/* Display driver handles */
static Display_Handle hDisplaySerial;

/***** Variable declarations *****/
static char currentFWVersion[4];
static bool oadInProgress = 0;
static uint16_t oadBlock = 0;
static uint16_t oadTotalBlocks = 0;
static uint32_t oadRetries = 0;
static int8_t oadStatus = -1;
uint8_t * ptrAppBuffer;

/* Event variables */
Event_Struct clientEvent;  /* not static so you can see in ROV */
static Event_Handle clientEventHandle;

/*
 *  ======== main ========
 */
void *mainThread(void *arg0)
{
    Display_init();

    /* Setup buttons 1, buttons 2 and green LED */
    GPIO_setInitialization();

    /* Setup display settings */
    Display_setInitialization();

    /* Initialize the radio */
    ptrAppBuffer = Radio_Init(rfClient_postNewOADMsg);

    /* OAD Init */
    OAD_Init();

    /* Enter receive mode for a single packet*/
    Radio_rxPacket();

#ifdef OAD_P_APP

    clientStorage_init();

    /* Check NVS for connection data. If found load that connection */
    if (clientStorage_verifyStorage())
    {
        OADProtocol_sendOadResetRsp((void*)0xFF);
    }

#endif /* OAD_P_APP */

    while (1)
    {
        /* Wait for event */
        uint32_t events = Event_pend(clientEventHandle, 0, CLIENT_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* Printing general OAD events */
        if (events & CLIENT_EVENT_STATUS_UPDATE)
        {
            /* Update display */
            rfClient_printUpdate();
        }

        /* OAD message is unknown so call parse to decode*/
        if (events & CLIENT_EVENT_NEW_OAD_MSG)
        {
            OADProtocol_ParseIncoming((void*)0xFF, ptrAppBuffer);
        }

        /* Handle OAD MSG Req */
        if (events & CLIENT_EVENT_OAD_REQ)
        {
            OADClient_processEvent(&events);
        }
    }

}

#if defined(OAD_U_APP) && !defined(MCUBOOT)
/*!
 This function is called when U-App receives OAD reset request
 */
void rfClient_resetUserApp(void* pDstAddr)
{
    (void) pDstAddr;
    clientStorage_Status status;

    /* Initialize the storage */
    clientStorage_init();

    /* Clear a sector in NVS */
    clientStorage_eraseSector();

    /* Write a data message into NVS*/
    status = clientStorage_writeStartSeq();

    if (status != clientStorage_Success)
    {
        /* If the write was not successful then clear the entire sector */
        clientStorage_eraseSector();
    }

    Display_printf(hDisplaySerial, 0, 0, "\033[2J \033[0;0HPerform reset now");

    /* Invalidate OAD image header so the bim will boot into the P-App */
    OADClient_invalidateHeader();

    /* Reset device */
    SysCtrlSystemReset();
}
#endif

void GPIO_setInitialization(void)
{
    /* Setup green LED */
    GPIO_setConfig(CONFIG_GPIO_GLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_OFF);

    /* Setup red LED */
    GPIO_setConfig(CONFIG_GPIO_RLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_OFF);
}

void Display_setInitialization(void)
{
    /* Initialize the UART terminal*/
    Display_Params params;
    Display_Params_init(&params);
    params.lineClearMode = DISPLAY_CLEAR_BOTH;
    hDisplaySerial = Display_open(Display_Type_UART, &params);

    OADImgHdr_getFWVersion(currentFWVersion);

    Display_printf(hDisplaySerial, 0, 0, "\033[2J \033[0;0HMain Menu - Client v%s",
                    currentFWVersion);
    Display_printf(hDisplaySerial, 0, 0, "Waiting for Server...");
}

void OAD_Init(void)
{
    OADClient_Params_t oadclientParams = {0};

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&clientEvent, &eventParam);
    clientEventHandle = Event_handle(&clientEvent);

    oadclientParams.eventHandle = clientEventHandle;
    oadclientParams.oadReqEventBit = CLIENT_EVENT_OAD_REQ;

    oadclientParams.oadRspPollEventBit = 0;
    OADClient_open(&oadclientParams);
}

void rfClient_printUpdate(void)
{
    if (!oadInProgress)
    {
        /* print to LCD */
        Display_clear(hDisplaySerial);
    }

    if (oadInProgress)
    {
        Display_printf(hDisplaySerial, 0, 0, "\033[2J \033[0;0HOAD Block: %d of %d", oadBlock, oadTotalBlocks);
        Display_printf(hDisplaySerial, 0, 0, "OAD Block Retries: %d", oadRetries);
    }

    else if (oadStatus != -1)
    {
        switch (((OADStorage_Status_t)oadStatus))
        {
        case OADStorage_Status_Success:
            Display_printf(hDisplaySerial, 0, 0, "OAD: completed successfully");

            /* close Serial display for UART Driver */
            Display_close(hDisplaySerial);

            break;
        case OADStorage_CrcError:
            Display_printf(hDisplaySerial, 0, 0, "OAD: CRC failed");
            break;
        case OADStorage_Failed:
            Display_printf(hDisplaySerial, 0, 0, "OAD: aborted");
            break;
        default:
            Display_printf(hDisplaySerial, 0, 0, "OAD: error");
            break;
        }
    }
}

void rfClient_postNewOADMsg(void)
{
    /* Post an event when it receives a OAD message */
    Event_post(clientEventHandle, CLIENT_EVENT_NEW_OAD_MSG);
}

void rfClient_displayOadBlockUpdate(uint16_t newOadBlock, uint16_t oadBNumBlocks, uint32_t retries)
{
    /* Once the OAD progress starts
     * this condition will run once */
    if(!oadInProgress)
    {
        oadInProgress = true;
    }

    /* Handle all block, totalblock and retries information */
    oadBlock = newOadBlock;
    oadTotalBlocks = oadBNumBlocks;
    oadRetries = retries;

    /* Post an event to update these values on the terminal */
    Event_post(clientEventHandle, CLIENT_EVENT_STATUS_UPDATE);
}

void rfClient_displayOadStatusUpdate(OADStorage_Status_t status)
{
    /* OAD progress stops so set the flag to be false */
    oadInProgress = false;
    oadStatus = (int8_t) status;

    /* Post event to see end result of OAD transfer */
    Event_post(clientEventHandle, CLIENT_EVENT_STATUS_UPDATE);
}
