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
#include <oad/native_oad/oad_server.h>
#include <rfServer.h>
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
#include <ti/display/DisplayExt.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/rf/RF.h>

/* Board Header files */
#include "ti_drivers_config.h"
#include <ti_radio_config.h>

/* Application Header files */
#if (defined(DeviceFamily_CC13X2) || defined(DeviceFamily_CC26X2) || \
     defined(DeviceFamily_CC13X2X7)  || defined(DeviceFamily_CC26X2X7) || \
     defined(DeviceFamily_CC13X1)   || defined(DeviceFamily_CC26X1) || \
     defined(DeviceFamily_CC13X4)   || defined(DeviceFamily_CC26X4))
#include "common/cc26xx/oad/ext_flash_layout.h"
#else
#include "oad/native_oad/ext_flash_layout.h"
#endif

#include "oad/native_oad/oad_storage.h"
#include "radio/radio.h"

/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Display driver handles */
static Display_Handle hDisplaySerial;

/* Event handles */
Event_Struct serverEvent;  /* not static so you can see in ROV */
static Event_Handle serverEventHandle;

/***** Variable declarations *****/
static char* selectedActionStr[] =
{
    "Update available FW",
    "Send FW Ver Req",
    "Update client FW",
};
static uint16_t oadBlock = 0;
static uint16_t totalBlocks = 0;
static char availableFwVersion[OADProtocol_FW_VERSION_STR_LEN] = "unknown";
static char currentClientFwVersion[OADProtocol_FW_VERSION_STR_LEN] = "unknown";
static int n = sizeof(selectedActionStr)/sizeof(selectedActionStr[0]);
static uint8_t selectedAction = 0;
static Server_Actions_t ChosenAction = Server_Actions_UpdateAvailableFw;
static NodeOadStatus_t nodeFwOadStatus = NodeOadStatus_NotInProgress;
static bool availableFwUpdateInProgress = false;
uint8_t * ptrAppBuffer;

/*
 *  ======== main ========
 */
void *mainThread(void *arg0)
{
    /* Setup uart terminal display */
    Display_init();

    /* Setup buttons 1, buttons 2 and LEDs */
    GPIO_setInitialization();

    /* Initialize the radio */
    ptrAppBuffer = Radio_Init(rfServer_postNewOadMsg);

    /* OAD Init */
    OAD_Init();

#if (defined(DeviceFamily_CC13X2) || defined(DeviceFamily_CC26X2) || \
     defined(DeviceFamily_CC13X2X7) || defined(DeviceFamily_CC26X2X7) || \
     defined(DeviceFamily_CC13X1)   || defined(DeviceFamily_CC26X1) || \
     defined(DeviceFamily_CC13X4)   || defined(DeviceFamily_CC26X4))
    OADStorage_imgIdentifyPld_t remoteAppImageId;

    OADStorage_init();

    /* Get available FW version*/
    if(OADStorage_imgIdentifyRead(OAD_IMG_TYPE_USR_BEGIN, &remoteAppImageId) != 0)
    {
        System_sprintf((xdc_Char*)availableFwVersion,"rfClient sv:%c%c%c%c bv:%02x",
                       remoteAppImageId.softVer[3],
                       remoteAppImageId.softVer[2],
                       remoteAppImageId.softVer[1],
                       remoteAppImageId.softVer[0],
                       remoteAppImageId.bimVer);
    }

    OADStorage_close();
#else
        OADTarget_ImgHdr_t remoteAppImageHdr;

    /* get Available FW version*/
    OADTarget_getImageHeader(EFL_OAD_IMG_TYPE_REMOTE_APP, &remoteAppImageHdr);
    if ((remoteAppImageHdr.ver != 0xFFFF) || (remoteAppImageHdr.ver == 0))
    {
        System_sprintf((xdc_Char*)availableFwVersion,"rfClient v%02d.%02d.00", ((remoteAppImageHdr.ver & 0xFF00)>>8), (remoteAppImageHdr.ver & 0xFF));
    }
#endif

    /* Setup display settings */
    Display_setInitialization();

    while (1)
    {
        /* Wait for event */
        uint32_t events = Event_pend(serverEventHandle, 0, SERVER_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* Choose selected action from menu */
        if (events & SERVER_EVENT_SELECT_ACTION)
        {
            /* Update current firmware for server */
            if ((ChosenAction == Server_Actions_UpdateAvailableFw) && (!availableFwUpdateInProgress))
            {
                /* Get FW from UART */
                availableFwUpdateInProgress = true;

                /* Free Serial display for UART Driver (needed to receive new FW image) */
                Display_print0(hDisplaySerial, 0, 0, "Waiting for Node FW update...");
                CPUdelay(8000*50);
                Display_close(hDisplaySerial);

                /* Start the uart firmware download */
                OADServer_updateAvailableFwVer();
            }

            /* Get firmware version from client */
            else if (ChosenAction == Server_Actions_FwVerReq)
            {
                /* Get FW version from client */
                /* Ignore argument input */
                OADServer_getFwVer(0xFF);
            }

            /* Update client firmware */
            else if (ChosenAction == Server_Actions_UpdateClientFw)
            {
#if (defined(DeviceFamily_CC13X2) || defined(DeviceFamily_CC26X2) || \
defined(DeviceFamily_CC26X2X7) || defined(DeviceFamily_CC13X2X7) || \
defined(DeviceFamily_CC13X1)   || defined(DeviceFamily_CC26X1) || \
defined(DeviceFamily_CC13X4)   || defined(DeviceFamily_CC26X4))

            /* Once a reset response is received then this condition will run*/
            if((nodeFwOadStatus == NodeOadStatus_ResetComplete) && !availableFwUpdateInProgress)
                {
                    /* update FW on selected node */
                    totalBlocks = OADServer_updateNodeFw(0xFF);
                }

            /* First send client a reset request */
            else if((nodeFwOadStatus != NodeOadStatus_InProgress) && (!availableFwUpdateInProgress))
                {
                    /* Send reset command to client
                        * Ignore address */
                    OADServer_resetNode(0xFF);
                }
#else

            if ((nodeFwOadStatus != NodeOadStatus_InProgress) && (!availableFwUpdateInProgress))
                {
                    /* OAD will start after receiving a response
                        * from this request
                        Ignore argument input */
                    totalBlocks = OADServer_updateNodeFw(0xFF);
                }
#endif
            }
        }

        /* Update any statuses */
        if (events & SERVER_EVENT_UPDATE_DISPLAY)
        {
            /* Update display from OAD events*/
            ServerStatusUpdate();
        }

        /* Received something from client */
        if (events & SERVER_EVENT_OAD_MSG)
        {
            OADProtocol_ParseIncoming((void*)0xFF, ptrAppBuffer);
        }

        /* This is not related to client-server OAD block exchanges */
        if (events & SERVER_EVENT_UART_FW_UPDATE)
        {
            /* Service UART firmware update */
            OADServer_processEvent(&events);
        }
    }
}

void buttonCallbackFunction(uint_least8_t index)
{
    /* Simple debounce logic, only toggle if the button is still pushed (low) */
    CPUdelay((uint32_t)((48000000/3)*0.050f));
    if (!GPIO_read(index))
    {
        /* After an OAD image is completed, it will ask for a button press to change the current state */
        if (nodeFwOadStatus == NodeOadStatus_Completed)
        {
            nodeFwOadStatus = NodeOadStatus_NotInProgress;
        }

        /* Button 1 - left button */
        if (index == CONFIG_GPIO_BTN1)
        {
            /* Choose action */
            if((nodeFwOadStatus != NodeOadStatus_InProgress) && (!availableFwUpdateInProgress))
            {
                /* Post select action event */
                Event_post(serverEventHandle, SERVER_EVENT_SELECT_ACTION);
            }
        }
        /* Button 2 - right button */
        else
        {
            /* Scroll action */
            if((nodeFwOadStatus != NodeOadStatus_InProgress) && (!availableFwUpdateInProgress))
            {
                selectedAction++;
                selectedAction = selectedAction % n;
                ChosenAction = (Server_Actions_t) selectedAction;

                /* Post update display event */
                Event_post(serverEventHandle, SERVER_EVENT_UPDATE_DISPLAY);
            }
        }
    }
}


void GPIO_setInitialization(void)
{
    /* Setup green LED */
    GPIO_setConfig(CONFIG_GPIO_GLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_OFF);

    /* Setup red LED */
    GPIO_setConfig(CONFIG_GPIO_RLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_OFF);

    /* Configure button 1 and button 2 pins */
    GPIO_setConfig(CONFIG_GPIO_BTN1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BTN2, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BTN1, buttonCallbackFunction);
    GPIO_setCallback(CONFIG_GPIO_BTN2, buttonCallbackFunction);

    /* Enable interrupts for buttons 1 and buttons 2*/
    GPIO_enableInt(CONFIG_GPIO_BTN1);
    GPIO_enableInt(CONFIG_GPIO_BTN2);
}

void Display_setInitialization(void)
{
    /* Initialize the UART terminal*/
    Display_Params params;
    Display_Params_init(&params);
    params.lineClearMode = DISPLAY_CLEAR_BOTH;
    hDisplaySerial = Display_open(Display_Type_UART, &params);
    Display_clear(hDisplaySerial);
    Display_printf(hDisplaySerial, 0, 0, "\033[2J \033[0;0HServer - Main Menu");
    Display_printf(hDisplaySerial, 1, 0, "Left Btn: Select Action, Right: Scroll Action Menu");
    Display_printf(hDisplaySerial, 2, 0, "Info: Current Server FW %s", availableFwVersion);
    Display_printf(hDisplaySerial, 3, 0, "Action: %s", selectedActionStr[selectedAction]);
}

void OAD_Init(void)
{
    OADServer_Params_t oadServerParams = {0};

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&serverEvent, &eventParam);
    serverEventHandle = Event_handle(&serverEvent);

    oadServerParams.eventBit = SERVER_EVENT_UART_FW_UPDATE;
    oadServerParams.eventHandle = serverEventHandle;
    OADProtocol_Status_t status = OADServer_open(&oadServerParams);

    if (status != OADProtocol_Status_Success)
    {
        System_abort("Error initializing OAD server\n");
    }
}

void ServerStatusUpdate(void)
{
    /* This ensures that the UART terminal is not in use
     * when the Concentrator is currently downloading the
     * latest FW
     */
    if (!availableFwUpdateInProgress)
    {
        uint8_t currentLcdLine = 0;

        Display_clear(hDisplaySerial);

        if(nodeFwOadStatus == NodeOadStatus_NotInProgress)
        {
            /* Clear the display and write header on first line */
            Display_printf(hDisplaySerial, 0, 0, "\033[2J \033[0;0HServer - Main Menu");
            Display_printf(hDisplaySerial, ++currentLcdLine, 0, "Left Btn: Select Action, Right: Scroll action");
            Display_printf(hDisplaySerial, ++currentLcdLine, 0, "Action: %s", selectedActionStr[selectedAction]);
        }

        switch(ChosenAction)
        {
        case Server_Actions_UpdateAvailableFw:
            /* Available FW*/
            Display_printf(hDisplaySerial, ++currentLcdLine, 0, "Info: Current Server FW %s", availableFwVersion);

            break;
        case Server_Actions_FwVerReq:
            /* FW version of selected node */
            Display_printf(hDisplaySerial, ++currentLcdLine, 0, "Info: Current client FW %s", currentClientFwVersion);

            break;

        case Server_Actions_UpdateClientFw:
            if(nodeFwOadStatus == NodeOadStatus_InProgress)
            {
                /* Print to UART */
                Display_printf(hDisplaySerial, 0, 0, "\033[2J \033[0;0HInfo: OAD Block %d of %d",
                           oadBlock,
                           totalBlocks);
            }
            else if(nodeFwOadStatus == NodeOadStatus_Completed)
            {
                /* Print to UART */
                Display_printf(hDisplaySerial, 0, 0, "Info: OAD Complete");
                Display_printf(hDisplaySerial, 1, 0, "Press any button to continue");
            }
            else if(nodeFwOadStatus == NodeOadStatus_Aborted)
            {
                /* Print to UART */
                Display_printf(hDisplaySerial, 0, 0, "Info: OAD Aborted");
            }

            break;
        default:
            break;
        }
    }
}

void rfServer_updateCurrentClientFWVer(char* fwVersionStr)
{
    /* Save firmware string to a local array */
    memcpy(currentClientFwVersion, fwVersionStr, OADProtocol_FW_VERSION_STR_LEN);

    /* Post an event to display*/
    Event_post(serverEventHandle, SERVER_EVENT_UPDATE_DISPLAY);
}

void rfServer_postNewOadMsg(void)
{
    /* Post event relating to incoming oad messages */
    Event_post(serverEventHandle, SERVER_EVENT_OAD_MSG);
}

void rfServer_updateNodeOadStatus(NodeOadStatus_t status)
{
    /* Update current nodeFwOadStatus variable from whatever state it is */
    nodeFwOadStatus = status;

    /* Post an event to display*/
    Event_post(serverEventHandle, SERVER_EVENT_UPDATE_DISPLAY);

}

void rfServer_updateNodeOadBlock(uint16_t block)
{
    /* Update the current block number coming from the client */
    oadBlock = block;

    /* Post an event to display*/
    Event_post(serverEventHandle, SERVER_EVENT_UPDATE_DISPLAY);
}

void rfServer_setNodeOadStatusReset(void)
{
    /* Change the variable to proceed to the next step of OAD process */
    nodeFwOadStatus = NodeOadStatus_ResetComplete;

    /* Post an event to proceed to the next oad process*/
    Event_post(serverEventHandle, SERVER_EVENT_SELECT_ACTION);
}


void rfServer_updateAvailableFWVer(uint8_t status)
{
    Display_Params params;
    Display_Params_init(&params);
    params.lineClearMode = DISPLAY_CLEAR_BOTH;

    availableFwUpdateInProgress = false;

    if(status == OADStorage_Status_Success)
    {
#if (defined(DeviceFamily_CC13X2) || defined(DeviceFamily_CC26X2) || \
     defined(DeviceFamily_CC13X2X7) || defined(DeviceFamily_CC26X2X7) || \
     defined(DeviceFamily_CC13X1)   || defined(DeviceFamily_CC26X1) || \
     defined(DeviceFamily_CC13X4)   || defined(DeviceFamily_CC26X4))
        /* Get available FW version*/
        OADStorage_imgIdentifyPld_t remoteAppImageId;

        OADStorage_init();

        /* Get available FW version*/
        OADStorage_imgIdentifyRead(OAD_IMG_TYPE_USR_BEGIN, &remoteAppImageId);

        System_sprintf((xdc_Char*)availableFwVersion,"rfClient sv:%c%c%c%c bv:%02x",
                       remoteAppImageId.softVer[0],
                       remoteAppImageId.softVer[1],
                       remoteAppImageId.softVer[2],
                       remoteAppImageId.softVer[3],
                       remoteAppImageId.bimVer);

        OADStorage_close();
#else
        OADTarget_ImgHdr_t remoteAppImageHdr;

        /* Get available FW version*/
        OADTarget_getImageHeader(EFL_OAD_IMG_TYPE_REMOTE_APP, &remoteAppImageHdr);
        if ((remoteAppImageHdr.ver != 0xFFFF) || (remoteAppImageHdr.ver == 0))
        {
            System_sprintf((xdc_Char*)availableFwVersion,"rfClient v%02d.%02d.00", ((remoteAppImageHdr.ver & 0xFF00)>>8), (remoteAppImageHdr.ver & 0xFF));
        }
#endif
    }
    else
    {
        /* display the error code */
        System_sprintf((xdc_Char*)availableFwVersion,"update failed ErrCode:%d", status);
    }

    /* Re-enable UART and LCD */
    hDisplaySerial = Display_open(Display_Type_UART, &params);

    /* Post an event to display*/
    Event_post(serverEventHandle, SERVER_EVENT_UPDATE_DISPLAY);
}
