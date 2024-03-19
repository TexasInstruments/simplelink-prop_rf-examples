/******************************************************************************

 @file oad_server.c

 @brief OAD Server

 Group: CMCU LPRF
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2016-2024, Texas Instruments Incorporated
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

/******************************************************************************
 Includes
 *****************************************************************************/
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include <ti/sysbios/knl/Clock.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/flash.h)

#include "oad_protocol.h"
#include "oad_server.h"
#include "oad_storage.h"
#include "radio/radio.h"
#include <common/cc26xx/oad/ext_flash_layout.h>
#include <common/cc26xx/oad/oad_image_header.h>
#include <rfServer.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>

#include "ti_drivers_config.h"

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/flash.h)

/******************************************************************************
 Constants and definitions
 *****************************************************************************/

/*!
 OAD block variables.
 */
/*static*/ uint16_t oadBNumBlocks = 0;
/*static*/ uint16_t oadBlock = 0;
static bool oadInProgress = false;
OADServer_Params_t oadServerParams;
static UART2_Handle uartHandle;

/*!
 * Clock for OAD abort
 */
Clock_Struct oadAbortTimeoutClock;     /* not static so you can see in ROV */
static Clock_Handle oadAbortTimeoutClockHandle;

/******************************************************************************
 Local function prototypes
 *****************************************************************************/

static void getNextBlock(uint16_t blkNum, uint8_t* oadBlockBuff);
static void fwVersionRspCb(void* pSrcAddr, char *fwVersionStr);
static void oadImgIdentifyRspCb(void* pSrcAddr, uint8_t status);
static void oadBlockReqCb(void* pSrcAddr, uint8_t imgId, uint16_t blockNum, uint16_t multiBlockSize);
static void oadResetRspCb(void* pSrcAddr);

static void oadAbortTimeoutCallback(UArg arg0);

void* oadRadioAccessAllocMsg(uint32_t msgLen);
static OADProtocol_Status_t oadRadioAccessPacketSend(void* pDstAddr, uint8_t *pMsg, uint32_t msgLen);

/******************************************************************************
 Callback tables
 *****************************************************************************/

static OADProtocol_RadioAccessFxns_t  oadRadioAccessFxns =
    {
      oadRadioAccessAllocMsg,
      oadRadioAccessPacketSend
    };

static OADProtocol_MsgCBs_t oadMsgCallbacks =
    {
      /*! Incoming FW Req */
      NULL,
      /*! Incoming FW Version Rsp */
      fwVersionRspCb,
      /*! Incoming Image Identify Req */
      NULL,
      /*! Incoming Image Identify Rsp */
      oadImgIdentifyRspCb,
      /*! Incoming OAD Block Req */
      oadBlockReqCb,
      /*! Incoming OAD Block Rsp */
      NULL,
      /*! Incoming OAD Reset Req */
      NULL,
      /*! Incoming OAD Reset Rsp */
      oadResetRspCb,
    };

/******************************************************************************
 Public Functions
 *****************************************************************************/

/*!
 Initialize this application.

 Public function defined in sensor.h
 */
OADProtocol_Status_t OADServer_open(OADServer_Params_t *params)
{
    OADProtocol_Params_t OADProtocol_params;
    OADProtocol_Status_t status = OADProtocol_Failed;

    /* Create clock object which is used for fast report timeout */
    Clock_Params clkParams;
    clkParams.period = 0;
    clkParams.startFlag = FALSE;
    Clock_construct(&oadAbortTimeoutClock, oadAbortTimeoutCallback, 1, &clkParams);
    oadAbortTimeoutClockHandle = Clock_handle(&oadAbortTimeoutClock);

    memcpy(&oadServerParams, params, sizeof(OADServer_Params_t));

    OADProtocol_Params_init(&OADProtocol_params);
    OADProtocol_params.pRadioAccessFxns = &oadRadioAccessFxns;

    if(OADProtocol_params.pRadioAccessFxns->pfnRadioAccessAllocMsg == NULL)
    {
        return status;
    }

    OADProtocol_params.pProtocolMsgCallbacks = &oadMsgCallbacks;

    OADProtocol_open(&OADProtocol_params);

    return OADProtocol_Status_Success;
}

/*!
 OAD event processing.

 Public function defined in oad_server.h
 */
void OADServer_processEvent(uint32_t *pEvent)
{
    /* Is it time to send the next sensor data message? */
    if(*pEvent & oadServerParams.eventBit)
    {
        /* allocate buffer for block + block number */
        uint8_t blkData[OADStorage_BLOCK_SIZE] = {0};

        /* is last block? */
        if(oadBlock < oadBNumBlocks)
        {
            uint16_t sentBlockNum;

            /* get block */
            getNextBlock(oadBlock, blkData);

            sentBlockNum = ((uint16_t)(((blkData[0]) & 0x00FF) + (((blkData[1]) & 0x00FF) << 8)));

            if (sentBlockNum == oadBlock)
            {
                /* write block */
                OADStorage_imgBlockWrite(oadBlock, blkData, OADStorage_BLOCK_SIZE);
                oadBlock++;
            }

            /* set event to get next block */
            Event_post(oadServerParams.eventHandle, oadServerParams.eventBit);
        }
        else
        {
            /* end available fw update */
            oadInProgress = false;

            /*
             * Check that CRC is correct and mark the image as new
             * image to be booted in to by BIM on next reset
             */
            uint8_t status = OADStorage_imgFinalise();

            /* Close resources */
            OADStorage_close();
            UART2_close(uartHandle);

            rfServer_updateAvailableFWVer(status);
        }

    }
}
/*!
 Get Node FW Version

 Public function defined in oad_server.h
 */
void OADServer_getFwVer(uint8_t dstAddr)
{
    OADProtocol_sendFwVersionReq(&dstAddr);
}

/*!
 Update Available FW

 Public function defined in oad_server.h
 */
void OADServer_updateAvailableFwVer(void)
{
    UART2_Params uartParams;
    uint8_t imgMetaData[sizeof(OADStorage_imgIdentifyPld_t)];

    if(!oadInProgress)
    {
        /* initialize UART */
        UART2_Params_init(&uartParams);
        uartParams.readReturnMode = UART2_ReadReturnMode_FULL;
        uartParams.baudRate = 115200;
        uartHandle = UART2_open(CONFIG_DISPLAY_UART, &uartParams);

        /* get image header */
        UART2_read(uartHandle, imgMetaData, sizeof(OADStorage_imgIdentifyPld_t), NULL);

        OADStorage_init();

        /* adjust the image type so that the image is stored in the User Image space */
        imgMetaData[IMG_TYPE_OFFSET] = OAD_IMG_TYPE_USR_BEGIN;

        oadBNumBlocks = OADStorage_imgIdentifyWrite(imgMetaData);
        oadBlock = 0;

        /* set event to get next block */
        Event_post(oadServerParams.eventHandle, oadServerParams.eventBit);
    }
}

/*!
 Initiate OAD.

 Public function defined in oad_server.h
 */
uint16_t OADServer_updateNodeFw(uint8_t dstAddr)
{
    OADStorage_imgIdentifyPld_t remoteImgId;

    if(!oadInProgress)
    {
        oadInProgress = true;

        OADStorage_init();

        /* get num blocks and setup OADStorage to store in user image region */
        oadBNumBlocks = OADStorage_imgIdentifyRead(OAD_IMG_TYPE_USR_BEGIN, &remoteImgId);

        /*
         * Hard code imgId to 0 - its not used in this
         * implementation as there is only 1 image available
         */
        OADProtocol_sendImgIdentifyReq(&dstAddr, 0, (uint8_t*)&remoteImgId);

        if(oadBNumBlocks == 0)
        {
            /* issue with image in ext flash */
            oadInProgress = false;
        }

        return oadBNumBlocks;
    }

    return 0;
}

void OADServer_resetNode(uint8_t dstAddr)
{
    OADProtocol_sendOadResetReq(&dstAddr);
}

static void oadResetRspCb(void* pSrcAddr)
{
    /* OAD client confirms that it is ready for OAD process */
    /* proceed to next step to start OAD progress */
    rfServer_setNodeOadStatusReset();
}

/******************************************************************************
 Local Functions
 *****************************************************************************/

/*!
 * @brief      OAD abort timer callback
 */
static void oadAbortTimeoutCallback(UArg arg0)
{
    if(oadInProgress)
    {
        /* end available fw update */
        oadInProgress = false;
        OADStorage_close();

        /* abort OAD */
        rfServer_updateNodeOadStatus(NodeOadStatus_Aborted);
    }
}


/*!
 * @brief      Get next block of available FW image from UART
 */
static void getNextBlock(uint16_t blkNum, uint8_t* oadBlockBuff)
{
    uint8_t blkNumLower = blkNum & 0xFF;
    uint8_t blkNumUpper = (blkNum & 0xFF00) >> 8;

    UART2_write(uartHandle, &blkNumUpper, 1, NULL);
    UART2_write(uartHandle, &blkNumLower, 1, NULL);
    GPIO_toggle(CONFIG_GPIO_GLED);
    UART2_read(uartHandle, oadBlockBuff, OADStorage_BLOCK_SIZE, NULL);
    GPIO_toggle(CONFIG_GPIO_RLED);
}

/*!
 * @brief      FW version response callback from OAD module
 */
static void fwVersionRspCb(void* pSrcAddr, char *fwVersionStr)
{
    /* ignore address */
    (void) pSrcAddr;
    rfServer_updateCurrentClientFWVer(fwVersionStr);
}

/*!
 * @brief      Image Identify response callback from OAD module
 */
static void oadImgIdentifyRspCb(void* pSrcAddr, uint8_t status)
{
    /* OAD progress starts now */
    rfServer_updateNodeOadStatus(NodeOadStatus_InProgress);
}

/*!
 * @brief      Image block request callback from OAD module
 */
static void oadBlockReqCb(void* pSrcAddr, uint8_t imgId, uint16_t blockNum, uint16_t multiBlockSize)
{
    uint8_t blockBuf[OADStorage_BLOCK_SIZE - OADStorage_BLK_NUM_HDR_SZ] = {0};
    (void) imgId;

    if(oadInProgress)
    {
        rfServer_updateNodeOadBlock(blockNum);

        /* read a block from Flash */
        OADStorage_imgBlockRead(blockNum, blockBuf);

        /* hard code imgId to 0 - its not used in this
         * implementation as there is only 1 image available
         */
        OADProtocol_sendOadImgBlockRsp(pSrcAddr, 0, blockNum, blockBuf);

        if(blockNum == oadBNumBlocks - 1)
        {
            /* OAD complete */
            oadInProgress = false;
            OADStorage_close();

            rfServer_updateNodeOadStatus(NodeOadStatus_Completed);
        }
        else
        {
            /* restart timeout in case of abort */
            Clock_stop(oadAbortTimeoutClockHandle);

            /* give 5s grace for the flash erase delay*/
            Clock_setTimeout(oadAbortTimeoutClockHandle,
                    (5000 * 1000) / Clock_tickPeriod);

            /* start timer */
            Clock_start(oadAbortTimeoutClockHandle);
        }
    }
}

/*!
 * @brief      Radio access function for OAD module to send messages
 */
void* oadRadioAccessAllocMsg(uint32_t msgLen)
{
    uint8_t *msgBuffer;

    /* Allocate buffer here */
    msgBuffer = (uint8_t*) malloc(msgLen);

    if(msgBuffer == NULL)
    {
        return NULL;
    }

    /* Zero buffer here */
    memset(msgBuffer, 0, msgLen);

    return msgBuffer;
}

/*!
 * @brief      Radio access function for OAD module to send messages
 */
static OADProtocol_Status_t oadRadioAccessPacketSend(void* pDstAddr, uint8_t *pMsgPayload, uint32_t msgLen)
{
    OADProtocol_Status_t status = OADProtocol_Failed;
    uint8_t * pMsg = pMsgPayload;

    /* Ignore this term */
    (void) pDstAddr;

    /* Transmit the packet via radio */
    Radio_txPacket(pMsg, msgLen);

    /* Free the memory allocated in oadRadioAccessAllocMsg
     * pMsg points to the same memory allocated space as msgBuffer */
    free(pMsg);

    return status;
}

