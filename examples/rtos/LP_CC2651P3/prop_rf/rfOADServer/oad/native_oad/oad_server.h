/******************************************************************************

 @file oad_server.h

 @brief OAD Server Header

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

#ifndef OADServer_H
#define OADServer_H

#include <ti/sysbios/knl/Event.h>

#include "oad/native_oad/oad_protocol.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * OAD User defined image type begin
 *
 * This is the beginning of the range of image types reserved for the user.
 * OAD applications will not attempt to use or load images with this type
 */
#define OAD_IMG_TYPE_USR_BEGIN            16

/** @brief RF parameter struct
 *  RF parameters are used with the OADServer_open() and OADServer_Params_t() call.
 */
typedef struct {
    Event_Handle eventHandle;     ///< Event handle to post to
    uint32_t eventBit;            ///< event to post
} OADServer_Params_t;

 /** @brief  Function to open the SOADProtocol module
 *
 *  @param  params      An pointer to OADServer_Params_t structure for initialization
 */
extern OADProtocol_Status_t OADServer_open(OADServer_Params_t *params);

 /** @brief  Function to process OAD events
 *
 *  @param  pEvent      Event to process
 */
extern void OADServer_processEvent(uint32_t *pEvent);

/** @brief  Function to get FW version of a node
*
*  @param  dstAddr      Address of node
*/
extern void OADServer_getFwVer(uint8_t dstAddr);

/** @brief  Function to get FW version of a node
*
*  @param  dstAddr      Address of node
*
*  @return blocks in image
*/
extern uint16_t OADServer_updateNodeFw(uint8_t dstAddr);

/** @brief  Send reset request to node before OAD transfer
*
*  @param  dstAddr      Address of node
*/
extern void OADServer_resetNode(uint8_t dstAddr);

/** @brief  Function to update available FW image
*
*/
void OADServer_updateAvailableFwVer(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OADServer_H */
