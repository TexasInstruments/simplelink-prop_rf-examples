/******************************************************************************

 @file oad_client.h

 @brief OAD Client Header

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
#ifndef OADClient_H
#define OADClient_H

#include <ti/sysbios/knl/Event.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief RF parameter struct
 *  RF parameters are used with the OADClient_open() and OADClient_Params_t() call.
 */
typedef struct {
    Event_Handle eventHandle;               ///< Event handle to post to
    uint32_t oadReqEventBit;                ///< event to post
    uint32_t oadRspPollEventBit;            ///< event to post
} OADClient_Params_t;

 /** @brief  Function to open the SOADProtocol module
 *
 *  @param  params      An pointer to OADClient_Params_t structure for initialization
 */
extern void OADClient_open(OADClient_Params_t *params);

 /** @brief  Function to process OAD events
 *
 *  @param  pEvent      Event to process
 */
extern void OADClient_processEvent(uint32_t *pEvent);

/** @brief  Function to invalidate the OAD image header of the U-App
 *  Called when the user app is performing a reset to the P-App context
 */
extern void OADClient_invalidateHeader(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OADClient_H */
