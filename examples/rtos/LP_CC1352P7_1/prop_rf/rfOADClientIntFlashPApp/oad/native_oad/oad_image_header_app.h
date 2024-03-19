/******************************************************************************

 @file  oad_image_header_app.h

 @brief This file extends the _imgHdr that is established in the 
    oad_image_header_app.c file for application specific code use.

 Group: CMCU LPRF
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2001-2024, Texas Instruments Incorporated
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
 
 
 ******************************************************************************/

#ifndef _OAD_IMAGE_HEADER_APP_H
#define _OAD_IMAGE_HEADER_APP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/
#include <common/cc26xx/oad/oad_image_header.h>

/*******************************************************************************
 * DEFINES
 ******************************************************************************/
#ifdef MCUBOOT
#define MCUBOOT_MAGIC            0x96f3b83d
#define MCUBOOT_HDR_IDENTIFIER   0x96
#endif

/*******************************************************************************
 * CONSTANTS
 ******************************************************************************/
 
/*******************************************************************************
 * Typedefs
 */
#ifdef MCUBOOT
typedef struct {
    uint8_t iv_major;
    uint8_t iv_minor;
    uint16_t iv_revision;
    uint32_t iv_build_num;
} mcuboot_image_version_t;

/** Image header.  All fields are in little endian byte order. */
typedef struct{
    uint32_t ih_magic;
    uint32_t ih_load_addr;
    uint16_t ih_hdr_size;           /* Size of image header (bytes). */
    uint16_t ih_protect_tlv_size;   /* Size of protected TLV area (bytes). */
    uint32_t ih_img_size;           /* Does not include header. */
    uint32_t ih_flags;              /* IMAGE_F_[...]. */
    mcuboot_image_version_t ih_ver;
    uint32_t _pad1;
} mcuboot_image_header_t;
#endif

/*******************************************************************************
 * Externs
 */
/* Make the header visible to other components */
extern const imgHdr_t _imgHdr;

#ifdef MCUBOOT
extern const mcuboot_image_header_t* _mcuBootHdr;
extern const uint32_t _PRIMARY_SLOT_BASE;
extern const uint32_t _ENTRY_SECTION_SZ;
extern const uint32_t _OAD_IMG_HDR_BASE;
#endif

/*!
 * @brief      Get firmware version from the image header
 */
extern void OADImgHdr_getFWVersion(char* fwVersion);

#endif /* _OAD_IMAGE_HEADER_APP_H */
