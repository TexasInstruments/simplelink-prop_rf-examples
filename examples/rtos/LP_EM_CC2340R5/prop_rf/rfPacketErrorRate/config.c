/*
 * Copyright (c) 2016-2018, Texas Instruments Incorporated
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

/* Application specific Header files */
#include "config.h"

FrequencyTableEntry config_frequencyTable_2_4[] =
{
    { "2405  ", 0x0965, 0x0000, 0x0 },
    { "2410  ", 0x096a, 0x0000, 0x0 },
    { "2415  ", 0x096f, 0x0000, 0x0 },
    { "2420  ", 0x0974, 0x0000, 0x0 },
    { "2425  ", 0x0979, 0x0000, 0x0 },
    { "2430  ", 0x097e, 0x0000, 0x0 },
    { "2435  ", 0x0983, 0x0000, 0x0 },
    { "2440  ", 0x0988, 0x0000, 0x0 },
    { "2445  ", 0x098d, 0x0000, 0x0 },
    { "2450  ", 0x0992, 0x0000, 0x0 },
    { "2455  ", 0x0997, 0x0000, 0x0 },
    { "2460  ", 0x099c, 0x0000, 0x0 },
    { "2465  ", 0x09a1, 0x0000, 0x0 },
    { "2470  ", 0x09a6, 0x0000, 0x0 },
    { "2475  ", 0x09ab, 0x0000, 0x0 },
    { "2480  ", 0x09b0, 0x0000, 0x0 },
    { "", 0xFFFF, 0, 0 }, // Freq of 0xFFFF indicates the end of the table
};

FrequencyTableEntry config_frequencyTable_ble[] =
{
    { "2402  ", 0x0962, 0x0000, 0x65 },
    { "2404  ", 0x0964, 0x0000, 0x40 },
    { "2406  ", 0x0966, 0x0000, 0x41 },
    { "2408  ", 0x0968, 0x0000, 0x42 },
    { "2410  ", 0x096a, 0x0000, 0x43 },
    { "2412  ", 0x096c, 0x0000, 0x44 },
    { "2414  ", 0x096e, 0x0000, 0x45 },
    { "2416  ", 0x0970, 0x0000, 0x46 },
    { "2418  ", 0x0972, 0x0000, 0x47 },
    { "2420  ", 0x0974, 0x0000, 0x48 },
    { "2422  ", 0x0976, 0x0000, 0x49 },
    { "2424  ", 0x0978, 0x0000, 0x4a },
    { "2426  ", 0x097a, 0x0000, 0x66 },
    { "2428  ", 0x097c, 0x0000, 0x4b },
    { "2430  ", 0x097e, 0x0000, 0x4c },
    { "2432  ", 0x0980, 0x0000, 0x4d },
    { "2434  ", 0x0982, 0x0000, 0x4e },
    { "2436  ", 0x0984, 0x0000, 0x4f },
    { "2438  ", 0x0986, 0x0000, 0x50 },
    { "2440  ", 0x0988, 0x0000, 0x51 },
    { "2442  ", 0x098a, 0x0000, 0x52 },
    { "2444  ", 0x098c, 0x0000, 0x53 },
    { "2446  ", 0x098e, 0x0000, 0x54 },
    { "2448  ", 0x0990, 0x0000, 0x55 },
    { "2480  ", 0x0992, 0x0000, 0x56 },
    { "2452  ", 0x0994, 0x0000, 0x57 },
    { "2454  ", 0x0996, 0x0000, 0x58 },
    { "2456  ", 0x0998, 0x0000, 0x59 },
    { "2458  ", 0x099a, 0x0000, 0x5a },
    { "2460  ", 0x099c, 0x0000, 0x5b },
    { "2462  ", 0x099e, 0x0000, 0x5c },
    { "2464  ", 0x09a0, 0x0000, 0x5d },
    { "2466  ", 0x09a2, 0x0000, 0x5e },
    { "2468  ", 0x09a4, 0x0000, 0x5f },
    { "2470  ", 0x09a6, 0x0000, 0x60 },
    { "2472  ", 0x09a8, 0x0000, 0x61 },
    { "2474  ", 0x09aa, 0x0000, 0x62 },
    { "2476  ", 0x09ac, 0x0000, 0x63 },
    { "2478  ", 0x09ae, 0x0000, 0x64 },
    { "2480  ", 0x09b0, 0x0000, 0x67 },
    { "", 0xFFFF, 0, 0 }, //Freq of 0xFFFF indicates the end of the table
};

/*
Frequency and data rate table look up. Maps Frequency table and data rate table
to specific RF Mode
*/
FrequencyTableEntry*  config_frequencyTable_Lut[] =
{
    config_frequencyTable_ble,    // RCL_Generic_BLE_1M
    config_frequencyTable_2_4,    // RCL_Generic_250K_MSK
    config_frequencyTable_2_4     // RCL_Generic_500K_MSK
};

uint32_t config_dataRateTable_Lut[] =
{
    1000000,  // RCL_Generic_BLE_1M
    250000,   // RCL_Generic_250K_MSK
    500000    // RCL_Generic_500K_MSK
};

const char* const config_rfSetupLabels[] =
{
    "BLE Mode ",
    "MSK 250K ",
    "MSK 500K"
};

const char* const config_testmodeLabels[] =
{
    "Rx",
    "Tx"
};

const uint32_t config_packetCountTable[] =
{
    10,
    100,
    1000,
    10000
};

const uint32_t config_payloadLengthTable[] =
{
    30,
    60,
    100,
    254
};

const char* const config_intervalLabels[] =
{
    "No ",
    "Yes"
};

const uint8_t config_NrOfPacketCounts = (sizeof(config_packetCountTable)/sizeof(uint32_t));
const uint8_t config_NrOfPayloadLengths = (sizeof(config_payloadLengthTable)/sizeof(uint32_t));
