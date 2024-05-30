/*
 * Copyright (c) 2016-2019, Texas Instruments Incorporated
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

#ifndef CONFIG_H
#define CONFIG_H

#include "ti_drivers_config.h"

/* This file defines data types and variables for the application configuration */

/* PER version */
#define PER_VERSION "Ver 1.00.00"

/*
RF basic settings as found in the typical settings section of Smart RF Studio.
Each one defines a test case in this application.
*/
typedef enum
{
    RCL_Generic_BLE_1M,
    RCL_Generic_250K_MSK,
    RCL_Generic_250_MSK_FEC,
    NrOfRfSetups
} RfSetup;

/* Whether packets are sent back-to-back or with predefined intervals */
typedef enum
{
    IntervalMode_No = 0,
    IntervalMode_Yes,
    NrOfIntervalModes,
} IntervalMode;

/* Whether the application works as sender or receiver */
typedef enum
{
    TestMode_Rx = 0,
    TestMode_Tx,
    NrOfTestModes
} TestMode;

/* Contains a pre-defined setting for frequency selection */
typedef struct
{
    const char* const label;
    const uint16_t frequency;
    const uint16_t fractFreq;
    const uint8_t whitening; // BLE has freq dependent whitening settings
} FrequencyTableEntry;

/*
Holds the application config that is prepared in the menu and
used in the rx and tx functions.
*/
typedef struct
{
    RfSetup rfSetup;                     // Test case index
    IntervalMode intervalMode;           // Packet interval
    TestMode testMode;                   // TX/RX mode index
    uint32_t packetCount;                // Desired packet count
    uint32_t  payloadLength;             // Desired payload length (bytes)
    FrequencyTableEntry* frequencyTable; // FrequencyTable for this test case
    uint8_t frequency;                   // Index in config_frequencyTable
} ApplicationConfig;

extern FrequencyTableEntry*  config_frequencyTable_Lut[]; // Lookup table for freq table
extern uint32_t config_dataRateTable_Lut[];               // Lookup table for data rates
extern const char* const config_testmodeLabels[];         // Lookup table for operation mode labels
extern const uint32_t config_packetCountTable[];          // Lookup table for different packet count options
extern const uint32_t config_payloadLengthTable[];        // Lookup table for different payload length options
extern const char* const config_rfSetupLabels[];          // Lookup table for RfSetup labels
extern const char* const config_intervalLabels[];         // Lookup table for interval mode labels
extern const char* const config_rangeExtenderLabels[];    // Lookup table for range extender labels
extern const char* const config_highPaLabels[];           // Lookup table for High PA labels
extern const uint8_t config_NrOfPacketCounts;             // Total amount of packet count options
extern const uint8_t config_NrOfPayloadLengths;           // Total size of payload options
#endif
