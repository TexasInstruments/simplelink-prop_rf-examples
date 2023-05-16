/*
 * Copyright (c) 2013-2022, Texas Instruments Incorporated
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
#include <stdio.h>
#include <unistd.h>

/* TI Drivers */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/rcl/RCL.h>
#include <ti/drivers/rcl/RCL_Scheduler.h>
#include <ti/drivers/rcl/commands/generic.h>

#if defined(USE_250KBPS_MSK)
#include <setup/rcl_settings_msk_250_kbps.h>
#elif defined(USE_500KBPS_MSK)
#include <setup/rcl_settings_msk_500_kbps.h>
#else
#include <setup/rcl_settings_ble_generic.h>
#endif

/***** Defines *****/
/* RF Frequency (Hz) to program */
#if defined(USE_250KBPS_MSK) || defined(USE_500KBPS_MSK)
#define FREQUENCY               (2433000000U)
#else
#define FREQUENCY               (2440000000U)
#endif

/***** Variable Declarations *****/
/* RCL Commands */
RCL_CmdGenericTxTest   txCmd;           // TX Test command

/* RCL Client used to open RCL */
static RCL_Client  rclClient;

/* Counters for RCL event callback */
volatile uint32_t gCmdDone = 0;         // Command done

/***** Callback functions *****/
void defaultCallback(RCL_Command *cmd, LRF_Events lrfEvents, RCL_Events rclEvents)
{
    if (rclEvents.lastCmdDone)
    {
        gCmdDone += 1;
    }
}

/***** Function definitions *****/

void *mainThread(void *arg0)
{
    /* Initialize and open RCL */
    RCL_init();
#if defined(USE_250KBPS_MSK)
    RCL_Handle rclHandle = RCL_open(&rclClient, &LRF_configMsk250Kbps);
#elif defined(USE_500KBPS_MSK)
    RCL_Handle rclHandle = RCL_open(&rclClient, &LRF_configMsk500Kbps);
#else
    RCL_Handle rclHandle = RCL_open(&rclClient, &LRF_configBle);
#endif

    /* Setup generic transmit test command */
    txCmd = RCL_CmdGenericTxTest_DefaultRuntime();

    /* Set RF frequency */
    txCmd.rfFrequency = FREQUENCY;
#if !(defined(USE_250KBPS_MSK) || defined(USE_500KBPS_MSK))
    txCmd.common.phyFeatures = RCL_PHY_FEATURE_SUB_PHY_1_MBPS_BLE;
#endif

    /* Start command as soon as possible */
    txCmd.common.scheduling = RCL_Schedule_Now;
    txCmd.common.status = RCL_CommandStatus_Idle;

    txCmd.config.sendCw = 1U;       // Send CW
    txCmd.config.whitenMode = 1U;   // Default whitening
    txCmd.config.txWord = 0U;       // Repeated word to transmit
    txCmd.config.fsOff = 1;         // Turn off FS

    /* Callback triggers on last command done */
    txCmd.common.runtime.callback = defaultCallback;
    txCmd.common.runtime.rclCallbackMask.value = RCL_EventLastCmdDone.value;

    /* Submit command */
    RCL_Command_submit(rclHandle, &txCmd);

    /* Pend on command completion */
    RCL_Command_pend(&txCmd);

    return NULL;
}
