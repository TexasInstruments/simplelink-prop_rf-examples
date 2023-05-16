/*
 * Copyright (c) 2017, Texas Instruments Incorporated
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

/*
 *  ======== main_nortos.c ========
 */
#include <stdint.h>
#include <stddef.h>

#include <NoRTOS.h>

#include <ti/drivers/Board.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/dbell_regs.h)

#include DeviceFamily_constructPath(inc/hw_types.h)
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_ioc.h)
#include DeviceFamily_constructPath(inc/hw_ckmd.h)

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC23X0.h>
#include <ti/drivers/Temperature.h>

extern void *mainThread(void *arg0);

/*
 *  ======== main ========
 */
int main(void)
{
	uint32_t hftrackctl =  HWREG( CKMD_BASE + CKMD_O_HFTRACKCTL );
	
    /* ==== /ti/drivers/Power initialization ==== */
    Power_init();
    Power_setConstraint(PowerLPF3_DISALLOW_IDLE);
    Power_setConstraint(PowerLPF3_DISALLOW_STANDBY);
	
    Temperature_init();

    /* High performance clock buffer enable */
    HWREG( CKMD_BASE + CKMD_O_HFXTCTL ) |= ( CKMD_HFXTCTL_HPBUFEN | CKMD_HFXTCTL_EN );

    /* Enable tracking loop */
    HWREG( CKMD_BASE + CKMD_O_HFTRACKCTL ) = hftrackctl | CKMD_HFTRACKCTL_EN;

    /* Start NoRTOS */
    NoRTOS_start();

    /* Call mainThread function */
    mainThread(NULL);

    while (1);
}
