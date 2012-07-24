/**************************************************************************//**
*   @file   main.c
*   @brief  XCOMM main program implementation.
*   @author acozma (andrei.cozma@analog.com)
*
*******************************************************************************
* Copyright 2011(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************
*   SVN Revision: $WCREV$
******************************************************************************/

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/
#include <stdio.h>
#include "platform.h"
#include "xcomm.h"

int main()
{
    int ret;
    float gain = 20.0f;
    float retGain;
    unsigned long long freqRx = 100000000;
    unsigned long long retFreqRx;
    unsigned long long freqTx = 150000000;
    unsigned long long retFreqTx;

    init_platform();

    xil_printf("Running XCOMM Test Program\n\r");

    xil_printf("Initializing XCOMM Components...\n\r");
    ret = XCOMM_Init();
	if(ret < 0)
	{
		xil_printf("XCOMM Init Failed!\n\r");
		return 0;
	}
	else
	{
		xil_printf("XCOMM Init OK!\n\r");
	}

	xil_printf("Setting the VGA gain to: %d.%d dB\n\r", (int)gain, (int)((gain - (int)gain) * 100));
	retGain = (float)XCOMM_SetRxGain((uint32_t)(gain*1000.0f)) / 1000.0f;
	xil_printf("Actual set VGA gain: %d.%d dB\n\r", (int)retGain, (int)((retGain - (int)retGain) * 100));

	xil_printf("Setting the Rx frequency to: %d\n\r", freqRx);
    retFreqRx = XCOMM_SetRxFrequency(freqRx);
    xil_printf("Actual set Rx frequency: %d\n\r", retFreqRx);

	xil_printf("Setting the Tx frequency to: %d\n\r", freqTx);
    retFreqTx = XCOMM_SetTxFrequency(freqTx);
    xil_printf("Actual set Tx frequency: %d\n\r", retFreqTx);

    xil_printf("Finished XCOMM Test Program\n\r");

	cleanup_platform();

    return 0;
}
