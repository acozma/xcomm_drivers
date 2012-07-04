/**************************************************************************//**
*   @file   xcomm.c
*   @brief  XCOMM interface functions implementation.
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
#include <stdint.h>
#include "ad9548.h"
#include "ad9523.h"
#include "adf4351.h"
#include "ad9122.h"
#include "ad9643.h"
#include "ad8366.h"
#include "spi_interface.h"
#include "xcomm.h"

/**************************************************************************//**
* @brief Initializes the XCOMM board
*
* @return Returns 0 in case of success or negative error code
******************************************************************************/
int XCOMM_Init()
{
    int ret = 0;

    ret = SPI_Init();
    if(ret < 0)
    	return ret;

    ret = ad9548_setup();
    if(ret < 0)
        return ret;

    ret = ad9523_setup();
    if(ret < 0)
        return ret;

    ret = adf4351_setup(ADF4351_RX_CHANNEL);
    if(ret < 0)
        return ret;

    ret = adf4351_setup(ADF4351_TX_CHANNEL);
    if(ret < 0)
        return ret;
    
    ret = ad9122_setup();
    if(ret < 0)
        return ret;

    ret = ad9643_setup();
    if(ret < 0)
        return ret;

    return ret;
}

/**************************************************************************//**
* @brief Sets the Rx center frequency
*
* @param frequency - frequency value in Hz
*
* @return Returns the actual frequency or negative error code
******************************************************************************/
long long XCOMM_SetFrequencyRx(long long frequency)
{
    return adf4351_out_altvoltage0_frequency(frequency, ADF4351_RX_CHANNEL);
}

/**************************************************************************//**
* @brief Sets the Tx center frequency
*
* @param frequency - frequency value in Hz
*
* @return Returns the actual frequency or negative error code
******************************************************************************/
long long XCOMM_SetFrequencyTx(long long frequency)
{
    return adf4351_out_altvoltage0_frequency(frequency, ADF4351_TX_CHANNEL);
}

/**************************************************************************//**
* @brief Sets the VGA gain
*
* @param gain - the VGA gain value in dB
*
* @return Returns the actual set gain or negative error code
******************************************************************************/
float XCOMM_SetGain(float gain)
{
    float retGain = 0;

    retGain = ad8366_out_voltage0_hardwaregain(gain);
    retGain = ad8366_out_voltage1_hardwaregain(gain);

    return retGain;
}

/**************************************************************************//**
* @brief Sets the test mode of the ADC
*
* @param mode - ADC test mode - see the Test Mode Definitions in AD9643.h
*
* @return Returns the set test mode or negative error code
******************************************************************************/
int XCOMM_SetAdcTestMode(int mode)
{
    return ad9643_test_mode(mode);
}

/**************************************************************************//**
* @brief Sets the sampling rate of the ADC
*
* @param rate - ADC sampling rate in samples/sec
*
* @return Returns the set sampling rate or negative error code
******************************************************************************/
long XCOMM_SetAdcSamplingRate(long rate)
{
    return ad9523_out_altvoltage2_ADC_CLK_frequency(rate);
}
