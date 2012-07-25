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

/****** XCOMM state structure ******/
struct stXCOMM_State
{
    /* Rx state variables */
    int64_t rxFreq;
    int32_t rxResolution;
    int32_t rxGain;
    XCOMM_IQCorrection rxIqCorrection;
    int16_t rxDcCorrection;

    /* Tx state variables */
    int64_t txFreq;
    int32_t txResolution;
    XCOMM_IQCorrection txIqCorrection;
    int16_t txDcCorrection;

    /* ADC state variables */
    int64_t adcSampleRate;
    XCOMM_AdcTestMode adcTestMode;

    /* DAC state variables */
    int64_t dacSampleRate;

}XCOMM_State;

/**************************************************************************//**
* @brief Initializes the XCOMM board
*
* @return If success, return 0
*         if error, return -1
******************************************************************************/
int32_t XCOMM_Init()
{
    int ret = 0;

    /* Reset the XCOMM state variables */
    int i = 0;
    char* pData = (char*)&XCOMM_State;
    for(i = 0; i < sizeof(XCOMM_State); i++)
    {
        pData[i] = 0;
    }

    /* Initialize the XCOMM board */
    ret = SPI_Init();
    if(ret < 0)
    	return -1;

    ret = ad9548_setup();
    if(ret < 0)
        return -1;

    ret = ad9523_setup();
    if(ret < 0)
        return -1;

    ret = adf4351_setup(ADF4351_RX_CHANNEL);
    if(ret < 0)
        return -1;

    ret = adf4351_setup(ADF4351_TX_CHANNEL);
    if(ret < 0)
        return -1;
    
    ret = ad9122_setup();
    if(ret < 0)
        return -1;

    ret = ad9643_setup();
    if(ret < 0)
        return -1;

    ret = ad8366_setup();
    if(ret < 0)
        return -1;

    return ret;
}

/**************************************************************************//**
* @brief Resync driver cached values by reading the XCOMM board 
*
* @return Returns 0 in case of success or -1 if error
******************************************************************************/
int32_t XCOMM_Sync(void)
{
    /* Resync Rx state variables */
    XCOMM_GetRxResolution(XCOMM_ReadMode_FromHW);
    XCOMM_GetRxGain(XCOMM_ReadMode_FromHW);
    XCOMM_GetRxIqCorrection(0, XCOMM_ReadMode_FromHW);
    XCOMM_GetRxDcOffset(0, XCOMM_ReadMode_FromHW);
    
    /* Resync Tx state variables */
    XCOMM_GetTxResolution(XCOMM_ReadMode_FromHW);
    XCOMM_GetTxIqCorrection(0, XCOMM_ReadMode_FromHW);
    XCOMM_GetTxDcOffset(0, XCOMM_ReadMode_FromHW);

    /* Resync ADC state variables */
    XCOMM_GetAdcSamplingRate(XCOMM_ReadMode_FromHW);
    XCOMM_GetAdcTestMode(XCOMM_ReadMode_FromHW);

    /* Resync DAC state variables */
    XCOMM_GetDacSamplingRate(XCOMM_ReadMode_FromHW);

    return 0;
}

/**************************************************************************//**
* @brief Gets the XCOMM board version string
*
* @param readMode - Read version from driver or HW
*
* @return If success, return version struct with version string and error set to 0
          If error, return version struct with error set to -1
******************************************************************************/
XCOMM_Version XCOMM_GetBoardVersion(XCOMM_ReadMode readMode)
{
    XCOMM_Version ver;

    ver.error = 0;

    return ver;
}


/************************ Rx Functions ***************************************/


/**************************************************************************//**
* @brief Sets the Rx center frequency
*
* @param frequency - desired frequency value in Hz
*
* @return If success, return exact calculated frequency in Hz
*         if error, return -1
******************************************************************************/
int64_t XCOMM_SetRxFrequency(uint64_t frequency)
{
    int64_t freq = adf4351_out_altvoltage0_frequency(frequency, ADF4351_RX_CHANNEL);    
    if(freq < 0)
        return -1;

    XCOMM_State.rxFreq = freq;

    return XCOMM_State.rxFreq;
}

/**************************************************************************//**
* @brief Gets the Rx center frequency 
*
* @return If success, return frequency in Hz stored in driver,
*         if error, return -1
******************************************************************************/
int64_t XCOMM_GetRxFrequency(void)
{
    return XCOMM_State.rxFreq;
}

/**************************************************************************//**
* @brief Sets the Rx center frequency resolution
*
* @param resolution: desired frequency resolution in Hz
*  
* @return If success, return exact calculated resolution in Hz,
*         if error, return -1
******************************************************************************/
int32_t XCOMM_SetRxResolution(uint32_t resolution)
{
    int32_t res = adf4351_out_altvoltage0_frequency_resolution(resolution, 
                                                               ADF4351_RX_CHANNEL);
    if(res < 0)
        return -1;

    XCOMM_State.rxResolution = res;

    return XCOMM_State.rxResolution;
}

/**************************************************************************//**
* @brief Gets the Rx center frequency resolution
*
* @param readMode: read frequency resolution from driver or HW
*
* @return If success, return frequency resolution in Hz,
*         if error, return -1
******************************************************************************/
int32_t XCOMM_GetRxResolution(XCOMM_ReadMode readMode)
{
    int32_t res;

    if(readMode == XCOMM_ReadMode_FromHW)
    {
        res = adf4351_out_altvoltage0_frequency_resolution(INT32_MAX, 
                                                           ADF4351_RX_CHANNEL);
        if(res < 0)
            return -1;
        
        XCOMM_State.rxResolution = res;
    }

    return XCOMM_State.rxResolution;
}

/**************************************************************************//**
* @brief Sets the Rx gain
*
* @param gain1000: desired gain (x1000) in dB
*
* @return If success, return calculated gain (x1000) in dB,
*         if error, return -1
******************************************************************************/
int32_t XCOMM_SetRxGain(int32_t gain1000)
{
    int32_t retGain = 0;

    retGain = ad8366_out_voltage0_hardwaregain(gain1000);
    retGain = ad8366_out_voltage1_hardwaregain(gain1000);
    if(retGain < 0)
        return -1;
    
    XCOMM_State.rxGain = retGain;
    
    return XCOMM_State.rxGain;
}

/**************************************************************************//**
* @brief Gets the Rx gain
*
* @param readMode: read gain from driver or HW
*
* @return If success, return gain (x1000) in dB
*         if error, return -1
******************************************************************************/
int32_t XCOMM_GetRxGain(XCOMM_ReadMode readMode)
{
    int32_t gain;

    if(readMode == XCOMM_ReadMode_FromHW)
    {
        gain = ad8366_out_voltage0_hardwaregain(2 * (int32_t)AD8366_MAX_GAIN * 1000);
        if(gain < 0)
            return -1;        
        XCOMM_State.rxGain = gain;
    }

    return XCOMM_State.rxGain;
}

/**************************************************************************//**
* @brief Gets the Rx gain and phase correction for I and Q
*
* @param frequency: center frequency used for the correction in Hz
* @param readMode: read gain and phase correction from driver or HW
*
* @return If success, return IQCorrection struct with gain and phase 
*         correction for the frequency and error set to 0
*         If error, return IQCorrection struct with error set to -1
******************************************************************************/
XCOMM_IQCorrection XCOMM_GetRxIqCorrection(uint64_t frequency, XCOMM_ReadMode readMode)
{
    return XCOMM_State.rxIqCorrection;
}

/**************************************************************************//**
* @brief Gets the Rx DC Offset correction
*
* @param frequency: center frequency used for the correction in Hz
* @param readMode: read DC offset correction from driver or HW
*
* @return If success, return DC offset correction
*         if error, return -1
******************************************************************************/
int16_t XCOMM_GetRxDcOffset(uint64_t frequency, XCOMM_ReadMode readMode)
{
    return XCOMM_State.rxDcCorrection;
}


/************************ Tx Functions ***************************************/


/**************************************************************************//**
* @brief Sets the Tx center frequency
*
* @param frequency: desired frequency in Hz
*
* @return If success, return calculated frequency in Hz
*         if error, return -1
******************************************************************************/
int64_t XCOMM_SetTxFrequency(uint64_t frequency)
{
    int64_t freq = adf4351_out_altvoltage0_frequency(frequency, ADF4351_TX_CHANNEL);

    if(freq < 0)
        return -1;

    XCOMM_State.txFreq = freq;

    return XCOMM_State.txFreq;
}

/**************************************************************************//**
* @brief Gets the Tx center frequency 
*
* @return If success, return frequency in Hz stored in driver,
*         if error, return -1
******************************************************************************/
int64_t XCOMM_GetTxFrequency(void)
{
    return XCOMM_State.txFreq;
}

/**************************************************************************//**
* @brief Sets the Tx center frequency resolution
*
* @param resolution: desired frequency resolution in Hz
*  
* @return If success, return exact calculated resolution in Hz,
*         if error, return -1
******************************************************************************/
int32_t XCOMM_SetTxResolution(uint32_t resolution)
{
    int32_t res = adf4351_out_altvoltage0_frequency_resolution(resolution, 
                                                               ADF4351_TX_CHANNEL);
    if(res < 0)
        return -1;

    XCOMM_State.txResolution = res;

    return XCOMM_State.txResolution;
}

/**************************************************************************//**
* @brief Gets the Tx center frequency resolution
*
* @param readMode: read frequency resolution from driver or HW
*
* @return If success, return frequency resolution in Hz,
*         if error, return -1
******************************************************************************/
int32_t XCOMM_GetTxResolution(XCOMM_ReadMode readMode)
{
    int32_t res;

    if(readMode == XCOMM_ReadMode_FromHW)
    {
        res = adf4351_out_altvoltage0_frequency_resolution(INT32_MAX, 
                                                           ADF4351_TX_CHANNEL);
        if(res < 0)
            return -1;
        
        XCOMM_State.txResolution = res;
    }

    return XCOMM_State.txResolution;
}

/**************************************************************************//**
* @brief Gets the Tx gain and phase correction for I and Q
*
* @param frequency: center frequency used for the correction in Hz
* @param readMode: read gain and phase correction from driver or HW
*
* @return If success, return IQCorrection struct with gain and phase 
*         correction for the frequency and error set to 0
*         If error, return IQCorrection struct with error set to -1
******************************************************************************/
XCOMM_IQCorrection XCOMM_GetTxIqCorrection(uint64_t frequency, XCOMM_ReadMode readMode)
{
    return XCOMM_State.txIqCorrection;
}

/**************************************************************************//**
* @brief Gets the Tx DC Offset correction
*
* @param frequency: center frequency used for the correction in Hz
* @param readMode: read DC offset correction from driver or HW
*
* @return If success, return DC offset correction
*         if error, return -1
******************************************************************************/
int16_t XCOMM_GetTxDcOffset(uint64_t frequency, XCOMM_ReadMode readMode)
{
    return XCOMM_State.txDcCorrection;
}


/************************ ADC Functions ***************************************/


/**************************************************************************//**
* @brief Sets the sampling rate of the ADC
*
* @param rate: desired rate in Hz
*
* @return  If success, return exact calculated rate in Hz
*          if error, return -1
******************************************************************************/
int64_t XCOMM_SetAdcSamplingRate(uint64_t rate)
{
    int64_t sampleRate = ad9523_out_altvoltage2_ADC_CLK_frequency(rate);

    if(sampleRate < 0)
        return -1;

    XCOMM_State.adcSampleRate = sampleRate;

    return XCOMM_State.adcSampleRate;
}

/**************************************************************************//**
* @brief Gets the sampling rate of the ADC
*
* @param readMode: read rate from driver or HW
*
* @return If success, return rate in Hz
*         if error, return -1
******************************************************************************/
int64_t XCOMM_GetAdcSamplingRate(XCOMM_ReadMode readMode)
{
    int64_t sampleRate;

    if(readMode == XCOMM_ReadMode_FromHW)
    {
        sampleRate = ad9523_out_altvoltage2_ADC_CLK_frequency(INT64_MAX);
        if(sampleRate < 0)
            return -1;
        
        XCOMM_State.adcSampleRate = sampleRate;
    }

    return XCOMM_State.adcSampleRate;;
}

/**************************************************************************//**
* @brief Sets the test mode of the ADC
*
* @param testMode - desired ADC test mode
*
* @return If success, return new ADC test mode if success
*         if error, return -1
******************************************************************************/
XCOMM_AdcTestMode XCOMM_SetAdcTestMode(XCOMM_AdcTestMode testMode)
{
    int32_t mode = ad9643_test_mode(testMode);

    if(mode < 0)
        return -1;

    XCOMM_State.adcTestMode = mode;

    return XCOMM_State.adcTestMode;
}

/**************************************************************************//**
/* @brief Gets the test mode of the ADC
*
* @param readMode: read ADC test mode from driver or HW
*
* @return If success, return ADC test mode
*         if error,return -1
******************************************************************************/
XCOMM_AdcTestMode XCOMM_GetAdcTestMode(XCOMM_ReadMode readMode)
{
    int32_t mode;

    if(readMode == XCOMM_ReadMode_FromHW)
    {
        mode = ad9643_test_mode(XCOMM_AdcTestMode_Ramp + 1);
        if(mode < 0)
            return -1;

        XCOMM_State.adcTestMode = mode;
    }

    return XCOMM_State.adcTestMode;
}


/************************ DAC Functions **************************************/


/**************************************************************************//**
* @brief Sets the sampling rate of the DAC
*
* @param rate: desired rate in Hz
*  
* @return If success, return exact calculated rate in Hz
*         if error, return -1
******************************************************************************/
int64_t XCOMM_SetDacSamplingRate(uint64_t rate)
{
    int64_t sampleRate = ad9523_out_altvoltage1_DAC_CLK_frequency(rate);

    if(sampleRate < 0)
        return -1;

    XCOMM_State.dacSampleRate = sampleRate;

    return XCOMM_State.dacSampleRate;
}

/**************************************************************************//**
/* @brief Gets the sampling rate of the DAC
*
* @param readMode: read rate from driver or HW
*
* @return If success, return rate in Hz
*         if error, return -1
******************************************************************************/
int64_t XCOMM_GetDacSamplingRate(XCOMM_ReadMode readMode)
{
    int64_t sampleRate;
    
    if(readMode == XCOMM_ReadMode_FromHW)
    {
		sampleRate = ad9523_out_altvoltage1_DAC_CLK_frequency(INT64_MAX);
		if(sampleRate < 0)
			return -1;

		XCOMM_State.dacSampleRate = sampleRate;
    }
    
    return XCOMM_State.dacSampleRate;
}