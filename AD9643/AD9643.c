/***************************************************************************//**
 *   @file   AD9643.c
 *   @brief  Implementation of AD9643 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: $WCREV$
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "spi_interface.h"
#include "adc_core.h"
#include "AD9643.h"

extern void msleep(uint32_t ms_count);

/**************************************************************************//**
* @brief Writes data into a register
*
* @param regAddr - The adress of the register to be written
* @param regVal - The value to be written into the register
*
* @return Returns 0 in case of success or negative error code
******************************************************************************/
int32_t ad9643_write(uint32_t regAddr, uint32_t regVal)
{
    return SPI_Write(SPI_SEL_AD9643, regAddr, regVal);
}

/**************************************************************************//**
* @brief Reads data from a register
*
* @param regAddr - The adress of the register to be read
*
* @return Returns the read data or negative error code
******************************************************************************/
int32_t ad9643_read(uint32_t regAddr)
{
	int32_t ret;
	uint32_t data;

	regAddr += 0x8000;
    ret = SPI_Read(SPI_SEL_AD9643, regAddr, &data);

    return (ret < 0 ? ret : (int32_t)data);
}

/***************************************************************************//**
 * @brief Sets the AD9643 test mode for DCO delay calibration
 *
 * @param chan_mask - Selects the internal ADC the command is applied to.
 * @param mode - ADC test mode
 *
 * @return Negative error code or 0 in case of success.
*******************************************************************************/
int32_t ad9643_testmode_set(uint32_t chan_mask, uint32_t mode)
{
	int32_t ret = 0;

	switch (mode)
	{
	case AD9643_TEST_MODE_PN23_SEQ:
	case AD9643_TEST_MODE_PN9_SEQ:
	case AD9643_TEST_MODE_ALTERNATING_CHECKERBOARD:
        ADC_Core_Write(ADC_CORE_ADC_CTRL,0);
		ret = ad9643_write(AD9643_REG_OUTPUT_MODE,
			        	  (AD9643_OUTPUT_MODE_DEF | AD9643_OUTPUT_MODE_TWOS_COMPLEMENT) &
			        	  ~AD9643_OUTPUT_MODE_TWOS_COMPLEMENT);
		break;
	default:
        ADC_Core_Write(ADC_CORE_ADC_CTRL,ADC_CORE_SIGNEXTEND);
		ret = ad9643_write(AD9643_REG_OUTPUT_MODE, (AD9643_OUTPUT_MODE_DEF |
                                           	   	 AD9643_OUTPUT_MODE_TWOS_COMPLEMENT));
	};
	if(ret < 0)
		return ret;

	ret = ad9643_write(AD9643_REG_CHANNEL_IDX, chan_mask);
	if(ret < 0)
		return ret;

	ret = ad9643_write(AD9643_REG_TEST_MODE, mode);
	if(ret < 0)
		return ret;

	ret = ad9643_write(AD9643_REG_CHANNEL_IDX, 0x3);
	if(ret < 0)
		return ret;

	ret = ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
	if(ret < 0)
		return ret;

	return ret;
}

/***************************************************************************//**
 * @brief Calibrates the DCO clock delay
 *
 * @return Negative error code or 0 in case of success.
*******************************************************************************/
int32_t ad9643_dco_calibrate_2c()
{
	int32_t dco, ret, cnt, start, max_start, max_cnt;
	uint32_t stat;
	uint8_t err_field[33];

	ad9643_testmode_set(0x2, AD9643_TEST_MODE_PN23_SEQ);
	ad9643_testmode_set(0x1, AD9643_TEST_MODE_PN9_SEQ);

    ADC_Core_Write(ADC_CORE_PN_ERR_CTRL, ADC_CORE_PN23_1_EN | ADC_CORE_PN9_0_EN);

	for(dco = 0; dco <= 32; dco++)
    {
		ret = 0;
		ad9643_write(AD9643_REG_DCO_OUTPUT_DELAY, dco > 0 ? ((dco - 1) | 0x80) : 0);
		ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
        ADC_Core_Write(ADC_CORE_ADC_STAT, ADC_CORE_ADC_STAT_MASK);
		cnt = 4;

		do {
			//msleep(8);
            ADC_Core_Read(ADC_CORE_ADC_STAT, &stat);
			if ((cnt-- < 0) | (stat & (ADC_CORE_ADC_STAT_PN_ERR0 |
				ADC_CORE_ADC_STAT_PN_ERR1))) {
				ret = -1;
				break;
			}
		} while (stat & (ADC_CORE_ADC_STAT_PN_OOS0 |
			             ADC_CORE_ADC_STAT_PN_OOS1));

		cnt = 4;

		if (!ret)
			do {
				//msleep(4);
                ADC_Core_Read(ADC_CORE_ADC_STAT, &stat);
				if (stat & (ADC_CORE_ADC_STAT_PN_ERR0 |
					        ADC_CORE_ADC_STAT_PN_ERR1)) {
					ret = -1;
					break;
				}
			} while (cnt--);

		err_field[dco] = !!ret;
	}

	ret = -1;
	for(dco = 0, cnt = 0, max_cnt = 0, start = -1, max_start = 0;
		dco <= 32; dco++) {
		if (err_field[dco] == 0) {
			if (start == -1)
				start = dco;
			cnt++;
			ret = 0;
		} else {
			if (cnt > max_cnt) {
				max_cnt = cnt;
				max_start = start;
			}
			start = -1;
			cnt = 0;
		}
	}

	if (cnt > max_cnt) {
		max_cnt = cnt;
		max_start = start;
	}

	dco = max_start + (max_cnt / 2);

	ad9643_testmode_set(0x3, AD9643_TEST_MODE_OFF);
	ad9643_write(AD9643_REG_DCO_OUTPUT_DELAY,
		         dco > 0 ? ((dco - 1) | 0x80) : 0);
	ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);

	return ret;
}

/***************************************************************************//**
 * @brief Initializes the AD9643. 
 *
 * @return Negative error code or 0 in case of success.
*******************************************************************************/
int32_t ad9643_setup()
{
    int32_t ret = 0;
	
	ad9643_reset();
	ad9643_write(AD9643_REG_CLK_PHASE_CTRL, AD9643_CLK_PHASE_CTRL_EVEN_ODD_MODE_EN);
    ADC_Core_Write(ADC_CORE_ADC_CTRL,ADC_CORE_SIGNEXTEND | ADC_CORE_SCALE_OFFSET_EN);
    ADC_Core_Write(ADC_CORE_CA_OFFS_SCALE,ADC_CORE_OFFSET(0) | ADC_CORE_SCALE(0x8000));
    ADC_Core_Write(ADC_CORE_CB_OFFS_SCALE,ADC_CORE_OFFSET(0) | ADC_CORE_SCALE(0x8000));
	ad9643_write(AD9643_REG_OUTPUT_MODE, AD9643_OUTPUT_MODE_DEF | AD9643_OUTPUT_MODE_TWOS_COMPLEMENT);
	ad9643_write(AD9643_REG_TEST_MODE, AD9643_TEST_MODE_OFF);
	ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
	ret = ad9643_dco_calibrate_2c();
	if(ret)
	{
		ad9643_dco_clock_invert(1);
		ret = ad9643_dco_calibrate_2c();
	}

    ADC_Core_Write(ADC_CORE_DMA_CHAN_SEL,0x02);

	return ret;
}

/***************************************************************************//**
 * @brief Resets the device.
 *
 * @return Returns negative error code or 0 in case of success.
*******************************************************************************/
int32_t ad9643_reset(void)
{
	int32_t ret;

    ret = ad9643_write(AD9643_REG_SPI_CONFIG, AD9643_SPI_CONFIG_SOFT_RESET);

    return ret;
}

/***************************************************************************//**
 * @brief Configures the external power-down pin function. 
 *
 * @param fnc - The external power-down pin function.
 *				    0 � power-down
 *					1 - standby
 *
 * @return The value set for the power-down pin function or negative error code
*******************************************************************************/
int32_t ad9643_ext_pwd_pin_fnc(int32_t fnc)
{
	int32_t ret;
    uint8_t regValue = 0;
	
	if( (fnc == 0) | (fnc == 1) )
	{
		regValue = ad9643_read(AD9643_REG_PWR_MODES);
		if(regValue < 0)
            return regValue;

        regValue &= ~AD9643_PWR_MODES_EXT_PWR_DOWN_PIN_FNC;
		regValue |= (fnc * AD9643_PWR_MODES_EXT_PWR_DOWN_PIN_FNC);
        
        ret = ad9643_write(AD9643_REG_PWR_MODES, regValue);
        if(ret < 0)
            return ret;
	}
    else
    {
        ret = ad9643_read(AD9643_REG_PWR_MODES);
        if(ret < 0)
            return ret;

        fnc = (ret & AD9643_PWR_MODES_EXT_PWR_DOWN_PIN_FNC) != 0;
    }

    return fnc;
}

/***************************************************************************//**
 * @brief Configures the power mode
 *
 * @param mode - The power mode.
 *				          00 � normal operation;
 *						  01 � full power-down;
 *						  10 � standby;
 *						  11 � reserved.
 *
 * @return The set power mode or negative error code
*******************************************************************************/
int32_t ad9643_pwd_mode(int32_t mode)
{
    int32_t ret;
    uint8_t regValue = 0;
	
	if( (mode == 0) | (mode == 1) | (mode == 2) )
	{
		regValue = ad9643_read(AD9643_REG_PWR_MODES);
        if(regValue < 0)
            return regValue;

		regValue &= ~AD9643_PWR_MODES_INT_PWR_DOWN_MODE(0x3);
		regValue |= AD9643_PWR_MODES_INT_PWR_DOWN_MODE(mode);
		
        ret = ad9643_write(AD9643_REG_PWR_MODES, regValue);
        if(ret < 0)
            return ret;

        ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
        if(ret < 0)
            return ret;
	}
    else
    {
        ret = ad9643_read(AD9643_REG_PWR_MODES);
        if(ret < 0)
            return ret;

        mode = (ret & AD9643_PWR_MODES_INT_PWR_DOWN_MODE(0x3));
    }

    return mode;
}

/***************************************************************************//**
 * @brief Enables (0) or disables (1) the duty cycle stabilizer.
 *
 * @param - en - Enable option.
 *				 Example: 0 - Enables the duty cycle stabilizer.
 *						  1 - Disables the duty cycle stabilizer.
 *
 * @return The status of the duty cycle stabilizer or negative error code
*******************************************************************************/
int32_t ad9643_clock_duty_cycle_stabilizer(int32_t en)
{
    int32_t ret;

	if(en == 0)
	{
		ret = ad9643_write(AD9643_REG_GLOBAL_CLK, 0x00);
        if(ret < 0)
            return ret;
        ret = ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
        if(ret < 0)
            return ret;
	}
	else if(en == 1)
	{
		ret = ad9643_write(AD9643_REG_GLOBAL_CLK,
					 AD9643_GLOBAL_CLK_DUTY_CYCLE_STABILIZER_EN);
        if(ret < 0)
            return ret;
        ret = ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
        if(ret < 0)
            return ret;
	}
    else
    {
	    ret = ad9643_read(AD9643_REG_GLOBAL_CLK);
        if(ret < 0)
            return ret;
		en = ret & AD9643_GLOBAL_CLK_DUTY_CYCLE_STABILIZER_EN;
    }

    return en;
}

/***************************************************************************//**
 * @brief Configures the input clock divide ratio and returns the set divide
 *		  ratio.
 *
 * @param ratio - The input clock divide ratio.
 *				  Example: 000 = divide by 1;
 *						   001 = divide by 2;
 *						   010 = divide by 3;
 *						   011 = divide by 4;
 *						   100 = divide by 5;
 *						   101 = divide by 6;
 *						   110 = divide by 7;
 *						   111 = divide by 8.
 *
 * @return The set divide ratio or negative error code
*******************************************************************************/
int32_t ad9643_clock_divide_ratio(int32_t ratio)
{
	int32_t ret;
    uint8_t regValue = 0;
	
	if( (ratio >= 0) & (ratio <= 7) )
	{
		regValue = ad9643_read(AD9643_REG_CLK_DIV);
        if(regValue < 0)
            return regValue;

		regValue &= ~AD9643_CLK_DIV_RATIO(0x7);
		regValue |= AD9643_CLK_DIV_RATIO(ratio);

		ret = ad9643_write(AD9643_REG_CLK_DIV, regValue);
        if(ret < 0)
            return ret;

        ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
        if(ret < 0)
            return ret;
	}
    else
    {
	    ret = ad9643_read(AD9643_REG_CLK_DIV);
        if(ret < 0)
            return ret;
        
        ratio = ret & AD9643_CLK_DIV_RATIO(0x7);
    }

    return ratio;
}

/***************************************************************************//**
 * @brief Configures the phase adjustment in clock cycles delay.
 *
 * @param adj - The phase adjustment in clock cycles delay.
 *				Example: 000 = no delay;
 *						 001 = 1 input clock cycle;
 *						 010 = 2 input clock cycles;
 *						 011 = 3 input clock cycles;
 *						 100 = 4 input clock cycles;
 *						 101 = 5 input clock cycles;
 *						 110 = 6 input clock cycles;
 *						 111 = 7 input clock cycles.
 *
 * @return The set phase adjustment or negative error code
*******************************************************************************/
int32_t ad9643_clock_phase_adj(int32_t adj)
{
	int32_t ret;
    uint8_t regValue = 0;
	
	if( (adj >= 0) & (adj <= 7) )
	{
		regValue = ad9643_read(AD9643_REG_CLK_DIV);
        if(regValue < 0)
            return regValue;

		regValue &= ~AD9643_CLK_IN_CLK_DIV_PHASE_ADJ(0x7);
		regValue |= AD9643_CLK_IN_CLK_DIV_PHASE_ADJ(adj);

		ret = ad9643_write(AD9643_REG_CLK_DIV, regValue);
        if(ret < 0)
            return ret;

        ret = ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
        if(ret < 0)
            return ret;
	}
	else
	{
		ret = ad9643_read(AD9643_REG_CLK_DIV);
		if(ret < 0)
            return ret;
			
		adj = (ret & AD9643_CLK_IN_CLK_DIV_PHASE_ADJ(0x7)) >> 3;
	}
	return adj;
}

/***************************************************************************//**
 * @brief Sets the offset adjustment.
 *
 * @param adj - The offset adjust value in LSBs from +31 to -32.
 *
 * @return The set offset adjustment or negative error code
*******************************************************************************/
int32_t ad9643_offset_adj(int32_t adj)
{
	int32_t ret;
    uint8_t regValue = 0;
	uint8_t twosComplementOffset = 0;
	
	if( (adj >= -32) & (adj <= 31) )
	{
		regValue = ad9643_read(AD9643_REG_OFFSET_ADJ);
		if(regValue < 0)
            return regValue;
        
        regValue &= ~AD9643_OFFSET_ADJ(0x3F);
		twosComplementOffset = (adj & 0x3F);
		regValue |= AD9643_OFFSET_ADJ(twosComplementOffset);
		
		ret = ad9643_write(AD9643_REG_OFFSET_ADJ, regValue);
        ret = ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
	}
	else
	{
		ret = ad9643_read(AD9643_REG_OFFSET_ADJ);
		if(ret < 0)
            return ret;
			
		adj = ret & AD9643_OFFSET_ADJ(0x3F);
	}
	
	return adj;
}

/***************************************************************************//**
 * @brief Enables (1) or disables (0) the data output.
 *
 * @param en - Enable option.
 *			   Example: 0 - Disables the data output.
 *						1 - Enables the data output.
 *
 * @return Output enable state or negative error code
*******************************************************************************/
int32_t ad9643_output_enable(int32_t en)
{
	int32_t ret;
    uint8_t regValue = 0;
	
	if( (en == 0) | (en == 1))
	{
		regValue = ad9643_read(AD9643_REG_OUTPUT_MODE);
		if(regValue < 0)
            return regValue;
        
        regValue &= ~AD9643_OUTPUT_MODE_OUTPUT_EN;
		regValue |= (en * AD9643_OUTPUT_MODE_OUTPUT_EN);
		
        ret = ad9643_write(AD9643_REG_OUTPUT_MODE, regValue);
        if(ret < 0)
            return ret;

        ret = ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
        if(ret < 0)
            return ret;
	}
	else
	{
		ret = ad9643_read(AD9643_REG_OUTPUT_MODE);
		if(ret < 0)
            return ret;
			
		en = (ret & AD9643_OUTPUT_MODE_OUTPUT_EN) != 0;
	}
	return en;
}

/***************************************************************************//**
 * @brief Activates the normal (1) or inverted (0) output mode.
 *
 * @param invert - Invert option.
 *				   Example: 0 - Activates the inverted output mode;
 *							1 - Activates the normal output mode.
 *
 * @return The set output mode or negative error code
*******************************************************************************/
int32_t ad9643_output_invert(int32_t invert)
{
	int32_t ret;
    uint8_t regValue = 0;
	
	if( (invert == 0) | (invert == 1))
	{
		regValue = ad9643_read(AD9643_REG_OUTPUT_MODE);
		if(regValue < 0)
            return regValue;
        
        regValue &= ~AD9643_OUTPUT_MODE_OUTPUT_INVERT;
		regValue |= (invert * AD9643_OUTPUT_MODE_OUTPUT_INVERT);
		
        ret = ad9643_write(AD9643_REG_OUTPUT_MODE, regValue);
        if(ret < 0)
            return ret;

        ret = ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
        if(ret < 0)
            return ret;
	}
	else
	{
		ret = ad9643_read(AD9643_REG_OUTPUT_MODE);
		if(ret < 0)
            return ret;
		
		invert = (ret & AD9643_OUTPUT_MODE_OUTPUT_INVERT) != 0;
	}
	
	return invert;
}

/***************************************************************************//**
 * @brief Specifies the output format.
 *
 * @param format - The output format.
 *				   Example: 00 � offset binary;
 *							01 � twos complement;
 *							10 � gray code;
 *							11 � reserved.
 *
 * @return The set output format or negative error code
*******************************************************************************/
int32_t ad9643_output_format(int32_t format)
{
	int32_t ret;
    uint8_t regValue = 0;
	
	if( (format == 0) | (format == 1) | (format == 2) )
	{
		regValue = ad9643_read(AD9643_REG_OUTPUT_MODE);
		if(regValue < 0)
            return regValue;
        
        regValue &= ~AD9643_OUTPUT_MODE_OUTPUT_FORMAT(0x3);
		regValue |= AD9643_OUTPUT_MODE_OUTPUT_FORMAT(format);
		
        ret = ad9643_write(AD9643_REG_OUTPUT_MODE, regValue);
        if(ret < 0)
            return ret;

        ret = ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
        if(ret < 0)
            return ret;
	}
	else
	{
		ret = ad9643_read(AD9643_REG_OUTPUT_MODE);		
		if(ret < 0)
            return ret;
		
		format = ret & AD9643_OUTPUT_MODE_OUTPUT_FORMAT(0x3);
	}
	
	return format;
}

/***************************************************************************//**
 * @brief Sets the output current adjustment.
 *
 * @param adj - The output current adjustment.
 *				Example: 0000 = 3.72 mA output drive current;
 *						 0001 = 3.5 mA output drive current (default);
 *						 0010 = 3.30 mA output drive current;
 *						 0011 = 2.96 mA output drive current;
 *						 0100 = 2.82 mA output drive current;
 *						 0101 = 2.57 mA output drive current;
 *						 0110 = 2.27 mA output drive current;
 *						 0111 = 2.0 mA output drive current (reduced range);
 *						 1000 to 1111 = reserved;
 *
 * @return The set current adjustment or negative error code
*******************************************************************************/
int32_t ad9643_output_current_adj(int32_t adj)
{
	int32_t ret;
    uint8_t regValue = 0;
	
	if( (adj >= 0) & (adj <= 7) )
	{
		regValue = ad9643_read(AD9643_REG_OUTPUT_ADJ);
		if(regValue < 0)
            return regValue;
        
        regValue &= ~AD9643_REG_OUTPUT_ADJ_VAL(0xF);
		regValue |= AD9643_REG_OUTPUT_ADJ_VAL(adj);
		
        ret = ad9643_write(AD9643_REG_OUTPUT_ADJ, regValue);
        if(ret < 0)
            return ret;

        ret = ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
        if(ret < 0)
            return ret;
	}
	else
	{
		ret = ad9643_read(AD9643_REG_OUTPUT_ADJ);
		if(ret < 0)
            return ret;
		
		adj = AD9643_REG_OUTPUT_ADJ_VAL(0xF);
	}
	
	return adj;
}

/***************************************************************************//**
 * @brief Activates the normal (0) or inverted (1) DCO clock.
 *
 * @param invert - Invert option.
 *				   Example: 0 - Activates the normal DCO clock;
 *							1 - Activates the inverted DCO clock.
 *
 * @return The DCO clock inversion status or negative error code
*******************************************************************************/
int32_t ad9643_dco_clock_invert(int32_t invert)
{
	int32_t ret;
    uint8_t regValue = 0;
	
	if( (invert == 0) | (invert == 1))
	{
		regValue = ad9643_read(AD9643_REG_CLK_PHASE_CTRL);
		if(regValue < 0)
            return regValue;
        
        regValue &= ~AD9643_CLK_PHASE_CTRL_INVERT_DCO_CLK;
		regValue |= (invert * AD9643_CLK_PHASE_CTRL_INVERT_DCO_CLK);
		
        ret = ad9643_write(AD9643_REG_CLK_PHASE_CTRL, regValue);
        if(ret < 0)
            return ret;

        ret = ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
        if(ret < 0)
            return ret;
	}
	else
	{
		ret = ad9643_read(AD9643_REG_CLK_PHASE_CTRL);
		if(ret < 0)
            return ret;
		
		invert = (ret & AD9643_CLK_PHASE_CTRL_INVERT_DCO_CLK) != 0;
	}
	
	return invert;
}

/***************************************************************************//**
 * @brief Enables (0) or disables (1) the even/odd mode output.
 *
 * @param mode - The even/odd mode output.
 *				 Example: 1 - Enables the even/odd mode output;
 *						  0 - Disables the even/odd mode output.
 *
 * @return The set clock mode or negative error code
*******************************************************************************/
int32_t ad9643_dco_clock_mode(int32_t mode)
{
	int32_t ret;
    uint8_t regValue = 0;
	
	if( (mode == 0) | (mode == 1))
	{
		regValue = ad9643_read(AD9643_REG_CLK_PHASE_CTRL);
		if(regValue < 0)
            return regValue;
        
        regValue &= ~AD9643_CLK_PHASE_CTRL_MODE;
		regValue |= (mode * AD9643_CLK_PHASE_CTRL_MODE);
		
        ret = ad9643_write(AD9643_REG_CLK_PHASE_CTRL, regValue);
        if(ret < 0)
            return ret;

        ret = ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
        if(ret < 0)
            return ret;
	}
	else
	{
		ret = ad9643_read(AD9643_REG_CLK_PHASE_CTRL);
		if(ret < 0)
            return ret;
		
		mode = (ret & AD9643_CLK_PHASE_CTRL_MODE) != 0;
	}
	
	return mode;
}

/***************************************************************************//**
 * @brief Configures the clock delay setting.
 *
 * @param delay - The clock delay setting in ps [0, 3200]. Setting the delay to 0
 * 				  disables the DCO output clock delay.
 *
 * @return The set clock delay or negative error code
*******************************************************************************/
int32_t ad9643_dco_output_clock_delay(int32_t delay)
{
    uint8_t regValue = 0;
	
	if (!delay) {
		ad9643_write(AD9643_REG_DCO_OUTPUT_DELAY, 0);
        ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
	} else if (delay <= 3200) {
		delay = (delay - 100) / 100;
		ad9643_write(AD9643_REG_DCO_OUTPUT_DELAY,
					 AD9643_DCO_OUTPUT_DELAY_EN_DCO_CLK_DELAY |
					 AD9643_DCO_OUTPUT_DELAY_DCO_CLK_DELAY(delay));
        ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
	}
	
	regValue = ad9643_read(AD9643_REG_DCO_OUTPUT_DELAY) & AD9643_DCO_OUTPUT_DELAY_EN_DCO_CLK_DELAY;
	if(regValue)
		return ((ad9643_read(AD9643_REG_DCO_OUTPUT_DELAY) &
			     AD9643_DCO_OUTPUT_DELAY_DCO_CLK_DELAY(0x1F)) * 100 + 100);
	else
		return 0;
}

/***************************************************************************//**
 * @brief Configures the full-scale input voltage selection.
 *
 * @param span - Full-scale input voltage selection.
 *				 Example: 01111 = 2.087 V p-p;
 *						  ...
 *						  00001 = 1.772 V p-p;
 *						  00000 = 1.75 V p-p (default);
 *						  11111 = 1.727 V p-p;
 *						  ...
 *						  10000 = 1.383 V p-p.
 *
 * @return The set input voltage selection or negative error code
*******************************************************************************/
int32_t ad9643_input_span(int32_t span)
{
	int32_t ret;
    uint8_t regValue = 0;
	uint8_t twosComplementSpan = 0;
	
	if( (span >= -16) & (span <= 15) )
	{
		regValue = ad9643_read(AD9643_REG_INPUT_SPAN_SEL);
		if(regValue < 0)
            return regValue;
        
        regValue &= ~AD9643_INPUT_SPAN_SEL_FULL_SCALE_VOLTAGE(0x1F);
		twosComplementSpan = (span & 0x1F);
		regValue |= AD9643_INPUT_SPAN_SEL_FULL_SCALE_VOLTAGE(twosComplementSpan);
		
        ret = ad9643_write(AD9643_REG_INPUT_SPAN_SEL, regValue);
        if(ret < 0)
            return ret;

        ret = ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
        if(ret < 0)
            return ret;
	}
	else
	{
		ret = ad9643_read(AD9643_REG_INPUT_SPAN_SEL);
		if(ret < 0)
            return ret;
		
		span = ret &  AD9643_INPUT_SPAN_SEL_FULL_SCALE_VOLTAGE(0x1F);
	}
	
	return span;
}

/***************************************************************************//**
 * @brief Sets the ADC's test mode. 
 *
 * @param mode - ADC test mode 
 *             - for valid values see the Test Modes Definitions in AD9643.h
 *
 * @return Returns the set test mode or negative error code
*******************************************************************************/
int32_t ad9643_test_mode(int32_t mode)
{
    int32_t ret;
	
	if(mode >= AD9643_TEST_MODE_OFF && mode <= AD9643_TEST_MODE_RAMP)
    {
        ret = ad9643_write(AD9643_REG_TEST_MODE, mode);
		if(ret < 0)
            return ret;
			
        ret = ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
		if(ret < 0)
            return ret;
    }
	else
	{
		mode = ad9643_read(AD9643_REG_TEST_MODE);
	}
    
	return  mode;
}

/***************************************************************************//**
 * @brief Sets the ADC's user test pattern. 
 *
 * @param pattern - Buffer holding the user test pattern (8 bytes)
  *
 * @return Returns 0 for success or negative error code
*******************************************************************************/
int32_t ad9643_user_test_pattern(uint8_t* pattern)
{
    int32_t ret, i;

    for(i = 0; i < 8; i++)
    {
        ret = ad9643_write(AD9643_REG_USER_TEST_PATTERN_1LSB + i, pattern[i]);
		if(ret < 0)
            break;
    }

    return ret;
}
