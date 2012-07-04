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
#include <stdint.h>
#include "spi_interface.h"
#include "AD9643.h"

/**************************************************************************//**
* @brief Writes data into a register
*
* @param regAddr - The adress of the register to be written
* @param regVal - The value to be written into the register
*
* @return Returns 0 in case of success or negative error code
******************************************************************************/
int ad9643_write(unsigned int regAddr, unsigned int regVal)
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
int ad9643_read(unsigned int regAddr)
{
	int ret;
	unsigned long data;

	regAddr += 0x8000;
    ret = SPI_Read(SPI_SEL_AD9643, regAddr, &data);

    return (ret < 0 ? ret : (int)data);
}

/***************************************************************************//**
 * @brief Initializes the AD9643. 
 *
 * @return Negative error code or 0 in case of success.
*******************************************************************************/
int32_t ad9643_setup()
{
    ad9643_write(AD9643_REG_CHANNEL_IDX, AD9643_CHANNEL_IDX_ADC_A |
                                         AD9643_CHANNEL_IDX_ADC_B);
	ad9643_dco_output_clock_delay(1000);
    ad9643_write(AD9643_REG_TEST_MODE, 0x00);
    ad9643_write(AD9643_REG_OUTPUT_MODE, 0x00);
    ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
    ad9643_write(AD9643_REG_TRANSFER, 0x00);
	
	return 0;
}

/***************************************************************************//**
 * @brief Configures the external power-down pin function. 
 *
 * @param fnc - The external power-down pin function.
 *				    0 – power-down;
 *					1 - standby.
 *
 * @return The value set for the power-down pin function or negative error code
*******************************************************************************/
int32_t ad9643_ext_pwd_pin_fnc(int32_t fnc)
{
	unsigned char regValue = 0;
	
	if( (fnc == 0) | (fnc == 1) )
	{
		regValue = ad9643_read(AD9643_REG_PWR_MODES);
		regValue &= ~AD9643_PWR_MODES_EXT_PWR_DOWN_PIN_FNC;
		regValue |= (fnc * AD9643_PWR_MODES_EXT_PWR_DOWN_PIN_FNC);
	}
	
	return (ad9643_read(AD9643_REG_PWR_MODES) &
			AD9643_PWR_MODES_EXT_PWR_DOWN_PIN_FNC);
}

/***************************************************************************//**
 * @brief Configures the power mode
 *
 * @param mode - The power mode.
 *				          00 – normal operation;
 *						  01 – full power-down;
 *						  10 – standby;
 *						  11 – reserved.
 *
 * @return The set power mode or negative error code
*******************************************************************************/
int32_t ad9643_pwd_mode(int32_t mode)
{
	unsigned char regValue = 0;
	
	if( (mode == 0) | (mode == 1) | (mode == 2) )
	{
		regValue = ad9643_read(AD9643_REG_PWR_MODES);
		regValue &= ~AD9643_PWR_MODES_INT_PWR_DOWN_MODE(0x3);
		regValue |= AD9643_PWR_MODES_INT_PWR_DOWN_MODE(mode);
		ad9643_write(AD9643_REG_PWR_MODES, regValue);
        ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
	}
	
	return (ad9643_read(AD9643_REG_PWR_MODES) &
			            AD9643_PWR_MODES_INT_PWR_DOWN_MODE(0x3));
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
	if(en == 0)
	{
		ad9643_write(AD9643_REG_GLOBAL_CLK, 0x00);
        ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
	}
	if(en == 1)
	{
		ad9643_write(AD9643_REG_GLOBAL_CLK,
					 AD9643_GLOBAL_CLK_DUTY_CYCLE_STABILIZER_EN);
        ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
	}
	
	return ad9643_read(AD9643_REG_GLOBAL_CLK);
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
	unsigned char regValue = 0;
	
	if( (ratio >= 0) & (ratio <= 7) )
	{
		regValue = ad9643_read(AD9643_REG_CLK_DIV);
		regValue &= ~AD9643_CLK_DIV_RATIO(0x7);
		regValue |= AD9643_CLK_DIV_RATIO(ratio);
		ad9643_write(AD9643_REG_CLK_DIV, regValue);
        ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
	}
	
	return (ad9643_read(AD9643_REG_CLK_DIV) &
			            AD9643_CLK_DIV_RATIO(0x7));
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
	unsigned char regValue = 0;
	
	if( (adj >= 0) & (adj <= 7) )
	{
		regValue = ad9643_read(AD9643_REG_CLK_DIV);
		regValue &= ~AD9643_CLK_IN_CLK_DIV_PHASE_ADJ(0x7);
		regValue |= AD9643_CLK_IN_CLK_DIV_PHASE_ADJ(adj);
		ad9643_write(AD9643_REG_CLK_DIV, regValue);
        ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
	}
	
	return (ad9643_read(AD9643_REG_CLK_DIV) &
			AD9643_CLK_IN_CLK_DIV_PHASE_ADJ(0x7));
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
	unsigned char regValue = 0;
	unsigned char twosComplementOffset = 0;
	
	if( (adj >= -32) & (adj <= 31) )
	{
		regValue = ad9643_read(AD9643_REG_OFFSET_ADJ);
		regValue &= ~AD9643_OFFSET_ADJ(0x3F);
		twosComplementOffset = (adj & 0x3F);
		regValue |= AD9643_OFFSET_ADJ(twosComplementOffset);
		
		ad9643_write(AD9643_REG_OFFSET_ADJ, regValue);
        ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
	}
	
	return (ad9643_read(AD9643_REG_OFFSET_ADJ) &
			            AD9643_OFFSET_ADJ(0x3F));
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
	unsigned char regValue = 0;
	
	if( (en == 0) | (en == 1))
	{
		regValue = ad9643_read(AD9643_REG_OUTPUT_MODE);
		regValue &= ~AD9643_OUTPUT_MODE_OUTPUT_EN;
		regValue |= (en * AD9643_OUTPUT_MODE_OUTPUT_EN);
		ad9643_write(AD9643_REG_OUTPUT_MODE, regValue);
        ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
	}
	
	return (ad9643_read(AD9643_REG_OUTPUT_MODE) &
			            AD9643_OUTPUT_MODE_OUTPUT_EN);
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
	unsigned char regValue = 0;
	
	if( (invert == 0) | (invert == 1))
	{
		regValue = ad9643_read(AD9643_REG_OUTPUT_MODE);
		regValue &= ~AD9643_OUTPUT_MODE_OUTPUT_INVERT;
		regValue |= (invert * AD9643_OUTPUT_MODE_OUTPUT_INVERT);
		ad9643_write(AD9643_REG_OUTPUT_MODE, regValue);
        ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
	}
	
	return (ad9643_read(AD9643_REG_OUTPUT_MODE) &
			            AD9643_OUTPUT_MODE_OUTPUT_INVERT);
}

/***************************************************************************//**
 * @brief Specifies the output format.
 *
 * @param format - The output format.
 *				   Example: 00 – offset binary;
 *							01 – twos complement;
 *							10 – gray code;
 *							11 – reserved.
 *
 * @return The set output format or negative error code
*******************************************************************************/
int32_t ad9643_output_format(int32_t format)
{
	unsigned char regValue = 0;
	
	if( (format == 0) | (format == 1) | (format == 2) )
	{
		regValue = ad9643_read(AD9643_REG_OUTPUT_MODE);
		regValue &= ~AD9643_OUTPUT_MODE_OUTPUT_FORMAT(0x3);
		regValue |= AD9643_OUTPUT_MODE_OUTPUT_FORMAT(format);
		ad9643_write(AD9643_REG_OUTPUT_MODE, regValue);
        ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
	}
	
	return (ad9643_read(AD9643_REG_OUTPUT_MODE) &
			            AD9643_OUTPUT_MODE_OUTPUT_FORMAT(0x3));
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
	unsigned char regValue = 0;
	
	if( (adj >= 0) & (adj <= 7) )
	{
		regValue = ad9643_read(AD9643_REG_OUTPUT_ADJ);
		regValue &= ~AD9643_REG_OUTPUT_ADJ_VAL(0xF);
		regValue |= AD9643_REG_OUTPUT_ADJ_VAL(adj);
		ad9643_write(AD9643_REG_OUTPUT_ADJ, regValue);
        ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
	}
	
	return (ad9643_read(AD9643_REG_OUTPUT_ADJ) &
			            AD9643_REG_OUTPUT_ADJ_VAL(0xF));
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
	unsigned char regValue = 0;
	
	if( (invert == 0) | (invert == 1))
	{
		regValue = ad9643_read(AD9643_REG_CLK_PHASE_CTRL);
		regValue &= ~AD9643_CLK_PHASE_CTRL_INVERT_DCO_CLK;
		regValue |= (invert * AD9643_CLK_PHASE_CTRL_INVERT_DCO_CLK);
		ad9643_write(AD9643_REG_CLK_PHASE_CTRL, regValue);
        ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
	}
	
	return (ad9643_read(AD9643_REG_CLK_PHASE_CTRL) &
			            AD9643_CLK_PHASE_CTRL_INVERT_DCO_CLK);
}

/***************************************************************************//**
 * @brief Enables (0) or disables (1) the even/odd mode output.
 *
 * @param mode - The even/odd mode output.
 *				 Example: 0 - Enables the even/odd mode output;
 *						  1 - Disables the even/odd mode output.
 *
 * @return The set clock mode or negative error code
*******************************************************************************/
int32_t ad9643_dco_clock_mode(int32_t mode)
{
	unsigned char regValue = 0;
	
	if( (mode == 0) | (mode == 1))
	{
		regValue = ad9643_read(AD9643_REG_CLK_PHASE_CTRL);
		regValue &= ~AD9643_CLK_PHASE_CTRL_MODE;
		regValue |= (mode * AD9643_CLK_PHASE_CTRL_MODE);
		ad9643_write(AD9643_REG_CLK_PHASE_CTRL, regValue);
        ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
	}
	
	return (ad9643_read(AD9643_REG_CLK_PHASE_CTRL) &
			AD9643_CLK_PHASE_CTRL_MODE);
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
	unsigned char regValue = 0;
	
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
	unsigned char regValue = 0;
	unsigned char twosComplementSpan = 0;
	
	if( (span >= -16) & (span <= 15) )
	{
		regValue = ad9643_read(AD9643_REG_INPUT_SPAN_SEL);
		regValue &= ~AD9643_INPUT_SPAN_SEL_FULL_SCALE_VOLTAGE(0x1F);
		twosComplementSpan = (span & 0x1F);
		regValue |= AD9643_INPUT_SPAN_SEL_FULL_SCALE_VOLTAGE(twosComplementSpan);
		ad9643_write(AD9643_REG_INPUT_SPAN_SEL, regValue);
        ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
	}
	
	return (ad9643_read(AD9643_REG_INPUT_SPAN_SEL) &
			            AD9643_INPUT_SPAN_SEL_FULL_SCALE_VOLTAGE(0x1F));
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
    if(mode >= AD9643_TEST_MODE_OFF && mode <= AD9643_TEST_MODE_RAMP)
    {
        ad9643_write(AD9643_REG_TEST_MODE, mode);
        ad9643_write(AD9643_REG_TRANSFER, AD9643_TRANSFER_EN);
    }

    return  ad9643_read(AD9643_REG_TEST_MODE);
}
