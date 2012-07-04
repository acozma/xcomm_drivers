/***************************************************************************//**
 *   @file   AD9523_cfg.h
 *   @brief  Header file of AD9523 Driver Configuration.
 *   @author ACozma (andrei.cozma@analog.com)
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
#ifndef __AD9523_CFG_H__
#define __AD9523_CFG_H__

#define AD9523_SPI_3_WIRE 1

struct ad9523_channel_spec ad9523_channels[] = 
{
    {	/* ZD output */
		.channel_num = 0,
		.divider_output_invert_en = 0,
		.sync_ignore_en = 0,
		.low_power_mode_en = 0,
		.use_alt_clock_src = 0,
		.output_dis = 0,
		.driver_mode = LVDS_4mA,
		.divider_phase = 0,
		.channel_divider = 8,
		.extended_name = "ZD_OUTPUT",
	},
	{	/* DAC CLK */
		.channel_num = 1,
		.divider_output_invert_en = 0,
		.sync_ignore_en = 0,
		.low_power_mode_en = 0,
		.use_alt_clock_src = 0,
		.output_dis = 0,
		.driver_mode = LVPECL_8mA,
		.divider_phase = 0,
		.channel_divider = 2,
		.extended_name = "DAC_CLK",
	},
	{	/* ADC CLK */
		.channel_num = 2,
		.divider_output_invert_en = 0,
		.sync_ignore_en = 0,
		.low_power_mode_en = 0,
		.use_alt_clock_src = 0,
		.output_dis = 0,
		.driver_mode = LVDS_7mA,
		.divider_phase = 0,
		.channel_divider = 4,
		.extended_name = "ADC_CLK",
	},
	{	/* DAC REF CLK */
		.channel_num = 4,
		.divider_output_invert_en = 0,
		.sync_ignore_en = 0,
		.low_power_mode_en = 0,
		.use_alt_clock_src = 0,
		.output_dis = 0,
		.driver_mode = LVDS_4mA,
		.divider_phase = 0,
		.channel_divider = 16,
		.extended_name = "DAC_REF_CLK",
	},
	{	/* TX LO REF */
		.channel_num = 5,
		.divider_output_invert_en = 0,
		.sync_ignore_en = 0,
		.low_power_mode_en = 0,
		.use_alt_clock_src = 0,
		.output_dis = 0,
		.driver_mode = CMOS_CONF3,
		.divider_phase = 0,
		.channel_divider = 8,
		.extended_name = "TX_LO_REF_CLK",
	},
	{	/* DAC DCO */
		.channel_num = 6,
		.divider_output_invert_en = 0,
		.sync_ignore_en = 0,
		.low_power_mode_en = 0,
		.use_alt_clock_src = 0,
		.output_dis = 0,
		.driver_mode = LVDS_7mA,
		.divider_phase = 0,
		.channel_divider = 2,
		.extended_name = "DAC_DCO_CLK",
	},
	{	/* ADC SYNC */
		.channel_num = 8,
		.divider_output_invert_en = 0,
		.sync_ignore_en = 0,
		.low_power_mode_en = 0,
		.use_alt_clock_src = 0,
		.output_dis = 0,
		.driver_mode = CMOS_CONF3,
		.divider_phase = 1,
		.channel_divider = 32,
		.extended_name = "ADC_SYNC_CLK",
	},
	{	/* RX LO REF */
		.channel_num = 9,
		.divider_output_invert_en = 0,
		.sync_ignore_en = 0,
		.low_power_mode_en = 0,
		.use_alt_clock_src = 0,
		.output_dis = 0,
		.driver_mode = CMOS_CONF3,
		.divider_phase = 0,
		.channel_divider = 8,
		.extended_name = "RX_LO_REF_CLK",
	},
};

struct ad9523_platform_data ad9523_pdata_lpc = 
{
	.vcxo_freq = 122880000,
 
	/* Single-Ended Input Configuration */
	.refa_diff_rcv_en = 1,
	.refb_diff_rcv_en = 0,
	.zd_in_diff_en = 1,
	.osc_in_diff_en = 0,

	.refa_cmos_neg_inp_en = 0,
	.refb_cmos_neg_inp_en = 1,
	.zd_in_cmos_neg_inp_en = 0,
	.osc_in_cmos_neg_inp_en = 1,
 
	.refa_r_div = 0,
	.refb_r_div = 0,
	.pll1_feedback_div = 4,
	.pll1_charge_pump_current_nA = 2000,
	.zero_delay_mode_internal_en = 1,
	.osc_in_feedback_en = 0,
	.pll1_loop_filter_rzero = 3,
 
	.ref_mode = REVERT_TO_REFA,
 
	.pll2_charge_pump_current_nA = 420000,
	.pll2_ndiv_a_cnt = 0,
	.pll2_ndiv_b_cnt = 3,
	.pll2_freq_doubler_en = 1,
	.pll2_r2_div = 1,
	.pll2_vco_diff_m1 = 3,
	.pll2_vco_diff_m2 = 3,
 
	.rpole2 = 0,
	.rzero = 2,
	.cpole1 = 2,
	.rzero_bypass_en = 0,
 
	/* Output Channel Configuration */
	.num_channels = ARRAY_SIZE(ad9523_channels),
	.channels = ad9523_channels,
	.name = "ad9523-lpc"
};

#endif // __AD9523_CFG_H__
