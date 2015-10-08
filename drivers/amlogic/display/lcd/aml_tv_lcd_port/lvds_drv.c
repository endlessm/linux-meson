/*
 * drivers/amlogic/display/lcd/aml_tv_lcd_port/lvds_drv.c
 *
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
*/

#include <mach/register.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <mach/am_regs.h>
#include <linux/amlogic/display/lcd.h>

#include "../aml_lcd_tv.h"
#include "lcd_common.h"

static void set_tcon_lvds(Lcd_Config_t *pConf)
{
	vpp_set_matrix_ycbcr2rgb(2, 0);
	aml_write_reg32(P_ENCL_VIDEO_RGBIN_CTRL, 3);
	aml_write_reg32(P_L_RGB_BASE_ADDR, 0);
	aml_write_reg32(P_L_RGB_COEFF_ADDR, 0x400);

	if (pConf->lcd_control.lvds_config->lvds_bits == 8)
		aml_write_reg32(P_L_DITH_CNTL_ADDR,  0x400);
	else if (pConf->lcd_control.lvds_config->lvds_bits == 6)
		aml_write_reg32(P_L_DITH_CNTL_ADDR,  0x600);
	else
		aml_write_reg32(P_L_DITH_CNTL_ADDR,  0);

	aml_write_reg32(P_VPP_MISC, aml_read_reg32(P_VPP_MISC) & ~(VPP_OUT_SATURATE));
}

static void init_phy_lvds(Lcd_Config_t *pConf)
{
	aml_write_reg32(P_VPU_VLOCK_GCLK_EN, 7);
	aml_write_reg32(P_VPU_VLOCK_ADJ_EN_SYNC_CTRL, 0x108010ff);
	aml_write_reg32(P_VPU_VLOCK_CTRL, 0xe0f50f1b);

	aml_write_reg32(P_LVDS_PHY_CNTL0, 0xffff);
	aml_write_reg32(P_LVDS_PHY_CNTL1, 0xff00);
	aml_write_reg32(P_LVDS_PHY_CNTL4, 0x007f);

	aml_write_reg32(P_HHI_DIF_CSI_PHY_CNTL1, 0x6c6cca80);
	aml_write_reg32(P_HHI_DIF_CSI_PHY_CNTL2, 0x0000006c);
	aml_write_reg32(P_HHI_DIF_CSI_PHY_CNTL3, 0x0fff0800);
	//od   clk 1039.5 / 2 = 519.75 = 74.25*7
	aml_write_reg32(P_HHI_LVDS_TX_PHY_CNTL0, 0x0fff0040);
}

static void set_control_lvds(Lcd_Config_t *pConf)
{
	unsigned int bit_num = 1;
	unsigned int pn_swap = 0;
	unsigned int dual_port = 1;
	unsigned int lvds_repack = 1;
	unsigned int port_reverse = 1;

	lvds_repack = (pConf->lcd_control.lvds_config->lvds_repack) & 0x1;
	pn_swap		= (pConf->lcd_control.lvds_config->pn_swap) & 0x1;
	dual_port	= (pConf->lcd_control.lvds_config->dual_port) & 0x1;
	port_reverse= (pConf->lcd_control.lvds_config->port_reverse) & 0x1;
	switch (pConf->lcd_control.lvds_config->lvds_bits) {
		case 10:
			bit_num=0;
			break;
		case 8:
			bit_num=1;
			break;
		case 6:
			bit_num=2;
			break;
		case 4:
			bit_num=3;
			break;
		default:
			bit_num=1;
			break;
	}

	aml_write_reg32(P_MLVDS_CONTROL,  (aml_read_reg32(P_MLVDS_CONTROL) & ~(1 << 0)));  //disable mlvds
	aml_write_reg32(P_LVDS_PACK_CNTL_ADDR,
					( lvds_repack<<0 ) | // repack
					( port_reverse?(0<<2):(1<<2)) | // odd_even
					( 0<<3 ) |			// reserve
					( 0<<4 ) |			// lsb first
					( pn_swap<<5 ) |	// pn swap
					( dual_port<<6 ) |	// dual port
					( 0<<7 ) |			// use tcon control
					( bit_num<<8 ) |	// 0:10bits, 1:8bits, 2:6bits, 3:4bits.
					( 0<<10 ) |			//r_select  //0:R, 1:G, 2:B, 3:0
					( 1<<12 ) |			//g_select  //0:R, 1:G, 2:B, 3:0
					( 2<<14 ));			//b_select  //0:R, 1:G, 2:B, 3:0;
}

static void set_venc_lvds(Lcd_Config_t *pConf)
{
	aml_write_reg32(P_ENCL_VIDEO_EN, 0);

	//aml_write_reg32(P_VPU_VIU_VENC_MUX_CTRL, (0<<0)|(0<<2) );    // viu1 select encl | viu2 select encl
	//aml_set_reg32_bits(P_VPU_VIU_VENC_MUX_CTRL, 0, 2, 2);// viu2 select encl
	//aml_set_reg32_bits(P_VPU_VIU_VENC_MUX_CTRL, 0, 0, 2);// viu1 select encl
	aml_set_reg32_bits(P_VPU_VIU_VENC_MUX_CTRL, 0, 0, 4);// viu1, viu2 select encl

	aml_write_reg32(P_ENCL_VIDEO_MODE, 		0); // Enable Hsync and equalization pulse switch in center; bit[14] cfg_de_v = 1
	aml_write_reg32(P_ENCL_VIDEO_MODE_ADV, 	0x0418); // Sampling rate: 1

	// bypass filter
	aml_write_reg32(P_ENCL_VIDEO_FILT_CTRL, 	0x1000);
	aml_write_reg32(P_ENCL_VIDEO_MAX_PXCNT, 	pConf->lcd_basic.h_period - 1);
	aml_write_reg32(P_ENCL_VIDEO_MAX_LNCNT,		pConf->lcd_basic.v_period - 1);
	aml_write_reg32(P_ENCL_VIDEO_HAVON_BEGIN,	pConf->lcd_basic.video_on_pixel);
	aml_write_reg32(P_ENCL_VIDEO_HAVON_END,		pConf->lcd_basic.h_active - 1 + pConf->lcd_basic.video_on_pixel);
	aml_write_reg32(P_ENCL_VIDEO_VAVON_BLINE,	pConf->lcd_basic.video_on_line);
	aml_write_reg32(P_ENCL_VIDEO_VAVON_ELINE,	pConf->lcd_basic.v_active - 1  + pConf->lcd_basic.video_on_line);

	aml_write_reg32(P_ENCL_VIDEO_HSO_BEGIN,		pConf->lcd_timing.sth1_hs_addr);
	aml_write_reg32(P_ENCL_VIDEO_HSO_END,		pConf->lcd_timing.sth1_he_addr);
	aml_write_reg32(P_ENCL_VIDEO_VSO_BEGIN,		pConf->lcd_timing.stv1_hs_addr);
	aml_write_reg32(P_ENCL_VIDEO_VSO_END,		pConf->lcd_timing.stv1_he_addr);
	aml_write_reg32(P_ENCL_VIDEO_VSO_BLINE,		pConf->lcd_timing.stv1_vs_addr);
	aml_write_reg32(P_ENCL_VIDEO_VSO_ELINE,		pConf->lcd_timing.stv1_ve_addr);
	aml_write_reg32(P_ENCL_VIDEO_RGBIN_CTRL, 	0);

	// enable encl
	aml_write_reg32(P_ENCL_VIDEO_EN, 1);
}

static void venc_change_lvds(Lcd_Config_t *pConf)
{
	aml_write_reg32(P_ENCL_VIDEO_MAX_PXCNT, pConf->lcd_basic.h_period - 1);
	aml_write_reg32(P_ENCL_VIDEO_MAX_LNCNT, pConf->lcd_basic.v_period - 1);
	printk("lcd: venc changed: %d,%d\n", pConf->lcd_basic.h_period, pConf->lcd_basic.v_period);

	lcd_notifier_call_chain(LCD_EVENT_BL_UPDATE, NULL);
}

static void set_pll_lvds(Lcd_Config_t *pConf)
{
	unsigned int sw_port 	= (pConf->lcd_control.lvds_config->lvds_fifo_wr_mode)&0x100;
	unsigned int lvds_fifo = (pConf->lcd_control.lvds_config->lvds_fifo_wr_mode)&0xff;
	unsigned int vx1_phy_div;
	unsigned int lvds_ports ;

	generate_clk_parameter(pConf);
	//printk("pll_ctrl=0x%x, div_ctrl=0x%x, clk_ctrl=0x%x.\n",
	//	pConf->lcd_timing.pll_ctrl, pConf->lcd_timing.div_ctrl,
	//	pConf->lcd_timing.clk_ctrl);
	set_vclk_lcd(pConf);

	lvds_ports = pConf->lcd_control.lvds_config->dual_port;
	if (lvds_ports < 2) {
		vx1_phy_div  = 2/2;
		//encl_div	 = vx1_phy_div*7;
		if (lvds_ports == 1) //dual port
			vx1_phy_div = vx1_phy_div*2;
	}else if (lvds_ports >= 2) {
		TV_LCD_ERR("lcd error :Quad-LVDS is not supported!\n");
		return;
	}

	aml_write_reg32(P_HHI_LVDS_TX_PHY_CNTL1,((3<<6)|((vx1_phy_div-1)<<1)|1)<<24);

	if ( sw_port == 0x100 )
		aml_write_reg32( P_HHI_VIID_DIVIDER_CNTL, ((aml_read_reg32(P_HHI_VIID_DIVIDER_CNTL) & ~(0x7 << 8)) | (1 << 8) | (0<<10)));
	else
		aml_write_reg32( P_HHI_VIID_DIVIDER_CNTL, ((aml_read_reg32(P_HHI_VIID_DIVIDER_CNTL) & ~(0x7 << 8)) | (2 << 8) | (0<<10)));
	aml_write_reg32(P_LVDS_GEN_CNTL, (aml_read_reg32(P_LVDS_GEN_CNTL)| (1 << 3) | (lvds_fifo << 0)));
}


unsigned int  lvds_init(struct aml_lcd *pDev)
{
	TV_LCD_INFO("lvds mode is selected\n");

	mutex_lock(&pDev->init_lock);

	switch (pDev->pConf->lcd_timing.frame_rate_adj_type) {
	case 0: /* clk adjust */
		set_pll_lvds(pDev->pConf);
		set_venc_lvds(pDev->pConf);
		set_tcon_lvds(pDev->pConf);
		set_control_lvds(pDev->pConf);
		init_phy_lvds(pDev->pConf);
		_enable_vsync_interrupt();
		break;
	case 1: /* htotal adjust */
	case 2: /* vtotal adjust */
		venc_change_lvds(pDev->pConf);
		break;
	default:
		break;
	}
	mutex_unlock(&pDev->init_lock);

	return 0;
}


