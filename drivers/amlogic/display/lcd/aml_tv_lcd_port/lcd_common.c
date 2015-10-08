/*
 * drivers/amlogic/display/lcd/aml_tv_lcd_port/lcd_common.c
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

#include "../aml_lcd_tv.h"
#include "lcd_common.h"

static unsigned od_table[4] = {1, 2, 4, 8};
#define TV_LCD_ENC_TST_NUM_MAX    8
unsigned int ss_level = 0;

static char *lcd_enc_tst_str[] = {
	"0-None",        /* 0 */
	"1-Color Bar",   /* 1 */
	"2-Thin Line",   /* 2 */
	"3-Dot Grid",    /* 3 */
	"4-Gray",        /* 4 */
	"5-Blue",         /* 5 */
	"6-Red",       /* 6 */
	"7-Green",        /* 7 */
};

static unsigned int lcd_enc_tst[][7] = {
/*tst_mode,    Y,       Cb,     Cr,     tst_en,  vfifo_en  rgbin*/
	{0,    0x200,   0x200,  0x200,   0,      1,        3},  /* 0 */
	{1,    0x200,   0x200,  0x200,   1,      0,        1},  /* 1 */
	{2,    0x200,   0x200,  0x200,   1,      0,        1},  /* 2 */
	{3,    0x200,   0x200,  0x200,   1,      0,        1},  /* 3 */
	{0,    0x200,   0x200,  0x200,   1,      0,        1},  /* 4 */
	{0,    0x130,   0x153,  0x3fd,   1,      0,        1},  /* 5 */
	{0,    0x256,   0x0ae,  0x055,   1,      0,        1},  /* 6 */
	{0,    0x074,   0x3fd,  0x1ad,   1,      0,        1},  /* 7 */
};

static const char *ss_level_table[] = {
	"0-0",
	"1-0.4%",
	"2-0.7%",
	"3-1.7%",
	"4-3%",
};

static unsigned int lcd_spread[][2] = {
/*HHI_HDMI_PLL_CNTL3, HHI_HDMI_PLL_CNTL4 */
	{0x135c5091,       0x801da72c},  /* 0% */
	{0x1bdc5091,       0xa0b1a72c},  /* 0.4% */
	{0x1bd05091,       0xbcb1a72c},  /* 0.7% */
	{0x1bc85091,       0xbcb1a72c},  /* 1.7% */
	{0x1bc45091,       0xbcb1a72c},  /* 3% */
};

int get_tv_lcd_spread_spectrum(void)
{
	unsigned int num = ss_level;
	int i ;

	for (i = 0; i < sizeof (ss_level_table) / sizeof (ss_level_table[1]); i++) {
		pr_info("spread sprectrum: %s\n",ss_level_table[i]);
	}

	pr_info("\n");
	pr_info("current spread sprectrum %d: %s\n",
		num, ss_level_table[num]);
	pr_info("HHI_HDMI_PLL_CNTL3 = 0x%x\n",
		aml_read_reg32(P_HHI_HDMI_PLL_CNTL3));
	pr_info("HHI_HDMI_PLL_CNTL4 = 0x%x\n",
		aml_read_reg32(P_HHI_HDMI_PLL_CNTL4));

	return num;
}

void  set_tv_lcd_spread_spectrum(unsigned int num)
{
	if (num < sizeof (ss_level_table) / sizeof (ss_level_table[1])) {
		ss_level = num;
		aml_write_reg32(P_HHI_HDMI_PLL_CNTL3, lcd_spread[num][0]);
		aml_write_reg32(P_HHI_HDMI_PLL_CNTL4, lcd_spread[num][1]);
		pr_info("show spread sprectrum %d: %s\n",
			num, ss_level_table[num]);
	} else {
		ss_level = 0;
		aml_write_reg32(P_HHI_HDMI_PLL_CNTL3, lcd_spread[0][0]);
		aml_write_reg32(P_HHI_HDMI_PLL_CNTL4, lcd_spread[0][1]);
		pr_info("error: Out of range ,not to spread spectrum!! \n ");
	}
}

void lcd_test(unsigned int num, struct aml_lcd *pDev)
{
	num = (num >= TV_LCD_ENC_TST_NUM_MAX) ? 0 : num;
	if (num >= 0) {
		aml_write_reg32(P_ENCL_VIDEO_RGBIN_CTRL, lcd_enc_tst[num][6]);
		aml_write_reg32(P_ENCL_TST_MDSEL, lcd_enc_tst[num][0]);
		aml_write_reg32(P_ENCL_TST_Y, lcd_enc_tst[num][1]);
		aml_write_reg32(P_ENCL_TST_CB, lcd_enc_tst[num][2]);
		aml_write_reg32(P_ENCL_TST_CR, lcd_enc_tst[num][3]);
		aml_write_reg32(P_ENCL_TST_CLRBAR_STRT,
						pDev->pConf->lcd_basic.video_on_pixel);
		aml_write_reg32(P_ENCL_TST_CLRBAR_WIDTH,
						(pDev->pConf->lcd_basic.h_active / 9));
		aml_write_reg32(P_ENCL_TST_EN, lcd_enc_tst[num][4]);
		aml_set_reg32_bits(P_ENCL_VIDEO_MODE_ADV, lcd_enc_tst[num][5], 3, 1);
		pr_info("show test pattern: %s\n", lcd_enc_tst_str[num]);
	} else {
		pr_info("disable test pattern\n");
	}
}

static int check_pll(struct pll_para_s *pll, unsigned int pll_fout)
{
	unsigned int fin, m, n;
	unsigned int od1_sel, od2_sel, od3_sel, od1, od2, od3;
	unsigned int pll_fod2_in, pll_fod3_in, pll_fvco;
	unsigned int od_fb = 0;
	unsigned int pll_frac = 0;
	int done;

	done = 0;
	fin = FIN_FREQ; /* kHz */
	/* od3 >= od2 */
	for (od3_sel = OD_SEL_MAX; od3_sel > 0; od3_sel--) {
		od3 = od_table[od3_sel - 1];
		pll_fod3_in = pll_fout * od3;
		for (od2_sel = od3_sel; od2_sel > 0; od2_sel--) {
			od2 = od_table[od2_sel - 1];
			pll_fod2_in = pll_fod3_in * od2;
			for (od1_sel = OD_SEL_MAX; od1_sel > 0; od1_sel--) {
				od1 = od_table[od1_sel - 1];
				pll_fvco = pll_fod2_in * od1;

				if ((pll_fvco < PLL_VCO_MIN) ||
					(pll_fvco > PLL_VCO_MAX)) {
					continue;
				}
				pll->od1_sel = od1_sel - 1;
				pll->od2_sel = od2_sel - 1;
				pll->od3_sel = od3_sel - 1;

				n = 1;
				od_fb = 0; /* pll default */
				pll_fvco = pll_fvco / ((od_fb + 1) * 2);

				m = pll_fvco / fin;
				pll_frac = (pll_fvco % fin) * 4096 / fin;

				pll->m = m;
				pll->n = n;
				pll->frac = pll_frac;
/*				printk("od1_sel=%d, od2_sel=%d, od3_sel=%d, pll_fvco=%d",
									(od1_sel - 1), (od2_sel - 1),
									(od3_sel - 1),	pll_fvco);
				printk(" pll_m=%d, pll_n=%d, pll_frac=%d\n",pll->m, pll->n, pll_frac);
*/				done = 1;
				return done;
			}
		}
	}

	return done;
}

static unsigned int clk_div_calc(unsigned int clk,unsigned int div_sel, int dir)
{
	unsigned int clk_ret;

	switch (div_sel) {
	case CLK_DIV_SEL_1:
		clk_ret = clk;
		break;
	case CLK_DIV_SEL_2:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 2;
		else
			clk_ret = clk * 2;
		break;
	case CLK_DIV_SEL_3:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 3;
		else
			clk_ret = clk * 3;
		break;
	case CLK_DIV_SEL_3p5:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk * 2 / 7;
		else
			clk_ret = clk * 7 / 2;
		break;
	case CLK_DIV_SEL_3p75:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk * 4 / 15;
		else
			clk_ret = clk * 15 / 4;
		break;
	case CLK_DIV_SEL_4:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 4;
		else
			clk_ret = clk * 4;
		break;
	case CLK_DIV_SEL_5:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 5;
		else
			clk_ret = clk * 5;
		break;
	case CLK_DIV_SEL_6:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 6;
		else
			clk_ret = clk * 6;
		break;
	case CLK_DIV_SEL_6p25:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk * 4 / 25;
		else
			clk_ret = clk * 25 / 4;
		break;
	case CLK_DIV_SEL_7:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 7;
		else
			clk_ret = clk * 7;
		break;
	case CLK_DIV_SEL_7p5:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk * 2 / 15;
		else
			clk_ret = clk * 15 / 2;
		break;
	case CLK_DIV_SEL_12:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 12;
		else
			clk_ret = clk * 12;
		break;
	case CLK_DIV_SEL_14:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 14;
		else
			clk_ret = clk * 14;
		break;
	case CLK_DIV_SEL_15:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 15;
		else
			clk_ret = clk * 15;
		break;
	case CLK_DIV_SEL_2p5:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk * 2 / 5;
		else
			clk_ret = clk * 5 / 2;
		break;
	default:
		clk_ret = clk;
		printk("[Error]clk_div_sel:  Invalid parameter\n");
		break;
	}

	return clk_ret;
}


void generate_clk_parameter(Lcd_Config_t *pconf)
{
	struct pll_para_s pll;
	int ret = 0;
	unsigned clk_div_sel, crt_xd;
	unsigned crt_xd_max;
	unsigned fout_pll, clk_div_out;
	unsigned tmp;
	unsigned fout;

	pll.frac = 0;
	pll.m = 0;
	pll.n = 0;
	pll.od1_sel = 0;
	pll.od2_sel = 0;
	pll.od3_sel = 0;

	//printk("lcd_clk = %dHz\n", pconf->lcd_timing.lcd_clk);
	fout = pconf->lcd_timing.lcd_clk / 1000; /* kHz */

	if (fout > ENCL_MAX_CLK_IN)
		goto generate_clk_done;

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_DIGITAL_LVDS:
		clk_div_sel = CLK_DIV_SEL_7; // CLK_DIV_SEL_1; 0
		crt_xd_max = CRT_VID_DIV_MAX; //255
		crt_xd = 1;
		clk_div_out = fout * crt_xd;
		if (clk_div_out > CRT_VID_MAX_CLK_IN)
			goto generate_clk_done;

		fout_pll = clk_div_calc(clk_div_out,clk_div_sel, CLK_DIV_O2I);
		if (fout_pll > CLK_DIV_MAX_CLK_IN)
			goto generate_clk_done;

		ret = check_pll(&pll, fout_pll);
/* printk("fout_pll=%d , clk_div_sel=%d clk_div_out=%d\n", fout_pll,clk_div_sel,clk_div_out); */
/* printk("od1_sel=%x od2_sel=%x od3_sel=%x n=%x m=%x  \n",
		pll.od1_sel,pll.od2_sel,pll.od3_sel,pll.n,pll.m);
 */
		if (ret)
			goto generate_clk_done;
		break;
	case LCD_DIGITAL_VBYONE:
		fout_pll = pconf->lcd_control.vbyone_config->bit_rate / 1000;
		if (fout_pll > CLK_DIV_MAX_CLK_IN)
			goto generate_clk_done;
		ret = check_pll(&pll, fout_pll);
		if (ret)
			goto generate_clk_done;
		break;
	default:
		break;
	}

generate_clk_done:
	if (ret) {
		pconf->lcd_timing.pll_ctrl =
			(pll.od1_sel << PLL_CTRL_OD1) |
			(pll.od2_sel << PLL_CTRL_OD2) |
			(pll.od3_sel << PLL_CTRL_OD3) |
			(pll.n << PLL_CTRL_N) |
			(pll.m << PLL_CTRL_M);
		pconf->lcd_timing.div_ctrl = (clk_div_sel << DIV_CTRL_CLK_DIV);
		tmp = (pconf->lcd_timing.clk_ctrl &
			~((0xff << CLK_CTRL_XD) | (0xfff << CLK_CTRL_FRAC)));
		pconf->lcd_timing.clk_ctrl = (tmp |
			((crt_xd << CLK_CTRL_XD) |
			(pll.frac << CLK_CTRL_FRAC)));

		printk("lcd: pll_ctrl=0x%x div_ctrl=0x%x clk_ctrl=0x%x \n",
			pconf->lcd_timing.pll_ctrl, pconf->lcd_timing.div_ctrl,pconf->lcd_timing.clk_ctrl);

	} else {
		pconf->lcd_timing.pll_ctrl = (0 << PLL_CTRL_OD1) |
			(1 << PLL_CTRL_OD2) | (1 << PLL_CTRL_OD3) |
			(1 << PLL_CTRL_N) | (65 << PLL_CTRL_M);
		pconf->lcd_timing.div_ctrl =
			(CLK_DIV_SEL_1 << DIV_CTRL_CLK_DIV);
		pconf->lcd_timing.clk_ctrl = (pconf->lcd_timing.clk_ctrl &
			~(0xff << CLK_CTRL_XD)) | (7 << CLK_CTRL_XD);
		printk("lcd: Out of clock range, reset to default setting\n");
	}
}

static void hpll_load_initial(void)
{
	//printk("lcd: %s\n", __func__);
	//hdmi load initial
	aml_write_reg32(P_HHI_VID_CLK_CNTL2, 0x2c);
	aml_write_reg32(P_HHI_VID_CLK_DIV, 0x100);
	aml_write_reg32(P_HHI_VIID_CLK_CNTL, 0x0);
	aml_write_reg32(P_HHI_VIID_CLK_DIV, 0x101);
	aml_write_reg32(P_HHI_VID_LOCK_CLK_CNTL, 0x80);

	//    aml_write_reg32(P_HHI_VPU_CLK_CNTL, 25);
	//    aml_set_reg32_bits(P_HHI_VPU_CLK_CNTL, 1, 8, 1);
	aml_write_reg32(P_AO_RTI_GEN_PWR_SLEEP0, 0x0);
	aml_write_reg32(P_HHI_HDMI_PLL_CNTL6, 0x100);

	aml_write_reg32(P_VPU_CLK_GATE, 0xffff);
	aml_write_reg32(P_ENCL_VIDEO_VSO_BLINE, 0x0);
	aml_write_reg32(P_ENCL_VIDEO_VSO_BEGIN, 0x0);
	aml_write_reg32(P_VPU_VLOCK_GCLK_EN, 0x7);
	aml_write_reg32(P_VPU_VLOCK_ADJ_EN_SYNC_CTRL, 0x108010ff);
	aml_write_reg32(P_VPU_VLOCK_CTRL, 0xe0f50f1b);
}

static void hpll_load_en(void)
{
	//printk("lcd: %s\n", __func__);
	//hdmi load gen
	aml_set_reg32_bits(P_HHI_VID_CLK_CNTL, 1, 19, 1);
	aml_set_reg32_bits(P_HHI_VID_CLK_CNTL, 7, 0 , 3);
	aml_set_reg32_bits(P_HHI_VID_CLK_CNTL, 1, 16, 3);  // tmp use fclk_div4
	aml_write_reg32(P_ENCL_VIDEO_EN, 0x1);
	//    msleep(20);
	aml_write_reg32(P_ENCL_VIDEO_EN, 0x0);
	//    msleep(20);
	//    printk("read Addr: 0x%x[0x%x]  Data: 0x%x\n", P_HHI_HDMI_PLL_CNTL, (P_HHI_HDMI_PLL_CNTL & 0xffff) >> 2, aml_read_reg32(P_HHI_HDMI_PLL_CNTL));
	aml_set_reg32_bits(P_HHI_VID_CLK_CNTL, 0, 16, 3);  // use vid_pll
}

static void lcd_set_pll(unsigned int pll_reg, unsigned int clk_ctrl_reg)
{
	unsigned m, n, od1, od2, od3, frac;
	int wait_loop = 10;
	unsigned pll_lock = 0;
	unsigned pll_ctrl, pll_ctrl2;

	m = (pll_reg >> PLL_CTRL_M) & 0x1ff;
	n = (pll_reg >> PLL_CTRL_N) & 0x1f;
	od1 = (pll_reg >> PLL_CTRL_OD1) & 0x3;
	od2 = (pll_reg >> PLL_CTRL_OD2) & 0x3;
	od3 = (pll_reg >> PLL_CTRL_OD3) & 0x3;
	frac = (clk_ctrl_reg >> CLK_CTRL_FRAC) & 0xfff;

	pll_ctrl = ((1 << 30) | (n << 9) | (m << 0));
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESONG9TV)
	pll_ctrl2 = ((od1 << 16) | (od2 << 18) | (od3 << 22));
#elif (MESON_CPU_TYPE == MESON_CPU_TYPE_MESONG9BB)
	pll_ctrl2 = ((od1 << 16) | (od2 << 18) | (od3 << 22));
#endif
	if (frac > 0)
		pll_ctrl2 |= ((1 << 14) | (frac << 0));

	//printk("pll_ctrl : 0x%x \n",pll_ctrl | (1 << 28));
	//printk("pll_ctrl2 : 0x%x \n",pll_ctrl2);

	hpll_load_initial();

	aml_write_reg32(P_HHI_HDMI_PLL_CNTL, pll_ctrl | (1 << 28));
	aml_write_reg32(P_HHI_HDMI_PLL_CNTL2, pll_ctrl2);
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESONG9TV)
	aml_write_reg32(P_HHI_HDMI_PLL_CNTL3, 0x135c5091);
	aml_write_reg32(P_HHI_HDMI_PLL_CNTL5, 0x71486900);
#elif (MESON_CPU_TYPE == MESON_CPU_TYPE_MESONG9BB)
	aml_write_reg32(P_HHI_HDMI_PLL_CNTL3, 0x0d5c5091);
	aml_write_reg32(P_HHI_HDMI_PLL_CNTL5, 0x714869c0);
#endif
	aml_write_reg32(P_HHI_HDMI_PLL_CNTL4, 0x801da72c);
	aml_write_reg32(P_HHI_HDMI_PLL_CNTL6, 0x00000a55);
	aml_write_reg32(P_HHI_HDMI_PLL_CNTL, pll_ctrl);
	do {
		hpll_load_en();
		mdelay(10);
		pll_lock = (aml_read_reg32(P_HHI_HDMI_PLL_CNTL) >> 31) & 0x1;
		wait_loop--;
	} while ((pll_lock == 0) && (wait_loop > 0));
	if (wait_loop == 0)
		printk("[error]: hpll lock failed\n");
#if 0
	printk("dump hpll registers:\n");
	printk("  0x10c8:    0x%08x\n", aml_read_reg32(P_HHI_HDMI_PLL_CNTL));
	printk("  0x10c9:    0x%08x\n", aml_read_reg32(P_HHI_HDMI_PLL_CNTL2));
	printk("  0x10ca:    0x%08x\n", aml_read_reg32(P_HHI_HDMI_PLL_CNTL3));
	printk("  0x10cb:    0x%08x\n", aml_read_reg32(P_HHI_HDMI_PLL_CNTL4));
	printk("  0x10cc:    0x%08x\n", aml_read_reg32(P_HHI_HDMI_PLL_CNTL5));
	printk("  0x10cd:    0x%08x\n", aml_read_reg32(P_HHI_HDMI_PLL_CNTL6));
#endif
}

static unsigned int lcd_clk_div_table[][3] = {
	/* divider,        shift_val,  shift_sel */
	{CLK_DIV_SEL_1,    0xffff,     0,},
	{CLK_DIV_SEL_2,    0x0aaa,     0,},
	{CLK_DIV_SEL_3,    0x0db6,     0,},
	{CLK_DIV_SEL_3p5,  0x36cc,     1,},
	{CLK_DIV_SEL_3p75, 0x6666,     2,},
	{CLK_DIV_SEL_4,    0x0ccc,     0,},
	{CLK_DIV_SEL_5,    0x739c,     2,},
	{CLK_DIV_SEL_6,    0x0e38,     0,},
	{CLK_DIV_SEL_6p25, 0x0000,     3,},
	{CLK_DIV_SEL_7,    0x3c78,     1,},
	{CLK_DIV_SEL_7p5,  0x78f0,     2,},
	{CLK_DIV_SEL_12,   0x0fc0,     0,},
	{CLK_DIV_SEL_14,   0x3f80,     1,},
	{CLK_DIV_SEL_15,   0x7f80,     2,},
	{CLK_DIV_SEL_2p5,  0x5294,     2,},
	{CLK_DIV_SEL_MAX,  0xffff,     0,},
};

void lcd_set_clk_div(unsigned long vid_div_reg)
{
	unsigned int  clk_div;
	unsigned int shift_val, shift_sel;
	int i;

	clk_div = (vid_div_reg >> DIV_CTRL_CLK_DIV) & 0xf;

	aml_set_reg32_bits(P_HHI_VIID_CLK_CNTL, 0, 19, 1);
	udelay(5);

	/* Disable the div output clock */
	aml_set_reg32_bits(P_HHI_VID_PLL_CLK_DIV, 0, 19, 1);
	aml_set_reg32_bits(P_HHI_VID_PLL_CLK_DIV, 0, 15, 1);

	i = 0;
	while (lcd_clk_div_table[i][0] != CLK_DIV_SEL_MAX) {
		if (clk_div == lcd_clk_div_table[i][0])
			break;
		i++;
	}
	if (lcd_clk_div_table[i][0] == CLK_DIV_SEL_MAX)
		printk("invalid clk divider\n");
	shift_val = lcd_clk_div_table[i][1];
	shift_sel = lcd_clk_div_table[i][2];

	if (shift_val == 0xffff ) {      // if divide by 1
		aml_set_reg32_bits(P_HHI_VID_PLL_CLK_DIV, 1, 18, 1);
	} else {
		aml_set_reg32_bits(P_HHI_VID_PLL_CLK_DIV, 0, 16, 2);
		aml_set_reg32_bits(P_HHI_VID_PLL_CLK_DIV, 0, 15, 1);
		aml_set_reg32_bits(P_HHI_VID_PLL_CLK_DIV, 0, 0, 14);

		aml_set_reg32_bits(P_HHI_VID_PLL_CLK_DIV, shift_sel, 16, 2);
		aml_set_reg32_bits(P_HHI_VID_PLL_CLK_DIV, 1, 15, 1);
		aml_set_reg32_bits(P_HHI_VID_PLL_CLK_DIV, shift_val, 0, 14);
		aml_set_reg32_bits(P_HHI_VID_PLL_CLK_DIV, 0, 15, 1);
	}
	/* Enable the final output clock */
	aml_set_reg32_bits(P_HHI_VID_PLL_CLK_DIV, 1, 19, 1);
}

static void lcd_set_vclk_crt(unsigned int clk_ctrl_reg)
{
	unsigned int xd;
/*	printk("%s\n", __func__); */

	xd = (clk_ctrl_reg >> CLK_CTRL_XD) & 0xff;
	/* setup the XD divider value */
	aml_set_reg32_bits(P_HHI_VIID_CLK_DIV, (xd-1), 0, 8);
	udelay(5);
	/* Bit[18:16] - v2_cntl_clk_in_sel */
	aml_set_reg32_bits(P_HHI_VIID_CLK_CNTL, 0, 16, 3);
	aml_set_reg32_bits(P_HHI_VIID_CLK_CNTL, 1, 19, 1);
	udelay(2);

	/* [15:12] encl_clk_sel, select vclk2_div1 */
	aml_set_reg32_bits(P_HHI_VIID_CLK_DIV, 8, 12, 4);
	/* release vclk2_div_reset and enable vclk2_div */
	aml_set_reg32_bits(P_HHI_VIID_CLK_DIV, 1, 16, 2);
	udelay(5);

	aml_set_reg32_bits(P_HHI_VIID_CLK_CNTL, 1, 0, 1);
	aml_set_reg32_bits(P_HHI_VIID_CLK_CNTL, 1, 15, 1);
	udelay(10);
	aml_set_reg32_bits(P_HHI_VIID_CLK_CNTL, 0, 15, 1);
	udelay(5);

	aml_set_reg32_bits(P_HHI_VID_CLK_CNTL2, 1, 3, 1);
}

void set_vclk_lcd(Lcd_Config_t *pconf)
{
	unsigned pll_reg, div_reg, clk_reg;
	int xd;
	int lcd_type;

	pll_reg = pconf->lcd_timing.pll_ctrl;
	div_reg = pconf->lcd_timing.div_ctrl;
	clk_reg = pconf->lcd_timing.clk_ctrl;
	xd = (clk_reg >> CLK_CTRL_XD) & 0xff;

	lcd_type = pconf->lcd_basic.lcd_type;

	switch (lcd_type) {
	case LCD_DIGITAL_LVDS:
		xd = 1;
		break;
	default:
		break;
	}

	clk_reg = (pconf->lcd_timing.clk_ctrl & ~(0xff << CLK_CTRL_XD)) |
		(xd << CLK_CTRL_XD);

	lcd_set_pll(pll_reg, clk_reg);
	lcd_set_clk_div(div_reg);
	lcd_set_vclk_crt(clk_reg);
}

void vpp_set_matrix_ycbcr2rgb (int vd1_or_vd2_or_post, int mode)
{
	if (vd1_or_vd2_or_post == 0) { //vd1
		aml_set_reg32_bits (P_VPP_MATRIX_CTRL, 1, 5, 1);
		aml_set_reg32_bits (P_VPP_MATRIX_CTRL, 1, 8, 2);
	}else if (vd1_or_vd2_or_post == 1) { //vd2
		aml_set_reg32_bits (P_VPP_MATRIX_CTRL, 1, 4, 1);
		aml_set_reg32_bits (P_VPP_MATRIX_CTRL, 2, 8, 2);
	}else{
		aml_set_reg32_bits (P_VPP_MATRIX_CTRL, 1, 0, 1);
		aml_set_reg32_bits (P_VPP_MATRIX_CTRL, 0, 8, 2);
		if (mode == 0) {
			aml_set_reg32_bits(P_VPP_MATRIX_CTRL, 1, 1, 2);
		}else if (mode == 1) {
			aml_set_reg32_bits(P_VPP_MATRIX_CTRL, 0, 1, 2);
		}
	}

	if (mode == 0) { //ycbcr not full range, 601 conversion
		aml_write_reg32(P_VPP_MATRIX_PRE_OFFSET0_1, 0x0064C8FF);
		aml_write_reg32(P_VPP_MATRIX_PRE_OFFSET2, 0x006400C8);
		//1.164     0       1.596
		//1.164   -0.392    -0.813
		//1.164   2.017     0
		aml_write_reg32(P_VPP_MATRIX_COEF00_01, 0x04000000);
		aml_write_reg32(P_VPP_MATRIX_COEF02_10, 0x059C0400);
		aml_write_reg32(P_VPP_MATRIX_COEF11_12, 0x1EA01D24);
		aml_write_reg32(P_VPP_MATRIX_COEF20_21, 0x04000718);
		aml_write_reg32(P_VPP_MATRIX_COEF22, 0x00000000);
		aml_write_reg32(P_VPP_MATRIX_OFFSET0_1, 0x00000000);
		aml_write_reg32(P_VPP_MATRIX_OFFSET2, 0x00000000);
		aml_write_reg32(P_VPP_MATRIX_PRE_OFFSET0_1, 0x00000E00);
		aml_write_reg32(P_VPP_MATRIX_PRE_OFFSET2, 0x00000E00);
	}else if (mode == 1) {//ycbcr full range, 601 conversion
		aml_write_reg32(P_VPP_MATRIX_PRE_OFFSET0_1, 0x0000600);
		aml_write_reg32(P_VPP_MATRIX_PRE_OFFSET2, 0x0600);
		//	1	0			1.402
		//	1	-0.34414	-0.71414
		//	1	1.772		0
		aml_write_reg32(P_VPP_MATRIX_COEF00_01, (0x400 << 16) |0);
		aml_write_reg32(P_VPP_MATRIX_COEF02_10, (0x59c << 16) |0x400);
		aml_write_reg32(P_VPP_MATRIX_COEF11_12, (0x1ea0 << 16) |0x1d25);
		aml_write_reg32(P_VPP_MATRIX_COEF20_21, (0x400 << 16) |0x717);
		aml_write_reg32(P_VPP_MATRIX_COEF22, 0x0);
		aml_write_reg32(P_VPP_MATRIX_OFFSET0_1, 0x0);
		aml_write_reg32(P_VPP_MATRIX_OFFSET2, 0x0);
	}
}



void _enable_vsync_interrupt(void)
{
	if ((aml_read_reg32(P_ENCL_VIDEO_EN) & 1) || (aml_read_reg32(P_ENCL_VIDEO_EN) & 1)) {
		aml_write_reg32(P_VENC_INTCTRL, 0x200);
	} else {
		aml_write_reg32(P_VENC_INTCTRL, 0x2);
	}
}


void _disable_display_driver(void)
{
	int vclk_sel;

	vclk_sel = 0;//((pConf->lcd_timing.clk_ctrl) >>4) & 0x1;

	lcd_function_remove();

	aml_set_reg32_bits(P_HHI_VIID_DIVIDER_CNTL, 0, 11, 1);	//close lvds phy clk gate: 0x104c[11]

	//aml_write_reg32(P_ENCT_VIDEO_EN, 0);	//disable enct
	aml_write_reg32(P_ENCL_VIDEO_EN, 0);	//disable encl

	if (vclk_sel)
		aml_set_reg32_bits(P_HHI_VIID_CLK_CNTL, 0, 0, 5);		//close vclk2 gate: 0x104b[4:0]
	else
		aml_set_reg32_bits(P_HHI_VID_CLK_CNTL, 0, 0, 5);		//close vclk1 gate: 0x105f[4:0]

	aml_write_reg32(P_HHI_DIF_CSI_PHY_CNTL1, 0x0);   //close phy
	aml_write_reg32(P_HHI_DIF_CSI_PHY_CNTL2, 0x0);
	aml_write_reg32(P_HHI_DIF_CSI_PHY_CNTL3, 0x0);

	TV_LCD_INFO("disable lcd display driver.\n");
}


