/*
 * AMLOGIC lcd controller driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the named License,
 * or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 * Author:  Tim Yao <timyao@amlogic.com>
 *
 * Modify:  Evoke Zhang <evoke.zhang@amlogic.com>
 * compatible dts
 *
 */
#include <linux/init.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/amlogic/vout/vinfo.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <plat/regops.h>
#include <mach/am_regs.h>
#include <mach/lcd_reg.h>
#include <linux/amlogic/vout/lcdoutc.h>
#include <linux/amlogic/vout/aml_lcd_common.h>
#include <mach/clock.h>
#include <mach/vpu.h>
#include <mach/mod_gate.h>
#include <asm/fiq.h>
#include <linux/delay.h>
#include <linux/of.h>
#include "lcd_config.h"

#define VPP_OUT_SATURATE	(1 << 0)

static spinlock_t gamma_write_lock;
static spinlock_t lcd_clk_lock;

static Lcd_Config_t *lcd_conf;

#define SS_LEVEL_MAX	1
static char *lcd_ss_level_table[] = {
	"0",
	"0.5%",
	"1%",
	"1.5%",
	"2%",
};

#define LVDS_VSWING_LEVEL_MAX  5
static unsigned int lvds_vswing_ctrl[] = {
/* vswing_ctrl   level   voltage */
	0x1,   /* 0      0.2V */
	0x3,   /* 1      0.4V */
	0x5,   /* 2      0.6V */
	0x6,   /* 3      0.7V */
	0x7,   /* 4      0.8V */
};

static void print_lcd_driver_version(void)
{
	pr_info("lcd driver version: %s%s\n\n", LCD_DRV_DATE, LCD_DRV_TYPE);
}

static void lcd_ports_ctrl_lvds(Bool_t status)
{
	unsigned int phy_reg, phy_bit, phy_width;
	unsigned int lane_cnt;
	LVDS_Config_t *lconf;

	lconf = lcd_conf->lcd_control.lvds_config;
	if (status) {
		WRITE_LCD_REG_BITS(LVDS_GEN_CNTL, 1, 3, 1); /*enable lvds fifo*/
		phy_reg = HHI_DIF_CSI_PHY_CNTL3;
		phy_bit = BIT_PHY_LANE;
		phy_width = WIDTH_PHY_LANE;
		if (lconf->port_sel == LVDS_PORT_A)
			lane_cnt = LVDS_PORT_A;
		else if (lconf->port_sel == LVDS_PORT_B)
			lane_cnt = LVDS_PORT_B;
		else
			lane_cnt = LVDS_PORT_AB;
		WRITE_LCD_CBUS_REG_BITS(phy_reg, lane_cnt, phy_bit, phy_width);
	} else {
		WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL1, 0x0);
		WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL2, 0x0);
		WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL3, 0x0);
	}

	lcd_print("%s: %s\n", __func__, (status ? "ON" : "OFF"));
}

/* the string must match pinctrl-names in dts */
const char *lcd_ports_ttl_pinmux_str[] = {
	"ttl_6bit_hvsync_on",      /* 0 */
	"ttl_6bit_de_on",          /* 1 */
	"ttl_6bit_hvsync_de_on",   /* 2 */
	"ttl_8bit_hvsync_on",      /* 3 */
	"ttl_8bit_de_on",          /* 4 */
	"ttl_8bit_hvsync_de_on",   /* 5 */
	"ttl_6bit_hvsync_de_off",  /* 6 */
	"ttl_8bit_hvsync_de_off",  /* 7 */
};

static void lcd_ports_ctrl_ttl(Bool_t status)
{
	struct pinctrl *pin;
	struct pinctrl_state *s;
	unsigned int pinmux_num;
	int ret;

	if (IS_ERR(lcd_conf->lcd_misc_ctrl.pin)) {
		pr_info("set ttl_ports_ctrl pinmux error.\n");
		return;
	}
	/* get pinmux control */
	pin = lcd_conf->lcd_misc_ctrl.pin;

	if (status) {
		if (lcd_conf->lcd_basic.lcd_bits == 6) {
			if (lcd_conf->lcd_timing.de_valid == 0)
				pinmux_num = 0;
			else if (lcd_conf->lcd_timing.hvsync_valid == 0)
				pinmux_num = 1;
			else
				pinmux_num = 2;
		} else {
			if (lcd_conf->lcd_timing.de_valid == 0)
				pinmux_num = 3;
			else if (lcd_conf->lcd_timing.hvsync_valid == 0)
				pinmux_num = 4;
			else
				pinmux_num = 5;
		}
	} else {
		if (lcd_conf->lcd_basic.lcd_bits == 6)
			pinmux_num = 6;
		else
			pinmux_num = 7;
	}

	/* select pinmux */
	s = pinctrl_lookup_state(pin, lcd_ports_ttl_pinmux_str[pinmux_num]);
	if (IS_ERR(s)) {
		pr_info("set ttl_ports_ctrl pinmux error\n");
		devm_pinctrl_put(pin); /* pinctrl_put(pin); //release pins */
		return;
	}

	/* set pinmux and lock pins */
	ret = pinctrl_select_state(pin, s);
	if (ret < 0) {
		pr_info("set ttl_ports_ctrl pinmux error\n");
		devm_pinctrl_put(pin);
		return;
	}

	lcd_print("%s: %s\n", __func__, (status ? "ON" : "OFF"));
}

static void lcd_ports_ctrl(Bool_t status)
{
	switch (lcd_conf->lcd_basic.lcd_type) {
	case LCD_DIGITAL_LVDS:
		lcd_ports_ctrl_lvds(status);
		break;
	case LCD_DIGITAL_TTL:
		lcd_ports_ctrl_ttl(status);
		break;
	default:
		pr_info("Invalid LCD type\n");
		break;
	}
}

#define LCD_GAMMA_RETRY_CNT            1000
static unsigned char lcd_gamma_init_err = 0;
static void write_gamma_table(u16 *data, u32 rgb_mask, u16 gamma_coeff, u32 gamma_reverse)
{
	int i;
	int cnt = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&gamma_write_lock, flags);
	rgb_mask = gamma_sel_table[rgb_mask];
	while ((!(READ_LCD_REG(L_GAMMA_CNTL_PORT) & (0x1 << LCD_ADR_RDY))) && (cnt < LCD_GAMMA_RETRY_CNT)) {
		udelay(10);
		cnt++;
	};
	WRITE_LCD_REG(L_GAMMA_ADDR_PORT, (0x1 << LCD_H_AUTO_INC) | (0x1 << rgb_mask) | (0x0 << LCD_HADR));
	if (gamma_reverse == 0) {
		for (i=0;i<256;i++) {
			cnt = 0;
			while ((!( READ_LCD_REG(L_GAMMA_CNTL_PORT) & (0x1 << LCD_WR_RDY))) && (cnt < LCD_GAMMA_RETRY_CNT)) {
				udelay(10);
				cnt++;
			};
			WRITE_LCD_REG(L_GAMMA_DATA_PORT, (data[i] * gamma_coeff / 100));
		}
	}
	else {
		for (i=0;i<256;i++) {
			cnt = 0;
			while ((!( READ_LCD_REG(L_GAMMA_CNTL_PORT) & (0x1 << LCD_WR_RDY))) && (cnt < LCD_GAMMA_RETRY_CNT)) {
				udelay(10);
				cnt++;
			};
			WRITE_LCD_REG(L_GAMMA_DATA_PORT, (data[255-i] * gamma_coeff / 100));
		}
	}
	cnt = 0;
	while ((!(READ_LCD_REG(L_GAMMA_CNTL_PORT) & (0x1 << LCD_ADR_RDY))) && (cnt < LCD_GAMMA_RETRY_CNT)) {
		udelay(10);
		cnt++;
	};
	WRITE_LCD_REG(L_GAMMA_ADDR_PORT, (0x1 << LCD_H_AUTO_INC) | (0x1 << rgb_mask) | (0x23 << LCD_HADR));
	
	if (cnt >= LCD_GAMMA_RETRY_CNT)
		lcd_gamma_init_err = 1;
	
	spin_unlock_irqrestore(&gamma_write_lock, flags);
}

static void set_gamma_table_lcd(unsigned int gamma_en)
{
	unsigned int reverse, coeff;
	unsigned short *gtable;

	lcd_print("%s\n", __func__);

	lcd_gamma_init_err = 0;
	reverse = ((lcd_conf->lcd_effect.gamma_ctrl >> GAMMA_CTRL_REVERSE) & 1);
	gtable = lcd_conf->lcd_effect.GammaTableR;
	coeff = lcd_conf->lcd_effect.gamma_r_coeff;
	write_gamma_table(gtable, GAMMA_SEL_R, coeff, reverse);
	gtable = lcd_conf->lcd_effect.GammaTableG;
	coeff = lcd_conf->lcd_effect.gamma_g_coeff;
	write_gamma_table(gtable, GAMMA_SEL_G, coeff, reverse);
	gtable = lcd_conf->lcd_effect.GammaTableB;
	coeff = lcd_conf->lcd_effect.gamma_b_coeff;
	write_gamma_table(gtable, GAMMA_SEL_B, coeff, reverse);

	if (lcd_gamma_init_err) {
		WRITE_LCD_REG_BITS(L_GAMMA_CNTL_PORT, 0, 0, 1);
		printk("[warning]: write gamma table error, gamma table disabled\n");
	}
	else
		WRITE_LCD_REG_BITS(L_GAMMA_CNTL_PORT, gamma_en, 0, 1);
}

static void set_tcon_lcd(Lcd_Config_t *pconf)
{
	Lcd_Timing_t *tcon_adr = &(pconf->lcd_timing);
	unsigned int tcon_pol_ctrl;
	unsigned hs_pol_adj, vs_pol_adj, clk_pol;
	unsigned int gamma_en;

	lcd_print("%s\n", __func__);

	gamma_en = (pconf->lcd_effect.gamma_ctrl >> GAMMA_CTRL_EN) & 1;
	set_gamma_table_lcd(gamma_en);

	WRITE_LCD_REG(L_RGB_BASE_ADDR,  pconf->lcd_effect.rgb_base_addr);
	WRITE_LCD_REG(L_RGB_COEFF_ADDR, pconf->lcd_effect.rgb_coeff_addr);
	if (pconf->lcd_effect.dith_user) {
		WRITE_LCD_REG(L_DITH_CNTL_ADDR,
			pconf->lcd_effect.dith_cntl_addr);
	} else {
		if (pconf->lcd_basic.lcd_bits == 8)
			WRITE_LCD_REG(L_DITH_CNTL_ADDR,  0x400);
		else
			WRITE_LCD_REG(L_DITH_CNTL_ADDR,  0x600);
	}

	tcon_pol_ctrl = pconf->lcd_timing.pol_ctrl;
	clk_pol = (tcon_pol_ctrl >> POL_CTRL_CLK) & 1;
	WRITE_LCD_REG(L_POL_CNTL_ADDR, (clk_pol << LCD_CPH1_POL));

	hs_pol_adj = (((tcon_pol_ctrl >> POL_CTRL_HS) & 1) ? 0 : 1);
	vs_pol_adj = (((tcon_pol_ctrl >> POL_CTRL_VS) & 1) ? 0 : 1);
	/* adjust hvsync pol */
	SET_LCD_REG_MASK(L_POL_CNTL_ADDR, ((0 << LCD_DE_POL) |
		(vs_pol_adj << LCD_VS_POL) | (hs_pol_adj << LCD_HS_POL)));
	/* enable tcon DE, Hsync, Vsync */
	SET_LCD_REG_MASK(L_POL_CNTL_ADDR, ((1 << LCD_TCON_DE_SEL) |
		(1 << LCD_TCON_VS_SEL) | (1 << LCD_TCON_HS_SEL)));

	/* DE signal for lvds */
	WRITE_LCD_REG(L_DE_HS_ADDR,    tcon_adr->de_hs_addr);
	WRITE_LCD_REG(L_DE_HE_ADDR,    tcon_adr->de_he_addr);
	WRITE_LCD_REG(L_DE_VS_ADDR,    tcon_adr->de_vs_addr);
	WRITE_LCD_REG(L_DE_VE_ADDR,    tcon_adr->de_ve_addr);
	/* DE signal for TTL */
	WRITE_LCD_REG(L_OEV1_HS_ADDR,  tcon_adr->de_hs_addr);
	WRITE_LCD_REG(L_OEV1_HE_ADDR,  tcon_adr->de_he_addr);
	WRITE_LCD_REG(L_OEV1_VS_ADDR,  tcon_adr->de_vs_addr);
	WRITE_LCD_REG(L_OEV1_VE_ADDR,  tcon_adr->de_ve_addr);
	WRITE_LCD_REG(L_OEH_HS_ADDR,  tcon_adr->de_hs_addr);
	WRITE_LCD_REG(L_OEH_HE_ADDR,  tcon_adr->de_he_addr);
	WRITE_LCD_REG(L_OEH_VS_ADDR,  tcon_adr->de_vs_addr);
	WRITE_LCD_REG(L_OEH_VE_ADDR,  tcon_adr->de_ve_addr);

	/* Hsync signal */
	WRITE_LCD_REG(L_HSYNC_HS_ADDR, tcon_adr->hs_hs_addr);
	WRITE_LCD_REG(L_HSYNC_HE_ADDR, tcon_adr->hs_he_addr);
	WRITE_LCD_REG(L_HSYNC_VS_ADDR, tcon_adr->hs_vs_addr);
	WRITE_LCD_REG(L_HSYNC_VE_ADDR, tcon_adr->hs_ve_addr);
	if ((tcon_pol_ctrl >> POL_CTRL_HS) & 1) {
		WRITE_LCD_REG(L_STH1_HS_ADDR, tcon_adr->hs_hs_addr);
		WRITE_LCD_REG(L_STH1_HE_ADDR, tcon_adr->hs_he_addr);
	} else {
		WRITE_LCD_REG(L_STH1_HS_ADDR, tcon_adr->hs_he_addr);
		WRITE_LCD_REG(L_STH1_HE_ADDR, tcon_adr->hs_hs_addr);
	}
	WRITE_LCD_REG(L_STH1_VS_ADDR, tcon_adr->hs_vs_addr);
	WRITE_LCD_REG(L_STH1_VE_ADDR, tcon_adr->hs_ve_addr);

	/* Vsync signal */
	WRITE_LCD_REG(L_VSYNC_HS_ADDR, tcon_adr->vs_hs_addr);
	WRITE_LCD_REG(L_VSYNC_HE_ADDR, tcon_adr->vs_he_addr);
	WRITE_LCD_REG(L_VSYNC_VS_ADDR, tcon_adr->vs_vs_addr);
	WRITE_LCD_REG(L_VSYNC_VE_ADDR, tcon_adr->vs_ve_addr);
	WRITE_LCD_REG(L_STV1_HS_ADDR, tcon_adr->vs_hs_addr);
	WRITE_LCD_REG(L_STV1_HE_ADDR, tcon_adr->vs_he_addr);
	if ((tcon_pol_ctrl >> POL_CTRL_VS) & 1) {
		WRITE_LCD_REG(L_STV1_VS_ADDR, tcon_adr->vs_vs_addr);
		WRITE_LCD_REG(L_STV1_VE_ADDR, tcon_adr->vs_ve_addr);
	} else {
		WRITE_LCD_REG(L_STV1_VS_ADDR, tcon_adr->vs_ve_addr);
		WRITE_LCD_REG(L_STV1_VE_ADDR, tcon_adr->vs_vs_addr);
	}

	WRITE_LCD_REG(L_INV_CNT_ADDR,       0);
	WRITE_LCD_REG(L_TCON_MISC_SEL_ADDR,
		((1 << LCD_STV1_SEL) | (1 << LCD_STV2_SEL)));

	if (pconf->lcd_misc_ctrl.vpp_sel)
		CLR_LCD_REG_MASK(VPP2_MISC, (VPP_OUT_SATURATE));
	else
		CLR_LCD_REG_MASK(VPP_MISC, (VPP_OUT_SATURATE));
}

static void lcd_set_pll(unsigned int pll_reg, unsigned int clk_ctrl_reg)
{
	unsigned m, n, od1, od2, od3, frac;
	int wait_loop = PLL_WAIT_LOCK_CNT;
	unsigned pll_lock = 0;
	unsigned pll_ctrl, pll_ctrl2;

	lcd_print("%s\n", __func__);

	m = (pll_reg >> PLL_CTRL_M) & 0x1ff;
	n = (pll_reg >> PLL_CTRL_N) & 0x1f;
	od1 = (pll_reg >> PLL_CTRL_OD1) & 0x3;
	od2 = (pll_reg >> PLL_CTRL_OD2) & 0x3;
	od3 = (pll_reg >> PLL_CTRL_OD3) & 0x3;
	frac = (clk_ctrl_reg >> CLK_CTRL_FRAC) & 0xfff;

	pll_ctrl = ((1 << 30) | (n << 9) | (m << 0));
	pll_ctrl2 = ((od1 << 16) | (od2 << 22) | (od3 << 18));
	if (frac > 0)
		pll_ctrl2 |= ((1 << 14) | (frac << 0));

	WRITE_LCD_CBUS_REG(HHI_HDMI_PLL_CNTL, pll_ctrl | (1 << 28));
	WRITE_LCD_CBUS_REG(HHI_HDMI_PLL_CNTL2, pll_ctrl2);
	WRITE_LCD_CBUS_REG(HHI_HDMI_PLL_CNTL3, 0x0d5c5091);
	WRITE_LCD_CBUS_REG(HHI_HDMI_PLL_CNTL4, 0x801da72c);
	WRITE_LCD_CBUS_REG(HHI_HDMI_PLL_CNTL5, 0x71486980);
	WRITE_LCD_CBUS_REG(HHI_HDMI_PLL_CNTL6, 0x00000a55);
	WRITE_LCD_CBUS_REG(HHI_HDMI_PLL_CNTL, pll_ctrl);

	do {
		udelay(50);
		pll_lock = (READ_LCD_CBUS_REG(HHI_HDMI_PLL_CNTL) >> 31) & 0x1;
		wait_loop--;
	} while ((pll_lock == 0) && (wait_loop > 0));
	if (wait_loop == 0)
		pr_info("[error]: hpll lock failed\n");
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

static void lcd_set_clk_div(unsigned long vid_div_reg)
{
	unsigned int  clk_div;
	unsigned int shift_val, shift_sel;
	int i;
	lcd_print("%s\n", __func__);

	clk_div = (vid_div_reg >> DIV_CTRL_CLK_DIV) & 0xf;

	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_CNTL, 0, 19, 1);
	udelay(5);

	/* Disable the div output clock */
	WRITE_LCD_CBUS_REG_BITS(HHI_VID_PLL_CLK_DIV, 0, 19, 1);
	WRITE_LCD_CBUS_REG_BITS(HHI_VID_PLL_CLK_DIV, 0, 15, 1);

	i = 0;
	while (lcd_clk_div_table[i][0] != CLK_DIV_SEL_MAX) {
		if (clk_div == lcd_clk_div_table[i][0])
			break;
		i++;
	}
	if (lcd_clk_div_table[i][0] == CLK_DIV_SEL_MAX)
		pr_info("invalid clk divider\n");
	shift_val = lcd_clk_div_table[i][1];
	shift_sel = lcd_clk_div_table[i][2];

	if (shift_val == 0xffff) { /* if divide by 1 */
		WRITE_LCD_CBUS_REG_BITS(HHI_VID_PLL_CLK_DIV, 1, 18, 1);
	} else {
		WRITE_LCD_CBUS_REG_BITS(HHI_VID_PLL_CLK_DIV, 0, 16, 2);
		WRITE_LCD_CBUS_REG_BITS(HHI_VID_PLL_CLK_DIV, 0, 15, 1);
		WRITE_LCD_CBUS_REG_BITS(HHI_VID_PLL_CLK_DIV, 0, 0, 14);

		WRITE_LCD_CBUS_REG_BITS(HHI_VID_PLL_CLK_DIV, shift_sel, 16, 2);
		WRITE_LCD_CBUS_REG_BITS(HHI_VID_PLL_CLK_DIV, 1, 15, 1);
		WRITE_LCD_CBUS_REG_BITS(HHI_VID_PLL_CLK_DIV, shift_val, 0, 14);
		WRITE_LCD_CBUS_REG_BITS(HHI_VID_PLL_CLK_DIV, 0, 15, 1);
	}
	/* Enable the final output clock */
	WRITE_LCD_CBUS_REG_BITS(HHI_VID_PLL_CLK_DIV, 1, 19, 1);
}

static void lcd_set_vclk_crt(unsigned int clk_ctrl_reg)
{
	unsigned int xd;
	lcd_print("%s\n", __func__);

	xd = (clk_ctrl_reg >> CLK_CTRL_XD) & 0xff;
	/* setup the XD divider value */
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_DIV, (xd-1), 0, 8);
	udelay(5);
	/* Bit[18:16] - v2_cntl_clk_in_sel */
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_CNTL, 0, 16, 3);
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_CNTL, 1, 19, 1);
	udelay(2);

	/* [15:12] encl_clk_sel, select vclk2_div1 */
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_DIV, 8, 12, 4);
	/* release vclk2_div_reset and enable vclk2_div */
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_DIV, 1, 16, 2);
	udelay(5);

	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_CNTL, 1, 0, 1);
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_CNTL, 1, 15, 1);
	udelay(10);
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_CNTL, 0, 15, 1);
	udelay(5);

	WRITE_LCD_CBUS_REG_BITS(HHI_VID_CLK_CNTL2, 1, 3, 1);
}

static void set_vclk_lcd(Lcd_Config_t *pconf)
{
	unsigned pll_reg, div_reg, clk_reg;
	int xd;
	int lcd_type;
	unsigned long flags = 0;

	lcd_print("%s\n", __func__);

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

	spin_lock_irqsave(&lcd_clk_lock, flags);
	lcd_set_pll(pll_reg, clk_reg);
	lcd_set_clk_div(div_reg);
	lcd_set_vclk_crt(clk_reg);
	spin_unlock_irqrestore(&lcd_clk_lock, flags);
}

static void set_venc_lcd(Lcd_Config_t *pconf)
{
	lcd_print("%s\n", __func__);

	WRITE_LCD_REG(ENCL_VIDEO_EN, 0);
#ifdef CONFIG_AMLOGIC_VOUT2
	if (pconf->lcd_misc_ctrl.vpp_sel) {
		/* viu2 select encl */
		WRITE_LCD_REG_BITS(VPU_VIU_VENC_MUX_CTRL, 0, 2, 2);
	} else {
		/* viu1 select encl */
		WRITE_LCD_REG_BITS(VPU_VIU_VENC_MUX_CTRL, 0, 0, 2);
	}
#else
	/* viu1, viu2 select encl */
	WRITE_LCD_REG_BITS(VPU_VIU_VENC_MUX_CTRL, 0, 0, 4);
#endif

	WRITE_LCD_REG(ENCL_VIDEO_MODE,        0);
	WRITE_LCD_REG(ENCL_VIDEO_MODE_ADV,    0x8); /* Sampling rate: 1 */

	WRITE_LCD_REG(ENCL_VIDEO_FILT_CTRL,   0x1000); /* bypass filter */

	WRITE_LCD_REG(ENCL_VIDEO_MAX_PXCNT,   pconf->lcd_basic.h_period - 1);
	WRITE_LCD_REG(ENCL_VIDEO_MAX_LNCNT,   pconf->lcd_basic.v_period - 1);

	WRITE_LCD_REG(ENCL_VIDEO_HAVON_BEGIN, pconf->lcd_timing.video_on_pixel);
	WRITE_LCD_REG(ENCL_VIDEO_HAVON_END,   pconf->lcd_basic.h_active - 1 +
		pconf->lcd_timing.video_on_pixel);
	WRITE_LCD_REG(ENCL_VIDEO_VAVON_BLINE, pconf->lcd_timing.video_on_line);
	WRITE_LCD_REG(ENCL_VIDEO_VAVON_ELINE, pconf->lcd_basic.v_active - 1 +
		pconf->lcd_timing.video_on_line);

	WRITE_LCD_REG(ENCL_VIDEO_HSO_BEGIN,   10);
	WRITE_LCD_REG(ENCL_VIDEO_HSO_END,     16);
	WRITE_LCD_REG(ENCL_VIDEO_VSO_BEGIN,   pconf->lcd_timing.vso_hstart);
	WRITE_LCD_REG(ENCL_VIDEO_VSO_END,     pconf->lcd_timing.vso_hstart);
	WRITE_LCD_REG(ENCL_VIDEO_VSO_BLINE,   pconf->lcd_timing.vso_vstart);
	WRITE_LCD_REG(ENCL_VIDEO_VSO_ELINE,   pconf->lcd_timing.vso_vstart + 2);

	WRITE_LCD_REG(ENCL_VIDEO_RGBIN_CTRL,  (1 << 0)); /*bit[0] 1:RGB, 0:YUV*/

	WRITE_LCD_REG(ENCL_VIDEO_EN,          1);
}

static void clk_util_lvds_set_clk_div(Lcd_Config_t *pconf)
{
	unsigned int phy_div2, wr_mode;

	if (pconf->lcd_control.lvds_config->dual_port == 0) {
		phy_div2 = 0;
		wr_mode = 1;
	} else {
		phy_div2 = 1;
		wr_mode = 3;
	}

	/* ---------------------------------------------
	// Configure the LVDS PHY
	// ---------------------------------------------
	// wire    [4:0]   cntl_ser_en         = control[20:16];
	// wire            cntl_prbs_en        = control[13];
	// wire            cntl_prbs_err_en    = control[12];
	// wire    [1:0]   cntl_mode_set_high  = control[11:10];
	// wire    [1:0]   cntl_mode_set_low   = control[9:8];
	//
	// wire    [1:0]   fifo_clk_sel        = control[7;6]
	//
	// wire            mode_port_rev       = control[4];
	// wire            mode_bit_rev        = control[3];
	// wire            mode_inv_p_n        = control[2];
	// wire            phy_clk_en          = control[1];
	// wire            soft_reset_int      = control[0];
	// enable all serializers, divide by 7 */
	WRITE_LCD_CBUS_REG(HHI_LVDS_TX_PHY_CNTL0, (0xfff << 16) | (0x1 << 6));
	WRITE_LCD_CBUS_REG(HHI_LVDS_TX_PHY_CNTL1,
			(1 << 30) | (phy_div2 << 25) | (1 << 24));

	/*    lvds_gen_cntl       <= {10'h0,     // [15:4] unused
	//                            2'h1,      // [5:4] divide by 7 in the PHY
	//                            1'b0,      // [3] fifo_en
	//                            1'b0,      // [2] wr_bist_gate
	//                            2'b00};    // [1:0] fifo_wr mode
	//FIFO_CLK_SEL = 1; // div7 */
	WRITE_LCD_REG_BITS(LVDS_GEN_CNTL, 1, 4, 2); /* lvds fifo clk div 7 */
	WRITE_LCD_REG_BITS(LVDS_GEN_CNTL, wr_mode, 0, 2);  /* fifo wr mode */

	/* lvds phy div reset */
	WRITE_LCD_CBUS_REG_BITS(HHI_LVDS_TX_PHY_CNTL0, 1, 0, 1);
	udelay(5);
	/* Release lvds div reset */
	WRITE_LCD_CBUS_REG_BITS(HHI_LVDS_TX_PHY_CNTL0, 0, 0, 1);
}

static void set_control_lvds(Lcd_Config_t *pconf)
{
	unsigned int lvds_repack, pn_swap, bit_num;
	unsigned int dual_port, port_swap;
	unsigned int data32;

	lcd_print("%s\n", __func__);
	clk_util_lvds_set_clk_div(pconf);

	WRITE_LCD_REG_BITS(LVDS_GEN_CNTL, 0, 3, 1); /* disable lvds fifo */

	data32 = (0x00 << LVDS_BLANK_DATA_R) |
		(0x00 << LVDS_BLANK_DATA_G) |
		(0x00 << LVDS_BLANK_DATA_B);
	WRITE_LCD_REG(LVDS_BLANK_DATA_HI, (data32 >> 16));
	WRITE_LCD_REG(LVDS_BLANK_DATA_LO, (data32 & 0xffff));

	dual_port = pconf->lcd_control.lvds_config->dual_port;
	port_swap = pconf->lcd_control.lvds_config->port_swap;
	lvds_repack = (pconf->lcd_control.lvds_config->lvds_repack) & 0x1;
	pn_swap = (pconf->lcd_control.lvds_config->pn_swap) & 0x1;

	switch (pconf->lcd_basic.lcd_bits) {
	case 10:
		bit_num = 0;
		break;
	case 8:
		bit_num = 1;
		break;
	case 6:
		bit_num = 2;
		break;
	case 4:
		bit_num = 3;
		break;
	default:
		bit_num = 1;
		break;
	}

	WRITE_LCD_REG(LVDS_PACK_CNTL_ADDR,
		(lvds_repack << 0) | /* repack */
		(port_swap << 2) | /* odd_even */
		(0 << 3) | /* reserve */
		(0 << 4) | /* lsb first */
		(pn_swap << 5) | /* pn swap */
		(dual_port << 6) | /* dual port */
		(0 << 7) | /* use tcon control */
		(bit_num << 8) | /* 0:10bits, 1:8bits, 2:6bits, 3:4bits. */
		(0 << 10) | /*r_select  //0:R, 1:G, 2:B, 3:0 */
		(1 << 12) | /*g_select  //0:R, 1:G, 2:B, 3:0 */
		(2 << 14));  /*b_select  //0:R, 1:G, 2:B, 3:0;  */

	/* WRITE_LCD_REG_BITS(LVDS_GEN_CNTL, 1, 3, 1);  //enable fifo */
}

static void set_control_ttl(Lcd_Config_t *pconf)
{
	unsigned rb_port_swap, rgb_bit_swap;

	rb_port_swap = (unsigned)(pconf->lcd_control.ttl_config->rb_swap);
	rgb_bit_swap = (unsigned)(pconf->lcd_control.ttl_config->bit_swap);

	WRITE_LCD_REG(L_DUAL_PORT_CNTL_ADDR,
		(rb_port_swap << LCD_RGB_SWP) | (rgb_bit_swap << LCD_BIT_SWP));
}

static void init_phy_lvds(Lcd_Config_t *pconf)
{
	unsigned int swing_level;
	unsigned int temp;
	lcd_print("%s\n", __func__);

	WRITE_LCD_REG(LVDS_SER_EN, 0xfff); /* Enable the serializers */

	WRITE_LCD_REG(LVDS_PHY_CNTL0, 0xffff);
	WRITE_LCD_REG(LVDS_PHY_CNTL1, 0xff00);
	WRITE_LCD_REG(LVDS_PHY_CNTL4, 0x007f);

	swing_level = pconf->lcd_control.lvds_config->lvds_vswing;
	swing_level = (swing_level >= LVDS_VSWING_LEVEL_MAX) ?
		(LVDS_VSWING_LEVEL_MAX - 1) : swing_level;

	temp = 0x606cca80 | (lvds_vswing_ctrl[swing_level] << 26);
	WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL1, temp);
	WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL2, 0x0000006c);
	WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL3, 0x00000800);
}

static void init_dphy(Lcd_Config_t *pconf)
{
	unsigned lcd_type = (unsigned)(pconf->lcd_basic.lcd_type);

	switch (lcd_type) {
	case LCD_DIGITAL_LVDS:
		WRITE_LCD_CBUS_REG(HHI_DSI_LVDS_EDP_CNTL0, lcd_type);
		init_phy_lvds(pconf);
		break;
	default:
		break;
	}
}

static void _init_lcd_driver(Lcd_Config_t *pconf)
{
	int lcd_type = pconf->lcd_basic.lcd_type;
	unsigned char ss_level;

	print_lcd_driver_version();
	request_vpu_clk_vmod(pconf->lcd_timing.lcd_clk, VMODE_LCD);
	switch_vpu_mem_pd_vmod(VMODE_LCD, VPU_MEM_POWER_ON);
	switch_lcd_mod_gate(ON);

	ss_level = (pconf->lcd_timing.clk_ctrl >> CLK_CTRL_SS) & 0xf;
	pr_info("Init LCD mode: %s, %s(%u) %ubit, %ux%u@%u.%uHz\n",
	pconf->lcd_basic.model_name, lcd_type_table[lcd_type],
	lcd_type, pconf->lcd_basic.lcd_bits, pconf->lcd_basic.h_active,
	pconf->lcd_basic.v_active, (pconf->lcd_timing.sync_duration_num / 10),
	(pconf->lcd_timing.sync_duration_num % 10));
	pr_info("ss_level=%u(%s)\n", ss_level, lcd_ss_level_table[ss_level]);

	set_vclk_lcd(pconf);
	set_venc_lcd(pconf);
	set_tcon_lcd(pconf);
	switch (lcd_type) {
	case LCD_DIGITAL_LVDS:
		set_control_lvds(pconf);
		init_dphy(pconf);
		break;
	case LCD_DIGITAL_TTL:
		set_control_ttl(pconf);
		break;
	default:
		pr_info("Invalid LCD type.\n");
		break;
	}
	pr_info("%s finished.\n", __func__);
}

static void _disable_lcd_driver(Lcd_Config_t *pconf)
{
	WRITE_LCD_REG_BITS(LVDS_GEN_CNTL, 0, 3, 1); /* disable lvds fifo */

	WRITE_LCD_REG(ENCL_VIDEO_EN, 0); /* disable encl */
	WRITE_LCD_CBUS_REG_BITS(HHI_VID_CLK_CNTL2, 0, 3, 1);
	/* close vclk2 gate: 0x104b[4:0] */
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_CNTL, 0, 0, 5);
	/* close vid2_pll gate: 0x104c[16] */
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_DIVIDER_CNTL, 0, 16, 1);
	/* disable vid_pll: 0x10c8[30] */
	WRITE_LCD_CBUS_REG_BITS(HHI_VID_PLL_CNTL, 0, 30, 1);

	switch_lcd_mod_gate(OFF);
	switch_vpu_mem_pd_vmod(VMODE_LCD, VPU_MEM_POWER_DOWN);
	release_vpu_clk_vmod(VMODE_LCD);
	pr_info("disable lcd display driver.\n");
}

static void _enable_vsync_interrupt(void)
{
	WRITE_LCD_REG(VENC_INTCTRL, 0x200);
}

#define LCD_ENC_TST_NUM_MAX    8
static char *lcd_enc_tst_str[] = {
	"None",        /* 0 */
	"Color Bar",   /* 1 */
	"Thin Line",   /* 2 */
	"Dot Grid",    /* 3 */
	"Gray",        /* 4 */
	"Red",         /* 5 */
	"Green",       /* 6 */
	"Blue",        /* 7 */
};

static unsigned int lcd_enc_tst[][6] = {
/*	tst_mode,  Y,       Cb,     Cr,    tst_en, vfifo_en */
	{0,       0x200,   0x200,  0x200,   0,      1},  /* 0 */
	{1,       0x200,   0x200,  0x200,   1,      0},  /* 1 */
	{2,       0x200,   0x200,  0x200,   1,      0},  /* 2 */
	{3,       0x200,   0x200,  0x200,   1,      0},  /* 3 */
	{0,       0x200,   0x200,  0x200,   1,      0},  /* 4 */
	{0,       0x130,   0x153,  0x3fd,   1,      0},  /* 5 */
	{0,       0x256,   0x0ae,  0x055,   1,      0},  /* 6 */
	{0,       0x074,   0x3fd,  0x1ad,   1,      0},  /* 7 */
};

static void lcd_test(unsigned int num)
{
	num = (num >= LCD_ENC_TST_NUM_MAX) ? 0 : num;

	WRITE_LCD_REG(ENCL_TST_MDSEL, lcd_enc_tst[num][0]);
	WRITE_LCD_REG(ENCL_TST_Y, lcd_enc_tst[num][1]);
	WRITE_LCD_REG(ENCL_TST_CB, lcd_enc_tst[num][2]);
	WRITE_LCD_REG(ENCL_TST_CR, lcd_enc_tst[num][3]);
	WRITE_LCD_REG(ENCL_TST_CLRBAR_STRT,
		lcd_conf->lcd_timing.video_on_pixel);
	WRITE_LCD_REG(ENCL_TST_CLRBAR_WIDTH,
		(lcd_conf->lcd_basic.h_active / 9));
	WRITE_LCD_REG(ENCL_TST_EN, lcd_enc_tst[num][4]);
	WRITE_LCD_REG_BITS(ENCL_VIDEO_MODE_ADV, lcd_enc_tst[num][5], 3, 1);

	if (num > 0) {
		pr_info("show test pattern %d: %s\n",
			num, lcd_enc_tst_str[num]);
	} else
		pr_info("disable test pattern\n");
}

/* ***********************************************
// sysfs api for video
// *********************************************** */
static ssize_t lcd_video_vso_read(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "read vso start: %u\n",
		lcd_conf->lcd_timing.vso_vstart);
}

static ssize_t lcd_video_vso_write(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int temp;

	temp = 10;
	ret = sscanf(buf, "%u", &temp);
	lcd_conf->lcd_timing.vso_vstart = (unsigned short)temp;
	lcd_conf->lcd_timing.vso_user = 1;
	WRITE_LCD_REG(ENCL_VIDEO_VSO_BLINE, lcd_conf->lcd_timing.vso_vstart);
	WRITE_LCD_REG(ENCL_VIDEO_VSO_ELINE,
		lcd_conf->lcd_timing.vso_vstart + 2);
	pr_info("set vso start: %u\n", lcd_conf->lcd_timing.vso_vstart);

	if ((ret != 1) || (ret != 2))
		return -EINVAL;

	return count;
	/* return 0; */
}

static struct class_attribute lcd_video_class_attrs[] = {
	__ATTR(vso,  S_IRUGO | S_IWUSR,
		lcd_video_vso_read, lcd_video_vso_write),
};

static int creat_lcd_video_attr(Lcd_Config_t *pconf)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(lcd_video_class_attrs); i++) {
		if (class_create_file(pconf->lcd_misc_ctrl.debug_class,
			&lcd_video_class_attrs[i])) {
			pr_info("create lcd_video attribute %s fail\n",
				lcd_video_class_attrs[i].attr.name);
		}
	}

	return 0;
}

static int remove_lcd_video_attr(Lcd_Config_t *pconf)
{
	int i;

	if (pconf->lcd_misc_ctrl.debug_class == NULL)
		return -1;

	for (i = 0; i < ARRAY_SIZE(lcd_video_class_attrs); i++) {
		class_remove_file(pconf->lcd_misc_ctrl.debug_class,
			&lcd_video_class_attrs[i]);
	}

	return 0;
}
/* *********************************************** */

static DEFINE_MUTEX(lcd_init_mutex);
static void lcd_module_enable(void)
{
	mutex_lock(&lcd_init_mutex);

	_init_lcd_driver(lcd_conf);
	lcd_conf->lcd_power_ctrl.power_ctrl(ON);
	_enable_vsync_interrupt();
	lcd_conf->lcd_misc_ctrl.lcd_status = 1;
	mutex_unlock(&lcd_init_mutex);
}

static void lcd_module_disable(void)
{
	mutex_lock(&lcd_init_mutex);
	lcd_conf->lcd_misc_ctrl.lcd_status = 0;
	lcd_conf->lcd_power_ctrl.power_ctrl(OFF);
	_disable_lcd_driver(lcd_conf);
	mutex_unlock(&lcd_init_mutex);
}

static unsigned int clk_div_calc(unsigned int clk,
		unsigned int div_sel, int dir)
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
		pr_info("[Error]clk_div_sel:  Invalid parameter\n");
		break;
	}

	return clk_ret;
}

struct pll_para_s {
	unsigned int m;
	unsigned int n;
	unsigned int frac;
	unsigned int od1_sel;
	unsigned int od2_sel;
	unsigned int od3_sel;
};

static int check_pll(struct pll_para_s *pll, unsigned int pll_fout)
{
	unsigned int fin, m, n;
	unsigned int od1_sel, od2_sel, od3_sel, od1, od2, od3;
	unsigned int pll_fod2_in, pll_fod3_in, pll_fvco;
	unsigned int od_fb = 0, pll_frac;
	int done;

	done = 0;
	fin = FIN_FREQ; /* kHz */
	for (od3_sel = OD_SEL_MAX; od3_sel > 0; od3_sel--) {
		od3 = od_table[od3_sel - 1];
		pll_fod3_in = pll_fout * od3;
		for (od2_sel = od3_sel; od2_sel > 0; od2_sel--) {
			od2 = od_table[od2_sel - 1];
			pll_fod2_in = pll_fod3_in * od2;
			for (od1_sel = od2_sel; od1_sel > 0; od1_sel--) {
				od1 = od_table[od1_sel - 1];
				pll_fvco = pll_fod2_in * od1;
				if ((pll_fvco < PLL_VCO_MIN) ||
					(pll_fvco > PLL_VCO_MAX)) {
					continue;
				}
				pll->od1_sel = od1_sel - 1;
				pll->od2_sel = od2_sel - 1;
				pll->od3_sel = od3_sel - 1;
				lcd_print("od1_sel=%d, od2_sel=%d, od3_sel=%d",
					(od1_sel - 1), (od2_sel - 1),
					(od3_sel - 1));
				lcd_print("pll_fvco=%d\n", pll_fvco);
				n = 1;
				od_fb = 0; /* pll default */
				pll_fvco = pll_fvco / ((od_fb + 1) * 2);
				m = pll_fvco / fin;
				pll_frac = (pll_fvco % fin) * 4096 / fin;
				pll->m = m;
				pll->n = n;
				pll->frac = pll_frac;
				lcd_print("pll_m=%d, pll_n=%d, pll_frac=%d\n",
					m, n, pll_frac);
				done = 1;
			}
		}
	}
	return done;
}

static void generate_clk_parameter(Lcd_Config_t *pconf)
{
	struct pll_para_s pll;
	int ret = 0;

	unsigned clk_div_sel, crt_xd;
	unsigned crt_xd_max;
	unsigned fout_pll, clk_div_out;
	unsigned tmp;
	unsigned fout;

	fout = pconf->lcd_timing.lcd_clk / 1000; /* kHz */

	if (fout > ENCL_MAX_CLK_IN)
		goto generate_clk_done;

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_DIGITAL_LVDS:
		clk_div_sel = CLK_DIV_SEL_1;//CLK_DIV_SEL_7;
		crt_xd_max = CRT_VID_DIV_MAX;
		crt_xd = 7;
		clk_div_out = fout * crt_xd;
		if (clk_div_out > CRT_VID_MAX_CLK_IN)
			goto generate_clk_done;
		lcd_print("fout=%d, crt_xd=%d, clk_div_out=%d\n",
			fout, crt_xd, clk_div_out);
		fout_pll = clk_div_calc(clk_div_out,
				clk_div_sel, CLK_DIV_O2I);
		if (fout_pll > CLK_DIV_MAX_CLK_IN)
			goto generate_clk_done;
		lcd_print("clk_div_sel=%d, fout_pll=%d\n",
			clk_div_sel, fout_pll);
		ret = check_pll(&pll, fout_pll);
		if (ret)
			goto generate_clk_done;
		break;
	case LCD_DIGITAL_TTL:
		clk_div_sel = CLK_DIV_SEL_1;
		crt_xd_max = CRT_VID_DIV_MAX;
		for (crt_xd = 1; crt_xd <= crt_xd_max; crt_xd++) {
			clk_div_out = fout * crt_xd;
			if (clk_div_out > CRT_VID_MAX_CLK_IN)
				continue;
			lcd_print("fout=%d, crt_xd=%d, clk_div_out=%d\n",
				fout, crt_xd, clk_div_out);
			fout_pll = clk_div_calc(clk_div_out,
					clk_div_sel, CLK_DIV_O2I);
			if (fout_pll > CLK_DIV_MAX_CLK_IN)
				continue;
			lcd_print("clk_div_sel=%d, fout_pll=%d\n",
				clk_div_sel, fout_pll);
			ret = check_pll(&pll, fout_pll);
			if (ret)
				goto generate_clk_done;
		}
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
	} else {
		pconf->lcd_timing.pll_ctrl = (0 << PLL_CTRL_OD1) |
			(1 << PLL_CTRL_OD2) | (1 << PLL_CTRL_OD3) |
			(1 << PLL_CTRL_N) | (65 << PLL_CTRL_M);
		pconf->lcd_timing.div_ctrl =
			(CLK_DIV_SEL_1 << DIV_CTRL_CLK_DIV);
		pconf->lcd_timing.clk_ctrl = (pconf->lcd_timing.clk_ctrl &
			~(0xff << CLK_CTRL_XD)) | (7 << CLK_CTRL_XD);
		pr_info("Out of clock range, reset to default setting!\n");
	}
}

static void lcd_sync_duration(Lcd_Config_t *pconf)
{
	unsigned m, n, od1, od2, od3, od_fb, frac, clk_div, xd;
	unsigned h_period, v_period, sync_duration_num, sync_duration_den;
	unsigned pll_out_clk, lcd_clk;

	m = ((pconf->lcd_timing.pll_ctrl) >> PLL_CTRL_M) & 0x1ff;
	n = ((pconf->lcd_timing.pll_ctrl) >> PLL_CTRL_N) & 0x1f;
	od1 = ((pconf->lcd_timing.pll_ctrl) >> PLL_CTRL_OD1) & 0x3;
	od2 = ((pconf->lcd_timing.pll_ctrl) >> PLL_CTRL_OD2) & 0x3;
	od3 = ((pconf->lcd_timing.pll_ctrl) >> PLL_CTRL_OD3) & 0x3;
	od1 = od_table[od1];
	od2 = od_table[od2];
	od3 = od_table[od3];
	frac = ((pconf->lcd_timing.clk_ctrl) >> CLK_CTRL_FRAC) & 0xfff;
	od_fb = 0;
	clk_div = ((pconf->lcd_timing.div_ctrl) >> DIV_CTRL_CLK_DIV) & 0xff;
	xd = ((pconf->lcd_timing.clk_ctrl) >> CLK_CTRL_XD) & 0xff;

	h_period = pconf->lcd_basic.h_period;
	v_period = pconf->lcd_basic.v_period;

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_DIGITAL_LVDS:
		xd = 1;
		clk_div = CLK_DIV_SEL_7;
		break;
	case LCD_DIGITAL_TTL:
		clk_div = CLK_DIV_SEL_1;
		break;
	default:
		break;
	}

	od_fb = (od_fb + 1) * 2;
	pll_out_clk = (frac * od_fb * FIN_FREQ) / 4096;
	pll_out_clk = ((m * od_fb * FIN_FREQ + pll_out_clk) /
		(n * od1 * od2 * od3));
	lcd_clk = clk_div_calc(pll_out_clk, clk_div, CLK_DIV_I2O) / xd;
	pconf->lcd_timing.lcd_clk = lcd_clk * 1000;
	sync_duration_num = ((lcd_clk * 1000 / h_period) * 100) / v_period;
	sync_duration_num = (sync_duration_num + 5) / 10;
	sync_duration_den = 10;

	pconf->lcd_timing.sync_duration_num = sync_duration_num;
	pconf->lcd_timing.sync_duration_den = sync_duration_den;
	pr_info("lcd_clk=%u.%03uMHz, frame_rate=%u.%uHz\n\n",
		(lcd_clk / 1000), (lcd_clk % 1000),
		(sync_duration_num / sync_duration_den),
		((sync_duration_num * 10 / sync_duration_den) % 10));
}

static void lcd_tcon_config(Lcd_Config_t *pconf)
{
	unsigned short de_hstart, de_vstart;
	unsigned short hstart, hend, vstart, vend;
	unsigned short h_delay = 0;
	unsigned short h_offset = 0, v_offset = 0, vsync_h_phase = 0;
	unsigned short h_period, v_period, h_active, v_active;

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_DIGITAL_LVDS:
		h_delay = LVDS_DELAY;
		break;
	case LCD_DIGITAL_TTL:
		h_delay = TTL_DELAY;
		break;
	default:
		h_delay = 0;
		break;
	}
	h_period = pconf->lcd_basic.h_period;
	v_period = pconf->lcd_basic.v_period;
	h_active = pconf->lcd_basic.h_active;
	v_active = pconf->lcd_basic.v_active;

	pconf->lcd_timing.video_on_pixel = h_period - h_active - 1 - h_delay;
	pconf->lcd_timing.video_on_line = v_period - v_active;

	h_offset = (pconf->lcd_timing.h_offset & 0xffff);
	v_offset = (pconf->lcd_timing.v_offset & 0xffff);
	if ((pconf->lcd_timing.h_offset >> 31) & 1) {
		de_hstart = (h_period - h_active - 1 + h_period - h_offset) %
			h_period;
	} else
		de_hstart = (h_period - h_active - 1 + h_offset) % h_period;
	if ((pconf->lcd_timing.v_offset >> 31) & 1) {
		de_vstart = (v_period - v_active + v_period - v_offset) %
			v_period;
	} else
		de_vstart = (v_period - v_active + v_offset) % v_period;

	hstart = (de_hstart + h_period - pconf->lcd_timing.hsync_bp -
		pconf->lcd_timing.hsync_width) % h_period;
	hend = (de_hstart + h_period - pconf->lcd_timing.hsync_bp) % h_period;
	pconf->lcd_timing.hs_hs_addr = hstart;
	pconf->lcd_timing.hs_he_addr = hend;
	pconf->lcd_timing.hs_vs_addr = 0;
	pconf->lcd_timing.hs_ve_addr = v_period - 1;

	vsync_h_phase = (pconf->lcd_timing.vsync_h_phase & 0xffff);
	if ((pconf->lcd_timing.vsync_h_phase >> 31) & 1) /* negative */
		vsync_h_phase = (hstart + h_period - vsync_h_phase) % h_period;
	else /* positive */
		vsync_h_phase = (hstart + h_period + vsync_h_phase) % h_period;
	pconf->lcd_timing.vs_hs_addr = vsync_h_phase;
	pconf->lcd_timing.vs_he_addr = vsync_h_phase;
	vstart = (de_vstart + v_period - pconf->lcd_timing.vsync_bp -
		pconf->lcd_timing.vsync_width) % v_period;
	vend = (de_vstart + v_period - pconf->lcd_timing.vsync_bp) % v_period;
	pconf->lcd_timing.vs_vs_addr = vstart;
	pconf->lcd_timing.vs_ve_addr = vend;

	pconf->lcd_timing.de_hs_addr = de_hstart;
	pconf->lcd_timing.de_he_addr = (de_hstart + h_active) % h_period;
	pconf->lcd_timing.de_vs_addr = de_vstart;
	pconf->lcd_timing.de_ve_addr = (de_vstart + v_active - 1) % v_period;

	if (pconf->lcd_timing.vso_user == 0) {
		/*pconf->lcd_timing.vso_hstart = pconf->lcd_timing.vs_hs_addr;*/
		pconf->lcd_timing.vso_vstart = pconf->lcd_timing.vs_vs_addr;
	}

	/*lcd_print("hs_hs_addr=%d, hs_he_addr=%d, ",
		pconf->lcd_timing.hs_hs_addr, pconf->lcd_timing.hs_he_addr);
	lcd_print("hs_vs_addr=%d, hs_ve_addr=%d\n",
		pconf->lcd_timing.hs_vs_addr, pconf->lcd_timing.hs_ve_addr);
	//lcd_print("vs_hs_addr=%d, vs_he_addr=%d, ",
		pconf->lcd_timing.vs_hs_addr, pconf->lcd_timing.vs_he_addr);
	lcd_print("vs_vs_addr=%d, vs_ve_addr=%d\n",
		pconf->lcd_timing.vs_vs_addr, pconf->lcd_timing.vs_ve_addr);
	//lcd_print("de_hs_addr=%d, de_he_addr=%d, ",
		pconf->lcd_timing.de_hs_addr, pconf->lcd_timing.de_he_addr);
	lcd_print("de_vs_addr=%d, de_ve_addr=%d\n",
		pconf->lcd_timing.de_vs_addr, pconf->lcd_timing.de_ve_addr); */
}

static void lcd_control_config_pre(Lcd_Config_t *pconf)
{
	unsigned int ss_level;

	/* prepare refer clock for frame_rate setting */
	if (pconf->lcd_timing.lcd_clk < 200) {
		pconf->lcd_timing.lcd_clk =
			(pconf->lcd_timing.lcd_clk * pconf->lcd_basic.h_period *
			pconf->lcd_basic.v_period);
	}

	ss_level = ((pconf->lcd_timing.clk_ctrl >> CLK_CTRL_SS) & 0xf);
	ss_level = ((ss_level >= SS_LEVEL_MAX) ? (SS_LEVEL_MAX - 1) : ss_level);

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_DIGITAL_LVDS:
		if (pconf->lcd_control.lvds_config->lvds_repack_user == 0) {
			if (pconf->lcd_basic.lcd_bits == 6)
				pconf->lcd_control.lvds_config->lvds_repack = 0;
			else
				pconf->lcd_control.lvds_config->lvds_repack = 1;
		}
		if (pconf->lcd_control.lvds_config->dual_port == 0) {
			if (pconf->lcd_control.lvds_config->port_swap == 0) {
				pconf->lcd_control.lvds_config->port_sel =
					LVDS_PORT_A;
			} else {
				pconf->lcd_control.lvds_config->port_sel =
					LVDS_PORT_B;
			}
		} else {
			pconf->lcd_control.lvds_config->port_sel = LVDS_PORT_AB;
		}
		break;
	default:
		break;
	}
	pconf->lcd_timing.clk_ctrl = ((pconf->lcd_timing.clk_ctrl &
		(~(0xf << CLK_CTRL_SS))) | (ss_level << CLK_CTRL_SS));
}

#ifdef CONFIG_USE_OF
static LVDS_Config_t lcd_lvds_config = {
	.lvds_vswing = 1,
	.lvds_repack_user = 0,
	.lvds_repack = 0,
	.pn_swap = 0,
};

static TTL_Config_t lcd_ttl_config = {
	.rb_swap = 0,
	.bit_swap = 0,
};

static Lcd_Config_t lcd_config = {
	.lcd_timing = {
		.lcd_clk = 40000000,
		.clk_ctrl = ((1 << CLK_CTRL_AUTO) | (0 << CLK_CTRL_SS)),
		.hvsync_valid = 1,
		.de_valid = 1,
		.pol_ctrl = ((0 << POL_CTRL_CLK) | (1 << POL_CTRL_DE) |
			(0 << POL_CTRL_VS) | (0 << POL_CTRL_HS)),
	},
	.lcd_effect = {
		.rgb_base_addr = 0xf0,
		.rgb_coeff_addr = 0x74a,
		.dith_user = 0,
		.vadj_brightness = 0x0,
		.vadj_contrast = 0x80,
		.vadj_saturation = 0x100,
		.gamma_ctrl = ((0 << GAMMA_CTRL_REVERSE) |
			(1 << GAMMA_CTRL_EN)),
		.gamma_r_coeff = 100,
		.gamma_g_coeff = 100,
		.gamma_b_coeff = 100,
		.set_gamma_table = set_gamma_table_lcd,
	},
	.lcd_control = {
		.lvds_config = &lcd_lvds_config,
		.ttl_config = &lcd_ttl_config,
	},
	.lcd_power_ctrl = {
		.power_on_step = 0,
		.power_off_step = 0,
		.power_ctrl = NULL,
	},
};

Lcd_Config_t *get_lcd_config(void)
{
	return &lcd_config;
}
#endif

static void lcd_config_assign(Lcd_Config_t *pconf)
{
	pconf->lcd_timing.vso_hstart = 10; /* for video process */
	pconf->lcd_timing.vso_vstart = 10; /* for video process */
	pconf->lcd_timing.vso_user = 0; /* use default config */

	pconf->lcd_power_ctrl.ports_ctrl = lcd_ports_ctrl;

	pconf->lcd_misc_ctrl.vpp_sel = 0;
	if (READ_LCD_REG(ENCL_VIDEO_EN) & 1)
		pconf->lcd_misc_ctrl.lcd_status = 1;
	else
		pconf->lcd_misc_ctrl.lcd_status = 0;
	pconf->lcd_misc_ctrl.module_enable = lcd_module_enable;
	pconf->lcd_misc_ctrl.module_disable = lcd_module_disable;
	pconf->lcd_misc_ctrl.lcd_test = lcd_test;
	pconf->lcd_misc_ctrl.print_version = print_lcd_driver_version;
}

void lcd_config_init(Lcd_Config_t *pconf)
{
	lcd_control_config_pre(pconf);

	if ((pconf->lcd_timing.clk_ctrl >> CLK_CTRL_AUTO) & 1) {
		pr_info("\nAuto generate clock parameters.\n");
		generate_clk_parameter(pconf);
	} else {
		pr_info("\nCustome clock parameters.\n");
	}
	pr_info("pll_ctrl=0x%x, div_ctrl=0x%x, clk_ctrl=0x%x.\n",
		pconf->lcd_timing.pll_ctrl, pconf->lcd_timing.div_ctrl,
		pconf->lcd_timing.clk_ctrl);

	lcd_sync_duration(pconf);
	lcd_tcon_config(pconf);
}

void lcd_config_probe(Lcd_Config_t *pconf)
{
	spin_lock_init(&gamma_write_lock);
	spin_lock_init(&lcd_clk_lock);

	lcd_conf = pconf;
	lcd_config_assign(pconf);

	creat_lcd_video_attr(pconf);
}

void lcd_config_remove(Lcd_Config_t *pconf)
{
	remove_lcd_video_attr(pconf);
}
