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
#include <linux/amlogic/vout/vout_notify.h>
#include <linux/amlogic/aml_bl.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/amlogic/logo/logo.h>
#include <plat/regops.h>
#include <mach/am_regs.h>
#include <linux/amlogic/vout/lcd_reg.h>
#include <linux/amlogic/vout/lcdoutc.h>
#include <linux/amlogic/vout/lcd_aml.h>
#include <mach/clock.h>
#if MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8
#include <mach/vpu.h>
#endif
#include <mach/mod_gate.h>
#include <asm/fiq.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/of.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/amlogic/aml_lcd_bl.h>
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
#include <mach/mlvds_regs.h>
#endif
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
#include "mipi_dsi_util.h"
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
#include "edp_drv.h"
#endif
#include <linux/amlogic/vout/aml_lcd_extern.h>
#ifdef CONFIG_AMLOGIC_BOARD_HAS_PMU
#include <linux/amlogic/aml_pmu_common.h>
#endif

#define VPP_OUT_SATURATE	(1 << 0)

#define PANEL_NAME		"panel"

#define FIQ_VSYNC
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
#define DRV_TYPE "c6"
#elif (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
#define DRV_TYPE "c8"
#elif (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B)
#define DRV_TYPE "c8b"
#endif
#define DRIVER_DATE		"20140522"

//#define LCD_DEBUG_INFO
#ifdef LCD_DEBUG_INFO
#define DBG_PRINT(...)		printk(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#endif

unsigned int vpp2_sel = 0; /*0,vpp; 1, vpp2 */
unsigned int lcd_status_flag = 1;

typedef struct {
	Lcd_Config_t *pConf;
	vinfo_t lcd_info;
	struct pinctrl *p;
} lcd_dev_t;

static lcd_dev_t *pDev = NULL;
static spinlock_t gamma_write_lock;
static spinlock_t lcd_clk_lock;
static Bool_t data_status = ON;
static int bl_status = ON;

static inline void lcd_mdelay(int n)
{
	mdelay(n);
}

#ifdef CONFIG_USE_OF
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
static DSI_Config_t lcd_mipi_config = {
    .lane_num = 4,
    .bit_rate_min = 0,
    .bit_rate_max = 0,
    .transfer_ctrl = 0,
    .dsi_init_on = NULL,
    .dsi_init_off = NULL,
    .lcd_extern_init = 0,
};
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
static EDP_Config_t lcd_edp_config = {
	.link_user = 0,
	.link_rate = 1,
	.lane_count = 4,
	.link_adaptive = 0,
	.vswing = 0,
	.preemphasis = 0,
};
#endif

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

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
static MLVDS_Config_t lcd_mlvds_config = {
    .mlvds_insert_start = 0x45,
    .total_line_clk = 1448,
    .test_dual_gate = 1,
    .test_pair_num = 6,
    .scan_function = 1,
    .phase_select = 1,
    .TL080_phase =3,
};
#endif

static Lcd_Config_t lcd_config = {
    .lcd_timing = {
        .lcd_clk = 40000000,
        .clk_ctrl = (1<<CLK_CTRL_AUTO) | (1<<CLK_CTRL_VCLK_SEL) | (7<<CLK_CTRL_XD),
        .video_on_pixel = 120,
        .video_on_line = 32,
        .hvsync_valid = 1,
        .de_valid = 1,
        .pol_cntl_addr = (0 << LCD_CPH1_POL) |(0 << LCD_HS_POL) | (0 << LCD_VS_POL),
        .inv_cnt_addr = (0 << LCD_INV_EN) | (0 << LCD_INV_CNT),
        .tcon_misc_sel_addr = (1 << LCD_STV1_SEL) | (1 << LCD_STV2_SEL),
    },
    .lcd_effect = {
        .gamma_cntl_port = (1 << LCD_GAMMA_EN),
        .rgb_base_addr = 0xf0,
        .rgb_coeff_addr = 0x74a,
        .dith_user = 0,
        .vadj_brightness = 0x0,
        .vadj_contrast = 0x80,
        .vadj_saturation = 0x100,
        .gamma_revert = 0,
        .gamma_r_coeff = 100,
        .gamma_g_coeff = 100,
        .gamma_b_coeff = 100,
    },
    .lcd_control = {
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
        .mipi_config = &lcd_mipi_config,
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
        .edp_config = &lcd_edp_config,
#endif
        .lvds_config = &lcd_lvds_config,
        .ttl_config = &lcd_ttl_config,
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
        .mlvds_config = &lcd_mlvds_config,
#endif
    },
    .lcd_power_ctrl = {
        .power_on_step = 0,
        .power_off_step = 0,
    }
};

static void lcd_setup_gamma_table(Lcd_Config_t *pConf, unsigned int rgb_flag)
{
	int i;
	
	const unsigned short gamma_adjust[256] = {
		0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,
		32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,
		64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,
		96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,
		128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,
		160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,
		192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,
		224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255
	};

	if (rgb_flag == 0) {	//r
		for (i=0; i<256; i++) {
			pConf->lcd_effect.GammaTableR[i] = gamma_adjust[i] << 2;
		}
	}
	else if (rgb_flag == 1) {	//g
		for (i=0; i<256; i++) {
			pConf->lcd_effect.GammaTableG[i] = gamma_adjust[i] << 2;
		}
	}
	else if (rgb_flag == 2) {	//b
		for (i=0; i<256; i++) {
			pConf->lcd_effect.GammaTableB[i] = gamma_adjust[i] << 2;
		}
	}
	else if (rgb_flag == 3) {	//rgb
		for (i=0; i<256; i++) {
			pConf->lcd_effect.GammaTableR[i] = gamma_adjust[i] << 2;
			pConf->lcd_effect.GammaTableG[i] = gamma_adjust[i] << 2;
			pConf->lcd_effect.GammaTableB[i] = gamma_adjust[i] << 2;
		}
	}
}

static void lcd_ports_ctrl_lvds(Bool_t status)
{
	if (status) {
		WRITE_LCD_REG_BITS(LVDS_GEN_CNTL, 1, 3, 1);	//enable lvds fifo
		if (pDev->pConf->lcd_basic.lcd_bits == 6) {
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
			WRITE_LCD_REG_BITS(LVDS_PHY_CNTL4, 0x27, 0, 7);	//enable LVDS 3 channels
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
			WRITE_LCD_CBUS_REG_BITS(HHI_DIF_CSI_PHY_CNTL3, 0x1e, 11, 5);	//enable LVDS phy 3 channels
#endif
		}
		else {
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
			WRITE_LCD_REG_BITS(LVDS_PHY_CNTL4, 0x2f, 0, 7);	//enable LVDS 4 channels
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
			WRITE_LCD_CBUS_REG_BITS(HHI_DIF_CSI_PHY_CNTL3, 0x1f, 11, 5);	//enable LVDS phy 4 channels
#endif
		}
	}
	else {
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
		WRITE_LCD_REG_BITS(LVDS_PHY_CNTL3, 0, 0, 1);
		WRITE_LCD_REG_BITS(LVDS_PHY_CNTL5, 0, 11, 1);	//shutdown lvds phy
		WRITE_LCD_REG_BITS(LVDS_PHY_CNTL4, 0, 0, 7);	//disable LVDS 4 channels
		WRITE_LCD_REG_BITS(LVDS_GEN_CNTL, 0, 3, 1);	//disable lvds fifo
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
		WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL1, 0x0);
		WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL2, 0x00060000);
		WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL3, 0x00200000);
#endif
	}

	DBG_PRINT("%s: %s\n", __FUNCTION__, (status ? "ON" : "OFF"));
}

#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
static void lcd_ports_ctrl_mipi(Bool_t status)
{
    if (status) {
        switch (pDev->pConf->lcd_control.mipi_config->lane_num) {
            case 1:
                WRITE_LCD_CBUS_REG_BITS(HHI_DIF_CSI_PHY_CNTL3, 0x11, 11, 5);
                break;
            case 2:
                WRITE_LCD_CBUS_REG_BITS(HHI_DIF_CSI_PHY_CNTL3, 0x19, 11, 5);
                break;
            case 3:
                WRITE_LCD_CBUS_REG_BITS(HHI_DIF_CSI_PHY_CNTL3, 0x1d, 11, 5);
                break;
            case 4:
                WRITE_LCD_CBUS_REG_BITS(HHI_DIF_CSI_PHY_CNTL3, 0x1f, 11, 5);
                break;
            default:
                break;
        }
    }
    else {
        WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL1, 0x0);
        WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL2, 0x00060000);
        WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL3, 0x00200000);
    }

    DBG_PRINT("%s: %s\n", __FUNCTION__, (status ? "ON" : "OFF"));
}
#endif

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
static void lcd_ports_ctrl_edp(Bool_t status)
{
	if (status) {
		switch (pDev->pConf->lcd_control.edp_config->lane_count) {
			case 1:
				WRITE_LCD_CBUS_REG_BITS(HHI_DIF_CSI_PHY_CNTL3, 0x18, 11, 5);
				break;
			case 2:
				WRITE_LCD_CBUS_REG_BITS(HHI_DIF_CSI_PHY_CNTL3, 0x1c, 11, 5);
				break;
			case 4:
				WRITE_LCD_CBUS_REG_BITS(HHI_DIF_CSI_PHY_CNTL3, 0x1f, 11, 5);
				break;
			default:
				break;
		}
	}
	else {
		WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL1, 0x0);
		WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL2, 0x00060000);
		WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL3, 0x00200000);
	}
	DBG_PRINT("%s: %s\n", __FUNCTION__, (status ? "ON" : "OFF"));
}
#endif

static void lcd_ports_ctrl_ttl(Bool_t status)
{
	struct pinctrl_state *s;
	int ret;
	
	if (IS_ERR(pDev->p)) {
		printk("set ttl_ports_ctrl pinmux error.\n");
		return;
	}

	if (status) {
	if (pDev->pConf->lcd_basic.lcd_bits == 6) {
			if (pDev->pConf->lcd_timing.de_valid == 0) {
				s = pinctrl_lookup_state(pDev->p, "ttl_6bit_hvsync_on");
			}
			else if (pDev->pConf->lcd_timing.hvsync_valid == 0) {
				s = pinctrl_lookup_state(pDev->p, "ttl_6bit_de_on");
			}
			else {
				s = pinctrl_lookup_state(pDev->p, "ttl_6bit_hvsync_de_on");	//select pinmux
			}
		}
		else {
			if (pDev->pConf->lcd_timing.de_valid == 0) {
				s = pinctrl_lookup_state(pDev->p, "ttl_8bit_hvsync_on");
			}	
			else if (pDev->pConf->lcd_timing.hvsync_valid == 0) {
				s = pinctrl_lookup_state(pDev->p, "ttl_8bit_de_on");
			}
			else {
				s = pinctrl_lookup_state(pDev->p, "ttl_8bit_hvsync_de_on");	//select pinmux
			}
		}
		if (IS_ERR(pDev->p)) {
			printk("set ttl_ports_ctrl pinmux error.\n");
			devm_pinctrl_put(pDev->p);
			return;
		}

		ret = pinctrl_select_state(pDev->p, s);	//set pinmux and lock pins
		if (ret < 0) {
			printk("set ttl_ports_ctrl pinmux error.\n");
			devm_pinctrl_put(pDev->p);
			return;
		}
	}else {
		//pinctrl_put(pDev->p);	//release pins
		if (pDev->pConf->lcd_basic.lcd_bits == 6) {
			s = pinctrl_lookup_state(pDev->p, "ttl_6bit_hvsync_de_off");	//select pinmux
		}
		else {
			s = pinctrl_lookup_state(pDev->p, "ttl_8bit_hvsync_de_off");	//select pinmux
		}
		if (IS_ERR(pDev->p)) {
			printk("set ttl_ports_ctrl pinmux error.\n");
			devm_pinctrl_put(pDev->p);
			return;
		}
		
		ret = pinctrl_select_state(pDev->p, s);	//set pinmux and lock pins
		if (ret < 0) {
			printk("set ttl_ports_ctrl pinmux error.\n");
			devm_pinctrl_put(pDev->p);
			return;
		}
	}
	DBG_PRINT("%s: %s\n", __FUNCTION__, (status ? "ON" : "OFF"));
}

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
static void lcd_ports_ctrl_mlvds(Bool_t status)
{
	return;
}
#endif

static void lcd_ports_ctrl(Bool_t status)
{
    switch(pDev->pConf->lcd_basic.lcd_type) {
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
        case LCD_DIGITAL_MIPI:
            lcd_ports_ctrl_mipi(status);
            break;
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
        case LCD_DIGITAL_EDP:
            lcd_ports_ctrl_edp(status);
            break;
#endif
        case LCD_DIGITAL_LVDS:
            lcd_ports_ctrl_lvds(status);
            break;
        case LCD_DIGITAL_TTL:
            lcd_ports_ctrl_ttl(status);
            break;
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
        case LCD_DIGITAL_MINILVDS:
            lcd_ports_ctrl_mlvds(status);
            break;
#endif
        default:
            printk("Invalid LCD type.\n");
            break;
    }
}

static void backlight_power_ctrl(Bool_t status)
{
	if( status == ON ){
		if ((data_status == OFF) || (bl_status == ON))
			return;
		bl_power_on(LCD_BL_FLAG);
	}
	else{
		if (bl_status == OFF)
			return;
		bl_power_off(LCD_BL_FLAG);
	}
	DBG_PRINT("%s(%s): bl_status=%s, data_status=%s\n", __FUNCTION__, (status ? "ON" : "OFF"), (bl_status ? "ON" : "OFF"), (data_status ? "ON" : "OFF"));
	bl_status = status;
}

#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
static void set_control_mipi(Lcd_Config_t *pConf);
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
static int set_control_edp(Lcd_Config_t *pConf);
#endif
//for special interface
static int lcd_power_ctrl_video(Bool_t status)
{
    int ret = 0;

    if (status) {
        switch(pDev->pConf->lcd_basic.lcd_type) {
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
            case LCD_DIGITAL_MIPI:
                set_control_mipi(pDev->pConf);
                break;
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
            case LCD_DIGITAL_EDP:
                ret = set_control_edp(pDev->pConf);
                break;
#endif
            default:
                break;
        }
    }
    else {
        switch(pDev->pConf->lcd_basic.lcd_type) {
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
            case LCD_DIGITAL_MIPI:
                mipi_dsi_link_off(pDev->pConf);  //link off command
                break;
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
            case LCD_DIGITAL_EDP:
                ret = dplpm_link_off();  //link off command
                break;
#endif
            default:
                break;
        }
    }
    DBG_PRINT("%s: %s\n", __FUNCTION__, (status ? "ON" : "OFF"));
    return ret;
}

static int lcd_power_ctrl(Bool_t status)
{
	int i;
	int ret = 0;
#ifdef CONFIG_AMLOGIC_BOARD_HAS_PMU
	struct aml_pmu_driver *pmu_driver;
#endif
	struct aml_lcd_extern_driver_t *lcd_extern_driver;

	DBG_PRINT("%s(): %s\n", __FUNCTION__, (status ? "ON" : "OFF"));
	if (status) {
		for (i=0; i<pDev->pConf->lcd_power_ctrl.power_on_step; i++) {
			DBG_PRINT("%s %s step %d\n", __FUNCTION__, (status ? "ON" : "OFF"), i+1);
			switch (pDev->pConf->lcd_power_ctrl.power_on_config[i].type) {
				case LCD_POWER_TYPE_CPU:
					if (pDev->pConf->lcd_power_ctrl.power_on_config[i].value == LCD_POWER_GPIO_OUTPUT_LOW) {
						lcd_gpio_direction_output(pDev->pConf->lcd_power_ctrl.power_on_config[i].gpio, 0);
					}
					else if (pDev->pConf->lcd_power_ctrl.power_on_config[i].value == LCD_POWER_GPIO_OUTPUT_HIGH) {
						lcd_gpio_direction_output(pDev->pConf->lcd_power_ctrl.power_on_config[i].gpio, 1);
					}
					else if (pDev->pConf->lcd_power_ctrl.power_on_config[i].value == LCD_POWER_GPIO_INPUT) {
						lcd_gpio_direction_input(pDev->pConf->lcd_power_ctrl.power_on_config[i].gpio);
					}
					break;
				case LCD_POWER_TYPE_PMU:
#ifdef CONFIG_AMLOGIC_BOARD_HAS_PMU
					pmu_driver = aml_pmu_get_driver();
					if (pmu_driver == NULL) {
						printk("no pmu driver\n");
					}
					else if (pmu_driver->pmu_set_gpio) {
						if (pDev->pConf->lcd_power_ctrl.power_on_config[i].value == LCD_POWER_GPIO_OUTPUT_LOW) {
							pmu_driver->pmu_set_gpio(pDev->pConf->lcd_power_ctrl.power_on_config[i].gpio, 0);
						}
						else {
							pmu_driver->pmu_set_gpio(pDev->pConf->lcd_power_ctrl.power_on_config[i].gpio, 1);
						}
					}
#endif
					break;
				case LCD_POWER_TYPE_SIGNAL:
					lcd_ports_ctrl(ON);
					break;
				case LCD_POWER_TYPE_INITIAL:
					lcd_extern_driver = aml_lcd_extern_get_driver();
					if (lcd_extern_driver == NULL) {
						printk("no lcd_extern driver\n");
					}
					else {
						if (lcd_extern_driver->power_on) {
							lcd_extern_driver->power_on();
							printk("%s power on init\n", lcd_extern_driver->name);
						}
					}
					break;
				default:
					printk("lcd power ctrl ON step %d is null.\n", i+1);
					break;
			}
			if (pDev->pConf->lcd_power_ctrl.power_on_config[i].delay > 0)
				lcd_mdelay(pDev->pConf->lcd_power_ctrl.power_on_config[i].delay);
		}
		ret = lcd_power_ctrl_video(ON);
	}
	else {
		lcd_mdelay(30);
		ret = lcd_power_ctrl_video(OFF);
		for (i=0; i<pDev->pConf->lcd_power_ctrl.power_off_step; i++) {
			DBG_PRINT("%s %s step %d\n", __FUNCTION__, (status ? "ON" : "OFF"), i+1);
			switch (pDev->pConf->lcd_power_ctrl.power_off_config[i].type) {
				case LCD_POWER_TYPE_CPU:
					if (pDev->pConf->lcd_power_ctrl.power_off_config[i].value == LCD_POWER_GPIO_OUTPUT_LOW) {
						lcd_gpio_direction_output(pDev->pConf->lcd_power_ctrl.power_off_config[i].gpio, 0);
					}
					else if (pDev->pConf->lcd_power_ctrl.power_off_config[i].value == LCD_POWER_GPIO_OUTPUT_HIGH) {
						lcd_gpio_direction_output(pDev->pConf->lcd_power_ctrl.power_off_config[i].gpio, 1);
					}
					else if (pDev->pConf->lcd_power_ctrl.power_off_config[i].value == LCD_POWER_GPIO_INPUT) {
						lcd_gpio_direction_input(pDev->pConf->lcd_power_ctrl.power_off_config[i].gpio);
					}
					break;
				case LCD_POWER_TYPE_PMU:
#ifdef CONFIG_AMLOGIC_BOARD_HAS_PMU
					pmu_driver = aml_pmu_get_driver();
					if (pmu_driver == NULL) {
						printk("no pmu driver\n");
					}
					else if (pmu_driver->pmu_set_gpio) {
						if (pDev->pConf->lcd_power_ctrl.power_off_config[i].value == LCD_POWER_GPIO_OUTPUT_LOW) {
							pmu_driver->pmu_set_gpio(pDev->pConf->lcd_power_ctrl.power_off_config[i].gpio, 0);
						}
						else {
							pmu_driver->pmu_set_gpio(pDev->pConf->lcd_power_ctrl.power_off_config[i].gpio, 1);
						}
					}
#endif
					break;
				case LCD_POWER_TYPE_SIGNAL:
					lcd_ports_ctrl(OFF);
					break;
				case LCD_POWER_TYPE_INITIAL:
					lcd_extern_driver = aml_lcd_extern_get_driver();
					if (lcd_extern_driver == NULL) {
						printk("no lcd_extern driver\n");
					}
					else {
						if (lcd_extern_driver->power_off) {
							lcd_extern_driver->power_off();
							printk("%s power off init\n", lcd_extern_driver->name);
						}
					}
					break;
				default:
					printk("lcd power ctrl OFF step %d is null.\n", i+1);
					break;
			}
			if (pDev->pConf->lcd_power_ctrl.power_off_config[i].delay > 0)
				lcd_mdelay(pDev->pConf->lcd_power_ctrl.power_off_config[i].delay);
		}
	}

	printk("%s(): %s finished.\n", __FUNCTION__, (status ? "ON" : "OFF"));
	return ret;
}
#endif

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
static void set_gamma_table_ttl(u16 *data, u32 rgb_mask, u16 gamma_coeff)
{
	int i;
	unsigned long flags = 0;
	spin_lock_irqsave(&gamma_write_lock, flags);
	
	while (!(READ_LCD_REG(GAMMA_CNTL_PORT) & (0x1 << LCD_ADR_RDY)));
	WRITE_LCD_REG(GAMMA_ADDR_PORT, (0x1 << LCD_H_AUTO_INC) | (0x1 << rgb_mask) | (0x0 << LCD_HADR));
	for (i=0;i<256;i++) {
		while (!( READ_LCD_REG(GAMMA_CNTL_PORT) & (0x1 << LCD_WR_RDY)));
		WRITE_LCD_REG(GAMMA_DATA_PORT, (data[i] * gamma_coeff / 100));
	}
	while (!(READ_LCD_REG(GAMMA_CNTL_PORT) & (0x1 << LCD_ADR_RDY)));
	WRITE_LCD_REG(GAMMA_ADDR_PORT, (0x1 << LCD_H_AUTO_INC) | (0x1 << rgb_mask) | (0x23 << LCD_HADR));
	spin_unlock_irqrestore(&gamma_write_lock, flags);
}
#endif

static void set_gamma_table_lcd(u16 *data, u32 rgb_mask, u16 gamma_coeff)
{
	int i;
	unsigned long flags = 0;
	spin_lock_irqsave(&gamma_write_lock, flags);
	
	DBG_PRINT("%s\n", __FUNCTION__);
	while (!(READ_LCD_REG(L_GAMMA_CNTL_PORT) & (0x1 << LCD_ADR_RDY)));
	WRITE_LCD_REG(L_GAMMA_ADDR_PORT, (0x1 << LCD_H_AUTO_INC) | (0x1 << rgb_mask) | (0x0 << LCD_HADR));
	for (i=0;i<256;i++) {
		while (!( READ_LCD_REG(L_GAMMA_CNTL_PORT) & (0x1 << LCD_WR_RDY)));
		WRITE_LCD_REG(L_GAMMA_DATA_PORT, (data[i] * gamma_coeff / 100));
	}
	while (!(READ_LCD_REG(L_GAMMA_CNTL_PORT) & (0x1 << LCD_ADR_RDY)));
	WRITE_LCD_REG(L_GAMMA_ADDR_PORT, (0x1 << LCD_H_AUTO_INC) | (0x1 << rgb_mask) | (0x23 << LCD_HADR));
	spin_unlock_irqrestore(&gamma_write_lock, flags);
}

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
static void write_tcon_double(MLVDS_Tcon_Config_t *mlvds_tcon)
{
    int channel_num = mlvds_tcon->channel_num;
    int hv_sel = (mlvds_tcon->hv_sel) & 1;
    int hstart_1 = mlvds_tcon->tcon_1st_hs_addr;
    int hend_1 = mlvds_tcon->tcon_1st_he_addr;
    int vstart_1 = mlvds_tcon->tcon_1st_vs_addr;
    int vend_1 = mlvds_tcon->tcon_1st_ve_addr;
    int hstart_2 = mlvds_tcon->tcon_2nd_hs_addr;
    int hend_2 = mlvds_tcon->tcon_2nd_he_addr;
    int vstart_2 = mlvds_tcon->tcon_2nd_vs_addr;
    int vend_2 = mlvds_tcon->tcon_2nd_ve_addr;

    switch(channel_num) {
        case 0 :
            WRITE_LCD_REG(MTCON0_1ST_HS_ADDR, hstart_1);
            WRITE_LCD_REG(MTCON0_1ST_HE_ADDR, hend_1);
            WRITE_LCD_REG(MTCON0_1ST_VS_ADDR, vstart_1);
            WRITE_LCD_REG(MTCON0_1ST_VE_ADDR, vend_1);
            WRITE_LCD_REG(MTCON0_2ND_HS_ADDR, hstart_2);
            WRITE_LCD_REG(MTCON0_2ND_HE_ADDR, hend_2);
            WRITE_LCD_REG(MTCON0_2ND_VS_ADDR, vstart_2);
            WRITE_LCD_REG(MTCON0_2ND_VE_ADDR, vend_2);
            WRITE_LCD_REG_BITS(L_TCON_MISC_SEL_ADDR, hv_sel, LCD_STH1_SEL, 1);
            break;
        case 1 :
            WRITE_LCD_REG(MTCON1_1ST_HS_ADDR, hstart_1);
            WRITE_LCD_REG(MTCON1_1ST_HE_ADDR, hend_1);
            WRITE_LCD_REG(MTCON1_1ST_VS_ADDR, vstart_1);
            WRITE_LCD_REG(MTCON1_1ST_VE_ADDR, vend_1);
            WRITE_LCD_REG(MTCON1_2ND_HS_ADDR, hstart_2);
            WRITE_LCD_REG(MTCON1_2ND_HE_ADDR, hend_2);
            WRITE_LCD_REG(MTCON1_2ND_VS_ADDR, vstart_2);
            WRITE_LCD_REG(MTCON1_2ND_VE_ADDR, vend_2);
            WRITE_LCD_REG_BITS(L_TCON_MISC_SEL_ADDR, hv_sel, LCD_CPV1_SEL, 1);
            break;
        case 2 :
            WRITE_LCD_REG(MTCON2_1ST_HS_ADDR, hstart_1);
            WRITE_LCD_REG(MTCON2_1ST_HE_ADDR, hend_1);
            WRITE_LCD_REG(MTCON2_1ST_VS_ADDR, vstart_1);
            WRITE_LCD_REG(MTCON2_1ST_VE_ADDR, vend_1);
            WRITE_LCD_REG(MTCON2_2ND_HS_ADDR, hstart_2);
            WRITE_LCD_REG(MTCON2_2ND_HE_ADDR, hend_2);
            WRITE_LCD_REG(MTCON2_2ND_VS_ADDR, vstart_2);
            WRITE_LCD_REG(MTCON2_2ND_VE_ADDR, vend_2);
            WRITE_LCD_REG_BITS(L_TCON_MISC_SEL_ADDR, hv_sel, LCD_STV1_SEL, 1);
            break;
        case 3 :
            WRITE_LCD_REG(MTCON3_1ST_HS_ADDR, hstart_1);
            WRITE_LCD_REG(MTCON3_1ST_HE_ADDR, hend_1);
            WRITE_LCD_REG(MTCON3_1ST_VS_ADDR, vstart_1);
            WRITE_LCD_REG(MTCON3_1ST_VE_ADDR, vend_1);
            WRITE_LCD_REG(MTCON3_2ND_HS_ADDR, hstart_2);
            WRITE_LCD_REG(MTCON3_2ND_HE_ADDR, hend_2);
            WRITE_LCD_REG(MTCON3_2ND_VS_ADDR, vstart_2);
            WRITE_LCD_REG(MTCON3_2ND_VE_ADDR, vend_2);
            WRITE_LCD_REG_BITS(L_TCON_MISC_SEL_ADDR, hv_sel, LCD_OEV1_SEL, 1);
            break;
        case 4 :
            WRITE_LCD_REG(MTCON4_1ST_HS_ADDR, hstart_1);
            WRITE_LCD_REG(MTCON4_1ST_HE_ADDR, hend_1);
            WRITE_LCD_REG(MTCON4_1ST_VS_ADDR, vstart_1);
            WRITE_LCD_REG(MTCON4_1ST_VE_ADDR, vend_1);
            WRITE_LCD_REG_BITS(L_TCON_MISC_SEL_ADDR, hv_sel, LCD_STH2_SEL, 1);
            break;
        case 5 :
            WRITE_LCD_REG(MTCON5_1ST_HS_ADDR, hstart_1);
            WRITE_LCD_REG(MTCON5_1ST_HE_ADDR, hend_1);
            WRITE_LCD_REG(MTCON5_1ST_VS_ADDR, vstart_1);
            WRITE_LCD_REG(MTCON5_1ST_VE_ADDR, vend_1);
            WRITE_LCD_REG_BITS(L_TCON_MISC_SEL_ADDR, hv_sel, LCD_CPV2_SEL, 1);
            break;
        case 6 :
            WRITE_LCD_REG(MTCON6_1ST_HS_ADDR, hstart_1);
            WRITE_LCD_REG(MTCON6_1ST_HE_ADDR, hend_1);
            WRITE_LCD_REG(MTCON6_1ST_VS_ADDR, vstart_1);
            WRITE_LCD_REG(MTCON6_1ST_VE_ADDR, vend_1);
            WRITE_LCD_REG_BITS(L_TCON_MISC_SEL_ADDR, hv_sel, LCD_OEH_SEL, 1);
            break;
        case 7 :
            WRITE_LCD_REG(MTCON7_1ST_HS_ADDR, hstart_1);
            WRITE_LCD_REG(MTCON7_1ST_HE_ADDR, hend_1);
            WRITE_LCD_REG(MTCON7_1ST_VS_ADDR, vstart_1);
            WRITE_LCD_REG(MTCON7_1ST_VE_ADDR, vend_1);
            WRITE_LCD_REG_BITS(L_TCON_MISC_SEL_ADDR, hv_sel, LCD_OEV3_SEL, 1);
            break;
        default:
            break;
    }
}
#endif

static void set_tcon_lcd(Lcd_Config_t *pConf)
{
	Lcd_Timing_t *tcon_adr = &(pConf->lcd_timing);
	unsigned hs_pol, vs_pol;
	int lcd_type;
	lcd_type = pConf->lcd_basic.lcd_type;
	DBG_PRINT("%s\n", __FUNCTION__);
	
	set_gamma_table_lcd(pConf->lcd_effect.GammaTableR, LCD_H_SEL_R, pConf->lcd_effect.gamma_r_coeff);
	set_gamma_table_lcd(pConf->lcd_effect.GammaTableG, LCD_H_SEL_G, pConf->lcd_effect.gamma_g_coeff);
	set_gamma_table_lcd(pConf->lcd_effect.GammaTableB, LCD_H_SEL_B, pConf->lcd_effect.gamma_b_coeff);

	WRITE_LCD_REG_BITS(L_GAMMA_CNTL_PORT, ((pConf->lcd_effect.gamma_cntl_port >> LCD_GAMMA_EN) & 1), 0, 1);
	//WRITE_LCD_REG(L_GAMMA_VCOM_HSWITCH_ADDR, pConf->lcd_effect.gamma_vcom_hswitch_addr);

	WRITE_LCD_REG(L_RGB_BASE_ADDR, pConf->lcd_effect.rgb_base_addr);
	WRITE_LCD_REG(L_RGB_COEFF_ADDR, pConf->lcd_effect.rgb_coeff_addr);
	
	if (pConf->lcd_effect.dith_user) {
		WRITE_LCD_REG(L_DITH_CNTL_ADDR,  pConf->lcd_effect.dith_cntl_addr);
	}
	else {
		if(pConf->lcd_basic.lcd_bits == 8)
			WRITE_LCD_REG(L_DITH_CNTL_ADDR,  0x400);
		else
			WRITE_LCD_REG(L_DITH_CNTL_ADDR,  0x600);
	}
	
	hs_pol = ((pConf->lcd_timing.pol_cntl_addr >> LCD_HS_POL) & 1);	//0 for low active, 1 for high active
	vs_pol = ((pConf->lcd_timing.pol_cntl_addr >> LCD_VS_POL) & 1);	//0 for low active, 1 for high active
	
	WRITE_LCD_REG(L_POL_CNTL_ADDR,   ((1 << LCD_TCON_DE_SEL) | (1 << LCD_TCON_VS_SEL) | (1 << LCD_TCON_HS_SEL))); //enable tcon DE, Hsync, Vsync
	SET_LCD_REG_MASK(L_POL_CNTL_ADDR, ((0 << LCD_DE_POL) | ((vs_pol ? 0 : 1) << LCD_VS_POL) | ((hs_pol ? 0 : 1) << LCD_HS_POL)));	//adjust hvsync pol
	
	//DE signal
	WRITE_LCD_REG(L_DE_HS_ADDR,		tcon_adr->oeh_hs_addr);
	WRITE_LCD_REG(L_DE_HE_ADDR,		tcon_adr->oeh_he_addr);
	WRITE_LCD_REG(L_DE_VS_ADDR,		tcon_adr->oeh_vs_addr);
	WRITE_LCD_REG(L_DE_VE_ADDR,		tcon_adr->oeh_ve_addr);
	
	//Hsync signal
	WRITE_LCD_REG(L_HSYNC_HS_ADDR,	tcon_adr->sth1_hs_addr);
	WRITE_LCD_REG(L_HSYNC_HE_ADDR,	tcon_adr->sth1_he_addr);
	WRITE_LCD_REG(L_HSYNC_VS_ADDR,	tcon_adr->sth1_vs_addr);
	WRITE_LCD_REG(L_HSYNC_VE_ADDR,	tcon_adr->sth1_ve_addr);
	
	//Vsync signal
	WRITE_LCD_REG(L_VSYNC_HS_ADDR,	tcon_adr->stv1_hs_addr);
	WRITE_LCD_REG(L_VSYNC_HE_ADDR,	tcon_adr->stv1_he_addr);
	WRITE_LCD_REG(L_VSYNC_VS_ADDR,	tcon_adr->stv1_vs_addr);
	WRITE_LCD_REG(L_VSYNC_VE_ADDR,	tcon_adr->stv1_ve_addr);

	if(vpp2_sel)
		CLR_LCD_REG_MASK(VPP2_MISC, (VPP_OUT_SATURATE));
	else
		CLR_LCD_REG_MASK(VPP_MISC, (VPP_OUT_SATURATE));
}

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
static void set_tcon_ttl(Lcd_Config_t *pConf)
{
	Lcd_Timing_t *tcon_adr = &(pConf->lcd_timing);
	unsigned hs_pol, vs_pol;
	
	DBG_PRINT("%s.\n", __FUNCTION__);	

	set_gamma_table_ttl(pConf->lcd_effect.GammaTableR, LCD_H_SEL_R, pConf->lcd_effect.gamma_r_coeff);
	set_gamma_table_ttl(pConf->lcd_effect.GammaTableG, LCD_H_SEL_G, pConf->lcd_effect.gamma_g_coeff);
	set_gamma_table_ttl(pConf->lcd_effect.GammaTableB, LCD_H_SEL_B, pConf->lcd_effect.gamma_b_coeff);

	WRITE_LCD_REG_BITS(GAMMA_CNTL_PORT, ((pConf->lcd_effect.gamma_cntl_port >> LCD_GAMMA_EN) & 1), 0, 1);
	//WRITE_LCD_REG(GAMMA_VCOM_HSWITCH_ADDR, pConf->lcd_effect.gamma_vcom_hswitch_addr);

	WRITE_LCD_REG(RGB_BASE_ADDR,   pConf->lcd_effect.rgb_base_addr);
	WRITE_LCD_REG(RGB_COEFF_ADDR,  pConf->lcd_effect.rgb_coeff_addr);
	WRITE_LCD_REG(POL_CNTL_ADDR,   (pConf->lcd_timing.pol_cntl_addr) & ((1 << LCD_CPH1_POL) | (1 << LCD_CPH2_POL) | (1 << LCD_CPH3_POL)));

	if (pConf->lcd_effect.dith_user) {
		WRITE_LCD_REG(DITH_CNTL_ADDR,  pConf->lcd_effect.dith_cntl_addr);
	}
	else {
		if(pConf->lcd_basic.lcd_bits == 8)
			WRITE_LCD_REG(DITH_CNTL_ADDR,  0x400);
		else
			WRITE_LCD_REG(DITH_CNTL_ADDR,  0x600);
	}
	
	hs_pol = (pConf->lcd_timing.pol_cntl_addr >> LCD_HS_POL) & 1;
	vs_pol = (pConf->lcd_timing.pol_cntl_addr >> LCD_VS_POL) & 1;
	
	if (hs_pol) {
		WRITE_LCD_REG(STH1_HS_ADDR,    tcon_adr->sth1_hs_addr);
		WRITE_LCD_REG(STH1_HE_ADDR,    tcon_adr->sth1_he_addr);
	}
	else {
		WRITE_LCD_REG(STH1_HS_ADDR,    tcon_adr->sth1_he_addr);
		WRITE_LCD_REG(STH1_HE_ADDR,    tcon_adr->sth1_hs_addr);
	}
    WRITE_LCD_REG(STH1_VS_ADDR,    tcon_adr->sth1_vs_addr);
    WRITE_LCD_REG(STH1_VE_ADDR,    tcon_adr->sth1_ve_addr);

    WRITE_LCD_REG(OEH_HS_ADDR,     tcon_adr->oeh_hs_addr);
    WRITE_LCD_REG(OEH_HE_ADDR,     tcon_adr->oeh_he_addr);
	WRITE_LCD_REG(OEH_VS_ADDR,     tcon_adr->oeh_vs_addr);
	WRITE_LCD_REG(OEH_VE_ADDR,     tcon_adr->oeh_ve_addr);

    WRITE_LCD_REG(STV1_HS_ADDR,    tcon_adr->stv1_hs_addr);
    WRITE_LCD_REG(STV1_HE_ADDR,    tcon_adr->stv1_he_addr);
	if (vs_pol) {
		WRITE_LCD_REG(STV1_VS_ADDR,    tcon_adr->stv1_vs_addr);
		WRITE_LCD_REG(STV1_VE_ADDR,    tcon_adr->stv1_ve_addr);
	}
	else {
		WRITE_LCD_REG(STV1_VS_ADDR,    tcon_adr->stv1_ve_addr);
		WRITE_LCD_REG(STV1_VE_ADDR,    tcon_adr->stv1_vs_addr);	
	}
	
    WRITE_LCD_REG(INV_CNT_ADDR,			tcon_adr->inv_cnt_addr);
    WRITE_LCD_REG(TCON_MISC_SEL_ADDR,	tcon_adr->tcon_misc_sel_addr);

	if(vpp2_sel)
		CLR_LCD_REG_MASK(VPP2_MISC, (VPP_OUT_SATURATE));
	else
		CLR_LCD_REG_MASK(VPP_MISC, (VPP_OUT_SATURATE));
}
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
static void set_tcon_ttl(Lcd_Config_t *pConf)
{
	Lcd_Timing_t *tcon_adr = &(pConf->lcd_timing);
	unsigned hs_pol, vs_pol;
	
	DBG_PRINT("%s.\n", __FUNCTION__);	

	set_gamma_table_lcd(pConf->lcd_effect.GammaTableR, LCD_H_SEL_R, pConf->lcd_effect.gamma_r_coeff);
	set_gamma_table_lcd(pConf->lcd_effect.GammaTableG, LCD_H_SEL_G, pConf->lcd_effect.gamma_g_coeff);
	set_gamma_table_lcd(pConf->lcd_effect.GammaTableB, LCD_H_SEL_B, pConf->lcd_effect.gamma_b_coeff);

	WRITE_LCD_REG_BITS(L_GAMMA_CNTL_PORT, ((pConf->lcd_effect.gamma_cntl_port >> LCD_GAMMA_EN) & 1), 0, 1);
	//WRITE_LCD_REG(L_GAMMA_VCOM_HSWITCH_ADDR, pConf->lcd_effect.gamma_vcom_hswitch_addr);

	WRITE_LCD_REG(L_RGB_BASE_ADDR,   pConf->lcd_effect.rgb_base_addr);
	WRITE_LCD_REG(L_RGB_COEFF_ADDR,  pConf->lcd_effect.rgb_coeff_addr);
	WRITE_LCD_REG(L_POL_CNTL_ADDR,   (pConf->lcd_timing.pol_cntl_addr) & ((1 << LCD_CPH1_POL) | (1 << LCD_CPH2_POL) | (1 << LCD_CPH3_POL)));

	if (pConf->lcd_effect.dith_user) {
		WRITE_LCD_REG(L_DITH_CNTL_ADDR,  pConf->lcd_effect.dith_cntl_addr);
	}
	else {
		if(pConf->lcd_basic.lcd_bits == 8)
			WRITE_LCD_REG(L_DITH_CNTL_ADDR,  0x400);
		else
			WRITE_LCD_REG(L_DITH_CNTL_ADDR,  0x600);
	}
	
	hs_pol = (pConf->lcd_timing.pol_cntl_addr >> LCD_HS_POL) & 1;
	vs_pol = (pConf->lcd_timing.pol_cntl_addr >> LCD_VS_POL) & 1;
	
	if (hs_pol) {
		WRITE_LCD_REG(L_STH1_HS_ADDR,    tcon_adr->sth1_hs_addr);
		WRITE_LCD_REG(L_STH1_HE_ADDR,    tcon_adr->sth1_he_addr);
	}
	else {
		WRITE_LCD_REG(L_STH1_HS_ADDR,    tcon_adr->sth1_he_addr);
		WRITE_LCD_REG(L_STH1_HE_ADDR,    tcon_adr->sth1_hs_addr);
	}
    WRITE_LCD_REG(L_STH1_VS_ADDR,    tcon_adr->sth1_vs_addr);
    WRITE_LCD_REG(L_STH1_VE_ADDR,    tcon_adr->sth1_ve_addr);

    WRITE_LCD_REG(L_OEH_HS_ADDR,     tcon_adr->oeh_hs_addr);
    WRITE_LCD_REG(L_OEH_HE_ADDR,     tcon_adr->oeh_he_addr);
    WRITE_LCD_REG(L_OEH_VS_ADDR,     tcon_adr->oeh_vs_addr);
    WRITE_LCD_REG(L_OEH_VE_ADDR,     tcon_adr->oeh_ve_addr);

    WRITE_LCD_REG(L_STV1_HS_ADDR,    tcon_adr->stv1_hs_addr);
    WRITE_LCD_REG(L_STV1_HE_ADDR,    tcon_adr->stv1_he_addr);
	
	if (vs_pol) {
		WRITE_LCD_REG(L_STV1_VS_ADDR,    tcon_adr->stv1_vs_addr);
		WRITE_LCD_REG(L_STV1_VE_ADDR,    tcon_adr->stv1_ve_addr);
	}
	else {
		WRITE_LCD_REG(L_STV1_VS_ADDR,    tcon_adr->stv1_ve_addr);
		WRITE_LCD_REG(L_STV1_VE_ADDR,    tcon_adr->stv1_vs_addr);
	}

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B)
	WRITE_LCD_REG(L_POL_CNTL_ADDR,   ((1 << LCD_TCON_DE_SEL) | (1 << LCD_TCON_VS_SEL) | (1 << LCD_TCON_HS_SEL))); //enable tcon DE, Hsync, Vsync 
	WRITE_LCD_REG(L_POL_CNTL_ADDR,   (READ_LCD_REG(L_POL_CNTL_ADDR) | ((0 << LCD_DE_POL) | ((vs_pol ? 0 : 1) << LCD_VS_POL) | ((hs_pol ? 0 : 1) << LCD_HS_POL))));	//adjust hvsync pol
	
	//DE signal
	WRITE_LCD_REG(L_DE_HS_ADDR,    tcon_adr->oeh_hs_addr);
	WRITE_LCD_REG(L_DE_HE_ADDR,    tcon_adr->oeh_he_addr);
	WRITE_LCD_REG(L_DE_VS_ADDR,    tcon_adr->oeh_vs_addr);
	WRITE_LCD_REG(L_DE_VE_ADDR,    tcon_adr->oeh_ve_addr);
	
	WRITE_LCD_REG(L_OEV1_HS_ADDR,  tcon_adr->oeh_hs_addr);
	WRITE_LCD_REG(L_OEV1_HE_ADDR,  tcon_adr->oeh_he_addr);
	WRITE_LCD_REG(L_OEV1_VS_ADDR,  tcon_adr->oeh_vs_addr);
	WRITE_LCD_REG(L_OEV1_VE_ADDR,  tcon_adr->oeh_ve_addr);
	
	//Hsync signal
	WRITE_LCD_REG(L_HSYNC_HS_ADDR, tcon_adr->sth1_hs_addr);
	WRITE_LCD_REG(L_HSYNC_HE_ADDR, tcon_adr->sth1_he_addr);
	WRITE_LCD_REG(L_HSYNC_VS_ADDR, tcon_adr->sth1_vs_addr);
	WRITE_LCD_REG(L_HSYNC_VE_ADDR, tcon_adr->sth1_ve_addr);
	
	//Vsync signal
	WRITE_LCD_REG(L_VSYNC_HS_ADDR, tcon_adr->stv1_hs_addr);
	WRITE_LCD_REG(L_VSYNC_HE_ADDR, tcon_adr->stv1_he_addr);
	WRITE_LCD_REG(L_VSYNC_VS_ADDR, tcon_adr->stv1_vs_addr);
	WRITE_LCD_REG(L_VSYNC_VE_ADDR, tcon_adr->stv1_ve_addr);
#endif

    WRITE_LCD_REG(L_INV_CNT_ADDR,       tcon_adr->inv_cnt_addr);
    WRITE_LCD_REG(L_TCON_MISC_SEL_ADDR, tcon_adr->tcon_misc_sel_addr);

	if(vpp2_sel)
		CLR_LCD_REG_MASK(VPP2_MISC, (VPP_OUT_SATURATE));
	else
		CLR_LCD_REG_MASK(VPP_MISC, (VPP_OUT_SATURATE));
}
#endif

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
// Set the mlvds TCON
// this function should support dual gate or singal gate TCON setting.
// singal gate TCON, Scan Function TO DO.
// scan_function   // 0 - Z1, 1 - Z2, 2- Gong
static void set_tcon_mlvds(Lcd_Config_t *pConf)
{
	MLVDS_Tcon_Config_t *mlvds_tconfig_l = pConf->lcd_control.mlvds_tcon_config;
    int dual_gate = pConf->lcd_control.mlvds_config->test_dual_gate;
    int bit_num = pConf->lcd_basic.lcd_bits;
    int pair_num = pConf->lcd_control.mlvds_config->test_pair_num;

    unsigned int data32;

    int pclk_div;
    int ext_pixel = dual_gate ? pConf->lcd_control.mlvds_config->total_line_clk : 0;
    int dual_wr_rd_start;
    int i = 0;
	
	DBG_PRINT("%s.\n", __FUNCTION__);

//    DBG_PRINT(" Notice: Setting VENC_DVI_SETTING[0x%4x] and GAMMA_CNTL_PORT[0x%4x].LCD_GAMMA_EN as 0 temporary\n", VENC_DVI_SETTING, GAMMA_CNTL_PORT);
//    DBG_PRINT(" Otherwise, the panel will display color abnormal.\n");
//    WRITE_LCD_REG(VENC_DVI_SETTING, 0);

    set_gamma_table_lcd(pConf->lcd_effect.GammaTableR, LCD_H_SEL_R, pConf->lcd_effect.gamma_r_coeff);
	set_gamma_table_lcd(pConf->lcd_effect.GammaTableG, LCD_H_SEL_G, pConf->lcd_effect.gamma_g_coeff);
	set_gamma_table_lcd(pConf->lcd_effect.GammaTableB, LCD_H_SEL_B, pConf->lcd_effect.gamma_b_coeff);

    WRITE_LCD_REG(L_GAMMA_CNTL_PORT, pConf->lcd_effect.gamma_cntl_port);
    WRITE_LCD_REG(L_GAMMA_VCOM_HSWITCH_ADDR, pConf->lcd_effect.gamma_vcom_hswitch_addr);

    WRITE_LCD_REG(L_RGB_BASE_ADDR, pConf->lcd_effect.rgb_base_addr);
    WRITE_LCD_REG(L_RGB_COEFF_ADDR, pConf->lcd_effect.rgb_coeff_addr);
    //WRITE_LCD_REG(L_POL_CNTL_ADDR, pConf->pol_cntl_addr);
	if (pConf->lcd_effect.dith_user) {
		WRITE_LCD_REG(L_DITH_CNTL_ADDR,  pConf->lcd_effect.dith_cntl_addr);
	}	
	else {
		if(pConf->lcd_basic.lcd_bits == 8)
			WRITE_LCD_REG(L_DITH_CNTL_ADDR,  0x400);
		else
			WRITE_LCD_REG(L_DITH_CNTL_ADDR,  0x600);
	}
	
//    WRITE_LCD_REG(L_INV_CNT_ADDR, pConf->inv_cnt_addr);
//    WRITE_LCD_REG(L_TCON_MISC_SEL_ADDR, pConf->tcon_misc_sel_addr);
//    WRITE_LCD_REG(L_DUAL_PORT_CNTL_ADDR, pConf->dual_port_cntl_addr);
//
    data32 = (0x9867 << tcon_pattern_loop_data) |
             (1 << tcon_pattern_loop_start) |
             (4 << tcon_pattern_loop_end) |
             (1 << ((mlvds_tconfig_l[6].channel_num)+tcon_pattern_enable)); // POL_CHANNEL use pattern generate

    WRITE_LCD_REG(L_TCON_PATTERN_HI,  (data32 >> 16));
    WRITE_LCD_REG(L_TCON_PATTERN_LO, (data32 & 0xffff));

    pclk_div = (bit_num == 8) ? 3 : // phy_clk / 8
                                2 ; // phy_clk / 6
   data32 = (1 << ((mlvds_tconfig_l[7].channel_num)-2+tcon_pclk_enable)) |  // enable PCLK_CHANNEL
            (pclk_div << tcon_pclk_div) |
            (
              (pair_num == 6) ?
              (
              ((bit_num == 8) & dual_gate) ?
              (
                (0 << (tcon_delay + 0*3)) |
                (0 << (tcon_delay + 1*3)) |
                (0 << (tcon_delay + 2*3)) |
                (0 << (tcon_delay + 3*3)) |
                (0 << (tcon_delay + 4*3)) |
                (0 << (tcon_delay + 5*3)) |
                (0 << (tcon_delay + 6*3)) |
                (0 << (tcon_delay + 7*3))
              ) :
              (
                (0 << (tcon_delay + 0*3)) |
                (0 << (tcon_delay + 1*3)) |
                (0 << (tcon_delay + 2*3)) |
                (0 << (tcon_delay + 3*3)) |
                (0 << (tcon_delay + 4*3)) |
                (0 << (tcon_delay + 5*3)) |
                (0 << (tcon_delay + 6*3)) |
                (0 << (tcon_delay + 7*3))
              )
              ) :
              (
              ((bit_num == 8) & dual_gate) ?
              (
                (0 << (tcon_delay + 0*3)) |
                (0 << (tcon_delay + 1*3)) |
                (0 << (tcon_delay + 2*3)) |
                (0 << (tcon_delay + 3*3)) |
                (0 << (tcon_delay + 4*3)) |
                (0 << (tcon_delay + 5*3)) |
                (0 << (tcon_delay + 6*3)) |
                (0 << (tcon_delay + 7*3))
              ) :
              (bit_num == 8) ?
              (
                (0 << (tcon_delay + 0*3)) |
                (0 << (tcon_delay + 1*3)) |
                (0 << (tcon_delay + 2*3)) |
                (0 << (tcon_delay + 3*3)) |
                (0 << (tcon_delay + 4*3)) |
                (0 << (tcon_delay + 5*3)) |
                (0 << (tcon_delay + 6*3)) |
                (0 << (tcon_delay + 7*3))
              ) :
              (
                (0 << (tcon_delay + 0*3)) |
                (0 << (tcon_delay + 1*3)) |
                (0 << (tcon_delay + 2*3)) |
                (0 << (tcon_delay + 3*3)) |
                (0 << (tcon_delay + 4*3)) |
                (0 << (tcon_delay + 5*3)) |
                (0 << (tcon_delay + 6*3)) |
                (0 << (tcon_delay + 7*3))
              )
              )
            );

    WRITE_LCD_REG(TCON_CONTROL_HI,  (data32 >> 16));
    WRITE_LCD_REG(TCON_CONTROL_LO, (data32 & 0xffff));


    WRITE_LCD_REG(L_TCON_DOUBLE_CTL,
                   (1<<(mlvds_tconfig_l[3].channel_num))   // invert CPV
                  );

	// for channel 4-7, set second setting same as first
    WRITE_LCD_REG(L_DE_HS_ADDR, (0x3 << 14) | ext_pixel);   // 0x3 -- enable double_tcon fir channel7:6
    WRITE_LCD_REG(L_DE_HE_ADDR, (0x3 << 14) | ext_pixel);   // 0x3 -- enable double_tcon fir channel5:4
    WRITE_LCD_REG(L_DE_VS_ADDR, (0x3 << 14) | 0);	// 0x3 -- enable double_tcon fir channel3:2
    WRITE_LCD_REG(L_DE_VE_ADDR, (0x3 << 14) | 0);	// 0x3 -- enable double_tcon fir channel1:0	

    dual_wr_rd_start = 0x5d;
    WRITE_LCD_REG(MLVDS_DUAL_GATE_WR_START, dual_wr_rd_start);
    WRITE_LCD_REG(MLVDS_DUAL_GATE_WR_END, dual_wr_rd_start + 1280);
    WRITE_LCD_REG(MLVDS_DUAL_GATE_RD_START, dual_wr_rd_start + ext_pixel - 2);
    WRITE_LCD_REG(MLVDS_DUAL_GATE_RD_END, dual_wr_rd_start + 1280 + ext_pixel - 2);

    WRITE_LCD_REG(MLVDS_SECOND_RESET_CTL, (pConf->lcd_control.mlvds_config->mlvds_insert_start + ext_pixel));

    data32 = (0 << ((mlvds_tconfig_l[5].channel_num)+mlvds_tcon_field_en)) |  // enable EVEN_F on TCON channel 6
             ( (0x0 << mlvds_scan_mode_odd) | (0x0 << mlvds_scan_mode_even)
             ) | (0 << mlvds_scan_mode_start_line);

	WRITE_LCD_REG(MLVDS_DUAL_GATE_CTL_HI,  (data32 >> 16));
	WRITE_LCD_REG(MLVDS_DUAL_GATE_CTL_LO, (data32 & 0xffff));

	DBG_PRINT("write minilvds tcon 0~7.\n");
	for(i = 0; i < 8; i++) {
		write_tcon_double(&mlvds_tconfig_l[i]);
	}
/*	
	if(vpp2_sel)
		CLR_LCD_REG_MASK(VPP2_MISC, (VPP_OUT_SATURATE));
	else
		CLR_LCD_REG_MASK(VPP_MISC, (VPP_OUT_SATURATE));
*/
}
#endif

static void set_lcd_spread_spectrum(int ss_level)
{
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
	unsigned pll_ctrl2, pll_ctrl3, pll_ctrl4;
	DBG_PRINT("%s.\n", __FUNCTION__);
	
	switch (ss_level) {
		case 1:  //about 0.5%
			pll_ctrl2 = 0x16110696;
			pll_ctrl3 = 0x6d625012;
			pll_ctrl4 = 0x130;
			break;
		case 2:  //about 1%
			pll_ctrl2 = 0x16110696;
			pll_ctrl3 = 0x4d625012;
			pll_ctrl4 = 0x130;
			break;
		case 3:  //about 2%
			pll_ctrl2 = 0x16110696;
			pll_ctrl3 = 0x2d425012;
			pll_ctrl4 = 0x130;
			break;
		case 4:  //about 3%
			pll_ctrl2 = 0x16110696;
			pll_ctrl3 = 0x1d425012;
			pll_ctrl4 = 0x130;
			break;
		case 5:  //about 4%
			pll_ctrl2 = 0x16110696;
			pll_ctrl3 = 0x0d125012;
			pll_ctrl4 = 0x130;
			break;
		case 6:  //about 5%
			pll_ctrl2 = 0x16110696;
			pll_ctrl3 = 0x0e425012;
			pll_ctrl4 = 0x130;
			break;
		case 0:	//disable ss
		default:
			pll_ctrl2 = 0x814d3928;
			pll_ctrl3 = 0x6b425012;
			pll_ctrl4 = 0x110;
			break;
	}

	WRITE_LCD_CBUS_REG(HHI_VIID_PLL_CNTL2, pll_ctrl2);
	WRITE_LCD_CBUS_REG(HHI_VIID_PLL_CNTL3, pll_ctrl3);
	WRITE_LCD_CBUS_REG(HHI_VIID_PLL_CNTL4, pll_ctrl4);
#endif
}

static void vclk_set_lcd(int lcd_type, int vclk_sel, unsigned long pll_reg, unsigned long vid_div_reg, unsigned int clk_ctrl_reg)
{
	unsigned edp_div0_sel = 0, edp_div1_sel = 0, xd = 0;
	unsigned pll_level = 0, pll_frac = 0;
	int wait_loop = PLL_WAIT_LOCK_CNT;
	unsigned pll_lock = 0;
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
	unsigned ss_level=0, pll_ctrl2, pll_ctrl3, pll_ctrl4, od_fb;
#endif
	unsigned long flags = 0;
	spin_lock_irqsave(&lcd_clk_lock, flags);
	
	DBG_PRINT("%s.\n", __FUNCTION__);

	edp_div0_sel = (vid_div_reg >> DIV_CTRL_EDP_DIV0) & 0xf;
	edp_div1_sel = (vid_div_reg >> DIV_CTRL_EDP_DIV1) & 0x7;
	vid_div_reg = ((vid_div_reg & 0x1ffff) | (1 << 16) | (1 << 15) | (0x3 << 0));	//select vid2_pll and enable clk
	xd = (clk_ctrl_reg >> CLK_CTRL_XD) & 0xf;
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
	pll_level = 0;
	pll_frac = 0;
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
	pll_level = (clk_ctrl_reg >> CLK_CTRL_LEVEL) & 0x7;
	pll_frac = (clk_ctrl_reg >> CLK_CTRL_FRAC) & 0xfff;
	ss_level = (clk_ctrl_reg >> CLK_CTRL_SS) & 0xf;
	pll_reg |= (1 << PLL_CTRL_EN);
#endif
	
	if(vclk_sel)
		WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_CNTL, 0, 19, 1);	//disable vclk2_en 
	else
		WRITE_LCD_CBUS_REG_BITS(HHI_VID_CLK_CNTL, 0, 19, 2);	//disable vclk1_en1,en0
	udelay(2);

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
	WRITE_LCD_CBUS_REG(HHI_EDP_TX_PHY_CNTL0, (1 << 16));	//reset edp tx phy
#endif

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_PLL_CNTL, 1, 29, 1);	//reset pll
	WRITE_LCD_CBUS_REG(HHI_VIID_PLL_CNTL, pll_reg|(1<<PLL_CTRL_RST));
	WRITE_LCD_CBUS_REG(HHI_VIID_PLL_CNTL2, 0x814d3928 );
	WRITE_LCD_CBUS_REG(HHI_VIID_PLL_CNTL3, 0x6b425012 );
	WRITE_LCD_CBUS_REG(HHI_VIID_PLL_CNTL4, 0x110 );
	WRITE_LCD_CBUS_REG(HHI_VIID_PLL_CNTL, pll_reg );
	do{
		udelay(50);
		pll_lock = (READ_LCD_CBUS_REG(HHI_VIID_PLL_CNTL) >> PLL_CTRL_LOCK) & 0x1;
		wait_loop--;
	}while((pll_lock == 0) && (wait_loop > 0));
	if (wait_loop == 0)
		printk("[error]: vid2_pll lock failed\n");
#elif (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
	if (pll_frac == 0)
		pll_ctrl2 = 0x0421a000;
	else
		pll_ctrl2 = (0x0431a000 | pll_frac);
	
	pll_ctrl4 = (0xd4000d67 & ~((1<<13) | (0xf<<14) | (0xf<<18)));
	switch (ss_level) {
		case 1:	//0.5%
			pll_ctrl4 |= ((1<<13) | (2<<18) | (1<<14));
			break;
		case 2:	//1%
			pll_ctrl4 |= ((1<<13) | (1<<18) | (1<<14));
			break;
		case 3:	//1.5%
			pll_ctrl4 |= ((1<<13) | (8<<18) | (1<<14));
			break;
		case 4: //2%
			pll_ctrl4 |= ((1<<13) | (0<<18) | (1<<14));
			break;
		case 0:
		default:
			ss_level = 0;
			break;
	}
	
	switch (pll_level) {
		case 1:
			pll_ctrl3 = (ss_level > 0) ? 0xca7e3823 : 0xca45b823;
			od_fb = 0;
			break;
		case 2:
			pll_ctrl2 |= (1<<19);//special adjust
			pll_ctrl3 = (ss_level > 0) ? 0xca7e3823 : 0xca49b823;
			od_fb = 1;
			break;
		case 3:
			pll_ctrl3 = (ss_level > 0) ? 0xca7e3823 : 0xca49b823;
			od_fb = 1;
			break;
		case 4:
			pll_ctrl3 = (ss_level > 0) ? 0xca7e3823 : 0xce49c022;
			od_fb = 1;
			break;
		default:
			pll_ctrl3 = 0xca7e3823;
			od_fb = 0;
			break;
	}
	WRITE_LCD_CBUS_REG_BITS(HHI_VID_PLL_CNTL5, 1, 16, 1);//enable bandgap
	WRITE_LCD_CBUS_REG(HHI_VID2_PLL_CNTL2, pll_ctrl2);
	WRITE_LCD_CBUS_REG(HHI_VID2_PLL_CNTL3, pll_ctrl3);
	WRITE_LCD_CBUS_REG(HHI_VID2_PLL_CNTL4, pll_ctrl4);
	WRITE_LCD_CBUS_REG(HHI_VID2_PLL_CNTL5, (0x00700001 | (od_fb << 8)));	//[8] od_fb
	WRITE_LCD_CBUS_REG(HHI_VID2_PLL_CNTL, pll_reg | (1 << PLL_CTRL_RST));
	WRITE_LCD_CBUS_REG(HHI_VID2_PLL_CNTL, pll_reg);
	
	do{
		udelay(50);
		pll_lock = (READ_LCD_CBUS_REG(HHI_VID2_PLL_CNTL) >> PLL_CTRL_LOCK) & 0x1;
		if (wait_loop == 100) {
			if (pll_level == 2) {
				//change setting if can't lock
				WRITE_LCD_CBUS_REG_BITS(HHI_VID2_PLL_CNTL2, 1, 18, 1);
				WRITE_LCD_CBUS_REG_BITS(HHI_VID2_PLL_CNTL, 1, PLL_CTRL_RST, 1);
				WRITE_LCD_CBUS_REG_BITS(HHI_VID2_PLL_CNTL, 0, PLL_CTRL_RST, 1);
				printk("change setting for vid2 pll stability\n");
			}
		}
		wait_loop--;
	}while((pll_lock == 0) && (wait_loop > 0));
	if (wait_loop == 0)
		printk("[error]: vid2_pll lock failed\n");
#elif (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B)
	if (pll_frac == 0)
		pll_ctrl2 = 0x59c88000;
	else
		pll_ctrl2 = (0x59c8c000 | pll_frac);
	
	pll_ctrl4 = (0x00238100 & ~((1<<9) | (0xf<<4) | (0xf<<0)));
	switch (ss_level) {
		case 1:	//0.5%
			pll_ctrl4 |= ((1<<9) | (2<<4) | (1<<0));
			break;
		case 2:	//1%
			pll_ctrl4 |= ((1<<9) | (1<<4) | (1<<0));
			break;
		case 3:	//1.5%
			pll_ctrl4 |= ((1<<9) | (8<<4) | (1<<0));
			break;
		case 4: //2%
			pll_ctrl4 |= ((1<<9) | (0<<4) | (1<<0));
			break;
		case 0:
		default:
			ss_level = 0;
			break;
	}
	
	switch (pll_level) {
		case 1: //<=1.7G
			pll_ctrl3 = (ss_level > 0) ? 0xca7e3823 : 0xca49b022;
			od_fb = 0;
			break;
		case 2: //1.7G~2.0G
			pll_ctrl2 |= (1<<13);//special adjust
			pll_ctrl3 = (ss_level > 0) ? 0xca7e3823 : 0xca493822;
			od_fb = 1;
			break;
		case 3: //2.0G~2.5G
			pll_ctrl3 = (ss_level > 0) ? 0xca7e3823 : 0xca493822;
			od_fb = 1;
			break;
		case 4: //>=2.5G
			pll_ctrl3 = (ss_level > 0) ? 0xca7e3823 : 0xce49c022;
			od_fb = 1;
			break;
		default:
			pll_ctrl3 = 0xca7e3823;
			od_fb = 0;
			break;
	}
	WRITE_LCD_CBUS_REG_BITS(HHI_VID2_PLL_CNTL2, 1, 16, 1);//enable ext LDO
	WRITE_LCD_CBUS_REG(HHI_VID_PLL_CNTL2, pll_ctrl2);
	WRITE_LCD_CBUS_REG(HHI_VID_PLL_CNTL3, pll_ctrl3);
	WRITE_LCD_CBUS_REG(HHI_VID_PLL_CNTL4, (pll_ctrl4 | (od_fb << 24))); //[24] od_fb
	WRITE_LCD_CBUS_REG(HHI_VID_PLL_CNTL5, 0x00012385);
	WRITE_LCD_CBUS_REG(HHI_VID_PLL_CNTL, pll_reg | (1 << PLL_CTRL_RST));
	WRITE_LCD_CBUS_REG(HHI_VID_PLL_CNTL, pll_reg);
	
	do{
		udelay(50);
		pll_lock = (READ_LCD_CBUS_REG(HHI_VID_PLL_CNTL) >> PLL_CTRL_LOCK) & 0x1;
		if (wait_loop == 100) {
			if (pll_level == 2) {
				//change setting if can't lock
				WRITE_LCD_CBUS_REG_BITS(HHI_VID_PLL_CNTL2, 1, 12, 1);
				WRITE_LCD_CBUS_REG_BITS(HHI_VID_PLL_CNTL, 1, PLL_CTRL_RST, 1);
				WRITE_LCD_CBUS_REG_BITS(HHI_VID_PLL_CNTL, 0, PLL_CTRL_RST, 1);
				printk("change setting for vid pll stability\n");
			}
		}
		wait_loop--;
	}while((pll_lock == 0) && (wait_loop > 0));
	if (wait_loop == 0)
		printk("[error]: vid_pll lock failed\n");
#endif

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
	//select logic & encl clock
	switch (lcd_type) {
		case LCD_DIGITAL_MIPI:
			WRITE_LCD_CBUS_REG_BITS(HHI_VID2_PLL_CNTL5, 3, 23, 3);	//pll_out mux to mipi-dsi phy & vid2_pll
			WRITE_LCD_CBUS_REG_BITS(HHI_DSI_LVDS_EDP_CNTL1, 0, 4, 1);
			break;
		case LCD_DIGITAL_EDP:
			WRITE_LCD_CBUS_REG_BITS(HHI_VID2_PLL_CNTL5, 4, 23, 3);	//pll_out mux to edp phy
			WRITE_LCD_CBUS_REG_BITS(HHI_DSI_LVDS_EDP_CNTL1, 1, 4, 1);
			
			WRITE_LCD_CBUS_REG(HHI_EDP_TX_PHY_CNTL0, (0xf << 0) | (1 << 4));	//enable edp phy channel & serializer clk, and release reset
			WRITE_LCD_CBUS_REG_BITS(HHI_EDP_TX_PHY_CNTL0, edp_div0_sel, 20, 4);	//set edptx_phy_clk_div0
			WRITE_LCD_CBUS_REG_BITS(HHI_EDP_TX_PHY_CNTL0, edp_div1_sel, 24, 3);	//set edptx_phy_clk_div1
			WRITE_LCD_CBUS_REG_BITS(HHI_EDP_TX_PHY_CNTL0, 1, 5, 1);	//enable divider N, for vid_pll2_in
			
			WRITE_LCD_CBUS_REG(HHI_EDP_APB_CLK_CNTL, (1 << 7) | (2 << 0));	//fclk_div5---fixed 510M, div to 170M, edp apb clk
			break;
		case LCD_DIGITAL_LVDS:
		case LCD_DIGITAL_TTL:
		default:
			WRITE_LCD_CBUS_REG_BITS(HHI_VID2_PLL_CNTL5, 2, 23, 3);	//pll_out mux to vid2_pll
			WRITE_LCD_CBUS_REG_BITS(HHI_DSI_LVDS_EDP_CNTL1, 0, 4, 1);
			break;
	}
	udelay(10);
#endif

	//pll_div2
	WRITE_LCD_CBUS_REG(HHI_VIID_DIVIDER_CNTL, vid_div_reg);
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_DIVIDER_CNTL, 1, 7, 1);    //0x104c[7]:SOFT_RESET_POST
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_DIVIDER_CNTL, 1, 3, 1);    //0x104c[3]:SOFT_RESET_PRE
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_DIVIDER_CNTL, 0, 1, 1);    //0x104c[1]:RESET_N_POST
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_DIVIDER_CNTL, 0, 0, 1);    //0x104c[0]:RESET_N_PRE
	udelay(5);
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_DIVIDER_CNTL, 0, 3, 1);
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_DIVIDER_CNTL, 0, 7, 1);
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_DIVIDER_CNTL, 3, 0, 2);
	udelay(5);

	if (vclk_sel) {
		WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_DIV, (xd-1), 0, 8);	// setup the XD divider value
		udelay(5);
		WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_CNTL, 4, 16, 3);	// Bit[18:16] - v2_cntl_clk_in_sel
		WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_CNTL, 1, 19, 1);	//vclk2_en0
	}
	else {
		WRITE_LCD_CBUS_REG_BITS(HHI_VID_CLK_DIV, (xd-1), 0, 8);	// setup the XD divider value
		udelay(5);
		WRITE_LCD_CBUS_REG_BITS(HHI_VID_CLK_CNTL, 4, 16, 3);  // Bit[18:16] - v2_cntl_clk_in_sel
		WRITE_LCD_CBUS_REG_BITS(HHI_VID_CLK_CNTL, 1, 19, 1);	//[19]vclk_en0
		//WRITE_LCD_CBUS_REG_BITS(HHI_VID_CLK_CNTL, 1, 20, 1);	//enable clk_div1 (en1 is for tcon_clko???)
	}
	udelay(2);

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
	if(lcd_type == LCD_DIGITAL_TTL){
		if (vclk_sel)
			WRITE_LCD_CBUS_REG_BITS(HHI_VID_CLK_DIV, 8, 20, 4); // [23:20] enct_clk_sel, select v2_clk_div1
		else
			WRITE_LCD_CBUS_REG_BITS(HHI_VID_CLK_DIV, 0, 20, 4); // [23:20] enct_clk_sel, select v1_clk_div1
	}
	else {
		if (vclk_sel)
			WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_DIV, 8, 12, 4); // [23:20] encl_clk_sel, select v2_clk_div1
		else
			WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_DIV, 0, 12, 4); // [23:20] encl_clk_sel, select v1_clk_div1
	}
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
	if (vclk_sel) {
		WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_DIV, 8, 12, 4); // [15:12] encl_clk_sel, select vclk2_div1
		WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_DIV, 1, 16, 2); // release vclk2_div_reset and enable vclk2_div
	}
	else {
		WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_DIV, 0, 12, 4); // [15:12] encl_clk_sel, select vclk_div1
		WRITE_LCD_CBUS_REG_BITS(HHI_VID_CLK_DIV, 1, 16, 2); // release vclk_div_reset and enable vclk_div	
	}
#endif
	udelay(5);

	if(vclk_sel) {
		WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_CNTL, 1, 0, 1);	//enable v2_clk_div1
		WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_CNTL, 1, 15, 1);  //soft reset
		udelay(10);
		WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_CNTL, 0, 15, 1);  //release soft reset
	}
	else {
		WRITE_LCD_CBUS_REG_BITS(HHI_VID_CLK_CNTL, 1, 0, 1);	//enable v1_clk_div1
		WRITE_LCD_CBUS_REG_BITS(HHI_VID_CLK_CNTL, 1, 15, 1);  //soft reset
		udelay(10);
		WRITE_LCD_CBUS_REG_BITS(HHI_VID_CLK_CNTL, 0, 15, 1);  //release soft reset
	}
	udelay(5);
	
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B)
	WRITE_LCD_CBUS_REG_BITS(HHI_VID_CLK_CNTL2, 1, 3, 1);	//enable encl clk gate //new add for M8b
#endif
	
	spin_unlock_irqrestore(&lcd_clk_lock, flags);
}

static void clk_util_lvds_set_clk_div(unsigned long divn_sel, unsigned long divn_tcnt, unsigned long div2_en)
{
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
    // assign          lvds_div_phy_clk_en     = tst_lvds_tmode ? 1'b1         : phy_clk_cntl[10];
    // assign          lvds_div_div2_sel       = tst_lvds_tmode ? atest_i[5]   : phy_clk_cntl[9];
    // assign          lvds_div_sel            = tst_lvds_tmode ? atest_i[7:6] : phy_clk_cntl[8:7];
    // assign          lvds_div_tcnt           = tst_lvds_tmode ? 3'd6         : phy_clk_cntl[6:4];
    // If dividing by 1, just select the divide by 1 path
	if( divn_tcnt == 1 )
		divn_sel = 0;

	WRITE_LCD_REG_BITS(LVDS_PHY_CLK_CNTL, 1, 10, 1);	
	WRITE_LCD_REG_BITS(LVDS_PHY_CLK_CNTL, divn_sel, 7, 2);
	WRITE_LCD_REG_BITS(LVDS_PHY_CLK_CNTL, div2_en, 9, 1);
	WRITE_LCD_REG_BITS(LVDS_PHY_CLK_CNTL, ((divn_tcnt-1)&0x7), 4, 3);
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
	// ---------------------------------------------
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
    WRITE_LCD_CBUS_REG(HHI_LVDS_TX_PHY_CNTL0, (0x1f << 16) | (0x1 << 6) ); // enable all serializers, divide by 7
#endif
}

static void set_pll_lcd(Lcd_Config_t *pConf)
{
    unsigned pll_reg, div_reg, clk_reg;
    int vclk_sel, xd;
    int lcd_type, ss_level;
    unsigned pll_div_post = 0, phy_clk_div2 = 0;

    DBG_PRINT("%s\n", __FUNCTION__);

    pll_reg = pConf->lcd_timing.pll_ctrl;
    div_reg = pConf->lcd_timing.div_ctrl;
    clk_reg = pConf->lcd_timing.clk_ctrl;
    ss_level = (clk_reg >> CLK_CTRL_SS) & 0xf;
    vclk_sel = (clk_reg >> CLK_CTRL_VCLK_SEL) & 0x1;
    xd = (clk_reg >> CLK_CTRL_XD) & 0xf;

    lcd_type = pConf->lcd_basic.lcd_type;

    switch(lcd_type){
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
        case LCD_DIGITAL_MIPI:
            break;
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
        case LCD_DIGITAL_EDP:
            xd = 1;
            break;
#endif
        case LCD_DIGITAL_LVDS:
            xd = 1;
            pll_div_post = 7;
            phy_clk_div2 = 0;
            div_reg = (div_reg | (1 << DIV_CTRL_POST_SEL) | (1 << DIV_CTRL_LVDS_CLK_EN) | ((pll_div_post-1) << DIV_CTRL_DIV_POST) | (phy_clk_div2 << DIV_CTRL_PHY_CLK_DIV2));
            break;
        case LCD_DIGITAL_TTL:
            break;
        default:
            break;
    }
    clk_reg = (pConf->lcd_timing.clk_ctrl & ~(0xf << CLK_CTRL_XD)) | (xd << CLK_CTRL_XD);

    DBG_PRINT("ss_level=%u(%s), pll_reg=0x%x, div_reg=0x%x, xd=%d.\n", ss_level, lcd_ss_level_table[ss_level], pll_reg, div_reg, xd);
    vclk_set_lcd(lcd_type, vclk_sel, pll_reg, div_reg, clk_reg);
    set_lcd_spread_spectrum(ss_level);

    switch(lcd_type){
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
        case LCD_DIGITAL_MIPI:
            WRITE_LCD_REG(MIPI_DSI_TOP_CNTL, (READ_LCD_REG(MIPI_DSI_TOP_CNTL) & ~(0x7<<4)) | (1 << 4) | (1 << 5) | (0 << 6));
            //WRITE_LCD_CBUS_REG( HHI_DSI_LVDS_EDP_CNTL0, 0x0);                                          // Select DSI as the output for u_dsi_lvds_edp_top
            WRITE_LCD_REG(MIPI_DSI_TOP_SW_RESET, (READ_LCD_REG(MIPI_DSI_TOP_SW_RESET) | 0xf) );     // Release mipi_dsi_host's reset
            WRITE_LCD_REG(MIPI_DSI_TOP_SW_RESET, (READ_LCD_REG(MIPI_DSI_TOP_SW_RESET) & 0xfffffff0) );     // Release mipi_dsi_host's reset
            WRITE_LCD_REG(MIPI_DSI_TOP_CLK_CNTL, (READ_LCD_REG(MIPI_DSI_TOP_CLK_CNTL) | 0x3) );            // Enable dwc mipi_dsi_host's clock 
            break;
#endif
        case LCD_DIGITAL_LVDS:
            clk_util_lvds_set_clk_div(1, pll_div_post, phy_clk_div2);
            //    lvds_gen_cntl       <= {10'h0,      // [15:4] unused
            //                            2'h1,       // [5:4] divide by 7 in the PHY
            //                            1'b0,       // [3] fifo_en
            //                            1'b0,       // [2] wr_bist_gate
            //                            2'b00};     // [1:0] fifo_wr mode
            //FIFO_CLK_SEL = 1; // div7
            WRITE_LCD_REG_BITS(LVDS_GEN_CNTL, 1, 4, 2);	//lvds fifo clk div 7

            WRITE_LCD_REG_BITS(LVDS_PHY_CLK_CNTL, 0, 15, 1);	// lvds div reset
            udelay(5);
            WRITE_LCD_REG_BITS(LVDS_PHY_CLK_CNTL, 1, 15, 1);	// Release lvds div reset
            break;
        case LCD_DIGITAL_TTL:
            break;
        default:
            break;
    }
}

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
static void set_pll_mlvds(Lcd_Config_t *pConf)
{
    int test_bit_num = pConf->lcd_basic.lcd_bits;
    int test_dual_gate = pConf->lcd_control.mlvds_config->test_dual_gate;
    int test_pair_num= pConf->lcd_control.mlvds_config->test_pair_num;
	
    int pll_div_post, phy_clk_div2, FIFO_CLK_SEL, MPCLK_DELAY, MCLK_half, MCLK_half_delay;
    unsigned int data32;
    unsigned long mclk_pattern_dual_6_6;
    int test_high_phase = (test_bit_num != 8) | test_dual_gate;
    unsigned long rd_data;

    unsigned pll_reg, div_reg, clk_reg;
    int vclk_sel, xd;
	int lcd_type, ss_level;
	
	DBG_PRINT("%s\n", __FUNCTION__);
	
    pll_reg = pConf->lcd_timing.pll_ctrl;
    div_reg = pConf->lcd_timing.div_ctrl;
	clk_reg = pConf->lcd_timing.clk_ctrl;
	ss_level = (clk_reg >> CLK_CTRL_SS) & 0xf;
	vclk_sel = (clk_reg >> CLK_CTRL_VCLK_SEL) & 0x1;
	xd = 1;
	
	lcd_type = pConf->lcd_basic.lcd_type;

	switch(pConf->lcd_control.mlvds_config->TL080_phase) {
		case 0 :
			mclk_pattern_dual_6_6 = 0xc3c3c3;
			MCLK_half = 1;
			break;
		case 1 :
			mclk_pattern_dual_6_6 = 0xc3c3c3;
			MCLK_half = 0;
			break;
		case 2 :
			mclk_pattern_dual_6_6 = 0x878787;
			MCLK_half = 1;
			break;
		case 3 :
			mclk_pattern_dual_6_6 = 0x878787;
			MCLK_half = 0;
			break;
		case 4 :
			mclk_pattern_dual_6_6 = 0x3c3c3c;
			MCLK_half = 1;
			break;
		case 5 :
			mclk_pattern_dual_6_6 = 0x3c3c3c;
			MCLK_half = 0;
			break;
		case 6 :
			mclk_pattern_dual_6_6 = 0x787878;
			MCLK_half = 1;
			break;
		default : // case 7
			mclk_pattern_dual_6_6 = 0x787878;
			MCLK_half = 0;
			break;
	}

	pll_div_post = (test_bit_num == 8) ?
					(test_dual_gate ? 4 : 8) :
					(test_dual_gate ? 3 : 6);

    phy_clk_div2 = (test_pair_num != 3);
	
	div_reg = (div_reg | (1 << 8) | (1 << 11) | ((pll_div_post-1) << 12) | (phy_clk_div2 << 10));
	DBG_PRINT("ss_level=%u(%s), pll_reg=0x%x, div_reg=0x%x, xd=%d.\n", ss_level, lcd_ss_level_table[ss_level], pll_reg, div_reg, xd);
    vclk_set_lcd(lcd_type, vclk_sel, pll_reg, div_reg, clk_reg);
	set_lcd_spread_spectrum(ss_level);
	
	clk_util_lvds_set_clk_div(1, pll_div_post, phy_clk_div2);
	
	//enable v2_clk div
    // WRITE_LCD_CBUS_REG(HHI_VIID_CLK_CNTL, READ_LCD_CBUS_REG(HHI_VIID_CLK_CNTL) | (0xF << 0) );
    // WRITE_LCD_CBUS_REG(HHI_VID_CLK_CNTL, READ_LCD_CBUS_REG(HHI_VID_CLK_CNTL) | (0xF << 0) );

    WRITE_LCD_REG(LVDS_PHY_CNTL0, 0xffff );

    //    lvds_gen_cntl       <= {10'h0,      // [15:4] unused
    //                            2'h1,       // [5:4] divide by 7 in the PHY
    //                            1'b0,       // [3] fifo_en
    //                            1'b0,       // [2] wr_bist_gate
    //                            2'b00};     // [1:0] fifo_wr mode

    FIFO_CLK_SEL = (test_bit_num == 8) ? 2 : // div8
                                    0 ; // div6
    rd_data = READ_LCD_REG(LVDS_GEN_CNTL);
    rd_data = (rd_data & 0xffcf) | (FIFO_CLK_SEL<< 4);
    WRITE_LCD_REG(LVDS_GEN_CNTL, rd_data);

    MPCLK_DELAY = (test_pair_num == 6) ?
                  ((test_bit_num == 8) ? (test_dual_gate ? 5 : 3) : 2) :
                  ((test_bit_num == 8) ? 3 : 3) ;

	MCLK_half_delay = pConf->lcd_control.mlvds_config->phase_select ? MCLK_half :
																			(test_dual_gate & (test_bit_num == 8) & (test_pair_num != 6));

    if(test_high_phase)
    {
        if(test_dual_gate)
        data32 = (MPCLK_DELAY << mpclk_dly) |
                 (((test_bit_num == 8) ? 3 : 2) << mpclk_div) |
                 (1 << use_mpclk) |
                 (MCLK_half_delay << mlvds_clk_half_delay) |
                 (((test_bit_num == 8) ? (
                                           (test_pair_num == 6) ? 0x999999 : // DIV4
                                                                  0x555555   // DIV2
                                         ) :
                                         (
                                           (test_pair_num == 6) ? mclk_pattern_dual_6_6 : //DIV8
                                                                  0x999999   // DIV4
                                         )
                 ) << mlvds_clk_pattern);
        else if(test_bit_num == 8)
            data32 = (MPCLK_DELAY << mpclk_dly) |
                     (((test_bit_num == 8) ? 3 : 2) << mpclk_div) |
                     (1 << use_mpclk) |
                     (0 << mlvds_clk_half_delay) |
                     (0xc3c3c3 << mlvds_clk_pattern);      // DIV 8
        else
            data32 = (MPCLK_DELAY << mpclk_dly) |
                     (((test_bit_num == 8) ? 3 : 2) << mpclk_div) |
                     (1 << use_mpclk) |
                     (0 << mlvds_clk_half_delay) |
                     (
                       (
                         (test_pair_num == 6) ? 0xc3c3c3 : // DIV8
                                                0x999999   // DIV4
                       ) << mlvds_clk_pattern
                     );
    }
    else {
        if(test_pair_num == 6) {
            data32 = (MPCLK_DELAY << mpclk_dly) |
                     (((test_bit_num == 8) ? 3 : 2) << mpclk_div) |
                     (1 << use_mpclk) |
                     (0 << mlvds_clk_half_delay) |
                     (
                       (
                         (test_pair_num == 6) ? 0x999999 : // DIV4
                                                0x555555   // DIV2
                       ) << mlvds_clk_pattern
                     );
        }
        else {
            data32 = (1 << mlvds_clk_half_delay) |
                   (0x555555 << mlvds_clk_pattern);      // DIV 2
        }
    }

    WRITE_LCD_REG(MLVDS_CLK_CTL_HI, (data32 >> 16));
    WRITE_LCD_REG(MLVDS_CLK_CTL_LO, (data32 & 0xffff));

	//pll2_div_sel
	// Set Soft Reset vid_pll_div_pre
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_DIVIDER_CNTL, 1, 3, 1);
	// Set Hard Reset vid_pll_div_post
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_DIVIDER_CNTL, 0, 1, 1);
	// Set Hard Reset lvds_phy_ser_top
	WRITE_LCD_REG_BITS(LVDS_PHY_CLK_CNTL, 0, 15, 1);
	// Release Hard Reset lvds_phy_ser_top
	WRITE_LCD_REG_BITS(LVDS_PHY_CLK_CNTL, 1, 15, 1);
	// Release Hard Reset vid_pll_div_post
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_DIVIDER_CNTL, 1, 1, 1);
	// Release Soft Reset vid_pll_div_pre
	WRITE_LCD_CBUS_REG_BITS(HHI_VIID_DIVIDER_CNTL, 0, 3, 1);
}
#endif

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
static void set_venc_ttl(Lcd_Config_t *pConf)
{
    DBG_PRINT("%s\n", __FUNCTION__);
	WRITE_LCD_REG(ENCT_VIDEO_EN,		0);
#ifdef CONFIG_AM_TV_OUTPUT2
    if(vpp2_sel)
        WRITE_LCD_REG_BITS(VPU_VIU_VENC_MUX_CTRL, 3, 2, 2);
    else
		WRITE_LCD_REG_BITS(VPU_VIU_VENC_MUX_CTRL, 0x44, 4, 8);//Select encT clock to VDIN, Enable VIU of ENC_T domain to VDIN
		WRITE_LCD_REG_BITS(VPU_VIU_VENC_MUX_CTRL, 3, 0, 2);//viu1 select enct
#else
    WRITE_LCD_REG(VPU_VIU_VENC_MUX_CTRL, (3<<0) | (3<<2));	// viu1 & viu2 select enct
#endif
    WRITE_LCD_REG(ENCT_VIDEO_MODE,        0);
    WRITE_LCD_REG(ENCT_VIDEO_MODE_ADV,    0x0008);
	  
	// bypass filter
    WRITE_LCD_REG(ENCT_VIDEO_FILT_CTRL,    0x1000);

    WRITE_LCD_REG(ENCT_VIDEO_MAX_PXCNT,    pConf->lcd_basic.h_period - 1);
    WRITE_LCD_REG(ENCT_VIDEO_MAX_LNCNT,    pConf->lcd_basic.v_period - 1);

    WRITE_LCD_REG(ENCT_VIDEO_HAVON_BEGIN,  pConf->lcd_timing.video_on_pixel);
    WRITE_LCD_REG(ENCT_VIDEO_HAVON_END,    pConf->lcd_basic.h_active - 1 + pConf->lcd_timing.video_on_pixel);
    WRITE_LCD_REG(ENCT_VIDEO_VAVON_BLINE,  pConf->lcd_timing.video_on_line);
    WRITE_LCD_REG(ENCT_VIDEO_VAVON_ELINE,  pConf->lcd_basic.v_active - 1  + pConf->lcd_timing.video_on_line);

    WRITE_LCD_REG(ENCT_VIDEO_HSO_BEGIN,    15);
    WRITE_LCD_REG(ENCT_VIDEO_HSO_END,      31);
    WRITE_LCD_REG(ENCT_VIDEO_VSO_BEGIN,    15);
    WRITE_LCD_REG(ENCT_VIDEO_VSO_END,      31);
    WRITE_LCD_REG(ENCT_VIDEO_VSO_BLINE,    0);
    WRITE_LCD_REG(ENCT_VIDEO_VSO_ELINE,    2);
    
	WRITE_LCD_REG(ENCT_VIDEO_RGBIN_CTRL, 	(1 << 0));//(1 << 1) | (1 << 0));	//bit[0] 1:RGB, 0:YUV
    // enable enct
    WRITE_LCD_REG(ENCT_VIDEO_EN,           1);
}

static void set_venc_mlvds(Lcd_Config_t *pConf)
{
    int ext_pixel = pConf->lcd_control.mlvds_config->test_dual_gate ? pConf->lcd_control.mlvds_config->total_line_clk : 0;
	int active_h_start = pConf->lcd_timing.video_on_pixel;
	int active_v_start = pConf->lcd_timing.video_on_line;
	int width = pConf->lcd_basic.h_active;
	int height = pConf->lcd_basic.v_active;
	int max_height = pConf->lcd_basic.v_period;
	
	DBG_PRINT("%s\n", __FUNCTION__);

    WRITE_LCD_REG(ENCL_VIDEO_EN,           0);

#ifdef CONFIG_AM_TV_OUTPUT2
    if(vpp2_sel){
        WRITE_LCD_REG_BITS(VPU_VIU_VENC_MUX_CTRL, 0, 2, 2);	//viu2 select encl
    }
    else{
		WRITE_LCD_REG_BITS(VPU_VIU_VENC_MUX_CTRL, 0, 0, 2);//viu1 select encl
    }
#else
    WRITE_LCD_REG(VPU_VIU_VENC_MUX_CTRL, (0<<0) | (0<<2));//viu1,viu2 select encl
#endif	

	WRITE_LCD_REG(ENCL_VIDEO_MODE,             0x0040 | (1<<14)); // Enable Hsync and equalization pulse switch in center; bit[14] cfg_de_v = 1
	WRITE_LCD_REG(ENCL_VIDEO_MODE_ADV,         0x0008); // Sampling rate: 1
	
	// bypass filter
 	WRITE_LCD_REG(ENCL_VIDEO_FILT_CTRL,			0x1000);
	
	WRITE_LCD_REG(ENCL_VIDEO_YFP1_HTIME,       active_h_start);
	WRITE_LCD_REG(ENCL_VIDEO_YFP2_HTIME,       active_h_start + width);

	WRITE_LCD_REG(ENCL_VIDEO_MAX_PXCNT,        pConf->lcd_control.mlvds_config->total_line_clk - 1 + ext_pixel);
	WRITE_LCD_REG(ENCL_VIDEO_MAX_LNCNT,        max_height - 1);

	WRITE_LCD_REG(ENCL_VIDEO_HAVON_BEGIN,      active_h_start);
	WRITE_LCD_REG(ENCL_VIDEO_HAVON_END,        active_h_start + width - 1);  // for dual_gate mode still read 1408 pixel at first half of line
	WRITE_LCD_REG(ENCL_VIDEO_VAVON_BLINE,      active_v_start);
	WRITE_LCD_REG(ENCL_VIDEO_VAVON_ELINE,      active_v_start + height -1);  //15+768-1);

	WRITE_LCD_REG(ENCL_VIDEO_HSO_BEGIN,        24);
	WRITE_LCD_REG(ENCL_VIDEO_HSO_END,          1420 + ext_pixel);
	WRITE_LCD_REG(ENCL_VIDEO_VSO_BEGIN,        1400 + ext_pixel);
	WRITE_LCD_REG(ENCL_VIDEO_VSO_END,          1410 + ext_pixel);
	WRITE_LCD_REG(ENCL_VIDEO_VSO_BLINE,        1);
	WRITE_LCD_REG(ENCL_VIDEO_VSO_ELINE,        3);

	WRITE_LCD_REG(ENCL_VIDEO_RGBIN_CTRL, 	(1 << 0));//(1 << 1) | (1 << 0));	//bit[0] 1:RGB, 0:YUV

	// enable encl
    WRITE_LCD_REG(ENCL_VIDEO_EN,		1);
}
#endif

static void set_venc_lcd(Lcd_Config_t *pConf)
{
	int lcd_type;
	lcd_type = pConf->lcd_basic.lcd_type;
	
	DBG_PRINT("%s\n",__FUNCTION__);

	WRITE_LCD_REG(ENCL_VIDEO_EN, 0);
#ifdef CONFIG_AM_TV_OUTPUT2
	if	(vpp2_sel) {
		WRITE_LCD_REG_BITS(VPU_VIU_VENC_MUX_CTRL, 0, 2, 2);	//viu2 select encl
		WRITE_LCD_REG(VPU_VIU_VENC_MUX_CTRL, (READ_LCD_REG(VPU_VIU_VENC_MUX_CTRL)&(~(0x3<<2)))|(0x0<<2)); //viu2 select encl
	}
	else {
		WRITE_LCD_REG_BITS(VPU_VIU_VENC_MUX_CTRL, 0x88, 4, 8);//Select encl clock to VDIN, Enable VIU of ENC_l domain to VDIN
		WRITE_LCD_REG_BITS(VPU_VIU_VENC_MUX_CTRL, 0, 0, 2);//viu1 select encl
	}
#else
	WRITE_LCD_REG(VPU_VIU_VENC_MUX_CTRL, (0<<0) | (0<<2));	// viu1 & viu2 select encl
#endif
	
	WRITE_LCD_REG(ENCL_VIDEO_MODE,			0);
	WRITE_LCD_REG(ENCL_VIDEO_MODE_ADV,		0x0008);	// Sampling rate: 1

 	WRITE_LCD_REG(ENCL_VIDEO_FILT_CTRL,		0x1000);	// bypass filter

	WRITE_LCD_REG(ENCL_VIDEO_MAX_PXCNT,		pConf->lcd_basic.h_period - 1);
	WRITE_LCD_REG(ENCL_VIDEO_MAX_LNCNT,		pConf->lcd_basic.v_period - 1);

	WRITE_LCD_REG(ENCL_VIDEO_HAVON_BEGIN,	pConf->lcd_timing.video_on_pixel);
	WRITE_LCD_REG(ENCL_VIDEO_HAVON_END,		pConf->lcd_basic.h_active - 1 + pConf->lcd_timing.video_on_pixel);
	WRITE_LCD_REG(ENCL_VIDEO_VAVON_BLINE,	pConf->lcd_timing.video_on_line);
	WRITE_LCD_REG(ENCL_VIDEO_VAVON_ELINE,	pConf->lcd_basic.v_active - 1  + pConf->lcd_timing.video_on_line);

	WRITE_LCD_REG(ENCL_VIDEO_HSO_BEGIN,		pConf->lcd_timing.sth1_hs_addr);
	WRITE_LCD_REG(ENCL_VIDEO_HSO_END,		pConf->lcd_timing.sth1_he_addr);
	WRITE_LCD_REG(ENCL_VIDEO_VSO_BEGIN,		pConf->lcd_timing.stv1_hs_addr);
	WRITE_LCD_REG(ENCL_VIDEO_VSO_END,		pConf->lcd_timing.stv1_he_addr);
	WRITE_LCD_REG(ENCL_VIDEO_VSO_BLINE,		pConf->lcd_timing.stv1_vs_addr);
	WRITE_LCD_REG(ENCL_VIDEO_VSO_ELINE,		pConf->lcd_timing.stv1_ve_addr);

	WRITE_LCD_REG(ENCL_VIDEO_RGBIN_CTRL, 	(1 << 0));//(1 << 1) | (1 << 0));	//bit[0] 1:RGB, 0:YUV

	WRITE_LCD_REG(ENCL_VIDEO_EN,			1);	// enable encl
}

static void set_control_lvds(Lcd_Config_t *pConf)
{
	unsigned lvds_repack, pn_swap, bit_num;
	unsigned data32;
	
	DBG_PRINT("%s\n", __FUNCTION__);

	WRITE_LCD_REG_BITS(LVDS_GEN_CNTL, 0, 3, 1); // disable lvds fifo
	
    data32 = (0x00 << LVDS_blank_data_r) |
             (0x00 << LVDS_blank_data_g) |
             (0x00 << LVDS_blank_data_b) ; 
    WRITE_LCD_REG(LVDS_BLANK_DATA_HI, (data32 >> 16));
    WRITE_LCD_REG(LVDS_BLANK_DATA_LO, (data32 & 0xffff));
	
	lvds_repack = (pConf->lcd_control.lvds_config->lvds_repack) & 0x1;
	pn_swap = (pConf->lcd_control.lvds_config->pn_swap) & 0x1;

	switch(pConf->lcd_basic.lcd_bits) {
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
	
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
	WRITE_LCD_REG_BITS(MLVDS_CONTROL, 0, 0, 1);  //disable mlvds
#endif

	WRITE_LCD_REG(LVDS_PACK_CNTL_ADDR, 
					( lvds_repack<<0 ) | // repack
					( 0<<2 ) | // odd_even
					( 0<<3 ) | // reserve
					( 0<<4 ) | // lsb first
					( pn_swap<<5 ) | // pn swap
					( 0<<6 ) | // dual port
					( 0<<7 ) | // use tcon control
					( bit_num<<8 ) | // 0:10bits, 1:8bits, 2:6bits, 3:4bits.
					( 0<<10 ) | //r_select  //0:R, 1:G, 2:B, 3:0
					( 1<<12 ) | //g_select  //0:R, 1:G, 2:B, 3:0
					( 2<<14 ));  //b_select  //0:R, 1:G, 2:B, 3:0; 
				   
    WRITE_LCD_REG_BITS(LVDS_GEN_CNTL, 1, 0, 1);  //fifo enable
	//WRITE_LCD_REG_BITS(LVDS_GEN_CNTL, 1, 3, 1);  //enable fifo
}

#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
static void set_control_mipi(Lcd_Config_t *pConf)
{
    set_mipi_dsi_control(pConf);
}
#endif

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
//**************************************************//
// for edp link maintain control
//**************************************************//
static void generate_clk_parameter(Lcd_Config_t *pConf);
static void lcd_sync_duration(Lcd_Config_t *pConf);
unsigned edp_clk_config_update(unsigned char link_rate)
{
	unsigned bit_rate;
	
	switch (link_rate) {
		case VAL_EDP_TX_LINK_BW_SET_162:
			pDev->pConf->lcd_control.edp_config->link_rate = 0;
			break;
		case VAL_EDP_TX_LINK_BW_SET_270:
		default:
			pDev->pConf->lcd_control.edp_config->link_rate = 1;
			break;
	}
	generate_clk_parameter(pDev->pConf);
	lcd_sync_duration(pDev->pConf);
	
	bit_rate = (pDev->pConf->lcd_timing.lcd_clk / 1000) * pDev->pConf->lcd_basic.lcd_bits * 3 / 1000;	//Mbps
	pDev->pConf->lcd_control.edp_config->bit_rate = bit_rate;
	
	//update lcd_info
	pDev->lcd_info.sync_duration_num = pDev->pConf->lcd_timing.sync_duration_num;
	pDev->lcd_info.sync_duration_den = pDev->pConf->lcd_timing.sync_duration_den;
	pDev->lcd_info.video_clk = pDev->pConf->lcd_timing.lcd_clk;
	
	request_vpu_clk_vmod(pDev->lcd_info.video_clk, pDev->lcd_info.mode);
	
	set_pll_lcd(pDev->pConf);	//real change the clk
	
	return bit_rate;
}

void edp_phy_config_update(unsigned char vswing_tx, unsigned char preemp_tx)
{
    unsigned vswing_ctrl, preemphasis_ctrl;

    switch (vswing_tx) {
        case 0:	//0.4V
            vswing_ctrl = 0x8018;	//0x8038;
            break;
        case 1:	//0.6V
            vswing_ctrl = 0x8088;
            break;
        case 2:	//0.8V
            vswing_ctrl = 0x80c8;
            break;
        case 3:	//1.2V
            vswing_ctrl = 0x80f8;
            break;
        default:
            vswing_ctrl = 0x80f8;
            break;
    }

    switch (preemp_tx) {
        case 0:	//0db
        case 1:	//3.5db
        case 2:	//6db
        case 3:	//9.5db
        default:
            preemphasis_ctrl = 0x0;	//to do
            break;
    }

    WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL1, vswing_ctrl);
    printk("edp link adaptive: vswing=0x%02x, preemphasis=0x%02x\n", vswing_tx, preemp_tx);
}
//**************************************************//

static int set_control_edp(Lcd_Config_t *pConf)
{
    int ret = 0;
    EDP_Video_Mode_t  vm;
    EDP_Link_Config_t link_config;

    DBG_PRINT("%s\n", __FUNCTION__);
    //edp link config
    link_config.max_lane_count = 4;
    link_config.max_link_rate = VAL_EDP_TX_LINK_BW_SET_270;
    //link_config.link_rate = pConf->lcd_control.edp_config->link_rate;
    link_config.lane_count = pConf->lcd_control.edp_config->lane_count;
    link_config.ss_level =((((pConf->lcd_timing.clk_ctrl) >> CLK_CTRL_SS) & 0xf) > 0 ? 1 : 0);
    link_config.link_adaptive = pConf->lcd_control.edp_config->link_adaptive;
    //link_config.vswing = pConf->lcd_control.edp_config->vswing;
    //link_config.preemphasis = pConf->lcd_control.edp_config->preemphasis;
    link_config.bit_rate = pConf->lcd_control.edp_config->bit_rate;

    switch (pConf->lcd_control.edp_config->link_rate) {
        case 0:
            link_config.link_rate = VAL_EDP_TX_LINK_BW_SET_162;
            break;
        case 1:
            link_config.link_rate = VAL_EDP_TX_LINK_BW_SET_270;
            break;
        default:
            link_config.link_rate = VAL_EDP_TX_LINK_BW_SET_270;
            break;
    }
    switch (pConf->lcd_control.edp_config->vswing) {
        case 0:
            link_config.vswing = VAL_EDP_TX_PHY_VSWING_0;
            break;
        case 1:
            link_config.vswing = VAL_EDP_TX_PHY_VSWING_1;
            break;
        case 2:
            link_config.vswing = VAL_EDP_TX_PHY_VSWING_2;
            break;
        case 3:
            link_config.vswing = VAL_EDP_TX_PHY_VSWING_3;
            break;
        default:
            link_config.vswing = VAL_EDP_TX_PHY_VSWING_0;
            break;
    }
    switch (pConf->lcd_control.edp_config->preemphasis) {
        case 0:
            link_config.preemphasis = VAL_EDP_TX_PHY_PREEMPHASIS_0;
            break;
        case 1:
            link_config.preemphasis = VAL_EDP_TX_PHY_PREEMPHASIS_1;
            break;
        case 2:
            link_config.preemphasis = VAL_EDP_TX_PHY_PREEMPHASIS_2;
            break;
        case 3:
            link_config.preemphasis = VAL_EDP_TX_PHY_PREEMPHASIS_3;
            break;
        default:
            link_config.preemphasis = VAL_EDP_TX_PHY_PREEMPHASIS_0;
            break;
    }

    //edp main stream attribute
    vm.h_active = pConf->lcd_basic.h_active;
    vm.v_active = pConf->lcd_basic.v_active;
    vm.h_period = pConf->lcd_basic.h_period;
    vm.v_period = pConf->lcd_basic.v_period;
    vm.clk = pConf->lcd_timing.lcd_clk;
    vm.hsync_pol = (pConf->lcd_timing.pol_cntl_addr >> LCD_HS_POL) & 1;
    vm.hsync_width = pConf->lcd_timing.hsync_width;
    vm.hsync_bp = pConf->lcd_timing.hsync_bp;
    vm.vsync_pol = (pConf->lcd_timing.pol_cntl_addr >> LCD_VS_POL) & 1;
    vm.vsync_width = pConf->lcd_timing.vsync_width;
    vm.vsync_bp = pConf->lcd_timing.vsync_bp;
    vm.de_hstart = pConf->lcd_timing.de_hstart;
    vm.de_vstart = pConf->lcd_timing.de_vstart;
    vm.ppc = 1;							//pixels per clock cycle
    vm.cformat = 0;						//color format(0=RGB, 1=4:2:2, 2=Y only)
    vm.bpc = pConf->lcd_basic.lcd_bits;	//bits per color

    //edp link maintain
    ret = dplpm_link_policy_maker(&link_config, &vm);

    //save feedback config by edp link maintain
    //pConf->lcd_control.edp_config->link_rate = link_config.link_rate;
    pConf->lcd_control.edp_config->lane_count = link_config.lane_count;
    //pConf->lcd_control.edp_config->vswing = link_config.vswing;
    //pConf->lcd_control.edp_config->preemphasis = link_config.preemphasis;
    pConf->lcd_control.edp_config->bit_rate = link_config.bit_rate;
    switch (link_config.link_rate) {
        case VAL_EDP_TX_LINK_BW_SET_162:
            pConf->lcd_control.edp_config->link_rate = 0;
            break;
        case VAL_EDP_TX_LINK_BW_SET_270:
            pConf->lcd_control.edp_config->link_rate = 1;
            break;
        default:
            pConf->lcd_control.edp_config->link_rate = 1;
            break; 
    }
    switch (link_config.vswing) {
        case VAL_EDP_TX_PHY_VSWING_0:
            pConf->lcd_control.edp_config->vswing = 0;
            break;
        case VAL_EDP_TX_PHY_VSWING_1:
            pConf->lcd_control.edp_config->vswing = 1;
            break;
        case VAL_EDP_TX_PHY_VSWING_2:
            pConf->lcd_control.edp_config->vswing = 2;
            break;
        case VAL_EDP_TX_PHY_VSWING_3:
            pConf->lcd_control.edp_config->vswing = 3;
            break;
        default:
            pConf->lcd_control.edp_config->vswing = 0;
            break;
    }
    switch (link_config.preemphasis) {
        case VAL_EDP_TX_PHY_PREEMPHASIS_0:
            pConf->lcd_control.edp_config->preemphasis = 0;
            break;
        case VAL_EDP_TX_PHY_PREEMPHASIS_1:
            pConf->lcd_control.edp_config->preemphasis = 1;
            break;
        case VAL_EDP_TX_PHY_PREEMPHASIS_2:
            pConf->lcd_control.edp_config->preemphasis = 2;
            break;
        case VAL_EDP_TX_PHY_PREEMPHASIS_3:
            pConf->lcd_control.edp_config->preemphasis = 3;
            break;
        default:
            pConf->lcd_control.edp_config->preemphasis = 0;
            break;
    }

    return ret;
}
#endif

static void set_control_ttl(Lcd_Config_t *pConf)
{
	unsigned rb_port_swap, rgb_bit_swap;
	
	rb_port_swap = (unsigned)(pConf->lcd_control.ttl_config->rb_swap);
	rgb_bit_swap = (unsigned)(pConf->lcd_control.ttl_config->bit_swap);
	
	WRITE_LCD_REG(L_DUAL_PORT_CNTL_ADDR, (rb_port_swap << LCD_RGB_SWP) | (rgb_bit_swap << LCD_BIT_SWP));
}

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
static void set_control_mlvds(Lcd_Config_t *pConf)
{
	int test_bit_num = pConf->lcd_basic.lcd_bits;
    int test_pair_num = pConf->lcd_control.mlvds_config->test_pair_num;
    int test_dual_gate = pConf->lcd_control.mlvds_config->test_dual_gate;
    int scan_function = pConf->lcd_control.mlvds_config->scan_function;     //0:U->D,L->R  //1:D->U,R->L
    int mlvds_insert_start;
    unsigned int reset_offset;
    unsigned int reset_length;

    unsigned long data32;
	
	DBG_PRINT("%s\n", __FUNCTION__);
    
    mlvds_insert_start = test_dual_gate ?
                           ((test_bit_num == 8) ? ((test_pair_num == 6) ? 0x9f : 0xa9) :
                                                  ((test_pair_num == 6) ? pConf->lcd_control.mlvds_config->mlvds_insert_start : 0xa7)
                           ) :
                           (
                             (test_pair_num == 6) ? ((test_bit_num == 8) ? 0xa9 : 0xa7) :
                                                    ((test_bit_num == 8) ? 0xae : 0xad)
                           );

    // Enable the LVDS PHY (power down bits)
	WRITE_LCD_REG_BITS(LVDS_PHY_CNTL1, 0x7f, 8, 7);

    data32 = (0x00 << LVDS_blank_data_r) |
             (0x00 << LVDS_blank_data_g) |
             (0x00 << LVDS_blank_data_b) ;
    WRITE_LCD_REG(LVDS_BLANK_DATA_HI,  (data32 >> 16));
    WRITE_LCD_REG(LVDS_BLANK_DATA_LO, (data32 & 0xffff));

    data32 = 0x7fffffff; //  '0'x1 + '1'x32 + '0'x2
    WRITE_LCD_REG(MLVDS_RESET_PATTERN_HI,  (data32 >> 16));
    WRITE_LCD_REG(MLVDS_RESET_PATTERN_LO, (data32 & 0xffff));
    data32 = 0x8000; // '0'x1 + '1'x32 + '0'x2
    WRITE_LCD_REG(MLVDS_RESET_PATTERN_EXT,  (data32 & 0xffff));

    reset_length = 1+32+2;
    reset_offset = test_bit_num - (reset_length%test_bit_num);

    data32 = (reset_offset << mLVDS_reset_offset) |
             (reset_length << mLVDS_reset_length) |
             ((test_pair_num == 6) << mLVDS_data_write_toggle) |
             ((test_pair_num != 6) << mLVDS_data_write_ini) |
             ((test_pair_num == 6) << mLVDS_data_latch_1_toggle) |
             (0 << mLVDS_data_latch_1_ini) |
             ((test_pair_num == 6) << mLVDS_data_latch_0_toggle) |
             (1 << mLVDS_data_latch_0_ini) |
             ((test_pair_num == 6) << mLVDS_reset_1_select) |
             (mlvds_insert_start << mLVDS_reset_start);
    WRITE_LCD_REG(MLVDS_CONFIG_HI, (data32 >> 16));
    WRITE_LCD_REG(MLVDS_CONFIG_LO, (data32 & 0xffff));

    data32 = (1 << mLVDS_double_pattern) |  //POL double pattern
			 (0x3f << mLVDS_ins_reset) |
             (test_dual_gate << mLVDS_dual_gate) |
             ((test_bit_num == 8) << mLVDS_bit_num) |
             ((test_pair_num == 6) << mLVDS_pair_num) |
             (0 << mLVDS_msb_first) |
             (0 << mLVDS_PORT_SWAP) |
             ((scan_function==1 ? 1:0) << mLVDS_MLSB_SWAP) |
             (0 << mLVDS_PN_SWAP) |
             (1 << mLVDS_en);
    WRITE_LCD_REG(MLVDS_CONTROL,  (data32 & 0xffff));

    WRITE_LCD_REG(LVDS_PACK_CNTL_ADDR,
                   ( 0 ) | // repack
                   ( 0<<2 ) | // odd_even
                   ( 0<<3 ) | // reserve
                   ( 0<<4 ) | // lsb first
                   ( 0<<5 ) | // pn swap
                   ( 0<<6 ) | // dual port
                   ( 0<<7 ) | // use tcon control
                   ( 1<<8 ) | // 0:10bits, 1:8bits, 2:6bits, 3:4bits.
                   ( (scan_function==1 ? 2:0)<<10 ) |  //r_select // 0:R, 1:G, 2:B, 3:0
                   ( 1<<12 ) |                        //g_select
                   ( (scan_function==1 ? 0:2)<<14 ));  //b_select

    WRITE_LCD_REG(L_POL_CNTL_ADDR,  (1 << LCD_DCLK_SEL) |
       //(0x1 << LCD_HS_POL) |
       (0x1 << LCD_VS_POL)
    );
	
	WRITE_LCD_REG_BITS(LVDS_GEN_CNTL, 1, 3, 1); // enable fifo
}
#endif

static void init_phy_lvds(Lcd_Config_t *pConf)
{
    unsigned swing_ctrl;
    DBG_PRINT("%s\n", __FUNCTION__);
	
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
    WRITE_LCD_REG(LVDS_PHY_CNTL3, 0xee1);
    WRITE_LCD_REG(LVDS_PHY_CNTL4 ,0);

	switch (pConf->lcd_control.lvds_config->lvds_vswing) {
		case 0:
			swing_ctrl = 0xaf20;
			break;
		case 1:
			swing_ctrl = 0xaf40;
			break;
		case 2:
			swing_ctrl = 0xa840;
			break;
		case 3:
			swing_ctrl = 0xa880;
			break;
		case 4:
			swing_ctrl = 0xa8c0;
			break;
		default:
			swing_ctrl = 0xaf40;
			break;
	}
	WRITE_LCD_REG(LVDS_PHY_CNTL5, swing_ctrl);

	WRITE_LCD_REG(LVDS_PHY_CNTL0,0x001f);
	WRITE_LCD_REG(LVDS_PHY_CNTL1,0xffff);

    WRITE_LCD_REG(LVDS_PHY_CNTL6,0xcccc);
    WRITE_LCD_REG(LVDS_PHY_CNTL7,0xcccc);
    WRITE_LCD_REG(LVDS_PHY_CNTL8,0xcccc);
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
	WRITE_LCD_REG(LVDS_SER_EN, 0xfff);	//Enable the serializers

    WRITE_LCD_REG(LVDS_PHY_CNTL0, 0xffff);
    WRITE_LCD_REG(LVDS_PHY_CNTL1, 0xff00);
	WRITE_LCD_REG(LVDS_PHY_CNTL4, 0x007f);
	
	//WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL1, 0x00000348);
	switch (pConf->lcd_control.lvds_config->lvds_vswing) {
		case 0:
			swing_ctrl = 0x028;
			break;
		case 1:
			swing_ctrl = 0x048;
			break;
		case 2:
			swing_ctrl = 0x088;
			break;
		case 3:
			swing_ctrl = 0x0c8;
			break;
		case 4:
			swing_ctrl = 0x0f8;
			break;
		default:
			swing_ctrl = 0x048;
			break;
	}
	WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL1, swing_ctrl);
	WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL2, 0x000665b7);
	WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL3, 0x84070000);
#endif
}

#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
static void init_phy_mipi(Lcd_Config_t *pConf)
{
    DBG_PRINT("%s\n", __FUNCTION__);

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B)
    WRITE_LCD_CBUS_REG_BITS(HHI_DSI_LVDS_EDP_CNTL1, 1, 4, 1);//swap mipi channels, only for m8baby
#endif
    WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL1, 0x8);//DIF_REF_CTL0
    WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL2, (0x3e << 16) | (0xa5b8 << 0));//DIF_REF_CTL2:31-16bit, DIF_REF_CTL1:15-0bit
    WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL3, (0x26e0 << 16) | (0x459 << 0));//DIF_TX_CTL1:31-16bit, DIF_TX_CTL0:15-0bit
}
#endif

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
static void init_phy_edp(Lcd_Config_t *pConf)
{
    unsigned swing_ctrl;
    DBG_PRINT("%s\n", __FUNCTION__);

    switch (pConf->lcd_control.edp_config->vswing) {
        case 0:	//0.4V
            swing_ctrl = 0x8018;
            break;
        case 1:	//0.6V
            swing_ctrl = 0x8088;
            break;
        case 2:	//0.8V
            swing_ctrl = 0x80c8;
            break;
        case 3:	//1.2V
            swing_ctrl = 0x80f8;
            break;
        default:
            swing_ctrl = 0x8018;
            break;
    }

    WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL1, swing_ctrl);//[7:4]swing b:800mv, step 50mv
    WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL2, ((0x6 << 16) | (0xf5d7 << 0)));
    WRITE_LCD_CBUS_REG(HHI_DIF_CSI_PHY_CNTL3, ((0xc2b2 << 16) | (0x600 << 0)));//0xd2b0fe00);
}
#endif

static void init_dphy(Lcd_Config_t *pConf)
{
	unsigned lcd_type = (unsigned)(pConf->lcd_basic.lcd_type);

	switch (lcd_type) {
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
		case LCD_DIGITAL_MIPI:
			WRITE_LCD_CBUS_REG(HHI_DSI_LVDS_EDP_CNTL0, lcd_type);	//dphy select by interface
			init_phy_mipi(pConf);
			break;
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
		case LCD_DIGITAL_EDP:
			WRITE_LCD_CBUS_REG(HHI_DSI_LVDS_EDP_CNTL0, lcd_type);	//dphy select by interface
			init_phy_edp(pConf);
			break;
#endif
		case LCD_DIGITAL_LVDS:
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
			WRITE_LCD_CBUS_REG(HHI_DSI_LVDS_EDP_CNTL0, lcd_type);	//dphy select by interface
#endif
			init_phy_lvds(pConf);
			break;
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
		case LCD_DIGITAL_MINILVDS:
			init_phy_lvds(pConf);
			break;
#endif
		default:
			break;
	}
}

static void set_video_adjust(Lcd_Config_t *pConf)
{
	DBG_PRINT("vadj_brightness = 0x%x, vadj_contrast = 0x%x, vadj_saturation = 0x%x.\n", pConf->lcd_effect.vadj_brightness, pConf->lcd_effect.vadj_contrast, pConf->lcd_effect.vadj_saturation);
	WRITE_LCD_REG(VPP_VADJ2_Y, (pConf->lcd_effect.vadj_brightness << 8) | (pConf->lcd_effect.vadj_contrast << 0));
	WRITE_LCD_REG(VPP_VADJ2_MA_MB, (pConf->lcd_effect.vadj_saturation << 16));
	WRITE_LCD_REG(VPP_VADJ2_MC_MD, (pConf->lcd_effect.vadj_saturation << 0));
	WRITE_LCD_REG(VPP_VADJ_CTRL, 0xf);	//enable video adjust
}

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
static unsigned error_abs(unsigned num1, unsigned num2)
{
	if (num1 >= num2)
		return num1 - num2;
	else
		return num2 - num1;
}
#endif

static void generate_clk_parameter(Lcd_Config_t *pConf)
{
    unsigned pll_n = 0, pll_m = 0, pll_od = 0, pll_frac = 0, pll_level = 0;
    unsigned edp_phy_div0 = 0, edp_phy_div1 = 0, vid_div_pre = 0;
    unsigned crt_xd = 0;

    unsigned m, n, od, div_pre, div_post, xd;
    unsigned od_sel, pre_div_sel;
    unsigned div_pre_sel_max, crt_xd_max;
    unsigned f_ref, pll_vco, fout_pll, div_pre_out, div_post_out, final_freq, iflogic_vid_clk_in_max;
    unsigned min_error = MAX_ERROR;
    unsigned error = MAX_ERROR;
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
    unsigned od_fb=0;
    unsigned int dsi_bit_rate_min=0, dsi_bit_rate_max=0;
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
    unsigned edp_div0, edp_div1, edp_div0_sel, edp_div1_sel;
    unsigned edp_tx_phy_out;
#endif
    unsigned clk_num = 0;
    unsigned tmp;
    unsigned fin = FIN_FREQ;
    unsigned fout = pConf->lcd_timing.lcd_clk;

    if (fout >= 200) {//clk
        fout = fout / 1000;  //kHz
    }
    else {//frame_rate
        fout = (fout * pConf->lcd_basic.h_period * pConf->lcd_basic.v_period) / 1000;	//kHz
    }

    switch (pConf->lcd_basic.lcd_type) {
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
        case LCD_DIGITAL_MIPI:
            div_pre_sel_max = DIV_PRE_SEL_MAX;
            div_post = 1;
            crt_xd_max = 16;
            dsi_bit_rate_min = pConf->lcd_control.mipi_config->bit_rate_min;
            dsi_bit_rate_max = pConf->lcd_control.mipi_config->bit_rate_max;
            iflogic_vid_clk_in_max = MIPI_MAX_VID_CLK_IN;
            break;
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
        case LCD_DIGITAL_EDP:
            div_pre_sel_max = 1;
            div_post = 1;
            crt_xd_max = 1;
            iflogic_vid_clk_in_max = EDP_MAX_VID_CLK_IN;
            min_error = 30 * 1000;
            break;
#endif
        case LCD_DIGITAL_LVDS:
            div_pre_sel_max = DIV_PRE_SEL_MAX;
            div_post = 7;
            crt_xd_max = 1;
            iflogic_vid_clk_in_max = LVDS_MAX_VID_CLK_IN;
            break;
        case LCD_DIGITAL_TTL:
            div_pre_sel_max = DIV_PRE_SEL_MAX;
            div_post = 1;
            crt_xd_max = CRT_VID_DIV_MAX;
            iflogic_vid_clk_in_max = TTL_MAX_VID_CLK_IN;
            break;
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
        case LCD_DIGITAL_MINILVDS:
            div_pre_sel_max = DIV_PRE_SEL_MAX;
            div_post = 6;
            crt_xd_max = 1;
            iflogic_vid_clk_in_max = MLVDS_MAX_VID_CLK_IN;
            break;
#endif
        default:
            div_pre_sel_max = DIV_PRE_SEL_MAX;
            div_post = 1;
            crt_xd_max = 1;
            iflogic_vid_clk_in_max = LCD_VENC_MAX_CLK_IN;
            break;
    }

    switch (pConf->lcd_basic.lcd_type) {
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
        case LCD_DIGITAL_MIPI:
            if (fout < LCD_VENC_MAX_CLK_IN) {
                for (xd = 1; xd <= crt_xd_max; xd++) {
                    div_post_out = fout * xd;
                    DBG_PRINT("div_post_out=%d, xd=%d, fout=%d\n",div_post_out, xd, fout);
                    if (div_post_out <= CRT_VID_MAX_CLK_IN) {
                        div_pre_out = div_post_out * div_post;
                        if (div_pre_out <= DIV_POST_MAX_CLK_IN) {
                            for (pre_div_sel = 0; pre_div_sel < div_pre_sel_max; pre_div_sel++) {
                                div_pre = div_pre_table[pre_div_sel];
                                fout_pll = div_pre_out * div_pre;
                                DBG_PRINT("pre_div_sel=%d, div_pre=%d, fout_pll=%d\n", pre_div_sel, div_pre, fout_pll);
                                if ((fout_pll <= dsi_bit_rate_max) && (fout_pll >= dsi_bit_rate_min)){
                                    for (od_sel = OD_SEL_MAX; od_sel > 0; od_sel--) {
                                        od = od_table[od_sel - 1];
                                        pll_vco = fout_pll * od;
                                        DBG_PRINT("od_sel=%d, od=%d, pll_vco=%d\n", od_sel, od, pll_vco);
                                        if ((pll_vco >= PLL_VCO_MIN) && (pll_vco <= PLL_VCO_MAX)) {
                                            if ((pll_vco >= 2500000) && (pll_vco <= PLL_VCO_MAX)) {
                                                od_fb = 1;
                                                pll_level = 4;
                                            }
                                            else if ((pll_vco >= 2000000) && (pll_vco < 2500000)) {
                                                od_fb = 1;
                                                pll_level = 3;
                                            }
                                            else if ((pll_vco >= 1700000) && (pll_vco < 2000000)) {//special adjust
                                                od_fb = 1;
                                                pll_level = 2;
                                            }
                                            else if ((pll_vco >= PLL_VCO_MIN) && (pll_vco < 1700000)) {
                                                od_fb = 0;
                                                pll_level = 1;
                                            }
                                            n = 1;
                                            m = pll_vco / (fin * (od_fb + 1));
                                            pll_frac = (pll_vco % (fin * (od_fb + 1))) * 4096 / (fin * (od_fb + 1));
                                            pll_m = m;
                                            pll_n = n;
                                            pll_od = od_sel - 1;
                                            vid_div_pre = pre_div_sel;
                                            crt_xd = xd;
                                            clk_num = 1;
                                            DBG_PRINT("pll_m=0x%x, pll_n=0x%x, pll_od=0x%x, vid_div_pre=0x%x, crt_xd=0x%x, pll_frac=0x%x, pll_level=%d\n",
                                                       pll_m, pll_n, pll_od, vid_div_pre, crt_xd, pll_frac, pll_level);
                                        }
                                        if (clk_num > 0)
                                            break;
                                    }
                                }
                                if (clk_num > 0)
                                    break;
                            }
                        }
                    }
                    if (clk_num > 0)
                        break;
                }
            }
            break;
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
        case LCD_DIGITAL_EDP:
            switch (pConf->lcd_control.edp_config->link_rate) {
                case 0:
                    n = 1;
                    m = 67;
                    od_sel = 0;
                    pll_level = 1;
                    pll_frac = 0x800;
                    fout_pll = 1620000;
                    break;
                case 1:
                default:
                    n = 1;
                    m = 56;
                    od_sel = 0;
                    pll_level = 4;
                    pll_frac = 0x400;
                    fout_pll = 2700000;
                    break;
            }
            pll_m = m;
            pll_n = n;
            pll_od = od_sel;

            for(edp_div1_sel=0; edp_div1_sel<EDP_DIV1_SEL_MAX; edp_div1_sel++) {
                edp_div1 = edp_div1_table[edp_div1_sel];
                for (edp_div0_sel=0; edp_div0_sel<EDP_DIV0_SEL_MAX; edp_div0_sel++) {
                    edp_div0 = edp_div0_table[edp_div0_sel];
                    edp_tx_phy_out = fout_pll / (edp_div0 * edp_div1);
                    if (edp_tx_phy_out <= DIV_PRE_MAX_CLK_IN) {
                        for (pre_div_sel = 0; pre_div_sel < div_pre_sel_max; pre_div_sel++) {
                            div_pre = div_pre_table[pre_div_sel];
                            div_pre_out = edp_tx_phy_out / div_pre;
                            if (div_pre_out <= DIV_POST_MAX_CLK_IN) {
                                div_post_out = div_pre_out / div_post;
                                if (div_post_out <= CRT_VID_MAX_CLK_IN) {
                                    for (xd = 1; xd <= crt_xd_max; xd++) {
                                        final_freq = div_post_out / xd;
                                        if (final_freq < LCD_VENC_MAX_CLK_IN) {
                                            if (final_freq < iflogic_vid_clk_in_max) {
                                                if (final_freq <= fout) {
                                                    error = fout - final_freq;
                                                    if (error < min_error) {
                                                        min_error = error;
                                                        edp_phy_div0 = edp_div0_sel;
                                                        edp_phy_div1 = edp_div1_sel;
                                                        vid_div_pre = pre_div_sel;
                                                        crt_xd = xd;
                                                        clk_num++;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            break;
#endif
        case LCD_DIGITAL_LVDS:
        case LCD_DIGITAL_TTL:
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
            for (n = PLL_N_MIN; n <= PLL_N_MAX; n++) {
                f_ref = fin / n;
                if ((f_ref >= PLL_FREF_MIN) && (f_ref <= PLL_FREF_MAX))    {
                    for (m = PLL_M_MIN; m <= PLL_M_MAX; m++) {
                        pll_vco = f_ref * m;
                        if ((pll_vco >= PLL_VCO_MIN) && (pll_vco <= PLL_VCO_MAX)) {
                            for (od_sel = OD_SEL_MAX; od_sel > 0; od_sel--) {
                                od = od_table[od_sel - 1];
                                fout_pll = pll_vco / od;
                            if (fout_pll <= DIV_PRE_MAX_CLK_IN) {
                                    for (pre_div_sel = 0; pre_div_sel < div_pre_sel_max; pre_div_sel++) {
                                        div_pre = div_pre_table[pre_div_sel];
                                        div_pre_out = fout_pll / div_pre;
                                        if (div_pre_out <= DIV_POST_MAX_CLK_IN) {
                                            div_post_out = div_pre_out / div_post;
                                            if (div_post_out <= CRT_VID_MAX_CLK_IN) {
                                                for (xd = 1; xd <= crt_xd_max; xd++) {
                                                    final_freq = div_post_out / xd;
                                                    if (final_freq < LCD_VENC_MAX_CLK_IN) {
                                                        if (final_freq < iflogic_vid_clk_in_max) {
                                                            error = error_abs(final_freq, fout);
                                                            if (error < min_error) {
                                                                min_error = error;
                                                                pll_m = m;
                                                                pll_n = n;
                                                                pll_od = od_sel - 1;
                                                                vid_div_pre = pre_div_sel;
                                                                crt_xd = xd;
                                                                clk_num++;
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
            if (fout < LCD_VENC_MAX_CLK_IN) {
                for (xd = 1; xd <= crt_xd_max; xd++) {
                    div_post_out = fout * xd;
                    DBG_PRINT("div_post_out=%d, xd=%d, fout=%d\n",div_post_out, xd, fout);
                    if (div_post_out <= CRT_VID_MAX_CLK_IN) {
                        div_pre_out = div_post_out * div_post;
                        if (div_pre_out <= DIV_POST_MAX_CLK_IN) {
                            for (pre_div_sel = 0; pre_div_sel < div_pre_sel_max; pre_div_sel++) {
                                div_pre = div_pre_table[pre_div_sel];
                                fout_pll = div_pre_out * div_pre;
                                DBG_PRINT("pre_div_sel=%d, div_pre=%d, fout_pll=%d\n", pre_div_sel, div_pre, fout_pll);
                                if (fout_pll <= DIV_PRE_MAX_CLK_IN) {
                                    for (od_sel = OD_SEL_MAX; od_sel > 0; od_sel--) {
                                        od = od_table[od_sel - 1];
                                        pll_vco = fout_pll * od;
                                        DBG_PRINT("od_sel=%d, od=%d, pll_vco=%d\n", od_sel, od, pll_vco);
                                        if ((pll_vco >= PLL_VCO_MIN) && (pll_vco <= PLL_VCO_MAX)) {
                                            if ((pll_vco >= 2500000) && (pll_vco <= PLL_VCO_MAX)) {
                                                od_fb = 1;
                                                pll_level = 4;
                                            }
                                            else if ((pll_vco >= 2000000) && (pll_vco < 2500000)) {
                                                od_fb = 1;
                                                pll_level = 3;
                                            }
                                            else if ((pll_vco >= 1700000) && (pll_vco < 2000000)) {
                                                od_fb = 1;
                                                pll_level = 2;
                                            }
                                            else if ((pll_vco >= PLL_VCO_MIN) && (pll_vco < 1700000)) {
                                                od_fb = 0;
                                                pll_level = 1;
                                            }
                                            n = 1;
                                            m = pll_vco / (fin * (od_fb + 1));
                                            pll_frac = (pll_vco % (fin * (od_fb + 1))) * 4096 / (fin * (od_fb + 1));

                                            pll_m = m;
                                            pll_n = n;
                                            pll_od = od_sel - 1;
                                            vid_div_pre = pre_div_sel;
                                            crt_xd = xd;
                                            DBG_PRINT("pll_m=0x%x, pll_n=0x%x, pll_od=0x%x, vid_div_pre=0x%x, crt_xd=0x%x, pll_frac=0x%x, pll_level=%d\n",
                                                       pll_m, pll_n, pll_od, vid_div_pre, crt_xd, pll_frac, pll_level);
                                            clk_num = 1;
                                        }
                                        if (clk_num > 0)
                                            break;
                                    }
                                }
                                if (clk_num > 0)
                                    break;
                            }
                        }
                    }
                    if (clk_num > 0)
                        break;
                }
            }
#endif
            break;
        default:
            break;
    }
    if (clk_num > 0) {
        pConf->lcd_timing.pll_ctrl = (pll_od << PLL_CTRL_OD) | (pll_n << PLL_CTRL_N) | (pll_m << PLL_CTRL_M);
        pConf->lcd_timing.div_ctrl = 0x18803 | (edp_phy_div1 << DIV_CTRL_EDP_DIV1) | (edp_phy_div0 << DIV_CTRL_EDP_DIV0) | (vid_div_pre << DIV_CTRL_DIV_PRE);
        tmp = (pConf->lcd_timing.clk_ctrl & ~((0xf << CLK_CTRL_XD) | (0x7 << CLK_CTRL_LEVEL) | (0xfff << CLK_CTRL_FRAC)));
        pConf->lcd_timing.clk_ctrl = (tmp | ((crt_xd << CLK_CTRL_XD) | (pll_level << CLK_CTRL_LEVEL) | (pll_frac << CLK_CTRL_FRAC)));
    }
    else {
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
        pConf->lcd_timing.pll_ctrl = (1 << PLL_CTRL_OD) | (1 << PLL_CTRL_N) | (32 << PLL_CTRL_M);
        pConf->lcd_timing.div_ctrl = 0x18803;
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
        pConf->lcd_timing.pll_ctrl = (1 << PLL_CTRL_OD) | (1 << PLL_CTRL_N) | (50 << PLL_CTRL_M);
        pConf->lcd_timing.div_ctrl = 0x18803 | (0 << DIV_CTRL_EDP_DIV1) | (0 << DIV_CTRL_EDP_DIV0) | (1 << DIV_CTRL_DIV_PRE);
#endif
        pConf->lcd_timing.clk_ctrl = (pConf->lcd_timing.clk_ctrl & ~(0xf << CLK_CTRL_XD)) | (7 << CLK_CTRL_XD);
        printk("Out of clock range, reset to default setting!\n");
    }
}

static void lcd_sync_duration(Lcd_Config_t *pConf)
{
	unsigned m, n, od, od_fb, frac, edp_div0, edp_div1, pre_div, xd, post_div;
	unsigned h_period, v_period, sync_duration;
	unsigned lcd_clk;

	m = ((pConf->lcd_timing.pll_ctrl) >> PLL_CTRL_M) & 0x1ff;
	n = ((pConf->lcd_timing.pll_ctrl) >> PLL_CTRL_N) & 0x1f;
	od = ((pConf->lcd_timing.pll_ctrl) >> PLL_CTRL_OD) & 0x3;
	od = od_table[od];
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
	frac = 0;
	od_fb = 0;
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
	frac = ((pConf->lcd_timing.clk_ctrl) >> CLK_CTRL_FRAC) & 0xfff;
	od_fb = ((((pConf->lcd_timing.clk_ctrl) >> CLK_CTRL_LEVEL) & 0x7) > 1) ? 1 : 0;

#endif	
	pre_div = ((pConf->lcd_timing.div_ctrl) >> DIV_CTRL_DIV_PRE) & 0x7;
	pre_div = div_pre_table[pre_div];
	
	h_period = pConf->lcd_basic.h_period;
	v_period = pConf->lcd_basic.v_period;
	
	edp_div0 = 0;
	edp_div1 = 0;
	switch(pConf->lcd_basic.lcd_type) {
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
		case LCD_DIGITAL_MIPI:
			xd = ((pConf->lcd_timing.clk_ctrl) >> CLK_CTRL_XD) & 0xf;
			post_div = 1;
			break;
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
		case LCD_DIGITAL_EDP:
			edp_div0 = ((pConf->lcd_timing.div_ctrl) >> DIV_CTRL_EDP_DIV0) & 0xf;
			edp_div1 = ((pConf->lcd_timing.div_ctrl) >> DIV_CTRL_EDP_DIV1) & 0x7;
			xd = 1;
			post_div = 1;
			break;
#endif
		case LCD_DIGITAL_LVDS:
			xd = 1;
			post_div = 7;
			break;
		case LCD_DIGITAL_TTL:
			xd = ((pConf->lcd_timing.clk_ctrl) >> CLK_CTRL_XD) & 0xf;
			post_div = 1;
			break;
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
		case LCD_DIGITAL_MINILVDS:
			xd = 1;
			post_div = 6;
			break;
#endif
		default:
			xd = ((pConf->lcd_timing.clk_ctrl) >> CLK_CTRL_XD) & 0xf;
			post_div = 1;
			break;
	}
	edp_div0 = edp_div0_table[edp_div0];
	edp_div1 = edp_div1_table[edp_div1];
	
	lcd_clk = (frac * (od_fb + 1) * FIN_FREQ) / 4096;
	lcd_clk = ((m * (od_fb + 1) * FIN_FREQ + lcd_clk) / (n * od * edp_div0 * edp_div1 * pre_div * post_div * xd)) * 1000;
	pConf->lcd_timing.lcd_clk = lcd_clk;
	sync_duration = ((lcd_clk / h_period) * 100) / v_period;
	sync_duration = (sync_duration + 5) / 10;
	
	pConf->lcd_timing.sync_duration_num = sync_duration;
	pConf->lcd_timing.sync_duration_den = 10;
	printk("lcd_clk=%u.%03uMHz, frame_rate=%u.%uHz.\n\n", (lcd_clk / 1000000), ((lcd_clk / 1000) % 1000), 
			(sync_duration / pConf->lcd_timing.sync_duration_den), ((sync_duration * 10 / pConf->lcd_timing.sync_duration_den) % 10));
}

static void lcd_tcon_config(Lcd_Config_t *pConf)
{
	unsigned short hstart, hend, vstart, vend;
	unsigned short h_delay = 0;
	unsigned short h_offset = 0, v_offset = 0, vsync_h_phase=0;
	
	switch (pConf->lcd_basic.lcd_type) {
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
		case LCD_DIGITAL_MIPI:
			h_delay = MIPI_DELAY;
			break;
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
		case LCD_DIGITAL_EDP:
			h_delay = EDP_DELAY;
			break;
#endif
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
#if 0
	h_offset = (pConf->lcd_timing.h_offset & 0xffff);
	v_offset = (pConf->lcd_timing.v_offset & 0xffff);
	if ((pConf->lcd_timing.h_offset >> 31) & 1)
		pConf->lcd_timing.de_hstart = (pConf->lcd_timing.video_on_pixel + pConf->lcd_basic.h_period + h_delay + h_offset) % pConf->lcd_basic.h_period;
	else
		pConf->lcd_timing.de_hstart = (pConf->lcd_timing.video_on_pixel + pConf->lcd_basic.h_period + h_delay - h_offset) % pConf->lcd_basic.h_period;
	if ((pConf->lcd_timing.v_offset >> 31) & 1)
		pConf->lcd_timing.de_vstart = (pConf->lcd_timing.video_on_line + pConf->lcd_basic.v_period + v_offset) % pConf->lcd_basic.v_period;
	else
		pConf->lcd_timing.de_vstart = (pConf->lcd_timing.video_on_line + pConf->lcd_basic.v_period - v_offset) % pConf->lcd_basic.v_period;
	
	hstart = (pConf->lcd_timing.de_hstart + pConf->lcd_basic.h_period - pConf->lcd_timing.hsync_bp) % pConf->lcd_basic.h_period;
	hend = (pConf->lcd_timing.de_hstart + pConf->lcd_basic.h_period - pConf->lcd_timing.hsync_bp + pConf->lcd_timing.hsync_width) % pConf->lcd_basic.h_period;	
	pConf->lcd_timing.sth1_hs_addr = hstart;
	pConf->lcd_timing.sth1_he_addr = hend;
	pConf->lcd_timing.sth1_vs_addr = 0;
	pConf->lcd_timing.sth1_ve_addr = pConf->lcd_basic.v_period - 1;
	
	vsync_h_phase = (pConf->lcd_timing.vsync_h_phase & 0xffff);
	if ((pConf->lcd_timing.vsync_h_phase >> 31) & 1) //negative
		vsync_h_phase = (hstart + pConf->lcd_basic.h_period - vsync_h_phase) % pConf->lcd_basic.h_period;
	else	//positive
		vsync_h_phase = (hstart + pConf->lcd_basic.h_period + vsync_h_phase) % pConf->lcd_basic.h_period;
	pConf->lcd_timing.stv1_hs_addr = vsync_h_phase;
	pConf->lcd_timing.stv1_he_addr = vsync_h_phase;
	vstart = (pConf->lcd_timing.de_vstart + pConf->lcd_basic.v_period - pConf->lcd_timing.vsync_bp) % pConf->lcd_basic.v_period;
	vend = (pConf->lcd_timing.de_vstart + pConf->lcd_basic.v_period - pConf->lcd_timing.vsync_bp + pConf->lcd_timing.vsync_width) % pConf->lcd_basic.v_period;
	pConf->lcd_timing.stv1_vs_addr = vstart;
	pConf->lcd_timing.stv1_ve_addr = vend;

	pConf->lcd_timing.de_hstart = pConf->lcd_timing.de_hstart;
	pConf->lcd_timing.de_vstart = pConf->lcd_timing.de_vstart;
	
	pConf->lcd_timing.oeh_hs_addr = pConf->lcd_timing.de_hstart;
	pConf->lcd_timing.oeh_he_addr = (pConf->lcd_timing.de_hstart + pConf->lcd_basic.h_active) % pConf->lcd_basic.h_period;
	pConf->lcd_timing.oeh_vs_addr = pConf->lcd_timing.de_vstart;
	pConf->lcd_timing.oeh_ve_addr = (pConf->lcd_timing.de_vstart + pConf->lcd_basic.v_active - 1) % pConf->lcd_basic.v_period;
#else
    pConf->lcd_timing.video_on_pixel = pConf->lcd_basic.h_period - pConf->lcd_basic.h_active - 1 -h_delay;
    pConf->lcd_timing.video_on_line = pConf->lcd_basic.v_period - pConf->lcd_basic.v_active;

    h_offset = (pConf->lcd_timing.h_offset & 0xffff);
    v_offset = (pConf->lcd_timing.v_offset & 0xffff);
    if ((pConf->lcd_timing.h_offset >> 31) & 1)
        pConf->lcd_timing.de_hstart = (pConf->lcd_basic.h_period - pConf->lcd_basic.h_active - 1 + pConf->lcd_basic.h_period - h_offset) % pConf->lcd_basic.h_period;
    else
        pConf->lcd_timing.de_hstart = (pConf->lcd_basic.h_period - pConf->lcd_basic.h_active - 1 + h_offset) % pConf->lcd_basic.h_period;
    if ((pConf->lcd_timing.v_offset >> 31) & 1)
        pConf->lcd_timing.de_vstart = (pConf->lcd_basic.v_period - pConf->lcd_basic.v_active + pConf->lcd_basic.v_period - v_offset) % pConf->lcd_basic.v_period;
    else
        pConf->lcd_timing.de_vstart = (pConf->lcd_basic.v_period - pConf->lcd_basic.v_active + v_offset) % pConf->lcd_basic.v_period;

    hstart = (pConf->lcd_timing.de_hstart + pConf->lcd_basic.h_period - pConf->lcd_timing.hsync_bp) % pConf->lcd_basic.h_period;
    hend = (pConf->lcd_timing.de_hstart + pConf->lcd_basic.h_period - pConf->lcd_timing.hsync_bp + pConf->lcd_timing.hsync_width) % pConf->lcd_basic.h_period;	
    pConf->lcd_timing.sth1_hs_addr = hstart;
    pConf->lcd_timing.sth1_he_addr = hend;
    pConf->lcd_timing.sth1_vs_addr = 0;
    pConf->lcd_timing.sth1_ve_addr = pConf->lcd_basic.v_period - 1;

    vsync_h_phase = (pConf->lcd_timing.vsync_h_phase & 0xffff);
    if ((pConf->lcd_timing.vsync_h_phase >> 31) & 1) //negative
        vsync_h_phase = (hstart + pConf->lcd_basic.h_period - vsync_h_phase) % pConf->lcd_basic.h_period;
    else	//positive
        vsync_h_phase = (hstart + pConf->lcd_basic.h_period + vsync_h_phase) % pConf->lcd_basic.h_period;
    pConf->lcd_timing.stv1_hs_addr = vsync_h_phase;
    pConf->lcd_timing.stv1_he_addr = vsync_h_phase;
    vstart = (pConf->lcd_timing.de_vstart + pConf->lcd_basic.v_period - pConf->lcd_timing.vsync_bp) % pConf->lcd_basic.v_period;
    vend = (pConf->lcd_timing.de_vstart + pConf->lcd_basic.v_period - pConf->lcd_timing.vsync_bp + pConf->lcd_timing.vsync_width) % pConf->lcd_basic.v_period;
    pConf->lcd_timing.stv1_vs_addr = vstart;
    pConf->lcd_timing.stv1_ve_addr = vend;

    pConf->lcd_timing.oeh_hs_addr = pConf->lcd_timing.de_hstart;
    pConf->lcd_timing.oeh_he_addr = (pConf->lcd_timing.de_hstart + pConf->lcd_basic.h_active) % pConf->lcd_basic.h_period;
    pConf->lcd_timing.oeh_vs_addr = pConf->lcd_timing.de_vstart;
    pConf->lcd_timing.oeh_ve_addr = (pConf->lcd_timing.de_vstart + pConf->lcd_basic.v_active - 1) % pConf->lcd_basic.v_period;
#endif

	DBG_PRINT("sth1_hs_addr=%d, sth1_he_addr=%d, sth1_vs_addr=%d, sth1_ve_addr=%d\n", pConf->lcd_timing.sth1_hs_addr, pConf->lcd_timing.sth1_he_addr, pConf->lcd_timing.sth1_vs_addr, pConf->lcd_timing.sth1_ve_addr);
	DBG_PRINT("stv1_hs_addr=%d, stv1_he_addr=%d, stv1_vs_addr=%d, stv1_ve_addr=%d\n", pConf->lcd_timing.stv1_hs_addr, pConf->lcd_timing.stv1_he_addr, pConf->lcd_timing.stv1_vs_addr, pConf->lcd_timing.stv1_ve_addr);
	DBG_PRINT("oeh_hs_addr=%d, oeh_he_addr=%d, oeh_vs_addr=%d, oeh_ve_addr=%d\n", pConf->lcd_timing.oeh_hs_addr, pConf->lcd_timing.oeh_he_addr, pConf->lcd_timing.oeh_vs_addr, pConf->lcd_timing.oeh_ve_addr);
}

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
static void select_edp_link_config(Lcd_Config_t *pConf)
{
    unsigned bit_rate;
    unsigned lane_cap;

    bit_rate = (pConf->lcd_timing.lcd_clk / 1000) * pConf->lcd_basic.lcd_bits * 3 / 1000;    //Mbps
    pConf->lcd_control.edp_config->bit_rate = bit_rate;

    if (pConf->lcd_control.edp_config->link_user == 0) {
        if (bit_rate < EDP_TX_LINK_CAPACITY_162 * 1) {
            pConf->lcd_control.edp_config->link_rate = 0;
            pConf->lcd_control.edp_config->lane_count = 1;
        }
        else if (bit_rate < EDP_TX_LINK_CAPACITY_270 * 1) {
            pConf->lcd_control.edp_config->link_rate = 1;
            pConf->lcd_control.edp_config->lane_count = 1;
        }
        else if (bit_rate < EDP_TX_LINK_CAPACITY_162 * 2) {
            pConf->lcd_control.edp_config->link_rate = 0;
            pConf->lcd_control.edp_config->lane_count = 2;
        }
        else if (bit_rate < EDP_TX_LINK_CAPACITY_270 * 2) {
            pConf->lcd_control.edp_config->link_rate = 1;
            pConf->lcd_control.edp_config->lane_count = 2;
        }
        else if (bit_rate < EDP_TX_LINK_CAPACITY_162 * 4) {
            pConf->lcd_control.edp_config->link_rate = 0;
            pConf->lcd_control.edp_config->lane_count = 4;
        }
        else if (bit_rate < EDP_TX_LINK_CAPACITY_270 * 4) {
            pConf->lcd_control.edp_config->link_rate = 1;
            pConf->lcd_control.edp_config->lane_count = 4;
        }
        else {
            printk("Error: bit rate is out edp of support, should reduce frame rate(pixel clock)\n");
            pConf->lcd_control.edp_config->link_rate = 1;
            pConf->lcd_control.edp_config->lane_count = 4;
        }
    }
    else {
        lane_cap = (pConf->lcd_control.edp_config->link_rate == 0) ? EDP_TX_LINK_CAPACITY_162 : EDP_TX_LINK_CAPACITY_270;
        while ((bit_rate > (lane_cap * pConf->lcd_control.edp_config->lane_count)) && (pConf->lcd_control.edp_config->lane_count < 4)) {
            switch (pConf->lcd_control.edp_config->lane_count) {
                case 1:
                    pConf->lcd_control.edp_config->lane_count = 2;
                    break;
                case 2:
                    pConf->lcd_control.edp_config->lane_count = 4;
                    break;
                default:
                    break;
            }
        }
        if (bit_rate > (lane_cap * pConf->lcd_control.edp_config->lane_count))
            printk("Error: bit rate is out edp of support, should reduce frame rate(pixel clock)\n");
    }
}
#endif

static void lcd_control_config_pre(Lcd_Config_t *pConf)
{
    unsigned vclk_sel, ss_level;

    vclk_sel = 1;
    ss_level = (pConf->lcd_timing.clk_ctrl >> CLK_CTRL_SS) & 0xf;

    switch (pConf->lcd_basic.lcd_type) {
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
        case LCD_DIGITAL_MIPI:
            ss_level = ((ss_level > 0) ? 1 : 0);
            set_mipi_dsi_control_config(pConf);
            break;
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
        case LCD_DIGITAL_EDP:
            ss_level = ((ss_level > 0) ? 1 : 0);
            select_edp_link_config(pConf);
            if (pConf->lcd_control.edp_config->link_adaptive == 1) {
                pConf->lcd_control.edp_config->vswing = 0;
                pConf->lcd_control.edp_config->preemphasis = 0;
            }
            printk("edp vswing=0x%x, preem=0x%x\n", pConf->lcd_control.edp_config->vswing, pConf->lcd_control.edp_config->preemphasis);
            break;
#endif
        case LCD_DIGITAL_LVDS:
            ss_level = ((ss_level >= SS_LEVEL_MAX) ? (SS_LEVEL_MAX-1) : ss_level);
            if (pConf->lcd_control.lvds_config->lvds_repack_user == 0) {
                if (pConf->lcd_basic.lcd_bits == 6)
                    pConf->lcd_control.lvds_config->lvds_repack = 0;
                else
                    pConf->lcd_control.lvds_config->lvds_repack = 1;
            }
            break;
        case LCD_DIGITAL_TTL:
            ss_level = ((ss_level >= SS_LEVEL_MAX) ? (SS_LEVEL_MAX-1) : ss_level);
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B)
            if (pConf->lcd_basic.lcd_bits != 6) {
                pConf->lcd_basic.lcd_bits = 6;
                printk("lcd change to 6bit for ttl support!\n");
            }
#endif
            break;
        default:
            ss_level = ((ss_level >= SS_LEVEL_MAX) ? (SS_LEVEL_MAX-1) : ss_level);
            break;
    }
    pConf->lcd_timing.clk_ctrl &= (~((1 << CLK_CTRL_VCLK_SEL) | (0xf << CLK_CTRL_SS)));
    pConf->lcd_timing.clk_ctrl |= ((vclk_sel << CLK_CTRL_VCLK_SEL) | (ss_level << CLK_CTRL_SS));
}

//for special interface config after clk setting
static void lcd_control_config_post(Lcd_Config_t *pConf)
{
    switch (pConf->lcd_basic.lcd_type) {
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
        case LCD_DIGITAL_MIPI:
            set_mipi_dsi_control_config_post(pConf);
            break;
#endif
        default:
            break;
    }
}

static void lcd_config_init(Lcd_Config_t *pConf)
{
	lcd_control_config_pre(pConf);//must before generate_clk_parameter, otherwise the clk parameter will not update base on the edp_link_rate
	
	if (pConf->lcd_timing.clk_ctrl & (1 << CLK_CTRL_AUTO)) {
		printk("\nAuto generate clock parameters.\n");
		generate_clk_parameter(pConf);
		DBG_PRINT("pll_ctrl=0x%x, div_ctrl=0x%x, clk_ctrl=0x%x.\n", pConf->lcd_timing.pll_ctrl, pConf->lcd_timing.div_ctrl, pConf->lcd_timing.clk_ctrl);
	}
	else {
		printk("\nCustome clock parameters.\n");
		printk("pll_ctrl=0x%x, div_ctrl=0x%x, clk_ctrl=0x%x.\n", pConf->lcd_timing.pll_ctrl, pConf->lcd_timing.div_ctrl, pConf->lcd_timing.clk_ctrl);
	}
	
	lcd_sync_duration(pConf);
	lcd_tcon_config(pConf);

	lcd_control_config_post(pConf);
}

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
static void switch_lcd_gates(Lcd_Type_t lcd_type)
{
	switch(lcd_type){
		case LCD_DIGITAL_TTL:
			switch_mod_gate_by_name("tcon", 1);
			switch_mod_gate_by_name("lvds", 0);
			break;
		case LCD_DIGITAL_LVDS:
		case LCD_DIGITAL_MINILVDS:
			switch_mod_gate_by_name("lvds", 1);
			switch_mod_gate_by_name("tcon", 0);
			break;
		default:
			break;
	}
}
#endif

static void _init_lcd_driver(Lcd_Config_t *pConf)
{
    int lcd_type = pConf->lcd_basic.lcd_type;
    unsigned char ss_level = (pConf->lcd_timing.clk_ctrl >> CLK_CTRL_SS) & 0xf;

    printk("lcd driver version: %s%s\n\n", DRIVER_DATE, DRV_TYPE);
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
    switch_lcd_gates(lcd_type);
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
    switch_vpu_mem_pd_vmod(pDev->lcd_info.mode, VPU_MEM_POWER_ON);
    switch_lcd_mod_gate(ON);
#endif

    printk("Init LCD mode: %s, %s(%u) %ubit, %ux%u@%u.%uHz, ss_level=%u(%s)\n", pConf->lcd_basic.model_name, lcd_type_table[lcd_type], lcd_type, pConf->lcd_basic.lcd_bits, pConf->lcd_basic.h_active, pConf->lcd_basic.v_active, (pConf->lcd_timing.sync_duration_num / 10), (pConf->lcd_timing.sync_duration_num % 10), ss_level, lcd_ss_level_table[ss_level]);

    switch(lcd_type){
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
        case LCD_DIGITAL_MIPI:
            set_pll_lcd(pConf);
            set_venc_lcd(pConf);
            set_tcon_lcd(pConf);
            init_dphy(pConf);
            break;
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
        case LCD_DIGITAL_EDP:
            set_pll_lcd(pConf);
            set_venc_lcd(pConf);
            set_tcon_lcd(pConf);
            init_dphy(pConf);
            break;
#endif
        case LCD_DIGITAL_LVDS:
            set_pll_lcd(pConf);
            set_venc_lcd(pConf);
            set_tcon_lcd(pConf);
            set_control_lvds(pConf);
            init_dphy(pConf);
            break;
        case LCD_DIGITAL_TTL:
            set_pll_lcd(pConf);
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
            set_venc_ttl(pConf);
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
            set_venc_lcd(pConf);
#endif
            set_tcon_ttl(pConf);
            set_control_ttl(pConf);
            break;
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
        case LCD_DIGITAL_MINILVDS:
            set_pll_mlvds(pConf);
            set_venc_mlvds(pConf);
            set_tcon_mlvds(pConf);
            set_control_mlvds(pConf);
            init_dphy(pConf);
            break;
#endif
        default:
            printk("Invalid LCD type.\n");
            break;
    }
    set_video_adjust(pConf);
    printk("%s finished.\n", __FUNCTION__);
}

static void _disable_lcd_driver(Lcd_Config_t *pConf)
{
    int vclk_sel;

    switch(pConf->lcd_basic.lcd_type){
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
        case LCD_DIGITAL_MIPI:
            mipi_dsi_off();
            break;
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
        case LCD_DIGITAL_EDP:
            dplpm_off();
            break;
#endif
        case LCD_DIGITAL_LVDS:
        case LCD_DIGITAL_TTL:
        case LCD_DIGITAL_MINILVDS:
        default:
            break;
    }

    vclk_sel = ((pConf->lcd_timing.clk_ctrl) >> CLK_CTRL_VCLK_SEL) & 0x1;

    WRITE_LCD_CBUS_REG_BITS(HHI_VIID_DIVIDER_CNTL, 0, 11, 1);	//close lvds phy clk gate: 0x104c[11]
    WRITE_LCD_REG_BITS(LVDS_GEN_CNTL, 0, 3, 1);	//disable lvds fifo

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
    WRITE_LCD_REG(ENCT_VIDEO_EN, 0);	//disable enct
#endif
    WRITE_LCD_REG(ENCL_VIDEO_EN, 0);	//disable encl

    if (vclk_sel)
        WRITE_LCD_CBUS_REG_BITS(HHI_VIID_CLK_CNTL, 0, 0, 5);	//close vclk2 gate: 0x104b[4:0]
    else
        WRITE_LCD_CBUS_REG_BITS(HHI_VID_CLK_CNTL, 0, 0, 5);		//close vclk1 gate: 0x105f[4:0]

    WRITE_LCD_CBUS_REG_BITS(HHI_VIID_DIVIDER_CNTL, 0, 16, 1);	//close vid2_pll gate: 0x104c[16]

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
    WRITE_LCD_CBUS_REG_BITS(HHI_VID2_PLL_CNTL5, 0, 23, 3);	//disable pll_out mux
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
    WRITE_LCD_CBUS_REG_BITS(HHI_VIID_PLL_CNTL, 1, 30, 1);		//power down vid2_pll: 0x1047[30]
#elif (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
    WRITE_LCD_CBUS_REG_BITS(HHI_VID2_PLL_CNTL, 0, 30, 1);		//disable vid2_pll: 0x10e0[30]
#elif (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B)
    WRITE_LCD_CBUS_REG_BITS(HHI_VID_PLL_CNTL, 0, 30, 1);		//disable vid_pll: 0x10c8[30]
#endif

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
    switch_mod_gate_by_name("tcon", 0);
    switch_mod_gate_by_name("lvds", 0);
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
    //switch_lcd_gates(OFF);
    switch_lcd_mod_gate(OFF);
    switch_vpu_mem_pd_vmod(pDev->lcd_info.mode, VPU_MEM_POWER_DOWN);
#endif
    printk("disable lcd display driver.\n");
}

static inline void _enable_vsync_interrupt(void)
{
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
	if ((READ_LCD_REG(ENCT_VIDEO_EN) & 1) || (READ_LCD_REG(ENCL_VIDEO_EN) & 1)) {
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
	if (READ_LCD_REG(ENCL_VIDEO_EN) & 1) {
#endif
		WRITE_LCD_REG(VENC_INTCTRL, 0x200);
	}
	else{
		WRITE_LCD_REG(VENC_INTCTRL, 0x2);
	}
}

void _enable_backlight(void)
{
	backlight_power_ctrl(ON);
}
void _disable_backlight(void)
{
	backlight_power_ctrl(OFF);
}

static DEFINE_MUTEX(lcd_init_mutex);
static void _lcd_module_enable(void)
{
	int ret = 0;

	mutex_lock(&lcd_init_mutex);
	BUG_ON(pDev==NULL);

	_init_lcd_driver(pDev->pConf);
	ret = lcd_power_ctrl(ON);
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
	if (pDev->pConf->lcd_basic.lcd_type == LCD_DIGITAL_EDP) {
		if (ret > 0) {
			lcd_power_ctrl(OFF);
			_disable_lcd_driver(pDev->pConf);
			mdelay(30);
			_init_lcd_driver(pDev->pConf);
			lcd_power_ctrl(ON);
		}
	}
#endif
	data_status = ON;
	_enable_vsync_interrupt();
	lcd_status_flag = 1;
	mutex_unlock(&lcd_init_mutex);
}

static void _lcd_module_disable(void)
{
	mutex_lock(&lcd_init_mutex);
	lcd_status_flag = 0;
	BUG_ON(pDev==NULL);
	data_status = OFF;
	lcd_power_ctrl(OFF);
	_disable_lcd_driver(pDev->pConf);
	mutex_unlock(&lcd_init_mutex);
}

static const vinfo_t *lcd_get_current_info(void)
{
    if (pDev == NULL) {
        printk("[error] no lcd device exist!\n");
        return NULL;
    }
    else 
        return &pDev->lcd_info;
}

static DEFINE_MUTEX(lcd_vout_mutex);
static int lcd_set_current_vmode(vmode_t mode)
{
	mutex_lock(&lcd_vout_mutex);
	if (VMODE_LCD != (mode & VMODE_MODE_BIT_MASK)) {
		mutex_unlock(&lcd_vout_mutex);
		return -EINVAL;
	}

	vpp2_sel = 0;
	WRITE_LCD_REG(VPP_POSTBLEND_H_SIZE, pDev->lcd_info.width);

	if( !(mode&VMODE_LOGO_BIT_MASK) ){
		_disable_backlight();
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
		request_vpu_clk_vmod(pDev->lcd_info.video_clk, pDev->lcd_info.mode);
#endif
		_lcd_module_enable();
		_enable_backlight();
	}
	if (VMODE_INIT_NULL == pDev->lcd_info.mode)
		pDev->lcd_info.mode = VMODE_LCD;
	
	mutex_unlock(&lcd_vout_mutex);
	return 0;
}

#ifdef CONFIG_AM_TV_OUTPUT2
static int lcd_set_current_vmode2(vmode_t mode)
{
	mutex_lock(&lcd_vout_mutex);
	if (mode != VMODE_LCD) {
		mutex_unlock(&lcd_vout_mutex);
		return -EINVAL;
	}
	_disable_backlight();
    vpp2_sel = 1;

    WRITE_LCD_REG(VPP2_POSTBLEND_H_SIZE, pDev->lcd_info.width);
	
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
	request_vpu_clk_vmod(pDev->lcd_info.video_clk, pDev->lcd_info.mode);
#endif
    _lcd_module_enable();
    if (VMODE_INIT_NULL == pDev->lcd_info.mode)
        pDev->lcd_info.mode = VMODE_LCD;
    _enable_backlight();
	mutex_unlock(&lcd_vout_mutex);
    return 0;
}
#endif

static vmode_t lcd_validate_vmode(char *mode)
{
    if ((strncmp(mode, PANEL_NAME, strlen(PANEL_NAME))) == 0)
        return VMODE_LCD;
    
    return VMODE_MAX;
}
static int lcd_vmode_is_supported(vmode_t mode)
{
    mode&=VMODE_MODE_BIT_MASK;
    if(mode == VMODE_LCD )
    return true;
    return false;
}

static int lcd_vout_disable(vmode_t cur_vmod)
{
	mutex_lock(&lcd_vout_mutex);
	_disable_backlight();
	_lcd_module_disable();
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
	release_vpu_clk_vmod(pDev->lcd_info.mode);
#endif
	mutex_unlock(&lcd_vout_mutex);
	return 0;
}

#ifdef  CONFIG_PM
static int lcd_suspend(void)
{
	mutex_lock(&lcd_vout_mutex);
	BUG_ON(pDev==NULL);
	printk("lcd_suspend\n");
	_disable_backlight();
	_lcd_module_disable();
	mutex_unlock(&lcd_vout_mutex);
	return 0;
}
static int lcd_resume(void)
{
	mutex_lock(&lcd_vout_mutex);
	printk("lcd_resume\n");
    _lcd_module_enable();
    _enable_backlight();
	mutex_unlock(&lcd_vout_mutex);
    return 0;
}
#endif
static vout_server_t lcd_vout_server={
    .name = "lcd_vout_server",
    .op = {    
        .get_vinfo = lcd_get_current_info,
        .set_vmode = lcd_set_current_vmode,
        .validate_vmode = lcd_validate_vmode,
        .vmode_is_supported=lcd_vmode_is_supported,
        .disable=lcd_vout_disable,
#ifdef  CONFIG_PM
        .vout_suspend=lcd_suspend,
        .vout_resume=lcd_resume,
#endif
    },
};

#ifdef CONFIG_AM_TV_OUTPUT2
static vout_server_t lcd_vout2_server={
    .name = "lcd_vout2_server",
    .op = {    
        .get_vinfo = lcd_get_current_info,
        .set_vmode = lcd_set_current_vmode2,
        .validate_vmode = lcd_validate_vmode,
        .vmode_is_supported=lcd_vmode_is_supported,
        .disable=lcd_vout_disable,
#ifdef  CONFIG_PM  
        .vout_suspend=lcd_suspend,
        .vout_resume=lcd_resume,
#endif
    },
};
#endif
static void _init_vout(lcd_dev_t *pDev)
{
    pDev->lcd_info.name = PANEL_NAME;
    pDev->lcd_info.mode = VMODE_LCD;
    pDev->lcd_info.width = pDev->pConf->lcd_basic.h_active;
    pDev->lcd_info.height = pDev->pConf->lcd_basic.v_active;
    pDev->lcd_info.field_height = pDev->pConf->lcd_basic.v_active;
    pDev->lcd_info.aspect_ratio_num = pDev->pConf->lcd_basic.screen_ratio_width;
    pDev->lcd_info.aspect_ratio_den = pDev->pConf->lcd_basic.screen_ratio_height;
    pDev->lcd_info.screen_real_width= pDev->pConf->lcd_basic.h_active_area;
    pDev->lcd_info.screen_real_height= pDev->pConf->lcd_basic.v_active_area;
    pDev->lcd_info.sync_duration_num = pDev->pConf->lcd_timing.sync_duration_num;
    pDev->lcd_info.sync_duration_den = pDev->pConf->lcd_timing.sync_duration_den;
    pDev->lcd_info.video_clk = pDev->pConf->lcd_timing.lcd_clk;
       
    //add lcd actual active area size
    printk("lcd actual active area size: %d %d (mm).\n", pDev->pConf->lcd_basic.h_active_area, pDev->pConf->lcd_basic.v_active_area);
    vout_register_server(&lcd_vout_server);
#ifdef CONFIG_AM_TV_OUTPUT2
    vout2_register_server(&lcd_vout2_server);
#endif   
}

static void _lcd_init(Lcd_Config_t *pConf)
{
	//logo_object_t  *init_logo_obj=NULL;
	_init_vout(pDev);
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
	request_vpu_clk_vmod(pDev->lcd_info.video_clk, pDev->lcd_info.mode);
#endif
	//init_logo_obj = get_current_logo_obj();    
	//if(NULL==init_logo_obj ||!init_logo_obj->para.loaded)
		//_lcd_module_enable();
}

static int lcd_reboot_notifier(struct notifier_block *nb, unsigned long state, void *cmd)
 {
	printk("[%s]: %lu\n", __FUNCTION__, state);
	_disable_backlight();
	_lcd_module_disable();

    return NOTIFY_DONE;
}

//****************************
//gamma debug
//****************************
#ifdef CONFIG_AML_GAMMA_DEBUG
static unsigned short gamma_adjust_r[256];
static unsigned short gamma_adjust_g[256];
static unsigned short gamma_adjust_b[256];
static unsigned short gamma_r_coeff, gamma_g_coeff, gamma_b_coeff;

static void save_original_gamma(Lcd_Config_t *pConf)
{
	int i;
	
	for (i=0; i<256; i++) {
        gamma_adjust_r[i] = pConf->lcd_effect.GammaTableR[i];
        gamma_adjust_g[i] = pConf->lcd_effect.GammaTableG[i];
		gamma_adjust_b[i] = pConf->lcd_effect.GammaTableB[i];
    }
	gamma_r_coeff = pConf->lcd_effect.gamma_r_coeff;
	gamma_g_coeff = pConf->lcd_effect.gamma_g_coeff;
	gamma_b_coeff = pConf->lcd_effect.gamma_b_coeff;
}

static void read_original_gamma_table(void)
{
    unsigned i;
	
	printk("original gamma r_coeff=%u%%, g_coeff=%u%%, b_coeff=%u%%\n", gamma_r_coeff, gamma_g_coeff, gamma_b_coeff);
    printk("read original gamma table R:\n");
    for (i=0; i<256; i++) {
        printk("%u,", gamma_adjust_r[i]);
    }
    printk("\n\nread original gamma table G:\n");
    for (i=0; i<256; i++) {
        printk("%u,", gamma_adjust_g[i]);
    }
    printk("\n\nread original gamma table B:\n");
    for (i=0; i<256; i++) {
        printk("%u,", gamma_adjust_b[i]);
    }
    printk("\n");
}

static void read_current_gamma_table(void)
{
	unsigned i;
	
	printk("current gamma r_coeff=%u%%, g_coeff=%u%%, b_coeff=%u%%\n", pDev->pConf->lcd_effect.gamma_r_coeff, pDev->pConf->lcd_effect.gamma_g_coeff, pDev->pConf->lcd_effect.gamma_b_coeff);
	printk("read current gamma table R:\n");
    for (i=0; i<256; i++) {
        printk("%u ", pDev->pConf->lcd_effect.GammaTableR[i]);
    }
    printk("\n\nread current gamma table G:\n");
    for (i=0; i<256; i++) {
        printk("%u ", pDev->pConf->lcd_effect.GammaTableG[i]);
    }
    printk("\n\nread current gamma table B:\n");
    for (i=0; i<256; i++) {
        printk("%u ", pDev->pConf->lcd_effect.GammaTableB[i]);
    }
    printk("\n");
}

static void write_gamma_table(void)
{
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
    if (pDev->pConf->lcd_basic.lcd_type == LCD_DIGITAL_TTL) {
		WRITE_LCD_REG_BITS(GAMMA_CNTL_PORT, 0, 1, 1);
        set_gamma_table_ttl(pDev->pConf->lcd_effect.GammaTableR, LCD_H_SEL_R, pDev->pConf->lcd_effect.gamma_r_coeff);
        set_gamma_table_ttl(pDev->pConf->lcd_effect.GammaTableG, LCD_H_SEL_G, pDev->pConf->lcd_effect.gamma_g_coeff);
        set_gamma_table_ttl(pDev->pConf->lcd_effect.GammaTableB, LCD_H_SEL_B, pDev->pConf->lcd_effect.gamma_b_coeff);
		WRITE_LCD_REG_BITS(GAMMA_CNTL_PORT, 1, 1, 1);
    }
    else {
        WRITE_LCD_REG_BITS(L_GAMMA_CNTL_PORT, 0, 1, 1);
        set_gamma_table_lcd(pDev->pConf->lcd_effect.GammaTableR, LCD_H_SEL_R, pDev->pConf->lcd_effect.gamma_r_coeff);
        set_gamma_table_lcd(pDev->pConf->lcd_effect.GammaTableG, LCD_H_SEL_G, pDev->pConf->lcd_effect.gamma_g_coeff);
        set_gamma_table_lcd(pDev->pConf->lcd_effect.GammaTableB, LCD_H_SEL_B, pDev->pConf->lcd_effect.gamma_b_coeff);
        WRITE_LCD_REG_BITS(L_GAMMA_CNTL_PORT, 1, 1, 1);
    }
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
	WRITE_LCD_REG_BITS(L_GAMMA_CNTL_PORT, 0, 1, 1);
	set_gamma_table_lcd(pDev->pConf->lcd_effect.GammaTableR, LCD_H_SEL_R, pDev->pConf->lcd_effect.gamma_r_coeff);
	set_gamma_table_lcd(pDev->pConf->lcd_effect.GammaTableG, LCD_H_SEL_G, pDev->pConf->lcd_effect.gamma_g_coeff);
	set_gamma_table_lcd(pDev->pConf->lcd_effect.GammaTableB, LCD_H_SEL_B, pDev->pConf->lcd_effect.gamma_b_coeff);
	WRITE_LCD_REG_BITS(L_GAMMA_CNTL_PORT, 1, 1, 1);
#endif
	printk("write gamma table ");
}

static void set_gamma_coeff(unsigned r_coeff, unsigned g_coeff, unsigned b_coeff)
{	    
	pDev->pConf->lcd_effect.gamma_r_coeff = (unsigned short)(r_coeff);
	pDev->pConf->lcd_effect.gamma_g_coeff = (unsigned short)(g_coeff);
	pDev->pConf->lcd_effect.gamma_b_coeff = (unsigned short)(b_coeff);
	write_gamma_table();
	printk("with scale factor R:%u%%, G:%u%%, B:%u%%.\n", (unsigned short)(r_coeff), (unsigned short)(g_coeff), (unsigned short)(b_coeff));
}

static const char * usage_str =
{"Usage:\n"
"    echo coeff <R_coeff> <G_coeff> <B_coeff> > write ; set R,G,B gamma scale factor\n"
"data format:\n"
"    <R/G/B_coeff>  : a number in Dec(0~100), means a percent value\n"
"\n"
"    echo [r|g|b] <step> <value> <value> <value> <value> <value> <value> <value> <value> > write ; input R/G/B gamma table\n"
"    echo w [0 | 8 | 10] > write ; apply the original/8bit/10bit gamma table\n"
"data format:\n"
"    <step>  : 0xX, 4bit in Hex, there are 8 steps(0~7, 8bit gamma) or 16 steps(0~f, 10bit gamma) for a single cycle\n"
"    <value> : 0xXXXXXXXX, 32bit in Hex, 2 or 4 gamma table values (8 or 10bit gamma) combia in one <value>\n"
"\n"
"    echo f[r | g | b | w] <level_value> > write ; write R/G/B/white gamma level with fixed level_value\n"
"data format:\n"
"    <level_value>  : a number in Dec(0~255)\n"
"\n"
"    echo [0 | 1] > read ; readback original/current gamma table\n"
};

static ssize_t gamma_help(struct class *class, struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n",usage_str);
}

static ssize_t aml_lcd_gamma_read(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	if (buf[0] == '0')
		read_original_gamma_table();
	else
		read_current_gamma_table();

	return count;
}

static unsigned gamma_adjust_r_temp[128];
static unsigned gamma_adjust_g_temp[128];
static unsigned gamma_adjust_b_temp[128];
static ssize_t aml_lcd_gamma_debug(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int i, j;
	unsigned t[8];

    switch (buf[0]) {
    case 'c':
        t[0] = 100;
        t[1] = 100;
        t[2] = 100;
        ret = sscanf(buf, "coeff %u %u %u", &t[0], &t[1], &t[2]);
        set_gamma_coeff(t[0], t[1], t[2]);
        break;
    case 'r':
        ret = sscanf(buf, "r %x %x %x %x %x %x %x %x %x", &i, &t[0], &t[1], &t[2], &t[3], &t[4], &t[5], &t[6], &t[7]);
        if (i<16) {
            i =  i * 8;
            for (j=0; j<8; j++) {
                gamma_adjust_r_temp[i+j] = t[j];
            }
            printk("write R table: step %u.\n", i/8);
        }
        break;
    case 'g':
        ret = sscanf(buf, "g %x %x %x %x %x %x %x %x %x", &i, &t[0], &t[1], &t[2], &t[3], &t[4], &t[5], &t[6], &t[7]);
        if (i<16) {
            i =  i * 8;
            for (j=0; j<8; j++) {
                gamma_adjust_g_temp[i+j] = t[j];
            }
            printk("write G table: step %u.\n", i/8);
        }
        break;
    case 'b':
        ret = sscanf(buf, "b %x %x %x %x %x %x %x %x %x", &i, &t[0], &t[1], &t[2], &t[3], &t[4], &t[5], &t[6], &t[7]);
        if (i<16) {
            i =  i * 8;
            for (j=0; j<8; j++) {
                gamma_adjust_b_temp[i+j] = t[j];
            }
            printk("write B table: step %u.\n", i/8);
        }
        break;
    case 'w':
        i = 0;
        ret = sscanf(buf, "w %u", &i);
        if (i == 8) {
            for (i=0; i<64; i++) {
                for (j=0; j<4; j++){
                    pDev->pConf->lcd_effect.GammaTableR[i*4+j] = (unsigned short)(((gamma_adjust_r_temp[i] >> (24-j*8)) & 0xff) << 2);
                    pDev->pConf->lcd_effect.GammaTableG[i*4+j] = (unsigned short)(((gamma_adjust_g_temp[i] >> (24-j*8)) & 0xff) << 2);
                    pDev->pConf->lcd_effect.GammaTableB[i*4+j] = (unsigned short)(((gamma_adjust_b_temp[i] >> (24-j*8)) & 0xff) << 2);
                }
            }
            write_gamma_table();
            printk("8bit finished.\n");
        }
        else if (i == 10) {
            for (i=0; i<128; i++) {
                for (j=0; j<2; j++){
                    pDev->pConf->lcd_effect.GammaTableR[i*2+j] = (unsigned short)((gamma_adjust_r_temp[i] >> (16-j*16)) & 0xffff);
                    pDev->pConf->lcd_effect.GammaTableG[i*2+j] = (unsigned short)((gamma_adjust_g_temp[i] >> (16-j*16)) & 0xffff);
                    pDev->pConf->lcd_effect.GammaTableB[i*2+j] = (unsigned short)((gamma_adjust_b_temp[i] >> (16-j*16)) & 0xffff);
                }
            }
            write_gamma_table();
            printk("10bit finished.\n");
        }
        else {
            for (i=0; i<256; i++) {
                pDev->pConf->lcd_effect.GammaTableR[i] = gamma_adjust_r[i];
                pDev->pConf->lcd_effect.GammaTableG[i] = gamma_adjust_g[i];
                pDev->pConf->lcd_effect.GammaTableB[i] = gamma_adjust_b[i];
            }
            write_gamma_table();
            printk("to original.\n");
        }
        break;
    case 'f':
        i=255;
        if (buf[1] == 'r') {
            ret = sscanf(buf, "fr %u", &i);
            i &= 0xff;
            for (j=0; j<256; j++) {
                pDev->pConf->lcd_effect.GammaTableR[j] = i<<2;
            }
            set_gamma_coeff(100, 0, 0);
            printk("with R fixed value %u finished.\n", i);
        }
        else if (buf[1] == 'g') {
            ret = sscanf(buf, "fg %u", &i);
            i &= 0xff; 
            for (j=0; j<256; j++) {
                pDev->pConf->lcd_effect.GammaTableG[j] = i<<2;
            }
            set_gamma_coeff(0, 100, 0);
            printk("with G fixed value %u finished.\n", i);
        }
        else if (buf[1] == 'b') {
            ret = sscanf(buf, "fb %u", &i);
            i &= 0xff;
            for (j=0; j<256; j++) {
                pDev->pConf->lcd_effect.GammaTableB[j] = i<<2;
            }
            set_gamma_coeff(0, 0, 100);
            printk("with B fixed value %u finished.\n", i);
        }
        else {
            ret = sscanf(buf, "fw %u", &i);
            i &= 0xff;
            for (j=0; j<256; j++) {
                pDev->pConf->lcd_effect.GammaTableR[j] = i<<2;
                pDev->pConf->lcd_effect.GammaTableG[j] = i<<2;
                pDev->pConf->lcd_effect.GammaTableB[j] = i<<2;
            }
            set_gamma_coeff(100, 100, 100);
            printk("with fixed value %u finished.\n", i);
        }
        break;
        default:
            printk("wrong format of gamma table writing.\n");
    }

	if (ret != 1 || ret !=2)
		return -EINVAL;

	return count;
	//return 0;
}

static struct class_attribute aml_lcd_class_attrs[] = {
	__ATTR(write,  S_IRUGO | S_IWUSR, gamma_help, aml_lcd_gamma_debug),
	__ATTR(read,  S_IRUGO | S_IWUSR, gamma_help, aml_lcd_gamma_read),
	__ATTR(help,  S_IRUGO | S_IWUSR, gamma_help, NULL),
    __ATTR_NULL
};

static struct class aml_gamma_class = {
    .name = "gamma",
    .class_attrs = aml_lcd_class_attrs,
};
#endif
//****************************

//****************************
//LCD debug
//****************************
static Lcd_Basic_t temp_lcd_basic;
static Lcd_Timing_t temp_lcd_timing;
static unsigned short temp_dith_user, temp_dith_ctrl;
static unsigned int temp_vadj_brightness, temp_vadj_contrast, temp_vadj_saturation;
static int temp_ttl_rb_swap, temp_ttl_bit_swap;
static int temp_lvds_repack, temp_pn_swap, temp_lvds_vswing;
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
static unsigned char temp_dsi_lane_num;
static unsigned temp_dsi_bit_rate_min, temp_dsi_bit_rate_max, temp_factor_denominator, temp_factor_numerator;
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
static unsigned char temp_edp_link_rate, temp_edp_lane_count, temp_edp_vswing, temp_edp_preemphasis;
#endif
static unsigned short last_h_active, last_v_active;

static const char * lcd_usage_str =
{"Usage:\n"
"    echo basic <h_active> <v_active> <h_period> <v_period> > debug ; write lcd basic config\n"
"    echo type <lcd_type> <lcd_bits> > debug ; write lcd type & bits\n"
"    echo clock <lcd_clk> <ss_level> <clk_pol> > debug ; write lcd clk (Hz)\n"
"    echo sync <hs_width> <hs_backporch> <hs_pol> <vs_width> <vs_backporch> <vs_pol> > debug ; write lcd sync timing\n"
"    echo valid <hvsync_valid> <de_valid> > debug ; enable lcd sync signals\n"
"data format:\n"
"    <lcd_type> : 0=mipi, 1=lvds, 2=edp, 3=ttl\n"
"    <lcd_bits> : 6=6bit(RGB18bit), 8=8bit(RGB24bit)\n"
"    <ss_level> : lcd clock spread spectrum level, 0~5, 0 for disable\n"
"    <xx_pol>   : 0=negative, 1=positive\n"
"    <xx_valid> : 0=disable, 1=enable\n"
"\n"
"    echo ttl <rb_swap> <bit_swap> > debug ; write ttl RGB swap config\n"
"    echo lvds <vswing_level> <lvds_repack> <pn_swap> > debug ; write lvds config\n"
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
"    echo mdsi <bit_rate_min> <bit_rate_max> <factor> > debug ; write mipi-dsi config\n"
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
"    echo edp <link_rate> <lane_count> <vswing_level> > debug ; write edp config\n"
#endif
"\n"
"data format:\n"
"    <xx_swap>      : 0=normal, 1=swap\n"
"    <vswing_level> : lvds support 5 levels (0,1,2,3,4. Default=1). edp support level 0,1,2,3.\n"
"    <lvds_repack>  : 0=JEIDA mode, 1=VESA mode\n"
"    <pn_swap>      : 0=normal, 1=swap lvds p/n channels\n"
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
"    <bit_rate_xxx> : unit in MHz\n"
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
"    <link_rate>    : 0=1.62G, 1=2.7G\n"
#endif
"\n"
"    echo offset <h_sign> <h_offset> <v_sign> <v_offset> > debug ; write ttl display offset\n"
"    echo dither <dither_user> <dither_ctrl> > debug ; write user dither ctrl config\n"
"    echo vadj <brightness> <contrast> <saturation> > debug ; write video adjust config\n"
"data format:\n"
"    <xx_sign>     : 0=positive, 1=negative\n"
"    <dither_user> : 0=disable user control, 1=enable user control\n"
"    <dither_ctrl> : dither ctrl in Hex, such as 0x400 or 0x600\n"
"    <brightness>  : negative 0x1ff~0x101, positive 0x0~0xff, signed value in Hex, default is 0x0\n"
"    <contrast>    : 0x0~0xff, unsigned value in Hex, default is 0x80\n"
"    <saturation>  : 0x0~0x1ff, unsigned value in Hex, default is 0x100\n"
"\n"
"    echo write > debug ; update lcd driver\n"
"    echo reset > debug ; reset lcd config & driver\n"
"    echo read > debug ; read current lcd config\n"
"    echo test <num> > debug ; bist pattern test, 0=pattern off, 1,2,3=different pattern \n"
"\n"
"    echo disable > debug ; power off lcd \n"
"    echo enable > debug ; power on lcd \n"
};

static ssize_t lcd_debug_help(struct class *class, struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n",lcd_usage_str);
}

static void read_current_lcd_config(Lcd_Config_t *pConf)
{
    unsigned lcd_clk;
    int h_adj, v_adj;

    lcd_clk = (pConf->lcd_timing.lcd_clk / 1000);
    h_adj = ((pConf->lcd_timing.h_offset >> 31) & 1);
    v_adj = ((pConf->lcd_timing.v_offset >> 31) & 1);

    printk("lcd driver version: %s%s\n\n", DRIVER_DATE, DRV_TYPE);
    printk("LCD mode: %s, %s %ubit, %ux%u@%u.%uHz\n"
           "lcd_clk           %u.%03uMHz\n"
           "ss_level          %d\n"
           "clk_pol           %d\n\n",
           pConf->lcd_basic.model_name, lcd_type_table[pConf->lcd_basic.lcd_type], pConf->lcd_basic.lcd_bits, pConf->lcd_basic.h_active, pConf->lcd_basic.v_active,
           (pConf->lcd_timing.sync_duration_num / 10), (pConf->lcd_timing.sync_duration_num % 10),
           (lcd_clk / 1000), (lcd_clk % 1000), ((pConf->lcd_timing.clk_ctrl >> CLK_CTRL_SS) & 0xf), ((pConf->lcd_timing.pol_cntl_addr >> LCD_CPH1_POL) & 1));

    printk("h_period          %d\n"
           "v_period          %d\n"
           "hs_width          %d\n"
           "hs_backporch      %d\n"
           "hs_pol            %d\n"
           "vs_width          %d\n"
           "vs_backporch      %d\n"
           "vs_pol            %d\n"
           "vs_h_phase        %s%d\n"
           "hvsync_valid      %d\n"
           "de_valid          %d\n"
           "h_offset          %s%d\n"
           "v_offset          %s%d\n\n",
           pConf->lcd_basic.h_period, pConf->lcd_basic.v_period,
           pConf->lcd_timing.hsync_width, pConf->lcd_timing.hsync_bp, ((pConf->lcd_timing.pol_cntl_addr >> LCD_HS_POL) & 1),
           pConf->lcd_timing.vsync_width, pConf->lcd_timing.vsync_bp, ((pConf->lcd_timing.pol_cntl_addr >> LCD_VS_POL) & 1),
           (((pConf->lcd_timing.vsync_h_phase >> 31) & 1) ? "-":""), (pConf->lcd_timing.vsync_h_phase & 0xffff), pConf->lcd_timing.hvsync_valid, pConf->lcd_timing.de_valid,
           (h_adj ? "-" : ""), (pConf->lcd_timing.h_offset & 0xffff), (v_adj ? "-" : ""), (pConf->lcd_timing.v_offset & 0xffff));

    switch (pConf->lcd_basic.lcd_type) {
        case LCD_DIGITAL_TTL:
            printk("rb_swap           %u\n"
                   "bit_swap          %u\n\n",
                   pConf->lcd_control.ttl_config->rb_swap, pConf->lcd_control.ttl_config->bit_swap);
            break;
        case LCD_DIGITAL_LVDS:
            printk("vswing_level      %u\n"
                   "lvds_repack       %u\n"
                   "pn_swap           %u\n\n",
                   pConf->lcd_control.lvds_config->lvds_vswing, pConf->lcd_control.lvds_config->lvds_repack, pConf->lcd_control.lvds_config->pn_swap);
            break;
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
        case LCD_DIGITAL_MIPI:
            printk("dsi_lane_num      %u\n"
                   "dsi_bit_rate      %u.%03uMHz\n"
                   "operation_mode    %u(%s), %u(%s)\n"
                   "transfer_ctrl     %u, %u\n\n",
                   pDev->pConf->lcd_control.mipi_config->lane_num,
                   (pDev->pConf->lcd_control.mipi_config->bit_rate / 1000000), ((pDev->pConf->lcd_control.mipi_config->bit_rate % 1000000) / 1000),
                   ((pDev->pConf->lcd_control.mipi_config->operation_mode>>BIT_OPERATION_MODE_INIT) &1), (((pDev->pConf->lcd_control.mipi_config->operation_mode>>BIT_OPERATION_MODE_INIT) & 1) ? "COMMAND" : "VIDEO"),
                   ((pDev->pConf->lcd_control.mipi_config->operation_mode>>BIT_OPERATION_MODE_DISP) & 1), (((pDev->pConf->lcd_control.mipi_config->operation_mode>>BIT_OPERATION_MODE_DISP) & 1) ? "COMMAND" : "VIDEO"),
                   ((pDev->pConf->lcd_control.mipi_config->transfer_ctrl>>BIT_TRANS_CTRL_CLK) & 1), ((pDev->pConf->lcd_control.mipi_config->transfer_ctrl>>BIT_TRANS_CTRL_SWITCH) & 3));
            break;
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
        case LCD_DIGITAL_EDP:
            printk("link_rate         %s\n"
                   "lane_count        %u\n"
                   "link_adaptive     %u\n"
                   "vswing            %u\n\n",
                   ((pConf->lcd_control.edp_config->link_rate == 0) ? "1.62G" : "2.7G"), pConf->lcd_control.edp_config->lane_count,
                   pConf->lcd_control.edp_config->link_adaptive, pConf->lcd_control.edp_config->vswing);
            break;
#endif
        default:
            break;
    }

    if (pConf->lcd_effect.dith_user)
        printk("dither_ctrl       0x%x\n", pConf->lcd_effect.dith_cntl_addr);

    printk("pll_ctrl          0x%08x\n"
           "div_ctrl          0x%08x\n"
           "clk_ctrl          0x%08x\n"
           "video_on_pixel    %d\n"
           "video_on_line     %d\n\n", 
           pConf->lcd_timing.pll_ctrl, pConf->lcd_timing.div_ctrl, pConf->lcd_timing.clk_ctrl,
           pConf->lcd_timing.video_on_pixel, pConf->lcd_timing.video_on_line);
}

static void scale_framebuffer(void)
{
	pDev->lcd_info.sync_duration_num = pDev->pConf->lcd_timing.sync_duration_num;
	pDev->lcd_info.sync_duration_den = pDev->pConf->lcd_timing.sync_duration_den;
	if ((pDev->pConf->lcd_basic.h_active != last_h_active) || (pDev->pConf->lcd_basic.v_active != last_v_active)) {
		pDev->lcd_info.width = pDev->pConf->lcd_basic.h_active;
		pDev->lcd_info.height = pDev->pConf->lcd_basic.v_active;
		pDev->lcd_info.field_height = pDev->pConf->lcd_basic.v_active;

		if (vpp2_sel)
			WRITE_LCD_REG(VPP2_POSTBLEND_H_SIZE, pDev->lcd_info.width);
		else
			WRITE_LCD_REG(VPP_POSTBLEND_H_SIZE, pDev->lcd_info.width);
		
		last_h_active = pDev->pConf->lcd_basic.h_active;
		last_v_active = pDev->pConf->lcd_basic.v_active;
	
		printk("\nPlease input below commands:\n");
		printk("echo 0 0 %d %d > /sys/class/video/axis\n", pDev->pConf->lcd_basic.h_active, pDev->pConf->lcd_basic.v_active);
		printk("echo %d > /sys/class/graphics/fb0/scale_width\n", temp_lcd_basic.h_active);
		printk("echo %d > /sys/class/graphics/fb0/scale_height\n", temp_lcd_basic.v_active);
		printk("echo 1 > /sys/class/graphics/fb0/free_scale\n\n");
	}	
}

static void save_lcd_config(Lcd_Config_t *pConf)
{
	temp_lcd_basic.h_active = pConf->lcd_basic.h_active;
	temp_lcd_basic.v_active = pConf->lcd_basic.v_active;
	temp_lcd_basic.h_period = pConf->lcd_basic.h_period;
	temp_lcd_basic.v_period = pConf->lcd_basic.v_period;
	temp_lcd_basic.lcd_type = pConf->lcd_basic.lcd_type;
	temp_lcd_basic.lcd_bits = pConf->lcd_basic.lcd_bits;

	temp_lcd_timing.pll_ctrl = pConf->lcd_timing.pll_ctrl;
	temp_lcd_timing.div_ctrl = pConf->lcd_timing.div_ctrl;
	temp_lcd_timing.clk_ctrl = pConf->lcd_timing.clk_ctrl;
	temp_lcd_timing.lcd_clk = pConf->lcd_timing.lcd_clk;
	temp_lcd_timing.hsync_width = pConf->lcd_timing.hsync_width;
	temp_lcd_timing.hsync_bp = pConf->lcd_timing.hsync_bp;
	temp_lcd_timing.vsync_width = pConf->lcd_timing.vsync_width;
	temp_lcd_timing.vsync_bp = pConf->lcd_timing.vsync_bp;
	temp_lcd_timing.hvsync_valid = pConf->lcd_timing.hvsync_valid;
	temp_lcd_timing.de_hstart = pConf->lcd_timing.de_hstart;
	temp_lcd_timing.de_vstart = pConf->lcd_timing.de_vstart;
	temp_lcd_timing.de_valid = pConf->lcd_timing.de_valid;
	temp_lcd_timing.h_offset = pConf->lcd_timing.h_offset;
	temp_lcd_timing.v_offset = pConf->lcd_timing.v_offset;
	temp_lcd_timing.pol_cntl_addr = pConf->lcd_timing.pol_cntl_addr;
	
	switch (pConf->lcd_basic.lcd_type) {
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
		case LCD_DIGITAL_MIPI:
			temp_dsi_lane_num = pConf->lcd_control.mipi_config->lane_num;
			temp_dsi_bit_rate_min = pConf->lcd_control.mipi_config->bit_rate_min;
			temp_dsi_bit_rate_max = pConf->lcd_control.mipi_config->bit_rate_max;
			temp_factor_denominator = pConf->lcd_control.mipi_config->factor_denominator;
			temp_factor_numerator = pConf->lcd_control.mipi_config->factor_numerator;
			break;
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
		case LCD_DIGITAL_EDP:
			temp_edp_link_rate = pConf->lcd_control.edp_config->link_rate;
			temp_edp_lane_count = pConf->lcd_control.edp_config->lane_count;
			temp_edp_vswing = pConf->lcd_control.edp_config->vswing;
			temp_edp_preemphasis = pConf->lcd_control.edp_config->preemphasis;
			break;
#endif
		case LCD_DIGITAL_LVDS:
			temp_lvds_repack = pConf->lcd_control.lvds_config->lvds_repack;
			temp_pn_swap = pConf->lcd_control.lvds_config->pn_swap;
			temp_lvds_vswing = pConf->lcd_control.lvds_config->lvds_vswing;
			break;
		case LCD_DIGITAL_TTL:
			temp_ttl_rb_swap = pConf->lcd_control.ttl_config->rb_swap;
			temp_ttl_bit_swap = pConf->lcd_control.ttl_config->bit_swap;
			break;
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
		case LCD_DIGITAL_MINILVDS:
			break;
#endif
		default:
			break;
	}
	
	temp_dith_user = pConf->lcd_effect.dith_user;
	temp_dith_ctrl = pConf->lcd_effect.dith_cntl_addr;
	temp_vadj_brightness = pConf->lcd_effect.vadj_brightness;
	temp_vadj_contrast = pConf->lcd_effect.vadj_contrast;
	temp_vadj_saturation = pConf->lcd_effect.vadj_saturation;
	
	last_h_active = pConf->lcd_basic.h_active;
	last_v_active = pConf->lcd_basic.v_active;
}

static void reset_lcd_config(Lcd_Config_t *pConf)
{
	int res = 0;
	
	_lcd_module_disable();
	printk("reset lcd config.\n");
	mdelay(200);
	if ((pConf->lcd_basic.h_active != temp_lcd_basic.h_active) || (pConf->lcd_basic.v_active != temp_lcd_basic.v_active))
		res = 1;
	
	pConf->lcd_basic.h_active = temp_lcd_basic.h_active;
	pConf->lcd_basic.v_active = temp_lcd_basic.v_active;
	pConf->lcd_basic.h_period = temp_lcd_basic.h_period;
	pConf->lcd_basic.v_period = temp_lcd_basic.v_period;
	pConf->lcd_basic.lcd_type = temp_lcd_basic.lcd_type;
	pConf->lcd_basic.lcd_bits = temp_lcd_basic.lcd_bits;

	pConf->lcd_timing.pll_ctrl = temp_lcd_timing.pll_ctrl;
	pConf->lcd_timing.div_ctrl = temp_lcd_timing.div_ctrl;
	pConf->lcd_timing.clk_ctrl = temp_lcd_timing.clk_ctrl;
	pConf->lcd_timing.lcd_clk = temp_lcd_timing.lcd_clk;
	pConf->lcd_timing.hsync_width = temp_lcd_timing.hsync_width;
	pConf->lcd_timing.hsync_bp = temp_lcd_timing.hsync_bp;
	pConf->lcd_timing.vsync_width = temp_lcd_timing.vsync_width;
	pConf->lcd_timing.vsync_bp = temp_lcd_timing.vsync_bp;
	pConf->lcd_timing.hvsync_valid = temp_lcd_timing.hvsync_valid;
	pConf->lcd_timing.de_hstart = temp_lcd_timing.de_hstart;
	pConf->lcd_timing.de_vstart = temp_lcd_timing.de_vstart;
	pConf->lcd_timing.de_valid = temp_lcd_timing.de_valid;
	pConf->lcd_timing.h_offset = temp_lcd_timing.h_offset;
	pConf->lcd_timing.v_offset = temp_lcd_timing.v_offset;
	pConf->lcd_timing.pol_cntl_addr = temp_lcd_timing.pol_cntl_addr;
	
	pConf->lcd_effect.dith_user = temp_dith_user;
	pConf->lcd_effect.dith_cntl_addr = temp_dith_ctrl;
	pConf->lcd_effect.vadj_brightness = temp_vadj_brightness;
	pConf->lcd_effect.vadj_contrast = temp_vadj_contrast;
	pConf->lcd_effect.vadj_saturation = temp_vadj_saturation;
	
	switch (pConf->lcd_basic.lcd_type) {
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
		case LCD_DIGITAL_MIPI:
			pConf->lcd_control.mipi_config->lane_num = temp_dsi_lane_num;
			pConf->lcd_control.mipi_config->bit_rate_min = temp_dsi_bit_rate_min;
			pConf->lcd_control.mipi_config->bit_rate_max = temp_dsi_bit_rate_max;
			pConf->lcd_control.mipi_config->factor_denominator = temp_factor_denominator;
			pConf->lcd_control.mipi_config->factor_numerator = temp_factor_numerator;
			break;
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
		case LCD_DIGITAL_EDP:
			//restore edp link config, for they are translate from user value to reg value
			pConf->lcd_control.edp_config->link_rate = temp_edp_link_rate;
			pConf->lcd_control.edp_config->lane_count = temp_edp_lane_count;
			pConf->lcd_control.edp_config->vswing = temp_edp_vswing;
			pConf->lcd_control.edp_config->preemphasis = temp_edp_preemphasis;
			break;
#endif
		case LCD_DIGITAL_LVDS:
			pConf->lcd_control.lvds_config->lvds_repack = temp_lvds_repack;
			pConf->lcd_control.lvds_config->pn_swap = temp_pn_swap;
			pConf->lcd_control.lvds_config->lvds_vswing = temp_lvds_vswing;
			break;
		case LCD_DIGITAL_TTL:
			pConf->lcd_control.ttl_config->rb_swap = temp_ttl_rb_swap;
			pConf->lcd_control.ttl_config->bit_swap = temp_ttl_bit_swap;
			break;
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
		case LCD_DIGITAL_MINILVDS:
			break;
#endif
		default:
			break;
	}
	
	lcd_config_init(pDev->pConf);
	_lcd_module_enable();
	
	pDev->lcd_info.sync_duration_num = pDev->pConf->lcd_timing.sync_duration_num;
    pDev->lcd_info.sync_duration_den = pDev->pConf->lcd_timing.sync_duration_den;
	if (res) {
		pDev->lcd_info.width = pDev->pConf->lcd_basic.h_active;
		pDev->lcd_info.height = pDev->pConf->lcd_basic.v_active;
		pDev->lcd_info.field_height = pDev->pConf->lcd_basic.v_active;
	
		if (vpp2_sel)
			WRITE_LCD_REG(VPP2_POSTBLEND_H_SIZE, pDev->lcd_info.width);
		else
			WRITE_LCD_REG(VPP_POSTBLEND_H_SIZE, pDev->lcd_info.width);
		
		last_h_active = pConf->lcd_basic.h_active;
		last_v_active = pConf->lcd_basic.v_active;
		
		printk("\nPlease input below commands:\n");
		printk("echo 0 > /sys/class/graphics/fb0/free_scale\n\n");
	}
}

static ssize_t lcd_debug(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	unsigned int ret;
	unsigned t[6];
	unsigned venc_video_mode, venc_test_base;
	
	switch (buf[0]) {
		case 'b':	//write basic config
			t[0] = 1024;
			t[1] = 768;
			t[2] = 1344;
			t[3] = 806;
			ret = sscanf(buf, "basic %d %d %d %d", &t[0], &t[1], &t[2], &t[3]);
			pDev->pConf->lcd_basic.h_active = t[0];
			pDev->pConf->lcd_basic.v_active = t[1];
			pDev->pConf->lcd_basic.h_period = t[2];
			pDev->pConf->lcd_basic.v_period = t[3];
			printk("h_active=%d, v_active=%d, h_period=%d, v_period=%d\n", t[0], t[1], t[2], t[3]);
			break;
		case 't':
			if (buf[1] == 'y') {//type
				t[0] = 1;
				t[1] = 6;
				ret = sscanf(buf, "type %d %d", &t[0], &t[1]);
				pDev->pConf->lcd_basic.lcd_type = t[0];
				pDev->pConf->lcd_basic.lcd_bits = t[1];
				printk("lcd_type: %s, lcd_bits: %d\n", lcd_type_table[t[0]], t[1]);
			}
			else if (buf[1] == 'e') {//test
				t[0] = 0;
				ret = sscanf(buf, "test %d", &t[0]);
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
				if (pDev->pConf->lcd_basic.lcd_type == LCD_DIGITAL_TTL) {
					venc_video_mode = ENCT_VIDEO_MODE_ADV;
					venc_test_base = ENCT_TST_EN;
				}
				else {
					venc_video_mode = ENCL_VIDEO_MODE_ADV;
					venc_test_base = ENCL_TST_EN;
				}
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
				venc_video_mode = ENCL_VIDEO_MODE_ADV;
				venc_test_base = ENCL_TST_EN;
#endif				
				switch (t[0]) {
					case 0:
						WRITE_LCD_REG(venc_video_mode, 0x8);
						printk("disable bist pattern\n");
						break;
					case 1:
						WRITE_LCD_REG(venc_video_mode, 0);
						WRITE_LCD_REG((venc_test_base+1), 1);
						WRITE_LCD_REG((venc_test_base+5), pDev->pConf->lcd_timing.video_on_pixel);
						WRITE_LCD_REG((venc_test_base+6), (pDev->pConf->lcd_basic.h_active / 9));
						WRITE_LCD_REG(venc_test_base, 1);
						printk("show bist pattern 1\n");
						break;
					case 2:
						WRITE_LCD_REG(venc_video_mode, 0);
						WRITE_LCD_REG((venc_test_base+1), 2);
						WRITE_LCD_REG(venc_test_base, 1);
						printk("show bist pattern 2\n");
						break;
					case 3:
						WRITE_LCD_REG(venc_video_mode, 0);
						WRITE_LCD_REG((venc_test_base+1), 3);
						WRITE_LCD_REG(venc_test_base, 1);
						printk("show bist pattern 3\n");
						break;
					default:
						printk("un-support pattern num\n");
						break;
				}
			}
			else if (buf[1] == 't') {//ttl
				t[0] = 0;
				t[1] = 0;
				ret = sscanf(buf, "ttl %d %d", &t[0], &t[1]);
				pDev->pConf->lcd_control.ttl_config->rb_swap = t[0];
				pDev->pConf->lcd_control.ttl_config->bit_swap = t[1];
				printk("ttl rb_swap: %s, bit_swap: %s\n", ((t[0] == 0) ? "disable" : "enable"), ((t[1] == 0) ? "disable" : "enable"));
			}
			break;
		case 'c':
			t[0] = 40000000;
			t[1] = 0;
			t[2] = 0;
			ret = sscanf(buf, "clock %d %d %d", &t[0], &t[1], &t[2]);
			pDev->pConf->lcd_timing.lcd_clk = t[0];
			pDev->pConf->lcd_timing.clk_ctrl = ((pDev->pConf->lcd_timing.clk_ctrl & ~(0xf << CLK_CTRL_SS)) | ((t[1] << CLK_CTRL_SS) | (1 << CLK_CTRL_AUTO)));
			pDev->pConf->lcd_timing.pol_cntl_addr = ((pDev->pConf->lcd_timing.pol_cntl_addr & ~(1 << LCD_CPH1_POL)) | (t[2] << LCD_CPH1_POL));
			printk("lcd_clk=%dHz, ss_level=%d, clk_pol=%s\n", t[0], t[1], ((t[2] == 0) ? "negative" : "positive"));
			break;
		case 's'://sync
			t[0] = 10;
			t[1] = 60;
			t[2] = 0;
			t[3] = 3;
			t[4] = 20;
			t[5] = 0;
			ret = sscanf(buf, "sync %d %d %d %d %d %d", &t[0], &t[1], &t[2], &t[3], &t[4], &t[5]);
			pDev->pConf->lcd_timing.hsync_width = t[0];
			pDev->pConf->lcd_timing.hsync_bp = t[1];
			pDev->pConf->lcd_timing.vsync_width = t[3];
			pDev->pConf->lcd_timing.vsync_bp = t[4];
			pDev->pConf->lcd_timing.pol_cntl_addr = ((pDev->pConf->lcd_timing.pol_cntl_addr & ~((1 << LCD_HS_POL) | (1 << LCD_VS_POL))) | ((t[2] << LCD_HS_POL) | (t[5] << LCD_VS_POL)));
			printk("hs_width=%d, hs_bp=%d, hs_pol=%s, vs_width=%d, vs_bp=%d, vs_pol=%s\n", t[0], t[1], ((t[2] == 0) ? "negative" : "positive"), t[3], t[4], ((t[5] == 0) ? "negative" : "positive"));
			break;
		case 'v':
			if (buf[2] == 'l') {	//valid
				t[0] = 0;
				t[1] = 0;
				t[2] = 1;
				ret = sscanf(buf, "valid %d %d", &t[0], &t[1]);
				pDev->pConf->lcd_timing.hvsync_valid = t[0];
				pDev->pConf->lcd_timing.de_valid = t[1];
				printk("hvsync: %s, de: %s\n", ((t[0] == 0) ? "disable" : "enable"), ((t[1] == 0) ? "disable" : "enable"));
			}
			else if (buf[2] == 'd') {	//vadj
				t[0] = 0x0;
				t[1] = 0x80;
				t[2] = 0x100;
				ret = sscanf(buf, "vadj %d %d %d", &t[0], &t[1], &t[2]);
				pDev->pConf->lcd_effect.vadj_brightness = t[0];
				pDev->pConf->lcd_effect.vadj_contrast = t[1];
				pDev->pConf->lcd_effect.vadj_saturation = t[2];
				printk("video adjust: brightness=0x%x, contrast=0x%x, stauration=0x%x\n", t[0], t[1], t[2]);
			}
			break;
		case 'o':
			t[0] = 1;
			t[1] = 0;
			t[2] = 1;
			t[3] = 0;
			ret = sscanf(buf, "offset %d %d %d %d", &t[0], &t[1], &t[2], &t[3]);
			pDev->pConf->lcd_timing.h_offset = ((t[0] << 31) | ((t[1] & 0xffff) << 0));
			pDev->pConf->lcd_timing.v_offset = ((t[2] << 31) | ((t[3] & 0xffff) << 0));
			printk("h_offset = %s%u, v_offset = %s%u\n", (t[0] ? "+" : "-"), (t[1] & 0xffff), (t[2] ? "+" : "-"), (t[3] & 0xffff));
			break;
		case 'l':	//write lvds config		//lvds_repack, pn_swap
			t[0] = 1;
			t[1] = 1;
			t[2] = 0;
			ret = sscanf(buf, "lvds %d %d %d", &t[0], &t[1], &t[2]);
			pDev->pConf->lcd_control.lvds_config->lvds_vswing = t[0];
			pDev->pConf->lcd_control.lvds_config->lvds_repack = t[1];
			pDev->pConf->lcd_control.lvds_config->pn_swap = t[2];
			printk("vswing_level: %u, lvds_repack: %s, rb_swap: %s\n", t[0], ((t[1] == 1) ? "VESA mode" : "JEIDA mode"), ((t[2] == 0) ? "disable" : "enable"));
			break;
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
        case 'm':	//write mipi config
            t[0] = 0;
            t[1] = 0;
            t[2] = 0;
            ret = sscanf(buf, "mdsi %d %d %d", &t[0],&t[1],&t[2]);
            pDev->pConf->lcd_control.mipi_config->bit_rate_min = t[0]*1000;
            pDev->pConf->lcd_control.mipi_config->bit_rate_max = t[1]*1000;
            pDev->pConf->lcd_control.mipi_config->factor_numerator=t[2];
            pDev->pConf->lcd_control.mipi_config->factor_denominator=10;
            lcd_config_init(pDev->pConf);
            printk("dsi bit_rate min=%dMHz, max=%dMHz, factor=%d",t[0], t[1], pDev->pConf->lcd_control.mipi_config->factor_numerator=t[2]);
            break;
#endif
		case 'd':
			if (buf[2] == 't') {
				t[0] = 0;
				t[1] = 0x600;
				ret = sscanf(buf, "dither %d %x", &t[0], &t[1]);
				pDev->pConf->lcd_effect.dith_user = t[0];
				pDev->pConf->lcd_effect.dith_cntl_addr = t[1];
				printk("dither user_ctrl: %s, 0x%x\n", ((t[0] == 0) ? "disable" : "enable"), t[1]);
			}
			else {
				printk("power off lcd.\n");
				_disable_backlight();
				lcd_power_ctrl(OFF);
			}
			break;
		case 'w':	//update display config
			if (pDev->pConf->lcd_basic.lcd_type == LCD_DIGITAL_MINILVDS) {
				printk("Don't support miniLVDS yet. Will reset to original lcd config.\n");
				reset_lcd_config(pDev->pConf);
			}
			else {
				_lcd_module_disable();
				mdelay(200);
				lcd_config_init(pDev->pConf);
				_lcd_module_enable();
				scale_framebuffer();
			}
			break;
		case 'r':	
			if (buf[2] == 'a') {	//read lcd config
				read_current_lcd_config(pDev->pConf);
			}
			else if (buf[2] == 's') {	//reset lcd config
				reset_lcd_config(pDev->pConf);
			}
			break;
		case 'e':
			if (buf[1] == 'n') {
				printk("power on lcd.\n");
				if (pDev->pConf->lcd_basic.lcd_type != LCD_DIGITAL_TTL) {
					init_phy_lvds(pDev->pConf);	
				}
				_init_lcd_driver(pDev->pConf);
				lcd_power_ctrl(ON);
				_enable_backlight();
			}
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
			else if (buf[1] == 'd') {
				t[0] = 1;
				t[1] = 4;
				t[2] = 0;
				ret = sscanf(buf, "edp %u %u %u", &t[0], &t[1], &t[2]);
				if (t[0] == 0)
					pDev->pConf->lcd_control.edp_config->link_rate = 0;
				else
					pDev->pConf->lcd_control.edp_config->link_rate = 1;
				switch (t[1]) {
					case 1:
					case 2:
						pDev->pConf->lcd_control.edp_config->lane_count = t[1];
						break;
					default:
						pDev->pConf->lcd_control.edp_config->lane_count = 4;
						break;
				}
				pDev->pConf->lcd_control.edp_config->vswing = t[2];
				printk("set edp link_rate = %sGbps, lane_count = %u, vswing_level = %u\n", ((pDev->pConf->lcd_control.edp_config->link_rate == 0) ? "1.62" : "2.70"), pDev->pConf->lcd_control.edp_config->lane_count, pDev->pConf->lcd_control.edp_config->vswing);
			}
#endif
			break;
		default:
			printk("wrong format of lcd debug command.\n");
	}	
	
	if (ret != 1 || ret !=2)
		return -EINVAL;
	
	return count;
	//return 0;
}

static ssize_t lcd_status_read(struct class *class, struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "read lcd status: %s\n", (lcd_status_flag ? "ON":"OFF"));
}

static ssize_t lcd_status_write(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	unsigned int ret;
	unsigned temp;

	temp = 1;
	ret = sscanf(buf, "%d", &temp);
	if (temp) {
		if (lcd_status_flag == 0) {
			mutex_lock(&lcd_vout_mutex);
			_lcd_module_enable();
			_enable_backlight();
			mutex_unlock(&lcd_vout_mutex);
		}
		else {
			printk("lcd has already ON\n");
		}
	}
	else {
		if (lcd_status_flag == 1) {
			mutex_lock(&lcd_vout_mutex);
			_disable_backlight();
			_lcd_module_disable();
			mutex_unlock(&lcd_vout_mutex);
		}
		else {
			printk("lcd has already OFF\n");
		}
	}

	if (ret != 1 || ret !=2)
		return -EINVAL;

	return count;
	//return 0;
}

static struct class_attribute lcd_debug_class_attrs[] = {
	__ATTR(debug,  S_IRUGO | S_IWUSR, lcd_debug_help, lcd_debug),
	__ATTR(help,  S_IRUGO | S_IWUSR, lcd_debug_help, NULL),
	__ATTR(status,  S_IRUGO | S_IWUSR, lcd_status_read, lcd_status_write),
    __ATTR_NULL
};

static struct class aml_lcd_debug_class = {
    .name = "lcd",
    .class_attrs = lcd_debug_class_attrs,
};
//****************************

static int amlogic_pmu_gpio_name_map_num(const char *name)
{
	int index;
	
	for(index = 0; index < LCD_POWER_PMU_GPIO_MAX; index++) {
		if(!strcasecmp(name, lcd_power_pmu_gpio_table[index]))
			break;
	}
	return index;
}

#ifdef CONFIG_USE_OF
static inline int _get_lcd_model_timing(struct platform_device *pdev)
{
	int ret=0;
	const char *str;
	unsigned int val;
	unsigned int lcd_para[100];
	int i, j;
	struct device_node *lcd_model_node;
	phandle fhandle;
	
	if (pdev->dev.of_node) {
		ret = of_property_read_u32(pdev->dev.of_node,"lcd_model_config",&fhandle);
		lcd_model_node = of_find_node_by_phandle(fhandle);
		ret = of_property_read_string(lcd_model_node,"model_name", &str);
		if(ret) {
			pDev->pConf->lcd_basic.model_name = "none";
			printk("lcd: faild to get lcd_model_name!\n");
		}
		else {
			pDev->pConf->lcd_basic.model_name = str;
			printk("load lcd model in dtb: %s\n", str);
		}
		ret = of_property_read_string(lcd_model_node, "interface", &str);
		if (ret) {
			printk("faild to get lcd_type!\n");
			str = "invalid";
		}	
		for(val = 0; val < LCD_TYPE_MAX; val++) {
			if(!strcasecmp(str, lcd_type_table[val]))
				break;
		}		
		pDev->pConf->lcd_basic.lcd_type = val;
		DBG_PRINT("lcd_type= %s(%u),\n", lcd_type_table[pDev->pConf->lcd_basic.lcd_type], pDev->pConf->lcd_basic.lcd_type);
		ret = of_property_read_u32_array(lcd_model_node,"active_area",&lcd_para[0],2);
		if(ret){
			printk("faild to get active_area\n");
		}
		else {
			pDev->pConf->lcd_basic.h_active_area = lcd_para[0];
			pDev->pConf->lcd_basic.v_active_area = lcd_para[1];
			pDev->pConf->lcd_basic.screen_ratio_width = lcd_para[0];
			pDev->pConf->lcd_basic.screen_ratio_height = lcd_para[1];
		}
		DBG_PRINT("h_active_area = %u, v_active_area =%u\n", pDev->pConf->lcd_basic.h_active_area, pDev->pConf->lcd_basic.v_active_area);
		ret = of_property_read_u32_array(lcd_model_node,"lcd_bits_option",&lcd_para[0],2);
		if(ret){
			printk("faild to get lcd_bits_option\n");
		}
		else {
			pDev->pConf->lcd_basic.lcd_bits = (unsigned short)(lcd_para[0]);
			pDev->pConf->lcd_basic.lcd_bits_option = (unsigned short)(lcd_para[1]);
		}
		DBG_PRINT("lcd_bits = %u, lcd_bits_option = %u\n", pDev->pConf->lcd_basic.lcd_bits, pDev->pConf->lcd_basic.lcd_bits_option);
		ret = of_property_read_u32_array(lcd_model_node,"resolution", &lcd_para[0], 2);
		if(ret){
			printk("faild to get resolution\n");
		}
		else {
			pDev->pConf->lcd_basic.h_active = (unsigned short)(lcd_para[0]);
			pDev->pConf->lcd_basic.v_active = (unsigned short)(lcd_para[1]);
		}		
		ret = of_property_read_u32_array(lcd_model_node,"period",&lcd_para[0],2);
		if(ret){
			printk("faild to get period\n");
		}
		else {
			pDev->pConf->lcd_basic.h_period = (unsigned short)(lcd_para[0]);
			pDev->pConf->lcd_basic.v_period = (unsigned short)(lcd_para[1]);
		}
		DBG_PRINT("h_active = %u, v_active =%u, h_period = %u, v_period = %u\n", pDev->pConf->lcd_basic.h_active, pDev->pConf->lcd_basic.v_active, pDev->pConf->lcd_basic.h_period, pDev->pConf->lcd_basic.v_period);
		ret = of_property_read_u32_array(lcd_model_node,"clock_hz_pol",&lcd_para[0], 2);
		if(ret){
			printk("faild to get clock_hz_pol\n");
		}
		else {
			pDev->pConf->lcd_timing.lcd_clk = lcd_para[0];
			pDev->pConf->lcd_timing.pol_cntl_addr = (lcd_para[1] << LCD_CPH1_POL);
		}
		DBG_PRINT("pclk = %uHz, pol=%u\n", pDev->pConf->lcd_timing.lcd_clk, (pDev->pConf->lcd_timing.pol_cntl_addr >> LCD_CPH1_POL) & 1);
		ret = of_property_read_u32_array(lcd_model_node,"hsync_width_backporch",&lcd_para[0], 2);
		if(ret){
			printk("faild to get hsync_width_backporch\n");
		}
		else {
			pDev->pConf->lcd_timing.hsync_width = (unsigned short)(lcd_para[0]);
			pDev->pConf->lcd_timing.hsync_bp = (unsigned short)(lcd_para[1]);
		}
		DBG_PRINT("hsync width = %u, backporch = %u\n", pDev->pConf->lcd_timing.hsync_width, pDev->pConf->lcd_timing.hsync_bp);
		ret = of_property_read_u32_array(lcd_model_node,"vsync_width_backporch",&lcd_para[0], 2);
		if(ret){
			printk("faild to get vsync_width_backporch\n");
		}
		else {
			pDev->pConf->lcd_timing.vsync_width = (unsigned short)(lcd_para[0]);
			pDev->pConf->lcd_timing.vsync_bp = (unsigned short)(lcd_para[1]);
		}
		DBG_PRINT("vsync width = %u, backporch = %u\n", pDev->pConf->lcd_timing.vsync_width, pDev->pConf->lcd_timing.vsync_bp);
		ret = of_property_read_u32_array(lcd_model_node,"pol_hsync_vsync",&lcd_para[0], 2);
		if(ret){
			printk("faild to get pol_hsync_vsync\n");
		}
		else {
			pDev->pConf->lcd_timing.pol_cntl_addr = (pDev->pConf->lcd_timing.pol_cntl_addr & ~((1 << LCD_HS_POL) | (1 << LCD_VS_POL))) | ((lcd_para[0] << LCD_HS_POL) | (lcd_para[1] << LCD_VS_POL));
		}
		DBG_PRINT("pol hsync = %u, vsync = %u\n", (pDev->pConf->lcd_timing.pol_cntl_addr >> LCD_HS_POL) & 1, (pDev->pConf->lcd_timing.pol_cntl_addr >> LCD_VS_POL) & 1);
		ret = of_property_read_u32_array(lcd_model_node,"vsync_horizontal_phase",&lcd_para[0], 2);
		if(ret){
			printk("faild to get vsync_horizontal_phase\n");
			pDev->pConf->lcd_timing.vsync_h_phase = 0;
		} else {
			pDev->pConf->lcd_timing.vsync_h_phase = ((lcd_para[0] << 31) | ((lcd_para[1] & 0xffff) << 0));
		}
		if (lcd_para[0] == 0)
			DBG_PRINT("vsync_horizontal_phase= %d\n", lcd_para[1]);
		else
			DBG_PRINT("vsync_horizontal_phase= -%d\n", lcd_para[1]);

#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
        if (LCD_DIGITAL_MIPI == pDev->pConf->lcd_basic.lcd_type) {
            ret = of_property_read_u32(lcd_model_node,"dsi_lane_num",&val);
            if(ret){
                printk("faild to get dsi_lane_num\n");
                pDev->pConf->lcd_control.mipi_config->lane_num = 4;
            }
            else {
                pDev->pConf->lcd_control.mipi_config->lane_num = (unsigned char)val;
            }
            DBG_PRINT("dsi_lane_num= %d\n",  pDev->pConf->lcd_control.mipi_config->lane_num);
            ret = of_property_read_u32_array(lcd_model_node,"dsi_bit_rate_min_max",&lcd_para[0], 2);
            if(ret){
                printk("faild to get dsi_bit_rate_min_max\n");
                lcd_para[0] = 0;
                lcd_para[1] = 0;
            }
            pDev->pConf->lcd_control.mipi_config->bit_rate_min = lcd_para[0];
            pDev->pConf->lcd_control.mipi_config->bit_rate_max = lcd_para[1];
            DBG_PRINT("dsi_bit_rate_min= %dMHz max=%dMHz\n", lcd_para[0], lcd_para[1]);
            ret = of_property_read_u32(lcd_model_node,"pclk_lanebyteclk_factor",&val);
            if(ret){
                printk("faild to get pclk_lanebyteclk_factor\n");
                pDev->pConf->lcd_control.mipi_config->factor_numerator = 0;
            }
            else {
                pDev->pConf->lcd_control.mipi_config->factor_numerator = val;
            }
            pDev->pConf->lcd_control.mipi_config->factor_denominator = 10;
            DBG_PRINT("pclk_lanebyteclk factor= %d\n", pDev->pConf->lcd_control.mipi_config->factor_numerator);
            ret = of_property_read_u32_array(lcd_model_node,"dsi_operation_mode",&lcd_para[0], 2);
            if(ret){
                printk("faild to get dsi_operation_mode\n");
                pDev->pConf->lcd_control.mipi_config->operation_mode = ((OPERATION_COMMAND_MODE << BIT_OPERATION_MODE_INIT) | (OPERATION_VIDEO_MODE << BIT_OPERATION_MODE_DISP));
            }
            else {
                pDev->pConf->lcd_control.mipi_config->operation_mode = ((lcd_para[0] << BIT_OPERATION_MODE_INIT) | (lcd_para[1] << BIT_OPERATION_MODE_DISP));
            }
            DBG_PRINT("dsi_operation_mode init=%d, display=%d\n", (pDev->pConf->lcd_control.mipi_config->operation_mode >> BIT_OPERATION_MODE_INIT) & 1, (pDev->pConf->lcd_control.mipi_config->operation_mode >> BIT_OPERATION_MODE_DISP) & 1);
            ret = of_property_read_u32_array(lcd_model_node,"dsi_transfer_ctrl",&lcd_para[0], 2);
            if(ret){
                printk("faild to get dsi_transfer_ctrl\n");
                pDev->pConf->lcd_control.mipi_config->transfer_ctrl = ((0 << BIT_TRANS_CTRL_CLK) | (0 << BIT_TRANS_CTRL_SWITCH));
            }
            else {
                pDev->pConf->lcd_control.mipi_config->transfer_ctrl = ((lcd_para[0] << BIT_TRANS_CTRL_CLK) | (lcd_para[1] << BIT_TRANS_CTRL_SWITCH));
            }
            DBG_PRINT("dsi_transfer_ctrl clk=%d, switch=%d\n", (pDev->pConf->lcd_control.mipi_config->transfer_ctrl >> BIT_TRANS_CTRL_CLK) & 1, (pDev->pConf->lcd_control.mipi_config->transfer_ctrl >> BIT_TRANS_CTRL_SWITCH) & 3);
            //detect dsi init on table
            pDev->pConf->lcd_control.mipi_config->dsi_init_on = get_dsi_init_table(1);//dsi_init_on
            ret = of_property_read_u32_index(lcd_model_node,"dsi_init_on", 0, &lcd_para[0]);
            if (ret) {
                printk("faild to get dsi_init_on\n");
            }
            else {
                i = 0;
                while (i < DSI_INIT_ON_MAX) {
                    ret = of_property_read_u32_index(lcd_model_node,"dsi_init_on", i, &val);
                    if (val == 0xff) {
                        ret = of_property_read_u32_index(lcd_model_node,"dsi_init_on", (i+1), &val);
                        i += 2;
                        if (val == 0xff)
                            break;
                    }
                    else {
                        ret = of_property_read_u32_index(lcd_model_node,"dsi_init_on", (i+2), &val);
                        i = i + 3 + val;
                    }
                }
                ret = of_property_read_u32_array(lcd_model_node,"dsi_init_on", &lcd_para[0], i);
                if(ret){
                    printk("faild to get dsi_init_on\n");
                }
                else {
                    DBG_PRINT("dsi_init_on: ");
                    for (j=0; j<i; j++) {
                        pDev->pConf->lcd_control.mipi_config->dsi_init_on[j] = (unsigned char)(lcd_para[j] & 0xff);
                        DBG_PRINT("0x%02x ", pDev->pConf->lcd_control.mipi_config->dsi_init_on[j]);
                    }
                    DBG_PRINT("\n");
                }
            }
            //detect dsi init off table
            pDev->pConf->lcd_control.mipi_config->dsi_init_off = get_dsi_init_table(0);//dsi_init_off
            ret = of_property_read_u32_index(lcd_model_node,"dsi_init_off", 0, &lcd_para[0]);
            if (ret) {
                printk("faild to get dsi_init_off\n");
            }
            else {
                i = 0;
                while (i < DSI_INIT_OFF_MAX) {
                    ret = of_property_read_u32_index(lcd_model_node,"dsi_init_off", i, &val);
                    if (val == 0xff) {
                        ret = of_property_read_u32_index(lcd_model_node,"dsi_init_off", (i+1), &val);
                        i += 2;
                        if (val == 0xff)
                            break;
                    }
                    else {
                        ret = of_property_read_u32_index(lcd_model_node,"dsi_init_off", (i+2), &val);
                        i = i + 3 + val;
                    }
                }
                ret = of_property_read_u32_array(lcd_model_node,"dsi_init_off", &lcd_para[0], i);
                if(ret){
                    printk("faild to get dsi_init_off\n");
                }
                else {
                    DBG_PRINT("dsi_init_off: ");
                    for (j=0; j<i; j++) {
                        pDev->pConf->lcd_control.mipi_config->dsi_init_off[j] = (unsigned char)(lcd_para[j] & 0xff);
                        DBG_PRINT("0x%02x ", pDev->pConf->lcd_control.mipi_config->dsi_init_off[j]);
                    }
                    DBG_PRINT("\n");
                }
            }
            ret = of_property_read_u32(lcd_model_node,"lcd_extern_init",&val);
            if(ret){
                printk("faild to get lcd_extern_init\n");
                pDev->pConf->lcd_control.mipi_config->lcd_extern_init =0;
            } else {
                pDev->pConf->lcd_control.mipi_config->lcd_extern_init =(unsigned char)(val);
            }
            DBG_PRINT("lcd_extern_init = %d\n",  pDev->pConf->lcd_control.mipi_config->lcd_extern_init);
        }
#endif
    }
    return ret;
}

static inline int _get_lcd_default_config(struct platform_device *pdev)
{
	int ret=0;
	unsigned int val;
	unsigned int lcd_para[5];
	unsigned int gamma_temp[256];
	int i;
	unsigned int lcd_gamma_multi = 0;
	
	//pdev->dev.of_node = of_find_node_by_name(NULL,"lcd");
	if (pdev->dev.of_node) {
		if (pDev->pConf->lcd_basic.lcd_bits_option == 1) {
			ret = of_property_read_u32(pdev->dev.of_node,"lcd_bits_user",&val);
			if(ret){
				printk("don't find to match lcd_bits_user, use panel typical setting.\n");
			}
			else {
				pDev->pConf->lcd_basic.lcd_bits = (unsigned short)(val);
				printk("lcd_bits = %u\n", pDev->pConf->lcd_basic.lcd_bits);
			}
		}
		//ttl & lvds config
		ret = of_property_read_u32_array(pdev->dev.of_node,"ttl_rb_bit_swap",&lcd_para[0], 2);
		if(ret){
			printk("don't find to match ttl_rb_bit_swap, use default setting.\n");
		}
		else {
			pDev->pConf->lcd_control.ttl_config->rb_swap = (unsigned char)(lcd_para[0]);
			pDev->pConf->lcd_control.ttl_config->bit_swap = (unsigned char)(lcd_para[1]);
			if (pDev->pConf->lcd_basic.lcd_type == LCD_DIGITAL_TTL)
				printk("ttl rb_swap = %u, bit_swap = %u\n", pDev->pConf->lcd_control.ttl_config->rb_swap, pDev->pConf->lcd_control.ttl_config->bit_swap);
			else
				DBG_PRINT("ttl rb_swap = %u, bit_swap = %u\n", pDev->pConf->lcd_control.ttl_config->rb_swap, pDev->pConf->lcd_control.ttl_config->bit_swap);
		}		
		ret = of_property_read_u32(pdev->dev.of_node,"lvds_channel_pn_swap",&val);
		if(ret){
			printk("don't find to match lvds_channel_pn_swap, use default setting.\n");
		}
		else {
			pDev->pConf->lcd_control.lvds_config->pn_swap = val;
			if (pDev->pConf->lcd_basic.lcd_type == LCD_DIGITAL_LVDS)
				printk("lvds_pn_swap = %u\n", pDev->pConf->lcd_control.lvds_config->pn_swap);
			else
				DBG_PRINT("lvds_pn_swap = %u\n", pDev->pConf->lcd_control.lvds_config->pn_swap);
		}

		//recommend setting
		ret = of_property_read_u32_array(pdev->dev.of_node,"valid_hvsync_de",&lcd_para[0], 2);
		if(ret){
			printk("don't find to match valid_hvsync_de, use default setting.\n");
		}
		else {
			pDev->pConf->lcd_timing.hvsync_valid = (unsigned short)(lcd_para[0]);
			pDev->pConf->lcd_timing.de_valid = (unsigned short)(lcd_para[1]);
			DBG_PRINT("valid hvsync = %u, de = %u\n", pDev->pConf->lcd_timing.hvsync_valid, pDev->pConf->lcd_timing.de_valid);
		}
		ret = of_property_read_u32_array(pdev->dev.of_node,"hsign_hoffset_vsign_voffset",&lcd_para[0], 4);
		if(ret){
			printk("don't find to match hsign_hoffset_vsign_voffset, use default setting.\n");
			pDev->pConf->lcd_timing.h_offset = 0;
			pDev->pConf->lcd_timing.v_offset = 0;
		}
		else {
			pDev->pConf->lcd_timing.h_offset = ((lcd_para[0] << 31) | ((lcd_para[1] & 0xffff) << 0));
			pDev->pConf->lcd_timing.v_offset = ((lcd_para[2] << 31) | ((lcd_para[3] & 0xffff) << 0));
			DBG_PRINT("h_offset = %s%u, ", (((pDev->pConf->lcd_timing.h_offset >> 31) & 1) ? "-" : ""), (pDev->pConf->lcd_timing.h_offset & 0xffff));
			DBG_PRINT("v_offset = %s%u\n", (((pDev->pConf->lcd_timing.v_offset >> 31) & 1) ? "-" : ""), (pDev->pConf->lcd_timing.v_offset & 0xffff));
		}
		ret = of_property_read_u32_array(pdev->dev.of_node,"dither_user_ctrl",&lcd_para[0], 2);
		if(ret){
			printk("don't find to match dither_user_ctrl, use default setting.\n");
			pDev->pConf->lcd_effect.dith_user = 0;
		}
		else {
			pDev->pConf->lcd_effect.dith_user = (unsigned short)(lcd_para[0]);
			pDev->pConf->lcd_effect.dith_cntl_addr = (unsigned short)(lcd_para[1]);
			DBG_PRINT("dither_user = %u, dither_ctrl = 0x%x\n", pDev->pConf->lcd_effect.dith_user, pDev->pConf->lcd_effect.dith_cntl_addr);
		}
		ret = of_property_read_u32_array(pdev->dev.of_node,"vadj_brightness_contrast_saturation",&lcd_para[0], 3);
		if(ret){
			printk("don't find to match vadj_brightness_contrast_saturation, use default setting.\n");
		}
		else {
			pDev->pConf->lcd_effect.vadj_brightness = lcd_para[0];
			pDev->pConf->lcd_effect.vadj_contrast = lcd_para[1];
			pDev->pConf->lcd_effect.vadj_saturation = lcd_para[2];
			DBG_PRINT("vadj_brightness = 0x%x, vadj_contrast = 0x%x, vadj_saturation = 0x%x\n", pDev->pConf->lcd_effect.vadj_brightness, pDev->pConf->lcd_effect.vadj_contrast, pDev->pConf->lcd_effect.vadj_saturation);
		}
		ret = of_property_read_u32_array(pdev->dev.of_node,"gamma_en_revert",&lcd_para[0], 2);
		if(ret){
			printk("don't find to match gamma_en_revert, use default setting.\n");
		}
		else {
			pDev->pConf->lcd_effect.gamma_cntl_port = (lcd_para[0] << LCD_GAMMA_EN);
			//pDev->pConf->lcd_effect.gamma_vcom_hswitch_addr = 0;
			pDev->pConf->lcd_effect.gamma_revert = (unsigned char)(lcd_para[1]);
			DBG_PRINT("gamma_en = %u, gamma_revert=%u\n", ((pDev->pConf->lcd_effect.gamma_cntl_port >> LCD_GAMMA_EN) & 1), pDev->pConf->lcd_effect.gamma_revert);
		}
		ret = of_property_read_u32_array(pdev->dev.of_node,"gamma_multi_rgb_coeff",&lcd_para[0], 4);
		if(ret){
			printk("don't find to match gamma_multi_rgb_coeff, use default setting.\n");
		}
		else {
			lcd_gamma_multi = lcd_para[0];
			pDev->pConf->lcd_effect.gamma_r_coeff = (unsigned short)(lcd_para[1]);
			pDev->pConf->lcd_effect.gamma_g_coeff = (unsigned short)(lcd_para[2]);
			pDev->pConf->lcd_effect.gamma_b_coeff = (unsigned short)(lcd_para[3]);
			DBG_PRINT("gamma_multi = %u, gamma_r_coeff = %u, gamma_g_coeff = %u, gamma_b_coeff = %u\n", lcd_gamma_multi, pDev->pConf->lcd_effect.gamma_r_coeff, pDev->pConf->lcd_effect.gamma_g_coeff, pDev->pConf->lcd_effect.gamma_b_coeff);
		}
		if (lcd_gamma_multi == 1) {
			ret = of_property_read_u32_array(pdev->dev.of_node,"gamma_table_r",&gamma_temp[0], 256);
			if(ret){
				printk("don't find to match gamma_table_r, use default table.\n");
				lcd_setup_gamma_table(pDev->pConf, 0);
			}
			else {
				for (i=0; i<256; i++) {
					pDev->pConf->lcd_effect.GammaTableR[i] = (unsigned short)(gamma_temp[i] << 2);
				}
				DBG_PRINT("load gamma_table_r.\n");
			}
			ret = of_property_read_u32_array(pdev->dev.of_node,"gamma_table_g",&gamma_temp[0], 256);
			if(ret){
				printk("don't find to match gamma_table_g, use default table.\n");
				lcd_setup_gamma_table(pDev->pConf, 1);
			}
			else {
				for (i=0; i<256; i++) {
					pDev->pConf->lcd_effect.GammaTableG[i] = (unsigned short)(gamma_temp[i] << 2);
				}
				DBG_PRINT("load gamma_table_g.\n");
			}
			ret = of_property_read_u32_array(pdev->dev.of_node,"gamma_table_b",&gamma_temp[0], 256);
			if(ret){
				printk("don't find to match gamma_table_b, use default table.\n");
				lcd_setup_gamma_table(pDev->pConf, 2);
			}
			else {
				for (i=0; i<256; i++) {
					pDev->pConf->lcd_effect.GammaTableB[i] = (unsigned short)(gamma_temp[i] << 2);
				}
				DBG_PRINT("load gamma_table_b.\n");
			}
		}
		else {
			ret = of_property_read_u32_array(pdev->dev.of_node,"gamma_table",&gamma_temp[0], 256);
			if(ret){
				printk("don't find to match gamma_table, use default table.\n");
				lcd_setup_gamma_table(pDev->pConf, 3);
			}
			else {
				for (i=0; i<256; i++) {
					pDev->pConf->lcd_effect.GammaTableR[i] = (unsigned short)(gamma_temp[i] << 2);
					pDev->pConf->lcd_effect.GammaTableG[i] = (unsigned short)(gamma_temp[i] << 2);
					pDev->pConf->lcd_effect.GammaTableB[i] = (unsigned short)(gamma_temp[i] << 2);
				}
				DBG_PRINT("load gamma_table.\n");
			}
		}
		
		//default setting
		ret = of_property_read_u32(pdev->dev.of_node,"clock_spread_spectrum",&val);
		if(ret){
			printk("don't find to match clock_spread_spectrum, use default setting.\n");
		}
		else {
			pDev->pConf->lcd_timing.clk_ctrl = (pDev->pConf->lcd_timing.clk_ctrl & ~(0xf << CLK_CTRL_SS)) | (val << CLK_CTRL_SS);
			DBG_PRINT("lcd clock spread spectrum = %u\n", (pDev->pConf->lcd_timing.clk_ctrl >> CLK_CTRL_SS) & 0xf);
		}
		ret = of_property_read_u32(pdev->dev.of_node,"clock_auto_generation",&val);
		if(ret){
			printk("don't find to match clock_auto_generation, use default setting.\n");
		}
		else {
			pDev->pConf->lcd_timing.clk_ctrl = ((pDev->pConf->lcd_timing.clk_ctrl & ~(1 << CLK_CTRL_AUTO)) | (val << CLK_CTRL_AUTO));
			DBG_PRINT("lcd clock auto calculate = %u\n", (pDev->pConf->lcd_timing.clk_ctrl >> CLK_CTRL_AUTO) & 1);
		}
		if (((pDev->pConf->lcd_timing.clk_ctrl >> CLK_CTRL_AUTO) & 1) == 0) {
			ret = of_property_read_u32_array(pdev->dev.of_node,"clk_pll_div_clk_ctrl",&lcd_para[0], 3);
			if(ret){
				printk("don't find to match clk_pll_div_clk_ctrl, use default setting.\n");
			}
			else {
				pDev->pConf->lcd_timing.pll_ctrl = lcd_para[0];
				pDev->pConf->lcd_timing.div_ctrl = lcd_para[1];
				pDev->pConf->lcd_timing.clk_ctrl = lcd_para[2];
				printk("pll_ctrl = 0x%x, div_ctrl = 0x%x, clk_ctrl=0x%x\n", pDev->pConf->lcd_timing.pll_ctrl, pDev->pConf->lcd_timing.div_ctrl, (pDev->pConf->lcd_timing.clk_ctrl & 0xffff));
			}
		}
		ret = of_property_read_u32(pdev->dev.of_node,"lvds_vswing",&val);
		if(ret){
			printk("don't find to match lvds_vswing, use default setting.\n");
		}
		else {
			pDev->pConf->lcd_control.lvds_config->lvds_vswing = val;
			if (pDev->pConf->lcd_basic.lcd_type == LCD_DIGITAL_LVDS)
				printk("lvds_vswing = %u\n", pDev->pConf->lcd_control.lvds_config->lvds_vswing = val);
			else
				DBG_PRINT("lvds_vswing = %u\n", pDev->pConf->lcd_control.lvds_config->lvds_vswing = val);
		}
		ret = of_property_read_u32_array(pdev->dev.of_node,"lvds_user_repack",&lcd_para[0], 2);
		if(ret){
			printk("don't find to match lvds_user_repack, use default setting.\n");
			pDev->pConf->lcd_control.lvds_config->lvds_repack_user = 0;
			pDev->pConf->lcd_control.lvds_config->lvds_repack = 1;
		}
		else {
			pDev->pConf->lcd_control.lvds_config->lvds_repack_user = lcd_para[0];
			pDev->pConf->lcd_control.lvds_config->lvds_repack = lcd_para[1];
			if (lcd_para[0] > 0) {
				if (pDev->pConf->lcd_basic.lcd_type == LCD_DIGITAL_LVDS)
					printk("lvds_repack = %u\n", pDev->pConf->lcd_control.lvds_config->lvds_repack);
				else
					DBG_PRINT("lvds_repack = %u\n", pDev->pConf->lcd_control.lvds_config->lvds_repack);
			}
			else {
				DBG_PRINT("lvds_repack_user = %u, lvds_repack = %u\n", pDev->pConf->lcd_control.lvds_config->lvds_repack_user, pDev->pConf->lcd_control.lvds_config->lvds_repack);
			}
		}
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
		ret = of_property_read_u32_array(pdev->dev.of_node,"edp_user_link_rate_lane_count",&lcd_para[0], 3);
		if(ret){
			pDev->pConf->lcd_control.edp_config->link_user = 0;
			pDev->pConf->lcd_control.edp_config->link_rate = 1;
			pDev->pConf->lcd_control.edp_config->lane_count = 4;
			printk("don't find to match edp_user_link_rate_lane_count, use default setting.\n");
		}
		else {
			pDev->pConf->lcd_control.edp_config->link_user = (unsigned char)(lcd_para[0]);
			pDev->pConf->lcd_control.edp_config->link_rate = (unsigned char)(lcd_para[1]);
			pDev->pConf->lcd_control.edp_config->lane_count = (unsigned char)(lcd_para[2]);
			if (lcd_para[0] > 0) {
				if (pDev->pConf->lcd_basic.lcd_type == LCD_DIGITAL_EDP)
					printk("edp link_rate = %s, lane_count = %u\n", (lcd_para[1] == 0) ? "1.62G":"2.7G", pDev->pConf->lcd_control.edp_config->lane_count);
				else
					DBG_PRINT("edp link_rate = %s, lane_count = %u\n", (lcd_para[1] == 0) ? "1.62G":"2.7G", pDev->pConf->lcd_control.edp_config->lane_count);
			}
			else {
				DBG_PRINT("edp user = %u, link_rate = %s, lane_count = %u\n", pDev->pConf->lcd_control.edp_config->link_user, (lcd_para[1] == 0) ? "1.62G":"2.7G", pDev->pConf->lcd_control.edp_config->lane_count);
			}
		}
		ret = of_property_read_u32_array(pdev->dev.of_node,"edp_link_adaptive_vswing",&lcd_para[0], 2);
		if(ret){
			printk("don't find to match edp_link_adaptive_vswing, use default setting.\n");
			pDev->pConf->lcd_control.edp_config->link_adaptive = 0;
			pDev->pConf->lcd_control.edp_config->vswing = 0;
			pDev->pConf->lcd_control.edp_config->preemphasis = 0;
		}
		else {
			pDev->pConf->lcd_control.edp_config->link_adaptive = (unsigned char)(lcd_para[0]);
			pDev->pConf->lcd_control.edp_config->vswing = (unsigned char)(lcd_para[1]);
			pDev->pConf->lcd_control.edp_config->preemphasis = 0;
			if (lcd_para[0] == 0) {
				if (pDev->pConf->lcd_basic.lcd_type == LCD_DIGITAL_EDP)
					printk("edp swing_level = %u\n", pDev->pConf->lcd_control.edp_config->vswing);
				else
					DBG_PRINT("edp swing_level = %u\n", pDev->pConf->lcd_control.edp_config->vswing);
			}
			else {
				DBG_PRINT("edp link_adaptive = %u, swing_level = %u\n", pDev->pConf->lcd_control.edp_config->link_adaptive, pDev->pConf->lcd_control.edp_config->vswing);
			}
		}
#endif
		ret = of_property_read_u32_array(pdev->dev.of_node,"rgb_base_coeff",&lcd_para[0], 2);
		if(ret){
			printk("don't find to match rgb_base_coeff, use default setting.\n");
		}
		else {
			pDev->pConf->lcd_effect.rgb_base_addr = (unsigned short)(lcd_para[0]);
			pDev->pConf->lcd_effect.rgb_coeff_addr = (unsigned short)(lcd_para[1]);
			DBG_PRINT("rgb_base = 0x%x, rgb_coeff = 0x%x\n", pDev->pConf->lcd_effect.rgb_base_addr, pDev->pConf->lcd_effect.rgb_coeff_addr);
		}
		// ret = of_property_read_u32_array(pdev->dev.of_node,"video_on_pixel_line",&lcd_para[0], 2);
		// if(ret){
			// printk("don't find to match video_on_pixel_line, use default setting.\n");
		// }
		// else {
			// pDev->pConf->lcd_timing.video_on_pixel = (unsigned short)(lcd_para[0]);
			// pDev->pConf->lcd_timing.video_on_line = (unsigned short)(lcd_para[1]);
			// DBG_PRINT("video_on_pixel = %u, video_on_line = %u\n", pDev->pConf->lcd_timing.video_on_pixel, pDev->pConf->lcd_timing.video_on_line);
		// }
	}
	
	return ret;
}

static inline int _get_lcd_power_config(struct platform_device *pdev)
{
	int ret=0;
	const char *str;
	unsigned char propname[20];
	int val;
	unsigned int lcd_para[LCD_POWER_CTRL_STEP_MAX];
	int i;
	int index;
	
	if (pdev->dev.of_node) {
		//lcd power on
		for (i=0; i < LCD_POWER_CTRL_STEP_MAX; i++) {
			//propname = kasprintf(GFP_KERNEL, "power_on_step_%d", i+1);
			sprintf(propname, "power_on_step_%d", i+1);
			ret = of_property_read_string_index(pdev->dev.of_node, propname, 0, &str);
			if (ret) {
				DBG_PRINT("faild to get %s\n", propname);
				break;
			}
			else if ((strcasecmp(str, "null") == 0) || ((strcasecmp(str, "n") == 0))) {
				break;
			}
			else {
				DBG_PRINT("%s 0: %s\n", propname, str);
				for(index = 0; index < LCD_POWER_TYPE_MAX; index++) {
					if(!strcasecmp(str, lcd_power_type_table[index]))
						break;
				}		
				pDev->pConf->lcd_power_ctrl.power_on_config[i].type = index;
				
				if (pDev->pConf->lcd_power_ctrl.power_on_config[i].type != LCD_POWER_TYPE_SIGNAL) {
					ret = of_property_read_string_index(pdev->dev.of_node, propname, 1, &str);
					if (ret) {
						printk("faild to get %s index 1\n", propname);
					}
					else {					
						if (pDev->pConf->lcd_power_ctrl.power_on_config[i].type == LCD_POWER_TYPE_CPU) {
							val = amlogic_gpio_name_map_num(str);
							ret = lcd_gpio_request(val);
							if (ret) {
							  printk("faild to alloc lcd power ctrl gpio (%s)\n", str);
							}
							pDev->pConf->lcd_power_ctrl.power_on_config[i].gpio = val;
						}
						else if (pDev->pConf->lcd_power_ctrl.power_on_config[i].type == LCD_POWER_TYPE_PMU) {
							pDev->pConf->lcd_power_ctrl.power_on_config[i].gpio = amlogic_pmu_gpio_name_map_num(str);
						}
					}
					ret = of_property_read_string_index(pdev->dev.of_node, propname, 2, &str);
					if (ret) {
						printk("faild to get %s\n", propname);
					}
					else {					
						if ((strcasecmp(str, "output_low") == 0) || (strcasecmp(str, "0") == 0)) {
							pDev->pConf->lcd_power_ctrl.power_on_config[i].value = LCD_POWER_GPIO_OUTPUT_LOW;
						}
						else if ((strcasecmp(str, "output_high") == 0) || (strcasecmp(str, "1") == 0)) {
							pDev->pConf->lcd_power_ctrl.power_on_config[i].value = LCD_POWER_GPIO_OUTPUT_HIGH;
						}
						else if ((strcasecmp(str, "input") == 0) || (strcasecmp(str, "2") == 0)) {
							pDev->pConf->lcd_power_ctrl.power_on_config[i].value = LCD_POWER_GPIO_INPUT;
						}
					}
				}
			}
		}
		pDev->pConf->lcd_power_ctrl.power_on_step = i;
		DBG_PRINT("lcd_power_on_step = %d\n", pDev->pConf->lcd_power_ctrl.power_on_step);
		
		ret = of_property_read_u32_array(pdev->dev.of_node,"power_on_delay",&lcd_para[0],pDev->pConf->lcd_power_ctrl.power_on_step);
		if (ret) {
			printk("faild to get power_on_delay\n");
		}
		else {
			for (i=0; i<pDev->pConf->lcd_power_ctrl.power_on_step; i++) {
				pDev->pConf->lcd_power_ctrl.power_on_config[i].delay = (unsigned short)(lcd_para[i]);
			}
		}
		//lcd power off
		for (i=0; i < LCD_POWER_CTRL_STEP_MAX; i++) {
			sprintf(propname, "power_off_step_%d", i+1);
			//propname = kasprintf(GFP_KERNEL, "power_off_step_%d", i+1);
			ret = of_property_read_string_index(pdev->dev.of_node, propname, 0, &str);
			if (ret) {
				DBG_PRINT("faild to get %s index 0\n", propname);
				break;
			}
			else if ((strcasecmp(str, "null") == 0) || ((strcasecmp(str, "n") == 0))) {
				break;
			}
			else {
				DBG_PRINT("%s 0: %s\n", propname, str);
				for(index = 0; index < LCD_POWER_TYPE_MAX; index++) {
					if(!strcasecmp(str, lcd_power_type_table[index]))
						break;
				}		
				pDev->pConf->lcd_power_ctrl.power_off_config[i].type = index;
			
				if (pDev->pConf->lcd_power_ctrl.power_off_config[i].type < LCD_POWER_TYPE_SIGNAL) {
					ret = of_property_read_string_index(pdev->dev.of_node, propname, 1, &str);
					if (ret) {
						printk("faild to get %s index 1\n", propname);
					}
					else {
						if (pDev->pConf->lcd_power_ctrl.power_off_config[i].type == LCD_POWER_TYPE_CPU) {
							val = amlogic_gpio_name_map_num(str);
							pDev->pConf->lcd_power_ctrl.power_off_config[i].gpio = val;
						}
						else if (pDev->pConf->lcd_power_ctrl.power_off_config[i].type == LCD_POWER_TYPE_PMU) {
							pDev->pConf->lcd_power_ctrl.power_off_config[i].gpio = amlogic_pmu_gpio_name_map_num(str);
						}
					}
					ret = of_property_read_string_index(pdev->dev.of_node, propname, 2, &str);
					if (ret) {
						printk("faild to get %s index 2\n", propname);
					}
					else {
						if ((strcasecmp(str, "output_low") == 0) || (strcasecmp(str, "0") == 0)) {
							pDev->pConf->lcd_power_ctrl.power_off_config[i].value = LCD_POWER_GPIO_OUTPUT_LOW;
						}
						else if ((strcasecmp(str, "output_high") == 0) || (strcasecmp(str, "1") == 0)) {
							pDev->pConf->lcd_power_ctrl.power_off_config[i].value = LCD_POWER_GPIO_OUTPUT_HIGH;
						}
						else if ((strcasecmp(str, "input") == 0) || (strcasecmp(str, "2") == 0)) {
							pDev->pConf->lcd_power_ctrl.power_off_config[i].value = LCD_POWER_GPIO_INPUT;
						}
					}
				}
			}
		}
		pDev->pConf->lcd_power_ctrl.power_off_step = i;
		DBG_PRINT("lcd_power_off_step = %d\n", pDev->pConf->lcd_power_ctrl.power_off_step);
		
		ret = of_property_read_u32_array(pdev->dev.of_node,"power_off_delay",&lcd_para[0],pDev->pConf->lcd_power_ctrl.power_off_step);
		if (ret) {
			printk("faild to get power_off_delay\n");
		}
		else {
			for (i=0; i<pDev->pConf->lcd_power_ctrl.power_off_step; i++) {
				pDev->pConf->lcd_power_ctrl.power_off_config[i].delay = (unsigned short)(lcd_para[i]);
			}
		}
		
		for (i=0; i<pDev->pConf->lcd_power_ctrl.power_on_step; i++) {
			DBG_PRINT("power on step %d: type = %s(%d)\n", i+1, lcd_power_type_table[pDev->pConf->lcd_power_ctrl.power_on_config[i].type], pDev->pConf->lcd_power_ctrl.power_on_config[i].type);
			DBG_PRINT("power on step %d: gpio = %d\n", i+1, pDev->pConf->lcd_power_ctrl.power_on_config[i].gpio);
			DBG_PRINT("power on step %d: value = %d\n", i+1, pDev->pConf->lcd_power_ctrl.power_on_config[i].value);
			DBG_PRINT("power on step %d: delay = %d\n", i+1, pDev->pConf->lcd_power_ctrl.power_on_config[i].delay);
		}
		
		for (i=0; i<pDev->pConf->lcd_power_ctrl.power_off_step; i++) {
			DBG_PRINT("power off step %d: type = %s(%d)\n", i+1, lcd_power_type_table[pDev->pConf->lcd_power_ctrl.power_off_config[i].type], pDev->pConf->lcd_power_ctrl.power_off_config[i].type);
			DBG_PRINT("power off step %d: gpio = %d\n", i+1, pDev->pConf->lcd_power_ctrl.power_off_config[i].gpio);
			DBG_PRINT("power off step %d: value = %d\n", i+1, pDev->pConf->lcd_power_ctrl.power_off_config[i].value);
			DBG_PRINT("power off step %d: delay = %d\n", i+1, pDev->pConf->lcd_power_ctrl.power_off_config[i].delay);
		}

		pDev->p = devm_pinctrl_get(&pdev->dev);
		if (IS_ERR(pDev->p))
			printk("get lcd ttl ports pinmux error.\n");
	}	
	return ret;
}
#endif

#ifdef CONFIG_USE_OF
static struct aml_lcd_platform meson_lcd_platform = {
	.lcd_conf = &lcd_config,
};

#define AMLOGIC_LCD_DRV_DATA ((kernel_ulong_t)&meson_lcd_platform)

static const struct of_device_id lcd_dt_match[] = {
	{
		.compatible = "amlogic,lcd",
		.data = (void *)AMLOGIC_LCD_DRV_DATA
	},
	{},
};
#else
#define lcd_dt_match NULL
#endif

#ifdef CONFIG_USE_OF
static inline struct aml_lcd_platform *lcd_get_driver_data(struct platform_device *pdev)
{
	const struct of_device_id *match;
	
	if(pdev->dev.of_node) {
		//DBG_PRINT("***of_device: get lcd driver data.***\n");		
		match = of_match_node(lcd_dt_match, pdev->dev.of_node);
		return (struct aml_lcd_platform *)match->data;
	}
	return NULL;
}
#endif

static struct notifier_block lcd_reboot_nb;
static int lcd_probe(struct platform_device *pdev)
{
    struct aml_lcd_platform *pdata;  
	int ret = 0;
	spin_lock_init(&gamma_write_lock);
	spin_lock_init(&lcd_clk_lock);
	
	printk("lcd driver version: %s%s\n\n", DRIVER_DATE, DRV_TYPE);
	
#ifdef 	CONFIG_USE_OF
	pdata = lcd_get_driver_data(pdev);
#else
	pdata = pdev->dev.platform_data;
#endif

    pDev = (lcd_dev_t *)kmalloc(sizeof(lcd_dev_t), GFP_KERNEL);
    if (!pDev) {
        printk("[lcd probe]: Not enough memory.\n");
        return -ENOMEM;
    }    

    pDev->pConf = (Lcd_Config_t *)(pdata->lcd_conf);
#ifdef CONFIG_USE_OF
	_get_lcd_model_timing(pdev);
	_get_lcd_default_config(pdev);
	_get_lcd_power_config(pdev);
#endif
	save_lcd_config(pDev->pConf);
	lcd_config_init(pDev->pConf);
    _lcd_init(pDev->pConf);
	
	lcd_reboot_nb.notifier_call = lcd_reboot_notifier;
    ret = register_reboot_notifier(&lcd_reboot_nb);
	if (ret) {
		printk("notifier register lcd_reboot_notifier fail!\n");
	}	
	
	ret = class_register(&aml_lcd_debug_class);
	if(ret){
		printk("class register aml_lcd_debug_class fail!\n");
	}
#ifdef CONFIG_AML_GAMMA_DEBUG
	save_original_gamma(pDev->pConf);
	ret = class_register(&aml_gamma_class);
	if(ret){
		printk("class register aml_gamma_class fail!\n");
	}
#endif

	switch (pDev->pConf->lcd_basic.lcd_type) {
#if ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
		case LCD_DIGITAL_MIPI :
			dsi_probe(pDev->pConf);
			break;
#endif
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
		case LCD_DIGITAL_EDP :
			edp_probe();
			break;
#endif
		default:
			break;
    }

	printk("LCD probe ok\n");

	return 0;
}

static int lcd_remove(struct platform_device *pdev)
{
	unregister_reboot_notifier(&lcd_reboot_nb);
	kfree(pDev);

    return 0;
}

//device tree
static struct platform_driver lcd_driver = {
	.probe = lcd_probe,
	.remove = lcd_remove,
	.driver = {
		.name = "mesonlcd",
#ifdef CONFIG_USE_OF
		.owner = THIS_MODULE,
		.of_match_table = lcd_dt_match,
#endif
	},
};

static int __init lcd_init(void)
{
	printk("LCD driver init\n");
    if (platform_driver_register(&lcd_driver)) {
        printk("failed to register lcd driver module\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit lcd_exit(void)
{
    platform_driver_unregister(&lcd_driver);
}

subsys_initcall(lcd_init);
module_exit(lcd_exit);

MODULE_DESCRIPTION("Meson LCD Panel Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Amlogic, Inc.");
