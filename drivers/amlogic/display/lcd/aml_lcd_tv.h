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
 * Modify:  <jiaming.huang@amlogic.com>
 *
 */

#ifndef AML_TV_LCD_H
#define AML_TV_LCD_H

#include <linux/amlogic/vout/vinfo.h>
#ifdef CONFIG_AML_LCD_EXTERN
#include <linux/amlogic/vout/aml_lcd_extern.h>
#endif

#define LCD_TV_DRIVER_VERSION        "20150916"

typedef enum {
    OFF = 0,
    ON = 1,
} Bool_t;

typedef enum {
	LCD_DIGITAL_LVDS = 0,
	LCD_DIGITAL_VBYONE,
	LCD_DIGITAL_TTL,
	//LCD_DIGITAL_MINILVDS,
	LCD_TYPE_MAX,
} Lcd_Type_t;


typedef struct {
	char *model_name;
	u16 h_active;		// Horizontal display area
	u16 v_active;		// Vertical display area
	u16 h_period;		// Horizontal total period time
	u16 v_period;		// Vertical total period time
	u16 video_on_pixel;
	u16 video_on_line;

	u32 screen_ratio_width;		// screen aspect ratio width
	u32 screen_ratio_height;		// screen aspect ratio height
	u32 h_active_area;				/* screen physical width in "mm" unit */
	u32 v_active_area;				/* screen physical height in "mm" unit */

	Lcd_Type_t lcd_type;
	//u16 lcd_bits;			// 6 or 8 bits
	//u16 lcd_bits_option;	//option=0, means the panel only support one lcd_bits option
}Lcd_Basic_t;
/*

typedef struct {
	u32 pll_ctrl;		//video PLL settings
	u32 div_ctrl;		// video pll div settings
	u32 clk_ctrl;		// video clock settings //[31]clk_auto, [11:8]ss_ctrl, [7:0]xd
	u32 lcd_clk;		// lcd clock
	u16 sync_duration_num;
	u16 sync_duration_den;

	u16 pol_ctrl;
	//u16 inv_cnt_addr;
	//u16 tcon_misc_sel_addr;

	u16 video_on_pixel;
	u16 video_on_line;

	u16 hsync_width;
	u16 hsync_bp;
	u16 vsync_width;
	u16 vsync_bp;
	u32 vsync_h_phase; //[31]sign [15:0]value
	u16 hvsync_valid;
	//u16 de_hstart;
	//u16 de_vstart;
	u16 de_valid;
	u32 h_offset;
	u32 v_offset;

	u16 de_hs_addr;
	u16 de_he_addr;
	u16 de_vs_addr;
	u16 de_ve_addr;

	u16 hs_hs_addr;
	u16 hs_he_addr;
	u16 hs_vs_addr;
	u16 hs_ve_addr;

	u16 vs_hs_addr;
	u16 vs_he_addr;
	u16 vs_vs_addr;
	u16 vs_ve_addr;

	u16 vso_hstart;
	u16 vso_vstart;
	u16 vso_user;

} Lcd_Timing_t;
*/

typedef struct {
	u32 hpll_clk;
	u32 hpll_od;
	u32 hdmi_pll_cntl5;

	u32 lcd_clk;		/* lcd clock = pixel clock*/
	u32 pll_ctrl;		/* video PLL settings */
	u32 div_ctrl;		/* video pll div settings */
	u32 clk_ctrl;		/* video clock settings */  //[31]clk_auto, [11:8]ss_ctrl, [7:0]xd
	unsigned char frame_rate_adj_type; /* 0=htotal adjust, 1=clock adjust */

    u16 sync_duration_num;
    u16 sync_duration_den;

    u16 sth1_hs_addr;
    u16 sth1_he_addr;
    u16 sth1_vs_addr;
    u16 sth1_ve_addr;

    u16 stv1_hs_addr;
    u16 stv1_he_addr;
    u16 stv1_vs_addr;
    u16 stv1_ve_addr;
} Lcd_Timing_t;

typedef struct {
	int lvds_bits;				// 6 / 8 /10  bits
    int lvds_repack;
    int pn_swap;
    int dual_port;
    int port_reverse;
    int lvds_fifo_wr_mode;
} Lvds_Config_t;

typedef struct {
	int lane_count;
    int byte_mode;
    int region;
    int color_fmt;
    int phy_div;
    unsigned int bit_rate;
} Vbyone_Config_t;

#ifdef CONFIG_AML_LCD_EXTERN
struct lcd_extern_config_s {
	unsigned int index;
	unsigned int on_delay;
	unsigned int off_delay;
};
#endif

typedef struct {
	Lvds_Config_t *lvds_config;
	Vbyone_Config_t *vbyone_config;
#ifdef CONFIG_AML_LCD_EXTERN
	struct lcd_extern_config_s *ext_config;
#endif
} Lcd_Control_Config_t;

//****panel power control only for uboot ***//
typedef struct {
	unsigned int gpio;
	unsigned short on_value;
	unsigned short off_value;
	unsigned short panel_on_delay;
	unsigned short panel_off_delay;
} Panel_Power_Config_t;

typedef struct {
	Panel_Power_Config_t *panel_power;
} Lcd_Power_Ctrl_t;

typedef struct {
    Lcd_Basic_t lcd_basic;
	Lcd_Timing_t lcd_timing;
	Lcd_Control_Config_t lcd_control;
	Lcd_Power_Ctrl_t lcd_power_ctrl;
} Lcd_Config_t;


struct aml_lcd {
	Lcd_Config_t *pConf;
	vinfo_t lcd_info;
	struct aml_lcd_platdata *pd;

	struct device		*dev;
	struct mutex init_lock;
	struct timer_list timer;
};

struct aml_lcd_platdata {
	Lcd_Config_t *pConf;
};

#define TV_LCD_INFO(format, arg...) printk(KERN_INFO "lcd: %s: " format, \
		__FUNCTION__ , ## arg)
#define TV_LCD_ERR(format, arg...)  printk(KERN_ERR "lcd error: %s: " format, \
		__FUNCTION__ , ## arg)

#define VX1_INTERVAL (HZ)

extern unsigned int  lvds_init(struct aml_lcd *pDev);
extern unsigned int vbyone_init(struct aml_lcd *pDev);
extern void lcd_function_remove(void);
extern void vbyone_timer_handler(unsigned long arg);
extern void vbyone_aln_clk_disable(void);

#endif
