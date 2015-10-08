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
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <mach/register.h>
#include <mach/am_regs.h>
#include <mach/lcd_reg.h>
#include <linux/amlogic/vout/vinfo.h>
#include <linux/amlogic/vout/vout_notify.h>
#include <linux/amlogic/display/lcd.h>
#ifdef CONFIG_AML_LCD_EXTERN
#include <linux/amlogic/vout/aml_lcd_extern.h>
#endif

#include "aml_lcd_tv.h"
#include "aml_tv_lcd_port/lcd_common.h"

#define PANEL_NAME		"aml_lcd_tv"

static DEFINE_MUTEX(lcd_vout_mutex);
static struct notifier_block lcd_power_nb;
static char lcd_propname[30] = "lvds_0";
struct aml_lcd *pDev = NULL;
static struct class *tv_lcd_class;
#define LCD_ARG_NUM 4

static unsigned short h_period_store, v_period_store;

static const char* lcd_type_table[]={
	"LVDS",
	"vbyone",
	"TTL",
	"invalid",
};

unsigned int (*init_lcd_port[])(struct aml_lcd *pDev_t) = {
	lvds_init,
	vbyone_init,
};

static unsigned int lcd_output_mode = 0;
enum {
	LCD_OUTPUT_MODE_1080P = 0,
	LCD_OUTPUT_MODE_1080P50HZ,
	LCD_OUTPUT_MODE_768P60HZ,
	LCD_OUTPUT_MODE_768P50HZ,
	LCD_OUTPUT_MODE_4K2K60HZ420,
	LCD_OUTPUT_MODE_4K2K50HZ420,
	LCD_OUTPUT_MODE_4K2K60HZ,
	LCD_OUTPUT_MODE_4K2K50HZ,
	LCD_OUTPUT_MODE_MAX,
};

static const vinfo_t lcd_info[] = {
	{
		.name              = "1080p",
		.mode              = VMODE_1080P,
		.width             = 1920,
		.height            = 1080,
		.field_height      = 1080,
		.aspect_ratio_num  = 16,
		.aspect_ratio_den  = 9,
		.sync_duration_num = 60,
		.sync_duration_den = 1,
		.viu_color_fmt = TVIN_RGB444,
	},
	{
		.name              = "1080p50hz",
		.mode              = VMODE_1080P_50HZ,
		.width             = 1920,
		.height            = 1080,
		.field_height      = 1080,
		.aspect_ratio_num  = 16,
		.aspect_ratio_den  = 9,
		.sync_duration_num = 50,
		.sync_duration_den = 1,
		.viu_color_fmt = TVIN_RGB444,
	},
	{
		.name              = "768p60hz",
		.mode              = VMODE_768P,
		.width             = 1366,
		.height            = 768,
		.field_height      = 768,
		.aspect_ratio_num  = 16,
		.aspect_ratio_den  = 9,
		.sync_duration_num = 60,
		.sync_duration_den = 1,
		.viu_color_fmt = TVIN_RGB444,
	},
	{
		.name              = "768p50hz",
		.mode              = VMODE_768P_50HZ,
		.width             = 1366,
		.height            = 768,
		.field_height      = 768,
		.aspect_ratio_num  = 16,
		.aspect_ratio_den  = 9,
		.sync_duration_num = 50,
		.sync_duration_den = 1,
		.viu_color_fmt = TVIN_RGB444,
	},
	{
		.name              = "4k2k60hz420",
		.mode              = VMODE_4K2K_60HZ_Y420,
		.width             = 3840,
		.height            = 2160,
		.field_height      = 2160,
		.aspect_ratio_num  = 16,
		.aspect_ratio_den  = 9,
		.sync_duration_num = 60,
		.sync_duration_den = 1,
		.viu_color_fmt = TVIN_RGB444,
	},
	{
		.name              = "4k2k50hz420",
		.mode              = VMODE_4K2K_50HZ_Y420,
		.width             = 3840,
		.height            = 2160,
		.field_height      = 2160,
		.aspect_ratio_num  = 16,
		.aspect_ratio_den  = 9,
		.sync_duration_num = 50,
		.sync_duration_den = 1,
		.viu_color_fmt = TVIN_RGB444,
	},
	{
		.name              = "4k2k60hz",
		.mode              = VMODE_4K2K_60HZ,
		.width             = 3840,
		.height            = 2160,
		.field_height      = 2160,
		.aspect_ratio_num  = 16,
		.aspect_ratio_den  = 9,
		.sync_duration_num = 60,
		.sync_duration_den = 1,
		.viu_color_fmt = TVIN_RGB444,
	},
	{
		.name              = "4k2k50hz",
		.mode              = VMODE_4K2K_50HZ,
		.width             = 3840,
		.height            = 2160,
		.field_height      = 2160,
		.aspect_ratio_num  = 16,
		.aspect_ratio_den  = 9,
		.sync_duration_num = 50,
		.sync_duration_den = 1,
		.viu_color_fmt = TVIN_RGB444,
	},
	{
		.name              = "invalid",
		.mode              = VMODE_INIT_NULL,
		.width             = 1920,
		.height            = 1080,
		.field_height      = 1080,
		.aspect_ratio_num  = 16,
		.aspect_ratio_den  = 9,
		.sync_duration_num = 60,
		.sync_duration_den = 1,
		.viu_color_fmt = TVIN_RGB444,
	},
};

static int get_lcd_vmode(vmode_t mode)
{
	int i, count = ARRAY_SIZE(lcd_info) - 1;

	for (i = 0; i < count; i++) {
		if (mode == lcd_info[i].mode)
			break;
	}
	return i;
}

static int lcd_vmode_is_mached(int index)
{
	if ((pDev->pConf->lcd_basic.h_active == lcd_info[index].width) &&
		(pDev->pConf->lcd_basic.v_active == lcd_info[index].height))
		return 0;
	else
		return -1;
}

static const vinfo_t *get_valid_vinfo(char *mode)
{
	const vinfo_t *vinfo = NULL;
	int i, count = ARRAY_SIZE(lcd_info);
	int mode_name_len=0;
	int ret;

	for (i = 0; i < count; i++) {
		if (strncmp(lcd_info[i].name, mode, strlen(lcd_info[i].name)) == 0) {
			if ((vinfo == NULL) || (strlen(lcd_info[i].name) > mode_name_len)) {
				ret = lcd_vmode_is_mached(i);
				if (ret == 0) {
					vinfo = &lcd_info[i];
					mode_name_len = strlen(lcd_info[i].name);
				}
			}
		}
	}
	return vinfo;
}

//******************************************************
//debug function
//******************************************************
static ssize_t tv_lcd_spread_show(struct class *class,
	struct class_attribute *attr, char *buf)
{
	int len = 0;
	unsigned int level;

	level = get_tv_lcd_spread_spectrum();
	len += sprintf(buf + len, "level = %d\n",level);

	return len;
}

static ssize_t tv_lcd_spread_stroe(struct class *cls,
	struct class_attribute *attr, const char *buf, size_t count)
{
	static unsigned int level = 0;
	sscanf(buf, "%d", (unsigned int*)&level);

	set_tv_lcd_spread_spectrum(level);

	return count;
}

static const char * lcd_usage_str = {"Usage:\n"
"    echo test <num> > debug ; bist pattern test, 0=pattern off, 1~7=different pattern\n"
"    echo fr_type <type> > debug ; set frame_rate change type: 0=hotal adjust, 1=clock adjust\n"
"    echo info > debug; dump lcd parameters\n"
"    echo reg > debug; dump lcd regs\n"
"    echo status > debug; dump lcd status\n"
"    echo dump > debug; dump all lcd information\n"
};

static void tv_lcd_timing_info_print(void)
{
	unsigned int hs_width, hs_bp, h_period;
	unsigned int vs_width, vs_bp, v_period;
	unsigned int video_on_pixel, video_on_line;
	unsigned int sync_start, sync_end;

	video_on_pixel = pDev->pConf->lcd_basic.video_on_pixel;
	video_on_line = pDev->pConf->lcd_basic.video_on_line;
	h_period = pDev->pConf->lcd_basic.h_period;
	v_period = pDev->pConf->lcd_basic.v_period;

	sync_start = pDev->pConf->lcd_timing.sth1_hs_addr;
	sync_end = pDev->pConf->lcd_timing.sth1_he_addr;
	hs_width = sync_end - sync_start;
	hs_bp = video_on_pixel - sync_end;

	sync_start = pDev->pConf->lcd_timing.stv1_vs_addr;
	sync_end = pDev->pConf->lcd_timing.stv1_ve_addr;
	vs_width = sync_end - sync_start;
	vs_bp = video_on_line - sync_end;

	printk("h_period          %d\n"
	   "v_period          %d\n"
	   "hs_width          %d\n"
	   "hs_backporch      %d\n"
	   "vs_width          %d\n"
	   "vs_backporch      %d\n"
	   "video_on_pixel    %d\n"
	   "video_on_line     %d\n\n",
	   h_period, v_period, hs_width, hs_bp, vs_width, vs_bp,
	   video_on_pixel, video_on_line);
}

static void tv_lcd_info_parameters(void)
{
	unsigned int lcd_clk;
	unsigned int sync_duration;

	printk("lcd: parameters info:\n");

	lcd_clk = (pDev->pConf->lcd_timing.lcd_clk / 1000);
	sync_duration = lcd_info[lcd_output_mode].sync_duration_num;
	sync_duration = (sync_duration * 10 / lcd_info[lcd_output_mode].sync_duration_den);
	printk("LCD mode: %s, %s, %ux%u@%u.%uHz\n"
	   "fr_adj_type       %d\n"
	   "lcd_clk           %u.%03uMHz\n\n",
	   pDev->pConf->lcd_basic.model_name,
	   lcd_type_table[pDev->pConf->lcd_basic.lcd_type],
	   pDev->pConf->lcd_basic.h_active, pDev->pConf->lcd_basic.v_active,
	   (sync_duration / 10), (sync_duration % 10),
	   pDev->pConf->lcd_timing.frame_rate_adj_type,
	   (lcd_clk / 1000), (lcd_clk % 1000));

	tv_lcd_timing_info_print();

	switch (pDev->pConf->lcd_basic.lcd_type) {
	case LCD_DIGITAL_LVDS:
		printk("lvds_bits         %u\n"
		   "lvds_repack       %u\n"
		   "pn_swap           %u\n"
		   "dual_port         %u\n"
		   "port_reverse      %u\n"
		   "lvds_fifo_wr_mode %u\n\n",
		   pDev->pConf->lcd_control.lvds_config->lvds_bits,
		   pDev->pConf->lcd_control.lvds_config->lvds_repack,
		   pDev->pConf->lcd_control.lvds_config->pn_swap,
		   pDev->pConf->lcd_control.lvds_config->dual_port,
		   pDev->pConf->lcd_control.lvds_config->port_reverse,
		   pDev->pConf->lcd_control.lvds_config->lvds_fifo_wr_mode);
		break;
	case LCD_DIGITAL_VBYONE:
		printk("lane_count        %u\n"
		   "byte_mode         %u\n"
		   "region            %u\n\n",
		   pDev->pConf->lcd_control.vbyone_config->lane_count,
		   pDev->pConf->lcd_control.vbyone_config->byte_mode,
		   pDev->pConf->lcd_control.vbyone_config->region);
		break;
	default:
		break;
	}

	printk("power gpio        %d\n"
	   "power on_value    %d\n"
	   "power off_value   %d\n"
	   "power on_delay    %d\n"
	   "power off_delay   %d\n\n",
	   pDev->pConf->lcd_power_ctrl.panel_power->gpio,
	   pDev->pConf->lcd_power_ctrl.panel_power->on_value,
	   pDev->pConf->lcd_power_ctrl.panel_power->off_value,
	   pDev->pConf->lcd_power_ctrl.panel_power->panel_on_delay,
	   pDev->pConf->lcd_power_ctrl.panel_power->panel_off_delay);
#ifdef CONFIG_AML_LCD_EXTERN
	printk("extern index      %d\n"
	   "extern on_delay   %d\n"
	   "extern off_delay  %d\n\n",
	   pDev->pConf->lcd_control.ext_config->index,
	   pDev->pConf->lcd_control.ext_config->on_delay,
	   pDev->pConf->lcd_control.ext_config->off_delay);
#endif
}

static void tv_lcd_reg_dump(void)
{
	unsigned int reg_start, reg_end, count, reg;
	int i;

	printk("lcd: reg dump:\n");

	/* clk setting: cbus */
	printk("clock regs:\n");
	reg_start = HHI_HDMI_PLL_CNTL;
	reg_end = HHI_HDMI_PLL_CNTL6;
	count = reg_end - reg_start;
	for (i = 0; i <= count; i++) {
		reg = reg_start + i;
		printk("0x%04x = 0x%08x\n", reg, READ_LCD_CBUS_REG(reg));
	}
	reg = HHI_VID_PLL_CLK_DIV;
	printk("0x%04x = 0x%08x\n", reg, READ_LCD_CBUS_REG(reg));
	reg = HHI_VID_CLK_DIV;
	printk("0x%04x = 0x%08x\n", reg, READ_LCD_CBUS_REG(reg));
	reg = HHI_VID_CLK_CNTL;
	printk("0x%04x = 0x%08x\n", reg, READ_LCD_CBUS_REG(reg));
	reg = HHI_VIID_CLK_DIV;
	printk("0x%04x = 0x%08x\n", reg, READ_LCD_CBUS_REG(reg));
	reg = HHI_VIID_CLK_CNTL;
	printk("0x%04x = 0x%08x\n", reg, READ_LCD_CBUS_REG(reg));
	reg = HHI_VID_CLK_CNTL2;
	printk("0x%04x = 0x%08x\n", reg, READ_LCD_CBUS_REG(reg));
	printk("\n");

	/* encl setting: vcbus */
	printk("encl regs:\n");
	reg_start = ENCL_VIDEO_EN;
	reg_end = ENCL_MAX_LINE_SWITCH_POINT;
	count = reg_end - reg_start;
	for (i = 0; i <= count; i++) {
		reg = reg_start + i;
		printk("0x%04x = 0x%08x\n", reg, READ_LCD_REG(reg));
	}
	reg = VPU_VIU_VENC_MUX_CTRL;
	printk("0x%04x = 0x%08x\n", reg, READ_LCD_REG(reg));
	printk("\n");

	/* interface controller setting */
	switch (pDev->pConf->lcd_basic.lcd_type) {
	case LCD_DIGITAL_LVDS:
		printk("lvds regs:\n");
		reg = HHI_LVDS_TX_PHY_CNTL0;
		printk("0x%04x = 0x%08x\n", reg, READ_LCD_CBUS_REG(reg));
		reg = HHI_LVDS_TX_PHY_CNTL1;
		printk("0x%04x = 0x%08x\n", reg, READ_LCD_CBUS_REG(reg));
		reg = LVDS_GEN_CNTL;
		printk("0x%04x = 0x%08x\n", reg, READ_LCD_REG(reg));
		reg = LVDS_PACK_CNTL_ADDR;
		printk("0x%04x = 0x%08x\n", reg, READ_LCD_REG(reg));
		printk("\n");

		reg_start = LVDS_PHY_CNTL0;
		reg_end = LVDS_PHY_CNTL4;
		count = reg_end - reg_start;
		for (i = 0; i <= count; i++) {
			reg = reg_start + i;
			printk("0x%04x = 0x%08x\n", reg, READ_LCD_REG(reg));
		}
		printk("\n");
		break;
	case LCD_DIGITAL_VBYONE:
		printk("vbyone regs:\n");
		reg_start = VBO_CTRL_L;
		reg_end = LCD_PORT_SWAP;
		count = reg_end - reg_start;
		for (i = 0; i <= count; i++) {
			reg = reg_start + i;
			printk("0x%04x = 0x%08x\n", reg, READ_LCD_REG(reg));
		}
		printk("\n");

		printk("pinmux & gpio regs:\n");
		reg = PERIPHS_PIN_MUX_7;
		printk("0x%04x = 0x%08x\n", reg, READ_LCD_CBUS_REG(reg));
		reg_start = PREG_PAD_GPIO1_EN_N;
		reg_end = PREG_PAD_GPIO1_I;
		count = reg_end - reg_start;
		for (i = 0; i <= count; i++) {
			reg = reg_start + i;
			printk("0x%04x = 0x%08x\n", reg, READ_LCD_CBUS_REG(reg));
		}
		printk("\n");
	default:
		break;
	}

	/* phy setting: cbus */
	printk("phy regs:\n");
	reg_start = HHI_DIF_CSI_PHY_CNTL1;
	reg_end = HHI_DIF_CSI_PHY_CNTL3;
	count = reg_end - reg_start;
	for (i = 0; i <= count; i++) {
		reg = reg_start + i;
		printk("0x%04x = 0x%08x\n", reg, READ_LCD_CBUS_REG(reg));
	}
	printk("\n");
}

static void tv_lcd_status_dump(void)
{
	unsigned int status;

	status = aml_read_reg32(P_ENCL_VIDEO_EN);
	printk("lcd: status: %s\n", (status ? "ON" : "OFF"));
}

static ssize_t tv_lcd_debug_show(struct class *class,
	struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n",lcd_usage_str);
}

static ssize_t tv_lcd_debug_store(struct class *cls,
	struct class_attribute *attr, const char *buf, size_t count)
{
	int argn;
	char * buf_work,*p,*para;
	char * argv[LCD_ARG_NUM];
	char cmd;
	unsigned int value;

	buf_work = kstrdup(buf, GFP_KERNEL);
	p = buf_work;

	for (argn = 0; argn < LCD_ARG_NUM; argn++) {
		para = strsep(&p," ");
		if (para == NULL)
			break;
		argv[argn] = para;
		//printk("argv[%d] = %s\n",argn,para);
	}

	if (argn < 1 || argn > LCD_ARG_NUM)
		goto end;

	if (pDev == NULL) {
		printk("lcd: [error]: no lcd device, exit\n");
		kfree(buf_work);
		return -EINVAL;
	}

	cmd = argv[0][0];
	switch (cmd) {
	case 't':
		if (argn != 2) {
			printk("%s",lcd_usage_str);
			goto end;
		}
		value = simple_strtol(argv[1], NULL, 16);
		lcd_test(value, pDev);
		break;
	case 'f':
		value = simple_strtol(argv[1], NULL, 10);
		pDev->pConf->lcd_timing.frame_rate_adj_type = (unsigned char)value;
		printk("change lcd frame rate change type: %d\n", value);
		break;
	case 'i':
		printk("lcd: driver version: %s\n", LCD_TV_DRIVER_VERSION);
		tv_lcd_info_parameters();
		break;
	case 'r':
		printk("lcd: driver version: %s\n", LCD_TV_DRIVER_VERSION);
		tv_lcd_reg_dump();
		break;
	case 's':
		printk("lcd: driver version: %s\n", LCD_TV_DRIVER_VERSION);
		tv_lcd_status_dump();
		break;
	case 'd':
		printk("lcd: driver version: %s\n", LCD_TV_DRIVER_VERSION);
		tv_lcd_info_parameters();
		tv_lcd_reg_dump();
		tv_lcd_status_dump();
		break;
	default:
		goto end;
	}

	kfree(buf_work);
	return count;
end:
	printk("error command!\n");
	kfree(buf_work);
	return -EINVAL;
}

static CLASS_ATTR(spread, S_IWUSR | S_IRUGO, tv_lcd_spread_show, tv_lcd_spread_stroe);
static CLASS_ATTR(debug, S_IWUSR | S_IRUGO, tv_lcd_debug_show, tv_lcd_debug_store);
//******************************************************

#ifdef CONFIG_USE_OF
static inline int aml_lcd_get_property_string(struct device_node *np,
	const char *propname, const char **out_string)
{
	int ret;

	ret = of_property_read_string(np, propname, out_string);
	if (ret) {
		TV_LCD_ERR("faild to get %s !\n",propname);
		*out_string = "invalid";
	}

	return ret;
}

static inline int aml_lcd_get_property_array(struct device_node* of_node,
	const char *propname, u32 *out_values, size_t sz)
{
	int ret;

	ret= of_property_read_u32_array(of_node,propname,out_values,sz);
	if (ret)
		TV_LCD_ERR("faild to get %s !\n",propname);

	return ret;
}

static struct aml_lcd_platdata * _get_lcd_config(struct platform_device *pdev)
{
	struct device_node* of_node = pdev->dev.of_node;
	struct device_node* child;
	const char *str;
	unsigned int val;
	unsigned int panel_power_pin;
	unsigned int *lcd_para = NULL;

	struct aml_lcd_platdata *pd;
	Lcd_Config_t *plcd_config;
	Panel_Power_Config_t *ppanel_power_config;
#ifdef CONFIG_AML_LCD_EXTERN
	struct lcd_extern_config_s *ext_conf;
#endif

	pd = kmalloc(sizeof(struct aml_lcd_platdata), GFP_KERNEL);
	if (pd == NULL) {
		TV_LCD_ERR("failed to get pd Not enough memory\n");
		return NULL;
	}
	memset(pd, 0, sizeof(*pd));

	plcd_config = kmalloc(sizeof(Lcd_Config_t), GFP_KERNEL);
	if (plcd_config == NULL) {
		TV_LCD_ERR("failed to get plcd_config Not enough memory\n");
		return NULL;
	}
	memset(plcd_config, 0, sizeof(*plcd_config));
	pd->pConf = plcd_config;

	ppanel_power_config = kmalloc(sizeof(Panel_Power_Config_t), GFP_KERNEL);
	if (ppanel_power_config == NULL) {
		TV_LCD_ERR("plcd_config ppanel_power_config Not enough memory\n");
		return NULL;
	}
	memset(ppanel_power_config, 0, sizeof(*ppanel_power_config));
	pd->pConf->lcd_power_ctrl.panel_power = ppanel_power_config;

#ifdef CONFIG_AML_LCD_EXTERN
	ext_conf = kmalloc(sizeof(struct lcd_extern_config_s), GFP_KERNEL);
	if (ext_conf == NULL) {
		TV_LCD_ERR("plcd_config ext_conf Not enough memory\n");
		return NULL;
	}
	memset(ext_conf, 0, sizeof(*ext_conf));
	pd->pConf->lcd_control.ext_config = ext_conf;
#endif

	lcd_para = (unsigned int *)kmalloc(sizeof(unsigned int)*20, GFP_KERNEL);
	if (lcd_para == NULL) {
		kfree(pd);
		kfree(plcd_config);
		kfree(ppanel_power_config);
		TV_LCD_ERR("ppanel_power_config lcd_para Not enough memory\n");
		return NULL;
	}
	memset(lcd_para, 0, sizeof(*lcd_para));

	if (of_node) {
		pd->pConf->lcd_basic.model_name = lcd_propname;
		child = of_get_child_by_name(of_node, lcd_propname);
		if (child == NULL) {
			kfree(lcd_para);
			kfree(pd);
			kfree(plcd_config);
			kfree(ppanel_power_config);
			TV_LCD_ERR("faild to get lcd_model_config!! \n");
			return NULL;
		}

		if (aml_lcd_get_property_string(child, "interface", &str)) {
			kfree(lcd_para);
			kfree(pd);
			kfree(plcd_config);
			kfree(ppanel_power_config);
			return NULL;
		} else {
			for (val = 0; val < LCD_TYPE_MAX; val++) {
			if (!strcasecmp(str, lcd_type_table[val]))
				break;
			}
			plcd_config->lcd_basic.lcd_type = val;
		}

		if (of_property_read_u32(child, "frame_rate_adjust_type", &val)) {
			TV_LCD_INFO("failed to get frame_rate_adjust_type, default to 0\n");
			/* default use clock adjust */
			plcd_config->lcd_timing.frame_rate_adj_type = 0;
		} else {
			plcd_config->lcd_timing.frame_rate_adj_type = (unsigned char)val;
		}

		if (aml_lcd_get_property_array(child, "basic_setting", &lcd_para[0], 6)) {
			kfree(lcd_para);
			kfree(pd);
			kfree(plcd_config);
			kfree(ppanel_power_config);
			return NULL;
		} else {
			plcd_config->lcd_basic.h_active = lcd_para[0];
			plcd_config->lcd_basic.v_active = lcd_para[1];
			plcd_config->lcd_basic.h_period = lcd_para[2];
			plcd_config->lcd_basic.v_period = lcd_para[3];
			plcd_config->lcd_basic.video_on_pixel = lcd_para[4];
			plcd_config->lcd_basic.video_on_line = lcd_para[5];
		}

		if (aml_lcd_get_property_array(child, "lcd_timing", &lcd_para[0], 11)) {
			kfree(lcd_para);
			kfree(pd);
			kfree(plcd_config);
			kfree(ppanel_power_config);
			return NULL;
		} else {
			plcd_config->lcd_timing.hpll_clk = lcd_para[0];
			plcd_config->lcd_timing.hpll_od  = lcd_para[1];
			plcd_config->lcd_timing.hdmi_pll_cntl5 = lcd_para[2];
			plcd_config->lcd_timing.sth1_hs_addr	 = lcd_para[3];
			plcd_config->lcd_timing.sth1_he_addr	 = lcd_para[4];
			plcd_config->lcd_timing.sth1_vs_addr	 = lcd_para[5];
			plcd_config->lcd_timing.sth1_ve_addr	 = lcd_para[6];
			plcd_config->lcd_timing.stv1_hs_addr	 = lcd_para[7];
			plcd_config->lcd_timing.stv1_he_addr	 = lcd_para[8];
			plcd_config->lcd_timing.stv1_vs_addr	 = lcd_para[9];
			plcd_config->lcd_timing.stv1_ve_addr	 = lcd_para[10];
		}

		if (aml_lcd_get_property_string(child, "panel_power_pin", &str)) {
			kfree(lcd_para);
			kfree(pd);
			kfree(plcd_config);
			kfree(ppanel_power_config);
			return NULL;
		} else {
			panel_power_pin = amlogic_gpio_name_map_num(str);
			if (panel_power_pin<0) {
				kfree(lcd_para);
				TV_LCD_ERR("wrong gpio number %s\n",str);
				return NULL;
			}
			ppanel_power_config->gpio	= panel_power_pin;
		}

		if (aml_lcd_get_property_array(child, "panel_power_att", &lcd_para[0], 4)) {
			kfree(lcd_para);
			kfree(pd);
			kfree(plcd_config);
			kfree(ppanel_power_config);
			return NULL;
		} else {
			ppanel_power_config->on_value  = lcd_para[0];
			ppanel_power_config->off_value = lcd_para[1];
			ppanel_power_config->panel_on_delay = lcd_para[2];
			ppanel_power_config->panel_off_delay = lcd_para[3];
		}

		if (plcd_config->lcd_basic.lcd_type == LCD_DIGITAL_LVDS) {
/*			if (aml_lcd_get_property_array(child, "pixel_clock", &lcd_para[0], 1)) {
				kfree(lcd_para);
				kfree(pd);
				kfree(plcd_config);
				kfree(ppanel_power_config);
				return NULL;
			} else {
				plcd_config->lcd_timing.lcd_clk = lcd_para[0];
			}
*/
			if (aml_lcd_get_property_array(child, "lvds_att", &lcd_para[0], 6)) {
				kfree(lcd_para);
				kfree(pd);
				kfree(plcd_config);
				kfree(ppanel_power_config);
				return NULL;
			}else {
				Lvds_Config_t *plvds_config;
				plvds_config = kmalloc(sizeof(Lvds_Config_t), GFP_KERNEL);
				if (plvds_config == NULL) {
					TV_LCD_ERR("pvbyone_config plvds_config Not enough memory\n");
					kfree(lcd_para);
					kfree(pd);
					kfree(plcd_config);
					kfree(ppanel_power_config);
					return NULL;
				}
				memset(plvds_config, 0, sizeof(*plvds_config));
				pd->pConf->lcd_control.lvds_config = plvds_config;

				plvds_config->lvds_bits	 = lcd_para[0];
				plvds_config->lvds_repack  = lcd_para[1];
				plvds_config->pn_swap 	 = lcd_para[2];
				plvds_config->dual_port	 = lcd_para[3];
				plvds_config->port_reverse		 = lcd_para[4];
				plvds_config->lvds_fifo_wr_mode	 = lcd_para[5];
			}

		} else if (plcd_config->lcd_basic.lcd_type == LCD_DIGITAL_VBYONE) {
			if (aml_lcd_get_property_array(child, "vbyone_att", &lcd_para[0], 4)) {
				kfree(lcd_para);
				kfree(pd);
				kfree(plcd_config);
				kfree(ppanel_power_config);
				return NULL;
			} else {
				Vbyone_Config_t *pvbyone_config;
				pvbyone_config = kmalloc(sizeof(Vbyone_Config_t), GFP_KERNEL);
				if (pvbyone_config == NULL) {
					kfree(lcd_para);
					kfree(pd);
					kfree(plcd_config);
					kfree(ppanel_power_config);
					TV_LCD_ERR("failed to get pvbyone_config Not enough memory\n");
					return NULL;
				}
				memset(pvbyone_config, 0, sizeof(*pvbyone_config));
				pd->pConf->lcd_control.vbyone_config= pvbyone_config;

				pvbyone_config->lane_count	= lcd_para[0];
				pvbyone_config->byte_mode	= lcd_para[1];
				pvbyone_config->region		= lcd_para[2];
				pvbyone_config->color_fmt 	= lcd_para[3];
			}
		} else if (plcd_config->lcd_basic.lcd_type == LCD_DIGITAL_TTL) {

		}

#ifdef CONFIG_AML_LCD_EXTERN
		if (aml_lcd_get_property_array(child, "lcd_extern_att", &lcd_para[0], 3)) {
			ext_conf->index = LCD_EXTERN_INDEX_INVALID;
			ext_conf->on_delay = 0;
			ext_conf->off_delay = 0;
		} else {
			ext_conf->index = lcd_para[0];
			ext_conf->on_delay = lcd_para[1];
			ext_conf->off_delay = lcd_para[2];
		}
#endif
	}

	TV_LCD_INFO("lcd_type = %s(%s)\n", lcd_type_table[pd->pConf->lcd_basic.lcd_type],lcd_propname);
	TV_LCD_INFO("h_active = %u, v_active = %u \n", pd->pConf->lcd_basic.h_active, pd->pConf->lcd_basic.v_active);
	TV_LCD_INFO("h_period = %u, v_period = %u \n", pd->pConf->lcd_basic.h_period, pd->pConf->lcd_basic.v_period );
	TV_LCD_INFO("video_on_pixel = %u, video_on_line = %u \n", pd->pConf->lcd_basic.video_on_pixel, pd->pConf->lcd_basic.video_on_line);
	TV_LCD_INFO("hpll_clk = %x, hpll_od =%x hdmi_pll_cntl5 = %x \n", pd->pConf->lcd_timing.hpll_clk, pd->pConf->lcd_timing.hpll_od, pd->pConf->lcd_timing.hdmi_pll_cntl5);
	TV_LCD_INFO("sth1_hs_addr = %u, sth1_he_addr = %u \n", pd->pConf->lcd_timing.sth1_hs_addr, pd->pConf->lcd_timing.sth1_he_addr);
	TV_LCD_INFO("sth1_vs_addr = %u, sth1_ve_addr = %u \n", pd->pConf->lcd_timing.sth1_vs_addr, pd->pConf->lcd_timing.sth1_ve_addr);
	TV_LCD_INFO("stv1_hs_addr = %u, stv1_he_addr = %u \n", pd->pConf->lcd_timing.stv1_hs_addr, pd->pConf->lcd_timing.stv1_he_addr);
	TV_LCD_INFO("stv1_vs_addr = %u, stv1_ve_addr = %u \n", pd->pConf->lcd_timing.stv1_vs_addr, pd->pConf->lcd_timing.stv1_ve_addr);
	TV_LCD_INFO("frame_rate_adjust_type = %u\n", pd->pConf->lcd_timing.frame_rate_adj_type);

	TV_LCD_INFO("panel_power_pin: %s--%d \n",str,pd->pConf->lcd_power_ctrl.panel_power->gpio);
	TV_LCD_INFO("on_value = %d \n",pd->pConf->lcd_power_ctrl.panel_power->on_value);
	TV_LCD_INFO("off_value = %d \n",pd->pConf->lcd_power_ctrl.panel_power->off_value);
	TV_LCD_INFO("panel_on_delay = %d \n",pd->pConf->lcd_power_ctrl.panel_power->panel_on_delay);
	TV_LCD_INFO("panel_off_delay = %d \n",pd->pConf->lcd_power_ctrl.panel_power->panel_off_delay);

	if (pd->pConf->lcd_basic.lcd_type == LCD_DIGITAL_LVDS) {
		TV_LCD_INFO("lvds_bits = %d \n",pd->pConf->lcd_control.lvds_config->lvds_bits);
		TV_LCD_INFO("lvds_repack = %d \n",pd->pConf->lcd_control.lvds_config->lvds_repack);
		TV_LCD_INFO("pn_swap = %d \n",pd->pConf->lcd_control.lvds_config->pn_swap);
		TV_LCD_INFO("dual_port = %d \n",pd->pConf->lcd_control.lvds_config->dual_port);
		TV_LCD_INFO("port_reverse = %d \n",pd->pConf->lcd_control.lvds_config->port_reverse);
		TV_LCD_INFO("lvds_fifo_wr_mode = %d \n",pd->pConf->lcd_control.lvds_config->lvds_fifo_wr_mode);
		//TV_LCD_INFO("pixel_clock = %d \n", pd->pConf->lcd_timing.lcd_clk);
	} else if (pd->pConf->lcd_basic.lcd_type == LCD_DIGITAL_VBYONE) {
		TV_LCD_INFO("lane_count = %d \n",pd->pConf->lcd_control.vbyone_config->lane_count);
		TV_LCD_INFO("byte_mode = %d \n",pd->pConf->lcd_control.vbyone_config->byte_mode);
		TV_LCD_INFO("region = %d \n",pd->pConf->lcd_control.vbyone_config->region);
		TV_LCD_INFO("color_fmt = %d \n",pd->pConf->lcd_control.vbyone_config->color_fmt);
	} else if (pd->pConf->lcd_basic.lcd_type == LCD_DIGITAL_TTL) {

	}
#ifdef CONFIG_AML_LCD_EXTERN
	if (pd->pConf->lcd_control.ext_config->index < LCD_EXTERN_INDEX_INVALID) {
		TV_LCD_INFO("lcd_extern index = %d \n",pd->pConf->lcd_control.ext_config->index);
		TV_LCD_INFO("lcd_extern on_delay = %d \n",pd->pConf->lcd_control.ext_config->on_delay);
		TV_LCD_INFO("lcd_extern off_delay = %d \n",pd->pConf->lcd_control.ext_config->off_delay);
	}
#endif

	kfree(lcd_para);

	return pd;
}

static void init_lcd_config(struct aml_lcd *pDev)
{
	h_period_store = pDev->pConf->lcd_basic.h_period;
	v_period_store = pDev->pConf->lcd_basic.v_period;

	/* prepare 60hz clock */
	pDev->pConf->lcd_timing.lcd_clk = h_period_store * v_period_store * 60;
}
#endif

static void panel_power_ctrl(Bool_t status)
{
	const char *owner = "aml_tv_lcd";
	Lcd_Config_t *pConf;
#ifdef CONFIG_AML_LCD_EXTERN
	struct aml_lcd_extern_driver_t *ext_drv;
	int index;
#endif

	pConf = pDev->pConf;
	TV_LCD_INFO("statu=%s gpio=%d value=%d \n",(status ? "ON" : "OFF"),
		pConf->lcd_power_ctrl.panel_power->gpio,
		(status ?pConf->lcd_power_ctrl.panel_power->on_value:
				pConf->lcd_power_ctrl.panel_power->off_value));

	if (ON == status) {
		amlogic_gpio_request(pConf->lcd_power_ctrl.panel_power->gpio,owner);
		amlogic_gpio_direction_output(pConf->lcd_power_ctrl.panel_power->gpio,
			pConf->lcd_power_ctrl.panel_power->on_value,owner);
		mdelay(pConf->lcd_power_ctrl.panel_power->panel_on_delay);
#ifdef CONFIG_AML_LCD_EXTERN
		index = pConf->lcd_control.ext_config->index;
		if (index < LCD_EXTERN_INDEX_INVALID) {
			ext_drv = aml_lcd_extern_get_driver(index);
			if (ext_drv)
				ext_drv->power_on();
			if (pConf->lcd_control.ext_config->on_delay > 0)
				mdelay(pConf->lcd_control.ext_config->on_delay);
		}
#endif
	} else {
#ifdef CONFIG_AML_LCD_EXTERN
		index = pConf->lcd_control.ext_config->index;
		if (index < LCD_EXTERN_INDEX_INVALID) {
			if (pConf->lcd_control.ext_config->off_delay > 0)
				mdelay(pConf->lcd_control.ext_config->off_delay);
			ext_drv = aml_lcd_extern_get_driver(index);
			if (ext_drv)
				ext_drv->power_off();
		}
#endif
		mdelay(pConf->lcd_power_ctrl.panel_power->panel_off_delay);
		amlogic_gpio_request(pConf->lcd_power_ctrl.panel_power->gpio,owner);
		amlogic_gpio_direction_output(pConf->lcd_power_ctrl.panel_power->gpio,
			pConf->lcd_power_ctrl.panel_power->off_value,owner);
	}
}

static inline void _init_display_driver(struct aml_lcd *pDev)
{
	if (pDev->pConf->lcd_basic.lcd_type < LCD_TYPE_MAX) {
		init_lcd_port[pDev->pConf->lcd_basic.lcd_type](pDev);
	} else {
		TV_LCD_ERR("no lcd port\n");
		init_lcd_port[LCD_DIGITAL_LVDS](pDev);
	}
}

/* change clock(frame_rate) for different vmode */
static int lcd_vmode_change(int index)
{
	unsigned int pclk;
	unsigned char type = pDev->pConf->lcd_timing.frame_rate_adj_type;
	unsigned int h_period = pDev->pConf->lcd_basic.h_period;
	unsigned int v_period = pDev->pConf->lcd_basic.v_period;
	unsigned int sync_duration_num = lcd_info[index].sync_duration_num;
	unsigned int sync_duration_den = lcd_info[index].sync_duration_den;

	/* frame rate adjust */
	switch (type) {
	case 1: /* htotal adjust */
		pclk = pDev->pConf->lcd_timing.lcd_clk;
		if ((sync_duration_den / sync_duration_num) < 55) { /* 50hz */
			h_period = ((pclk / v_period) * sync_duration_den * 10) / sync_duration_num;
			h_period = (h_period + 5) / 10; /* round off */
		} else { /* 60hz */
			h_period = h_period_store;
		}
		printk("lcd: %s: adjust h_period %u -> %u\n",
			__func__, pDev->pConf->lcd_basic.h_period, h_period);
		pDev->pConf->lcd_basic.h_period = h_period;
		break;
	case 2: /* vtotal adjust */
		pclk = pDev->pConf->lcd_timing.lcd_clk;
		if ((sync_duration_den / sync_duration_num) < 55) { /* 50hz */
			v_period = ((pclk / h_period) * sync_duration_den * 10) / sync_duration_num;
			v_period = (v_period + 5) / 10; /* round off */
		} else { /* 60hz */
			v_period = v_period_store;
		}
		printk("lcd: %s: adjust v_period %u -> %u\n",
			__func__, pDev->pConf->lcd_basic.v_period, v_period);
		pDev->pConf->lcd_basic.v_period = v_period;
		break;
	case 0: /* pixel clk adjust */
	default:
		pclk = (h_period * v_period * sync_duration_num) / sync_duration_den;
		printk("lcd: %s: adjust pclk %u.%03uMHz -> %u.%03uMHz\n",
			__func__, (pDev->pConf->lcd_timing.lcd_clk / 1000000),
			((pDev->pConf->lcd_timing.lcd_clk / 1000) % 1000),
			(pclk / 1000000), ((pclk / 1000) % 1000));
		pDev->pConf->lcd_timing.lcd_clk = pclk;
		break;
	}

	return 0;
}

static void _lcd_module_enable(void)
{
	printk("lcd: driver version: %s\n", LCD_TV_DRIVER_VERSION);
	if (pDev == NULL) {
		printk("lcd: [error]: no lcd device, exit\n");
		return;
	}

	lcd_vmode_change(lcd_output_mode);
	_init_display_driver(pDev);
}

static const vinfo_t *lcd_get_current_info(void)
{
	if (lcd_output_mode >= LCD_OUTPUT_MODE_MAX)
		lcd_output_mode = LCD_OUTPUT_MODE_MAX;
	return &lcd_info[lcd_output_mode];
}

static int lcd_set_current_vmode(vmode_t mode)
{
	unsigned int ret = 0;

	mutex_lock(&lcd_vout_mutex);
/*	printk("initcall %s mode = %d\n", __func__, (mode & VMODE_MODE_BIT_MASK)); */
	lcd_output_mode = get_lcd_vmode(mode & VMODE_MODE_BIT_MASK);

	if (!(mode & VMODE_LOGO_BIT_MASK)) {
		switch (mode & VMODE_MODE_BIT_MASK) {
		case VMODE_1080P:
		case VMODE_1080P_50HZ:
		case VMODE_768P:
		case VMODE_768P_50HZ:
		case VMODE_4K2K_60HZ_Y420:
		case VMODE_4K2K_50HZ_Y420:
		case VMODE_4K2K_60HZ:
		case VMODE_4K2K_50HZ:
			_lcd_module_enable();
			break;
		default:
			ret = -EINVAL;
		}
	}
	mutex_unlock(&lcd_vout_mutex);

	return ret;
}

static vmode_t lcd_validate_vmode(char *mode)
{
	const vinfo_t *info;

	if (mode == NULL)
		return VMODE_MAX;

	info = get_valid_vinfo(mode);
	if (info)
		return info->mode;

	return VMODE_MAX;
}

static int lcd_vmode_is_supported(vmode_t mode)
{
	int m;

	mode &= VMODE_MODE_BIT_MASK;
	m = get_lcd_vmode(mode);
	printk("initcall vmode = %d, lcd mode = %d(%s)\n", mode, m, lcd_info[m].name);

	switch (mode) {
	case VMODE_1080P:
	case VMODE_1080P_50HZ:
	case VMODE_768P:
	case VMODE_768P_50HZ:
	case VMODE_4K2K_60HZ_Y420:
	case VMODE_4K2K_50HZ_Y420:
	case VMODE_4K2K_60HZ:
	case VMODE_4K2K_50HZ:
		return true;
		break;
	default:
		return false;
	}
}

static int lcd_vout_disable(vmode_t cur_vmod)
{
    return 0;
}

#ifdef  CONFIG_PM
static int lcd_suspend(int pm_event)
{
	TV_LCD_INFO("lcd_suspend \n");
	/* in freeze process do not turn off the display devices */
	if (pm_event == PM_EVENT_FREEZE)
		return 0;

	_disable_display_driver();

	return 0;
}
static int lcd_resume(int pm_event)
{
	TV_LCD_INFO("lcd_resume\n");
	/* in thaw/restore process do not reset the display mode */
	if (pm_event == PM_EVENT_THAW ||
		pm_event == PM_EVENT_RESTORE)
		return 0;

	_lcd_module_enable();

	return 0;
}
#endif

static vout_server_t lcd_vout_server = {
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

static void _init_vout(struct aml_lcd *pDev)
{
	aml_write_reg32(P_VPP2_POSTBLEND_H_SIZE, pDev->pConf->lcd_basic.h_active);

	vout_register_server(&lcd_vout_server);
}

static int lcd_power_notifier(struct notifier_block *nb, unsigned long event, void *cmd)
 {
	/* If we aren't interested in this event, skip it immediately */
	if (event != LCD_EVENT_POWERON && event != LCD_EVENT_POWEROFF)
		return 0;

	TV_LCD_INFO("event = %lu\n", event);
	if (event == LCD_EVENT_POWEROFF) {
		_disable_display_driver();
		panel_power_ctrl(OFF);
	}

	return NOTIFY_OK;
}

void lcd_function_remove(void)
{
	switch (pDev->pConf->lcd_basic.lcd_type) {
	case LCD_DIGITAL_LVDS:
		break;
	case LCD_DIGITAL_VBYONE:
		del_timer_sync(&pDev->timer);
		printk("remove vbyone timer handler\n");
		break;
	default:
		break;
	}
}

static void lcd_function_probe(struct aml_lcd *pDev)
{
	switch (pDev->pConf->lcd_basic.lcd_type) {
	case LCD_DIGITAL_LVDS:
		break;
	case LCD_DIGITAL_VBYONE:
		vbyone_aln_clk_disable();
		init_timer(&pDev->timer);
		pDev->timer.data = (ulong)pDev;
		pDev->timer.function = vbyone_timer_handler;
		pDev->timer.expires = jiffies + VX1_INTERVAL;
		add_timer(&pDev->timer);
		printk("add vbyone timer handler\n");
		break;
	default:
		break;
	}
}

static int lcd_probe(struct platform_device *pdev)
{
	unsigned int ret = 0;
	//struct aml_lcd *pDev;
	struct aml_lcd_platdata *pdata;

	printk("lcd: driver version: %s\n", LCD_TV_DRIVER_VERSION);

	pDev = (struct aml_lcd *)kmalloc(sizeof(struct aml_lcd), GFP_KERNEL);
	if (!pDev) {
		TV_LCD_ERR("lcd error: Not enough memory.\n");
		return -ENOMEM;
	}
	memset(pDev, 0, sizeof(*pDev));

	pDev->dev = &pdev->dev;

	pdata = _get_lcd_config(pdev);
	if (pdata == NULL) {
		kfree(pDev);
		TV_LCD_ERR("can not find lcd dtd config \n");
		return -ENOMEM;
	}
	pDev->pConf = pdata->pConf;
	pDev->pd = pdata;
	init_lcd_config(pDev);

	mutex_init(&pDev->init_lock);

	//panel_power_ctrl(ON);
	//udelay(50);
	_init_vout(pDev);
	//lcd_set_current_vmode(VMODE_LCD);
	//lcd_notifier_call_chain(LCD_EVENT_POWERON, NULL);

	platform_set_drvdata(pdev, pDev);

	tv_lcd_class =  class_create(THIS_MODULE, PANEL_NAME);
	ret = class_create_file(tv_lcd_class, &class_attr_spread );
	ret = class_create_file(tv_lcd_class, &class_attr_debug );

	lcd_power_nb.notifier_call = lcd_power_notifier;
	ret = lcd_register_client(&lcd_power_nb);
	if (ret) {
		TV_LCD_ERR("lcd_register_client fail!\n");
	}

	lcd_function_probe(pDev);

	TV_LCD_INFO("LCD probe ok\n");

	return 0;
}

static int lcd_remove(struct platform_device *pdev)
{
	struct aml_lcd *pDev = platform_get_drvdata(pdev);

	lcd_unregister_client(&lcd_power_nb);

	class_remove_file(tv_lcd_class, &class_attr_debug);
	class_remove_file(tv_lcd_class, &class_attr_spread);
	class_destroy(tv_lcd_class);

	platform_set_drvdata(pdev, NULL);

	kfree(pDev->pConf);
	kfree(pDev->pConf->lcd_power_ctrl.panel_power);
	kfree(pDev->pd);
	if (pDev)
		kfree(pDev);

    return 0;
}

#ifdef CONFIG_USE_OF
  static const struct of_device_id lcd_dt_match[] = {
	  {
		  .compatible = "amlogic,lcd_tv",
	  },
	  {},
  };
#endif

static struct platform_driver lcd_driver = {
	 .probe = lcd_probe,
	 .remove = lcd_remove,
	 .driver = {
		 .name = "amltvlcd",
		 .owner = THIS_MODULE,
#ifdef CONFIG_USE_OF
		 .of_match_table = lcd_dt_match,
#endif
	 },
 };

static int __init lcd_init(void)
{
	/* printk("LCD driver init\n"); */
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

int __init outputmode_setup(char *s)
{
	if (!strcmp(s,"1080p")) {
		lcd_output_mode = LCD_OUTPUT_MODE_1080P;
	} else  if (!strcmp(s,"1080p50hz")){
	        lcd_output_mode = LCD_OUTPUT_MODE_1080P50HZ;
	} else  if (!strcmp(s,"4k2k60hz420")){
		lcd_output_mode = LCD_OUTPUT_MODE_4K2K60HZ420;
	} else  if (!strcmp(s,"4k2k50hz420")){
		lcd_output_mode = LCD_OUTPUT_MODE_4K2K50HZ420;
	} else  if (!strcmp(s,"4k2k60hz")){
		lcd_output_mode = LCD_OUTPUT_MODE_4K2K60HZ;
	} else  if (!strcmp(s,"4k2k50hz")){
		lcd_output_mode = LCD_OUTPUT_MODE_4K2K50HZ;
	} else  if (!strcmp(s,"768p60hz")){
		lcd_output_mode = LCD_OUTPUT_MODE_768P60HZ;
	} else  if (!strcmp(s,"768p50hz")){
		lcd_output_mode = LCD_OUTPUT_MODE_768P50HZ;
	} else {
		lcd_output_mode = LCD_OUTPUT_MODE_MAX;
		printk("the output mode is not support!\n");
	}
	printk("the output mode is %d\n", lcd_output_mode);
	return 0;
}
__setup("vmode=",outputmode_setup);

static int __init lcd_boot_para_setup(char *s)
{
	if (NULL != s) {
		sprintf(lcd_propname, "%s", s);
	}

	return 0;
}
 __setup("panel_type=",lcd_boot_para_setup);


 MODULE_DESCRIPTION("Meson LCD Panel Driver");
 MODULE_LICENSE("GPL");
 MODULE_AUTHOR("Amlogic, Inc.");
