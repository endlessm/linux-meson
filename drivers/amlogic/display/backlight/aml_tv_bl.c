/*
 * drivers/amlogic/display/backlight/aml_tv_bl.c
 *
 * This driver don't power on backlight when booting. It has been done
 * in uboot when booting. But this driver will power off backlight and
 * notify lcd to power off.
 *
 * This driver also has be able to power on/off backlight on the fly.
 *
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/printk.h>
#include <linux/reboot.h>

#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/amlogic/display/lcd.h>
#include <mach/register.h>
#include <mach/io.h>
#include <plat/io.h>
#include "aml_tv_bl.h"

#define AML_BL_NAME		"aml-bl"

#define INFO(format, arg...) printk(KERN_INFO "%s: " format, \
	__FUNCTION__ , ## arg)
#define ERR(format, arg...)  printk(KERN_ERR  "%s: " format, \
	__FUNCTION__ , ## arg)

static int aml_bl_brightness_update_force = 0;

static int aml_bl_reboot_notify_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct aml_bl *bdev;

	bdev = container_of(self, struct aml_bl, reboot_nb);
	mutex_lock(&bdev->bd->ops_lock);
	bdev->bd->props.state &= ~AML_BL_STATE_POWERON;
	bdev->reboot = true;
	backlight_update_status(bdev->bd);
	mutex_unlock(&bdev->bd->ops_lock);
	return NOTIFY_OK;
}

static void aml_bl_pwm_init(struct backlight_device *bd)
{
	unsigned int port;
	unsigned int div;
	struct aml_bl *bdev;

	bdev = bl_get_data(bd);;
	port = bdev->d->pwm_port;
	div  = bdev->pwm_clk_div;

	switch (port) {
	case AML_BL_PWM_A:
		/* pwm_a_clk_div: bit[14-8] */
		aml_set_reg32_bits(P_PWM_MISC_REG_AB, div, 8, 7);
		/* pwm_a_clk_sel: bit[5-4] */
		aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 4, 2);
		/* pwm_a_clk_en: bit[15] */
		aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 15, 1);
		break;
	case AML_BL_PWM_B:
		/* pwm_b_clk_div: bit[22-16] */
		aml_set_reg32_bits(P_PWM_MISC_REG_AB, div, 16, 7);
		/* pwm_b_clk_sel: bit[7-6] */
		aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 6, 2);
		/* pwm_b_clk_en: bit[23] */
		aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 23, 1);
		break;
	case AML_BL_PWM_C:
		/* pwm_c_clk_div: bit[14-8] */
		aml_set_reg32_bits(P_PWM_MISC_REG_CD, div, 8, 7);
		/* pwm_c_clk_sel: bit[5-4] */
		aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 4, 2);
		/* pwm_c_clk_en: bit[15] */
		aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 15, 1);
		break;
	case AML_BL_PWM_D:
		/* pwm_d_clk_div: bit[22-16] */
		aml_set_reg32_bits(P_PWM_MISC_REG_CD, div, 16, 7);
		/* pwm_d_clk_sel: bit[7-6] */
		aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 6, 2);
		/* pwm_d_clk_en: bit[23] */
		aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 23, 1);
		break;
	case AML_BL_PWM_VS:
		break;
	default:
		ERR("unsupport pwm port %u\n", port);
		break;
	}
	INFO("pwm select port %d ok\n", port);
}


static void aml_bl_pwm_on(struct backlight_device *bd)
{
	unsigned int port;
	struct aml_bl *bdev;
	struct pinctrl_state *s;
	int ret;

	bdev = bl_get_data(bd);
	port = bdev->d->pwm_port;

	amlogic_gpio_free(bdev->d->pwm_gpio, AML_BL_NAME);

	if (bdev->d->pwm_port == AML_BL_PWM_VS) {
		s = pinctrl_lookup_state(bdev->p, "pwm_vs");
		if (IS_ERR(s)) {
			ERR("failed to pinctrl_lookup_state pwm_vs\n");
			devm_pinctrl_put(bdev->p);
			return;
		}
	} else {
		s = pinctrl_lookup_state(bdev->p, "default");
		if (IS_ERR(s)) {
			ERR("failed to pinctrl_lookup_state default\n");
			devm_pinctrl_put(bdev->p);
			return;
		}
	}

	ret = pinctrl_select_state(bdev->p, s);
	if (ret < 0) {
		ERR("failed to pinctrl_select_state\n");
		devm_pinctrl_put(bdev->p);
		return;
	}

	switch (port) {
	case AML_BL_PWM_A:
		/* pwm_a_en bit[0] */
		aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 0, 1);
		break;
	case AML_BL_PWM_B:
		/* pwm_b_en bit[1] */
		aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 1, 1);
		break;
	case AML_BL_PWM_C:
		/* pwm_c_en bit[0] */
		aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 0, 1);
		break;
	case AML_BL_PWM_D:
		/* pwm_d_en bit[1] */
		aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 1, 1);
		break;
	case AML_BL_PWM_VS:
		break;
	default:
		ERR("unsupport pwm port %u\n", port);
		break;
	}
	INFO("pwm on\n");
}


static void aml_bl_pwm_off(struct backlight_device *bd)
{
	unsigned int port;
	int low;
	struct aml_bl *bdev;
	struct pinctrl_state *s;
	int ret;

	bdev = bl_get_data(bd);
	port = bdev->d->pwm_port;
	low = bdev->d->pwm_gpio_on ? 0 : 1;

	switch (port) {
	case AML_BL_PWM_A:
		/* pwm_a_en bit[0] */
		aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 0, 1);
		break;
	case AML_BL_PWM_B:
		/* pwm_b_en bit[1] */
		aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 1, 1);
		break;
	case AML_BL_PWM_C:
		/* pwm_c_en bit[0] */
		aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 0, 1);
		break;
	case AML_BL_PWM_D:
		/* pwm_d_en bit[1] */
		aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 1, 1);
		break;
	case AML_BL_PWM_VS:
		break;
	default:
		ERR("unsupport pwm port %u\n", port);
		break;
	}

	s = pinctrl_lookup_state(bdev->p, "aml_tv_bl_dummy");
	if (IS_ERR(s)) {
		ERR("failed to pinctrl_lookup_state aml_tv_bl_dummy\n");
		devm_pinctrl_put(bdev->p);
	}

	ret = pinctrl_select_state(bdev->p, s);
	if (ret < 0) {
		ERR("failed to pinctrl_select_state\n");
		devm_pinctrl_put(bdev->p);
	}
	amlogic_gpio_request(bdev->d->pwm_gpio, AML_BL_NAME);
	amlogic_gpio_direction_output(bdev->d->pwm_gpio, low, AML_BL_NAME);
	INFO("pwm off\n");
}


static void aml_bl_power_on(struct backlight_device *bd)
{
	struct aml_bl *bdev;
	int on;

	bdev = bl_get_data(bd);
	on = bdev->d->bl_en_gpio_on;

	mutex_lock(&bdev->power_lock);
	aml_bl_pwm_on(bd);
	msleep(bdev->d->power_on_delay);
	amlogic_gpio_direction_output(bdev->d->bl_en_gpio, on, AML_BL_NAME);

	/* set state flag */
	bdev->bd->props.state |= AML_BL_FLAG_POWERON;
	mutex_unlock(&bdev->power_lock);
	INFO("power on\n");
}


static void aml_bl_power_off(struct backlight_device *bd)
{
	struct aml_bl *bdev;
	int off;

	bdev = bl_get_data(bd);
	off = bdev->d->bl_en_gpio_on ? 0 : 1;

	mutex_lock(&bdev->power_lock);
	amlogic_gpio_direction_output(bdev->d->bl_en_gpio, off, AML_BL_NAME);
	msleep(bdev->d->power_off_delay);
	aml_bl_pwm_off(bd);
	bdev->bd->props.state &= ~AML_BL_FLAG_POWERON;
	/* notify lcd to power off if reboot */
	if (bdev->reboot) {
		//aml_bl_reboot_notify_delay_work(bdev->bd);
		mdelay(bdev->d->pwm_off_delay);
		lcd_notifier_call_chain(LCD_EVENT_POWEROFF, bdev);
	}
	mutex_unlock(&bdev->power_lock);
	INFO("power off\n");
}


static void aml_bl_pwm_duty_set(struct backlight_device *bd,
	unsigned int pwm_high, unsigned int pwm_low)
{
	struct aml_bl *bdev = bl_get_data(bd);
	unsigned int high = 0, low = 0;
	unsigned int vs[4], ve[4], sw, n, i;

	//INFO("pwm_high=%d, pwm_low=%d\n", pwm_high, pwm_low);
	if (bdev->d->pwm_port == AML_BL_PWM_VS) {
		memset(vs, 0, sizeof(unsigned int) * 4);
		memset(ve, 0, sizeof(unsigned int) * 4);
		n = bdev->d->pwm_freq;
		sw = (bdev->pwm_cnt * 10 / n + 5) / 10;
		if (bdev->d->pwm_positive)
			high = (pwm_high * 10 / n + 5) / 10;
		else
			high = (pwm_low * 10 / n + 5) / 10;
		high = (high > 1) ? high : 1;
		//INFO("n=%d, sw=%d, high=%d\n", n, sw, high);
		for (i = 0; i < n; i++) {
			vs[i] = 1 + (sw * i);
			ve[i] = vs[i] + high - 1;
			//INFO("vs[%d]=%d, ve[%d]=%d\n", i, vs[i], i, ve[i]);
		}
	} else {
		if (bdev->d->pwm_positive) {
			high = pwm_high;
			low = pwm_low;
		} else {
			high = pwm_low;
			low = pwm_high;
		}
	}

	switch (bdev->d->pwm_port) {
	case AML_BL_PWM_A:
		aml_write_reg32(P_PWM_PWM_A, (high << 16) | (low));
		break;
	case AML_BL_PWM_B:
		aml_write_reg32(P_PWM_PWM_B, (high << 16) | (low));
		break;
	case AML_BL_PWM_C:
		aml_write_reg32(P_PWM_PWM_C, (high << 16) | (low));
		break;
	case AML_BL_PWM_D:
		aml_write_reg32(P_PWM_PWM_D, (high << 16) | (low));
		break;
	case AML_BL_PWM_VS:
		aml_write_reg32(P_VPU_VPU_PWM_V0, (ve[0] << 16) | (vs[0]));
		aml_write_reg32(P_VPU_VPU_PWM_V1, (ve[1] << 16) | (vs[1]));
		aml_write_reg32(P_VPU_VPU_PWM_V2, (ve[2] << 16) | (vs[2]));
		aml_write_reg32(P_VPU_VPU_PWM_V3, (ve[3] << 16) | (vs[3]));
		break;
	default:
		break;
	}
}

static void aml_bl_set_level(struct backlight_device *bd, unsigned level)
{
	unsigned int pw, lw, lv;
	unsigned int pwm_high, pwm_low;
	struct aml_bl *bdev = bl_get_data(bd);

	//INFO("input level %u\n", level);

	if (aml_bl_brightness_update_force == 0) {
		if (bdev->curr_brightness == level)
			return;
	}

	/* validate value */
	if (level > bdev->d->level_max)
		level = bdev->d->level_max;
	if (level < bdev->d->level_min)
		level = bdev->d->level_min;

	/* save the validated level */
	bdev->curr_brightness = level;

	pw = bdev->pwm_max - bdev->pwm_min;
	lw = bdev->d->level_max - bdev->d->level_min;
	lv = level - bdev->d->level_min;
	level = pw * lv / lw + bdev->pwm_min;

	pwm_high = level;
	pwm_low = bdev->pwm_cnt - level;
	aml_bl_pwm_duty_set(bd, pwm_high, pwm_low);

	INFO("set level ok\n");
}

static int aml_bl_get_brightness(struct backlight_device *bd)
{
	struct aml_bl *bdev = bl_get_data(bd);
	return bdev->curr_brightness;
}

static int aml_bl_update_status(struct backlight_device *bd)
{
	int brightness = bd->props.brightness;

	if (brightness < 0)
		brightness = 0;
	if (brightness > 255)
		brightness = 255;

	if (bd->props.power != FB_BLANK_UNBLANK) {
		INFO("!FB_BLANK_UNBLANK props.power = %d\n", bd->props.power);
		brightness = 0;
	}

	if (!(bd->props.state & AML_BL_STATE_POWERON)) {
		INFO("!AML_BL_STATE_POWERON props.state = 0x%x\n", bd->props.state);
		brightness = 0;
	}

	INFO("new brightness = %d\n", brightness);

	/* backlight is disable */
	if (!brightness) {
		aml_bl_power_off(bd);
		return 0;
	}

	aml_bl_set_level(bd, brightness);

	if (!(bd->props.state & AML_BL_FLAG_POWERON)) {
		aml_bl_power_on(bd);
	}
	return 0;
}

static const struct backlight_ops aml_bl_ops = {
	.get_brightness = aml_bl_get_brightness,
	.update_status = aml_bl_update_status,
};

static int aml_bl_register_backlight(struct platform_device *pdev)
{
	struct aml_bl *bdev;
	struct backlight_properties props;
	int ret = 0;

	memset(&props, 0, sizeof(struct backlight_properties));

	bdev = platform_get_drvdata(pdev);

	if (bdev->d->level_max > 0)
		props.max_brightness = bdev->d->level_max;
	else
		props.max_brightness = AML_BL_LEVEL_MAX;

	if (bdev->d->level_def > 0)
		props.brightness = bdev->d->level_def;
	else
		props.brightness = AML_BL_LEVEL_DEF;

	props.type = BACKLIGHT_RAW;
	props.power = FB_BLANK_UNBLANK;
	props.state |= AML_BL_STATE_POWERON;
	props.state |= AML_BL_FLAG_POWERON;

	bdev->bd = backlight_device_register(AML_BL_NAME, &pdev->dev, bdev,
					&aml_bl_ops, &props);
	if (IS_ERR(bdev->bd)) {
		ERR("failed to register backlight\n");
		return -ENODEV;
	}
	INFO("register backlight ok\n");
	return ret;
}

#define XTAL_FREQ_HZ		(24*1000*1000) /* 24M in HZ */
#define XTAL_HALF_FREQ_HZ	(24*1000*500)  /* 24M/2 in HZ */

static void aml_bl_pwm_param_init(struct aml_bl *bdev)
{
	unsigned int pwm_freq;
	unsigned int pwm_clk_div;
	unsigned int pwm_cnt;
	int i;

	pwm_freq = bdev->d->pwm_freq;

	if (bdev->d->pwm_port == AML_BL_PWM_VS) {
		pwm_cnt = aml_read_reg32(P_ENCL_VIDEO_MAX_LNCNT) + 1;
		bdev->pwm_cnt = pwm_cnt;
		INFO("pwm_cnt = %u\n", bdev->pwm_cnt);
	} else {
		/* 0x7f(7bit): PWMA[14:8] or PWMB[22:6]*/
		/*
		 * pwm_clk_in = xtal / clk_div
		 * pwm_out = pwm_clk_in/((high + 1) + (low + 1))
		 *
		 */
		for (i = 0; i < 0x7f; i++) {
			pwm_clk_div = i;
			pwm_cnt = XTAL_FREQ_HZ / (pwm_freq * (pwm_clk_div + 1)) - 2;
			if (pwm_cnt <= 0xffff) /* 16bit */
				break;
		}

		bdev->pwm_cnt = pwm_cnt;
		bdev->pwm_clk_div = pwm_clk_div;
		INFO("pwm_cnt = %u, pwm_clk_div = %u\n", pwm_cnt, pwm_clk_div);
	}

	bdev->pwm_max = (pwm_cnt * bdev->d->pwm_duty_max / 100);
	bdev->pwm_min = (pwm_cnt * bdev->d->pwm_duty_min / 100);
	//INFO("pwm_max = %u, pwm_min = %u\n", bdev->pwm_max, bdev->pwm_min);
}

static struct aml_bl_dt aml_bl_dt_def = {
	.level_def = AML_BL_LEVEL_DEF,
	.pwm_positive = 1,
	.bl_en_gpio_on = 0,
	//...
};

static char aml_bl_sel[15] = "backlight_0";
static int __init aml_bl_select_setup(char *s)
{
	char *sel;
	if (NULL != s) {
		sel= strchr(s, '_');
		sprintf(aml_bl_sel, "%s%s", "backlight", sel);
	}

	INFO("select backlight: %s\n", aml_bl_sel);
	return 0;
}
__setup("panel_type=", aml_bl_select_setup);


static int aml_bl_dt_parse(struct platform_device *pdev)
{
	struct device_node *node;
	struct device_node *child;
	struct aml_bl *bdev;
	unsigned int val;
	const char *str;
	unsigned int array[2];
	int ret;

	bdev = platform_get_drvdata(pdev);
	node = pdev->dev.of_node;
	if (!node) {
		ERR("failed to find backlight node\n");
		return -ENODEV;
	}

	child = of_get_child_by_name(node, aml_bl_sel);
	if (!child) {
		ERR("failed to find child node\n");
		return -ENODEV;
	}
	INFO("child %s\n", aml_bl_sel);

	/* BL_EN_GPIO */
	ret = of_property_read_string(child, "bl_en_gpio", &str);
	if (ret) {
		ERR("faild to get bl_en_gpio\n");
		bdev->d->bl_en_gpio = GPIOY_6;
	}
	else {
		bdev->d->bl_en_gpio = amlogic_gpio_name_map_num(str);
		ret = amlogic_gpio_request(bdev->d->bl_en_gpio, AML_BL_NAME);
		if (ret)
			ERR("failed to request gpio %s(%u)\n",
			str, bdev->d->bl_en_gpio);
	}
	INFO("bl_en_gpio = %s(%u)\n", str, bdev->d->bl_en_gpio);

	/* BL_EN_GPIO_ON */
	ret = of_property_read_u32(child, "bl_en_gpio_on", &val);
	if (ret) {
		ERR("faild to get bl_en_gpio_on\n");
	}
	else {
		bdev->d->bl_en_gpio_on = val;
	}
	INFO("bl_en_gpio_on = %u\n", bdev->d->bl_en_gpio_on);

	/* BL_PWM PORT */
	ret = of_property_read_string(child, "bl_pwm_port", &str);
	if (ret) {
		ERR("faild to get bl_pwm_port\n");
		bdev->d->pwm_port = AML_BL_PWM_B;
	}
	else {
		if (strcmp(str, "PWM_A") == 0)
			bdev->d->pwm_port = AML_BL_PWM_A;
		else if (strcmp(str, "PWM_B") == 0)
			bdev->d->pwm_port = AML_BL_PWM_B;
		else if (strcmp(str, "PWM_C") == 0)
			bdev->d->pwm_port = AML_BL_PWM_C;
		else if (strcmp(str, "PWM_D") == 0)
			bdev->d->pwm_port = AML_BL_PWM_D;
		else if (strcmp(str, "PWM_VS") == 0)
			bdev->d->pwm_port = AML_BL_PWM_VS;
		else
			bdev->d->pwm_port = AML_BL_PWM_MAX;
	}
	INFO("bl_pwm_port = %u\n", bdev->d->pwm_port);

	/* BL_PWM POSITIVE */
	ret = of_property_read_u32(child, "bl_pwm_positive", &val);
	if (ret) {
		ERR("faild to get bl_pwm_positive\n");
	}
	else {
		bdev->d->pwm_positive = val;
	}
	INFO("bl_pwm_positive = %u\n", bdev->d->pwm_positive);

	/* BL PWM FREQ */
	ret = of_property_read_u32(child, "bl_pwm_freq", &val);
	if (ret) {
		ERR("failed to get bl_pwm_freq\n");
		if (bdev->d->pwm_port == AML_BL_PWM_VS)
			val = AML_BL_FREQ_VS_DEF;
		else
			val = AML_BL_FREQ_DEF;
	}
	if (bdev->d->pwm_port == AML_BL_PWM_VS) {
		if (val > 4) {
			ERR("invalid pwm_vs freq\n");
			val = AML_BL_FREQ_VS_DEF;
		} else {
			bdev->d->pwm_freq = val;
		}
		INFO("bl pwm_vs_freq = %u x vfreq\n",  bdev->d->pwm_freq);
	} else {
		/* validate: less than the half of xtal */
		if (val > XTAL_HALF_FREQ_HZ)
			bdev->d->pwm_freq = XTAL_HALF_FREQ_HZ;
		else
			bdev->d->pwm_freq = val;
		INFO("bl pwm_freq = %uHz\n",  bdev->d->pwm_freq);
	}

	/* BL_PWM_GPIO */
	ret = of_property_read_string(child, "bl_pwm_gpio", &str);
	if (ret) {
		ERR("faild to get bl_pwm_gpio\n");
		bdev->d->pwm_gpio = GPIOY_7;
	}
	else {
		bdev->d->pwm_gpio = amlogic_gpio_name_map_num(str);
	}
	INFO("bl_pwm_gpio = %s(%u)\n", str, bdev->d->pwm_gpio);

	/* BL_PWM_GPIO_ON */
	ret = of_property_read_u32(child, "bl_pwm_gpio_on", &val);
	if (ret) {
		ERR("faild to get bl_pwm_gpio_on\n");
	}
	else {
		bdev->d->pwm_gpio_on = val;
	}
	INFO("bl_pwm_gpio_on = %u\n", bdev->d->pwm_gpio_on);

	ret = of_property_read_u32(child, "bl_pwm_on_delay", &val);
	if (ret) {
		ERR("faild to get bl_pwm_on_delay\n");
		bdev->d->pwm_on_delay = AML_BL_PWM_DEALY_DEF;
	}
	else {
		bdev->d->pwm_on_delay = val & 0xffff;
	}
	INFO("bl_pwm_on_delay = %u\n", bdev->d->pwm_on_delay);

	ret = of_property_read_u32(child, "bl_pwm_off_delay", &val);
	if (ret) {
		ERR("faild to get bl_pwm_off_delay\n");
		bdev->d->pwm_off_delay = AML_BL_PWM_DEALY_DEF;
	}
	else {
		bdev->d->pwm_off_delay = val & 0xffff;
	}
	INFO("bl_pwm_off_delay = %u\n", bdev->d->pwm_off_delay);

	ret = of_property_read_u32(child, "bl_power_on_delay", &val);
	if (ret) {
		ERR("faild to get bl_power_on_delay\n");
		bdev->d->power_on_delay = AML_BL_PWR_DEALY_DEF;
	}
	else {
		bdev->d->power_on_delay = val & 0xffff;
	}
	INFO("bl_power_on_delay = %u\n", bdev->d->power_on_delay);

	ret = of_property_read_u32(child, "bl_power_off_delay", &val);
	if (ret) {
		ERR("faild to get bl_power_off_delay\n");
		bdev->d->power_off_delay = AML_BL_PWR_DEALY_DEF;
	}
	else {
		bdev->d->power_off_delay = val & 0xffff;
	}
	INFO("bl_power_off_delay = %u\n", bdev->d->power_off_delay);

	ret = of_property_read_u32(child, "bl_level_default_kernel", &val);
	if (ret) {
		ERR("faild to get bl_level_default_kernel\n");
		bdev->d->level_def = AML_BL_LEVEL_DEF;
	}
	else {
		bdev->d->level_def = val;
	}
	INFO("bl_level_default_kernel = %u\n", bdev->d->level_def);

	ret = of_property_read_u32_array(child, "bl_level_max_min",
		array, ARRAY_SIZE(array));
	if (ret) {
		ERR("failed to bl_level_max_min\n");
		bdev->d->level_min = AML_BL_LEVEL_MIN;
		bdev->d->level_max = AML_BL_LEVEL_MAX;
	}
	else {
		bdev->d->level_max = array[0];
		bdev->d->level_min = array[1];
	}
	INFO("bl_level_max_min = <%u, %u>\n", bdev->d->level_max,
		bdev->d->level_min);

	ret = of_property_read_u32_array(child, "bl_pwm_duty_max_min",
		array, ARRAY_SIZE(array));
	if (ret) {
		ERR("failed to get bl_pwm_duty_max_min\n");
		bdev->d->pwm_duty_max = AML_BL_DUTY_MIN;
		bdev->d->pwm_duty_min = AML_BL_DUTY_MIN;
	}
	else {
		bdev->d->pwm_duty_max = array[0];
		bdev->d->pwm_duty_min = array[1];
	}
	INFO("bl_pwm_duty_max_min = <%u, %u>\n", bdev->d->pwm_duty_max,
		bdev->d->pwm_duty_min);

	return 0;
}

static int aml_bl_lcd_update_notifier(struct notifier_block *nb, unsigned long event, void *cmd)
 {
	struct aml_bl *bdev;

	/* If we aren't interested in this event, skip it immediately */
	if (event != LCD_EVENT_BL_UPDATE)
		return 0;

	printk("bl_lcd_update_notifier: event = %lu\n", event);
	bdev = container_of(nb, struct aml_bl, bl_update_nb);
	if (bdev->d->pwm_port == AML_BL_PWM_VS) {
		aml_bl_brightness_update_force = 1;
		aml_bl_pwm_param_init(bdev);
		aml_bl_update_status(bdev->bd);
		aml_bl_brightness_update_force = 0;
	}

	return NOTIFY_OK;
}

static const struct of_device_id aml_bl_dt_match[] = {
	{
		.compatible = "amlogic,backlight",
	},
	{},
};

static int aml_bl_probe(struct platform_device *pdev)
{
	struct aml_bl *bdev;
	int ret;

	aml_bl_brightness_update_force = 0;

	bdev = kzalloc(sizeof(struct aml_bl), GFP_KERNEL);
	if (!bdev ) {
	    ERR("kzalloc error\n");
	    return -ENOMEM;
	}

	platform_set_drvdata(pdev, bdev);
	bdev->pdev = pdev;

	/* parse dt param */
	bdev->d = &aml_bl_dt_def;
	ret = aml_bl_dt_parse(pdev);
	if (ret)
		return ret;
	aml_bl_pwm_param_init(bdev);


	/* setup pinmux */
	bdev->p = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(bdev->p))
		ERR("failed to get pinmux\n");

	mutex_init(&bdev->power_lock);

	ret = aml_bl_register_backlight(pdev);
	if (ret < 0) {
		kfree(bdev);
		return ret;
	}

	/* pwm init */
	aml_bl_pwm_init(bdev->bd);

	/* register rebot notify */
	bdev->reboot = false;
	bdev->reboot_nb.notifier_call = aml_bl_reboot_notify_callback;
	ret = register_reboot_notifier(&bdev->reboot_nb);

	bdev->bl_update_nb.notifier_call = aml_bl_lcd_update_notifier;
	ret = lcd_register_client(&bdev->bl_update_nb);
	if (ret) {
		ERR("bl_register_client fail\n");
	}

	INFO("module probed ok\n");
	return 0;
}

static int __exit aml_bl_remove(struct platform_device *pdev)
{
	struct aml_bl *bdev = platform_get_drvdata(pdev);

	register_reboot_notifier(&bdev->reboot_nb);
	amlogic_gpio_free(bdev->d->bl_en_gpio, AML_BL_NAME);
	backlight_device_unregister(bdev->bd);
	platform_set_drvdata(pdev, NULL);
	kfree(bdev);

	INFO("module removed ok\n");
	return 0;
}

static struct platform_driver aml_bl_driver = {
	.driver = {
		.name = AML_BL_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_USE_OF
		.of_match_table = aml_bl_dt_match,
#endif
	},
	.probe = aml_bl_probe,
	.remove = __exit_p(aml_bl_remove),
};

static int __init aml_bl_init(void)
{
	INFO("module init\n");
	if (platform_driver_register(&aml_bl_driver)) {
		ERR("failed to register bl driver module\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit aml_bl_exit(void)
{
	INFO("module exit\n");
	platform_driver_unregister(&aml_bl_driver);
}

module_init(aml_bl_init);
module_exit(aml_bl_exit);

MODULE_DESCRIPTION("Meson Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Amlogic, Inc.");
