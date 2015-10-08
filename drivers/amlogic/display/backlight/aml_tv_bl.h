/*
 * drivers/amlogic/display/backlight/aml_tv_bl.h
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

#ifndef __AML_TV_BL_H
#define __AML_TV_BL_H

#include <linux/backlight.h>
#include <linux/notifier.h>

#define AML_BL_FREQ_DEF			1000	/* unit: HZ */
#define AML_BL_FREQ_VS_DEF		2 /* multiple 2 of vfreq */

#define AML_BL_LEVEL_DEF		128
#define AML_BL_LEVEL_MAX		255
#define AML_BL_LEVEL_MIN		10
#define AML_BL_LEVEL_OFF		1

#define AML_BL_DUTY_MAX			100	/* unit: % */
#define AML_BL_DUTY_MIN			20

#define AML_BL_PWM_DEALY_DEF		100	/* unit: s */
#define AML_BL_PWR_DEALY_DEF		5	/* unit: s */

typedef enum {
	AML_BL_PWM_A = 0,
	AML_BL_PWM_B,
	AML_BL_PWM_C,
	AML_BL_PWM_D,
	AML_BL_PWM_VS,
	AML_BL_PWM_MAX,
} aml_bl_pwm_port_t;

enum {
	AML_BL_PWM_OFF = 0,
	AML_BL_PWM_ON,
};

/* backlight properties state flag */
#define AML_BL_FLAG_POWERON		BL_CORE_DRIVER1 // (1 << 31)
/* 1: power n 0: power off */
#define AML_BL_STATE_POWERON		BL_CORE_DRIVER2 // (1 << 30)


struct aml_bl_dt {
	unsigned int bl_en_gpio; /* bl_en gpio */
	unsigned int bl_en_gpio_on;
	unsigned int pwm_port;
	unsigned int pwm_gpio;
	unsigned int pwm_gpio_on;

	unsigned int level_def;
	unsigned int level_mid;
	unsigned int level_mid_mapping;
	unsigned int level_min;
	unsigned int level_max;
	unsigned int level_off;
	unsigned int pwm_on_delay; /* unit: ms */
	unsigned int pwm_off_delay; /* unit: ms */
	unsigned int power_on_delay; /* unit: ms */
	unsigned int power_off_delay; /* unit: ms */

	unsigned int pwm_freq; /* pwm_vs: 1~4(vfreq multiple), other pwm: real freq(unit: Hz) */
	unsigned int pwm_duty_min;
	unsigned int pwm_duty_max;
	unsigned int pwm_positive; /* 1: positive 0: negative */
};

struct aml_bl {
	struct aml_bl_dt *d;
	struct pinctrl *p;
	struct backlight_device *bd;
	struct platform_device *pdev;

	unsigned int pwm_cnt;
	unsigned int pwm_clk_div;
	unsigned int pwm_max;
	unsigned int pwm_min;

	int curr_brightness;

	struct mutex power_lock;
	struct notifier_block reboot_nb;
	struct notifier_block bl_update_nb;
	bool reboot; /* true: notify reboot, false: other */
};

#endif
