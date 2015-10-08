/*
 * drivers/amlogic/display/led/aml_tv_led_gpio.h
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

#ifndef __AML_TV_LED_H
#define __AML_TV_LED_H

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>

struct aml_led_dt {
	bool led_use_gpio;
	const char  *led_gpio_name;
	unsigned int led_gpio_pin;
	unsigned int led_gpio_low; /* the value to be set gpio low or high */

	bool led_use_gpio_pullup;
	unsigned int led_gpio_pullup_low;

	bool led_has_jtag;
	unsigned int led_jtag_select;
};

struct aml_led {
	struct aml_led_dt *d;
	struct pinctrl *p;
	struct platform_device *pdev;
	struct class cls;
};

#endif
