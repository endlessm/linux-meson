/*
 * drivers/amlogic/display/led/aml_tv_led_gpio.c
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
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>
#include <linux/string.h>
#include <linux/printk.h>

#include <linux/amlogic/aml_gpio_consumer.h>
#include <mach/register.h>
#include <mach/io.h>
#include <plat/io.h>

#include "aml_tv_led.h"

#define AML_LED_NAME		"aml_led"

#define INFO(format, arg...) printk(KERN_INFO "%s: " format, \
	__FUNCTION__ , ## arg)
#define ERR(format, arg...)  printk(KERN_ERR  "%s: " format, \
	__FUNCTION__ , ## arg)


static struct aml_led_dt aml_led_dt_def = {
	.led_gpio_name = "GPIOAO_11",
	.led_use_gpio = 1,
	.led_gpio_pin = GPIOAO_11,
	.led_gpio_low = 0,
	.led_use_gpio_pullup = 1,
	.led_gpio_pullup_low = 0,
	.led_has_jtag = 1,
	.led_jtag_select = 0,
};


enum {
	JTAG_DISABLE = 0,	/* disable */
	JTAG_FROM_AO_CPU ,	/* from ao cpu */
	JTAG_FROM_SYS_CPU,	/* from sys cpu */
	JTAG_FROM_AUDIO_CPU,	/* from audio cpu */
};

static void aml_led_jtag_select(unsigned int from)
{
	switch (from) {
	case JTAG_DISABLE:
	case JTAG_FROM_AO_CPU:
	case JTAG_FROM_SYS_CPU:
	case JTAG_FROM_AUDIO_CPU:
		aml_clr_reg32_mask(P_AO_SECURE_REG1, 0x3 << 0);
		aml_set_reg32_mask(P_AO_SECURE_REG1, from << 0);
		break;
	default: /* none */
		break;
	}
}

static ssize_t aml_led_jtag_store(struct class *cls,
			 struct class_attribute *attr,
			 const char *buffer, size_t count)
{
	unsigned int from;
	from = simple_strtol(buffer, NULL, 16);
	INFO("from = 0x%x\n", from);
	aml_led_jtag_select(from);
	return count;
}


static void aml_led_gpio_setup(struct aml_led *ldev)
{
	unsigned int low;
	unsigned int gpio;

	low = !!ldev->d->led_gpio_low;
	gpio = ldev->d->led_gpio_pin;
	amlogic_gpio_request(gpio, AML_LED_NAME);
	amlogic_gpio_direction_output(gpio, low, AML_LED_NAME);
}

static ssize_t aml_led_gpio_store(struct class *cls,
			 struct class_attribute *attr,
			 const char *buffer, size_t count)
{
	unsigned int low;
	unsigned int gpio;
	struct aml_led *ldev;

	ldev = container_of(cls, struct aml_led, cls);
	gpio = ldev->d->led_gpio_pin;
	low = !!simple_strtol(buffer, NULL, 10);
	INFO("low = %u\n", low);

	amlogic_gpio_request(gpio, AML_LED_NAME);
	amlogic_gpio_direction_output(gpio, !!low, AML_LED_NAME);

	return count;
}


static void aml_led_pullup_setup(struct aml_led *ldev)
{
	unsigned int low;
	unsigned int gpio;

	low = !!ldev->d->led_gpio_pullup_low;
	gpio = ldev->d->led_gpio_pin;
	amlogic_set_pull_up_down(gpio, low, AML_LED_NAME);
}

static ssize_t aml_led_pullup_store(struct class *cls,
			 struct class_attribute *attr,
			 const char *buffer, size_t count)
{
	unsigned int low;
	unsigned int gpio;
	struct aml_led *ldev;

	ldev = container_of(cls, struct aml_led, cls);
	gpio = ldev->d->led_gpio_pin;
	low = !!simple_strtol(buffer, NULL, 10);
	INFO("low = %u\n", low);

	amlogic_set_pull_up_down(gpio, !!low, AML_LED_NAME);

	return count;
}


/* this function only clear the pinmux settting defined by clrmask in dtd */
static void aml_led_pinctrl_clrmask(struct aml_led *ldev)
{
	struct pinctrl_state *s;
	unsigned int ret;

	/* first free gpio */
	amlogic_gpio_free(ldev->d->led_gpio_pin, AML_LED_NAME);
	/* then select pinmux */
	s = pinctrl_lookup_state(ldev->p, "default");
	if (IS_ERR(s)) {
		ERR("failed to pinctrl_lookup_state default\n");
		devm_pinctrl_put(ldev->p);
		return;
	}

	ret = pinctrl_select_state(ldev->p, s);
	if (ret < 0) {
		ERR("failed to pinctrl_select_state\n");
		devm_pinctrl_put(ldev->p);
		return;
	}

	/* then select pinmux */
	s = pinctrl_lookup_state(ldev->p, " aml_led_dummy");
	if (IS_ERR(s)) {
		ERR("failed to pinctrl_lookup_state default\n");
		devm_pinctrl_put(ldev->p);
		return;
	}

	ret = pinctrl_select_state(ldev->p, s);
	if (ret < 0) {
		ERR("failed to pinctrl_select_state\n");
		devm_pinctrl_put(ldev->p);
		return;
	}
}

static void aml_led_setup(struct aml_led *ldev)
{
	/* clear pinmux */
	if (ldev->d->led_use_gpio)
		aml_led_pinctrl_clrmask(ldev);

	/* clear jtag pin */
	if (ldev->d->led_has_jtag)
		aml_led_jtag_select(ldev->d->led_jtag_select);

	/* set gpio output */
	if (ldev->d->led_use_gpio)
		aml_led_gpio_setup(ldev);

	/* set gpio pullup */
	if (ldev->d->led_use_gpio_pullup)
		aml_led_pullup_setup(ldev);
}


/* print all the register related to this feature */
static ssize_t aml_led_debug_show(struct class *cls,
			struct class_attribute *attr, char *buf)
{
	unsigned int val;
	struct aml_led *ldev;

	ldev = container_of(cls, struct aml_led, cls);

	INFO("%-20s = %s\n", "led_gpio_name", ldev->d->led_gpio_name);
	INFO("%-20s = %u\n", "led_use_gpio", ldev->d->led_use_gpio);
	INFO("%-20s = %u\n", "led_gpio_low", ldev->d->led_gpio_low);
	INFO("%-20s = %u\n", "led_use_gpio_pullup", ldev->d->led_use_gpio_pullup);
	INFO("%-20s = %u\n", "led_gpio_pullup_low", ldev->d->led_gpio_pullup_low);
	INFO("%-20s = %u\n", "led_has_jtag", ldev->d->led_has_jtag);
	INFO("%-20s = %u\n", "led_jtag_select", ldev->d->led_jtag_select);

	val = aml_read_reg32(P_AO_RTI_PIN_MUX_REG);
	INFO("%-20s 0x%4x = 0x%-8x	[22] = %u [28] = %u\n",
		"P_AO_RTI_PIN_MUX_REG", P_AO_RTI_PIN_MUX_REG, val,
		!!(val&(1<<22)), !!(val&(1<<28)));

	val = aml_read_reg32(P_AO_SECURE_REG1);
	INFO("%-20s 0x%4x = 0x%-8x	[1:0] = %u\n",
		"P_AO_SECURE_REG1", P_AO_SECURE_REG1, val,
		!!(val&(0x3<<0)));

	val = aml_read_reg32(P_AO_GPIO_O_EN_N);
	INFO("%-20s 0x%4x = 0x%-8x	[11] = %u [27] = %u\n",
		"P_AO_GPIO_O_EN_N", P_AO_GPIO_O_EN_N, val,
		!!(val&(1<<11)), !!(val&(1<<27)));

	val = aml_read_reg32(P_AO_RTI_PULL_UP_REG);
	INFO("%-20s 0x%4x = 0x%-8x	[11] = %u [27] = %u\n",
		"P_AO_RTI_PULL_UP_REG", P_AO_RTI_PULL_UP_REG, val,
		!!(val&(1<<11)), !!(val&(1<<27)));
	return 0;
}

static ssize_t aml_led_debug_store(struct class *cls,
			 struct class_attribute *attr,
			 const char *buffer, size_t count)
{
	struct aml_led *ldev;
	ldev = container_of(cls, struct aml_led, cls);
	aml_led_setup(ldev);
	return count;
}


static struct class_attribute aml_tv_led_attrs[] = {
	__ATTR(gpio,   0644, NULL, aml_led_gpio_store),
	__ATTR(pullup, 0644, NULL, aml_led_pullup_store),
	__ATTR(jtag,   0644, NULL, aml_led_jtag_store),
	__ATTR(debug,  0644, aml_led_debug_show, aml_led_debug_store),
	__ATTR_NULL,
};


static void aml_led_dt_parse(struct platform_device *pdev)
{
	struct aml_led *ldev;
	struct device_node *node;
	unsigned int val;
	const char *str;
	int ret;

	ldev = platform_get_drvdata(pdev);
	node = pdev->dev.of_node;

	/* set default value */
	ldev->d = &aml_led_dt_def;

	/* start to parse dt value */
	ret = of_property_read_u32(node, "led_use_gpio", &val);
	if (ret) {
		ERR("faild to get led_use_gpio\n");
		INFO("use defalut led_use_gpio = %u\n",
			ldev->d->led_use_gpio);
	}
	else {
		INFO("led_gpio_enable = %u\n", val);
		ldev->d->led_use_gpio = !!val; /* revert to 0 or 1*/
	}

	ret = of_property_read_string(node, "led_gpio_name", &str);
	if (ret) {
		ERR("faild to get led_gpio_name\n");
		INFO("use defalut led_gpio_name = %s\n",
			ldev->d->led_gpio_name);
	}
	else {
		INFO("led_gpio_name = %s\n", str);
		ldev->d->led_gpio_name = str;
		ldev->d->led_gpio_pin = amlogic_gpio_name_map_num(str);
	}

	ret = of_property_read_u32(node, "led_gpio_low", &val);
	if (ret) {
		ERR("faild to get led_gpio_low\n");
		INFO("use defalut led_gpio_low = %u\n",
			ldev->d->led_gpio_low);
	}
	else {
		INFO("led_gpio_low = %u\n", val);
		ldev->d->led_gpio_low = !!val;
	}

	ret = of_property_read_u32(node, "led_use_gpio_pullup", &val);
	if (ret) {
		ERR("faild to get led_use_gpio_pullup\n");
		INFO("use defalut led_use_gpio_pullup = %u\n",
			ldev->d->led_use_gpio_pullup);
	}
	else {
		INFO("led_use_gpio_pullup = %u\n", val);
		ldev->d->led_use_gpio_pullup = !!val;
	}

	ret = of_property_read_u32(node, "led_gpio_pullup_low", &val);
	if (ret) {
		ERR("faild to get led_gpio_pullup_low\n");
		INFO("use defalut led_gpio_pullup_low = %u\n",
			ldev->d->led_gpio_pullup_low);
	}
	else {
		INFO("led_gpio_pullup_low = %u\n", val);
		ldev->d->led_gpio_pullup_low = !!val;
	}

	ret = of_property_read_u32(node, "led_has_jtag", &val);
	if (ret) {
		ERR("faild to get led_has_jtag\n");
		INFO("use defalut led_has_jtag = %u\n",
			ldev->d->led_has_jtag);
	}
	else {
		INFO("led_has_jtag = %u\n", val);
		ldev->d->led_has_jtag = !!val;
	}

	ret = of_property_read_u32(node, "led_jtag_select", &val);
	if (ret) {
		ERR("faild to get led_jtag_select\n");
		INFO("use defalut led_jtag_select = %u\n",
			ldev->d->led_jtag_select);
	}
	else {
		INFO("led_jtag_select = %u\n", val);
		ldev->d->led_jtag_select = val;
	}
}


static int aml_led_probe(struct platform_device *pdev)
{
	struct aml_led *ldev;

	ldev = kzalloc(sizeof(struct aml_led), GFP_KERNEL);
	if (!ldev ) {
	    ERR("kzalloc error\n");
	    return -ENOMEM;
	}

	/* set driver data */
	platform_set_drvdata(pdev, ldev);
	ldev->pdev = pdev;

	/* parse dt */
	aml_led_dt_parse(pdev);
	/* get pinmux */
	ldev->p = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(ldev->p)) {
		ERR("failed to get pinmux\n");
		goto exit;
	}

	/* create class attributes */
	ldev->cls.name = AML_LED_NAME;
	ldev->cls.owner = THIS_MODULE;
	ldev->cls.class_attrs = aml_tv_led_attrs;
	class_register(&ldev->cls);

	/* setup led */
	aml_led_setup(ldev);

	INFO("module probed ok\n");
	return 0;;

exit:
	kfree(ldev);

	INFO("module probed failed\n");
	return -ENODEV;;
}

static int __exit aml_led_remove(struct platform_device *pdev)
{
	struct aml_led *ldev;
	ldev = platform_get_drvdata(pdev);
	class_unregister(&ldev->cls);
	platform_set_drvdata(pdev, NULL);
	kfree(ldev);
	INFO("module removed ok\n");
	return 0;
}
static const struct of_device_id aml_led_dt_match[] = {
	{
		.compatible = "amlogic,tv_led",
	},
	{},
};

static struct platform_driver aml_led_driver = {
	.driver = {
		.name = AML_LED_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_USE_OF
		.of_match_table = aml_led_dt_match,
#endif
	},
	.probe = aml_led_probe,
	.remove = __exit_p(aml_led_remove),
};

static int __init aml_led_init(void)
{
	INFO("module init\n");
	if (platform_driver_register(&aml_led_driver)) {
		ERR("failed to register led driver module\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit aml_led_exit(void)
{
	INFO("module exit\n");
	platform_driver_unregister(&aml_led_driver);
}


module_init(aml_led_init);
module_exit(aml_led_exit);

MODULE_DESCRIPTION("Meson LED Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Amlogic, Inc.");

