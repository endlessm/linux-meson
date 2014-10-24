/*
 * Amlogic Watchdog Timer Driver for Meson Chip
 *
 * Author: Bobby Yang <bo.yang@amlogic.com>
 *
 * Copyright (C) 2011 Amlogic Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <mach/am_regs.h>

#ifndef WATCHDOG_ENABLE_BIT
#define WATCHDOG_ENABLE_BIT     (22)
#endif

#define AML_WDT_PING_TIMEOUT        1   /* 1 seconds  */
#define AML_WDT_STARTUP_TIMEOUT     30  /* 30 seconds */
#define AML_WDT_USER_PET_TIMEOUT    45  /* 45 seconds */

#define AML_WDT_DEV_NAME        "aml_wdt"

/* watchdog timer hardware routines */
static void aml_wdt_settimeout(unsigned int timeout);
static unsigned int aml_wdt_gettimeout(void);
static void aml_wdt_set_enable(bool enalbe);
static bool aml_wdt_get_enable(void);
static void aml_wdt_keepalive(void);

/* internal timer service routine */
static void aml_wdt_timer_ping(unsigned long data);

/* internal timer && timerout */
static struct timer_list ping_timer;
static int ping_timeout = AML_WDT_PING_TIMEOUT;
static bool ping_enable = true;
static bool reset_enable = true;

/*
 * user space will set this value to indicate still alive.
 * the default value indicate that the user space has not startup.
 */
static unsigned int user_pet = 1;
static unsigned int user_pet_old = 0;
static unsigned int user_pet_timeout = AML_WDT_USER_PET_TIMEOUT;
static unsigned int user_pet_timer_count_max = (AML_WDT_USER_PET_TIMEOUT/AML_WDT_PING_TIMEOUT) + 1;
static unsigned int user_pet_timer_count = 0;

static unsigned int user_pet_debug = 0; /* print user_pet and  user_pet_old when aml_wdt_timer_ping is called */
static unsigned int user_pet_reset_enable = 1; /* reset when software watchdog is triggered */

/*
 * set watchdog timer timeout.
 * the watchdog timer issues a chip reset when the watchdog timer timeout.
 * @timeout watchdog reset timeout in second
 * @todo timeout limits supported.
 */
static inline void aml_wdt_settimeout(unsigned int timeout)
{
	unsigned int enable = READ_CBUS_REG_BITS(WATCHDOG_TC, WATCHDOG_ENABLE_BIT, 1);
	WRITE_CBUS_REG(WATCHDOG_RESET, 0);
	WRITE_CBUS_REG(WATCHDOG_TC, (enable << WATCHDOG_ENABLE_BIT) | (0x186a0 * timeout));
}

/*
 * return watchdog timer timeout in second.
 */
static inline unsigned int aml_wdt_gettimeout(void)
{
	return (READ_CBUS_REG_BITS(WATCHDOG_TC, 0, 22) / 0x186a0);
}

/*
 * enable/disable the watchdog reset
 */
static inline void aml_wdt_set_enable(bool enable)
{
	/* NOTES: if we only write the enalbe bit, the other bits of the register
	* will be reset to zero, so we must save the other bits value before we
	* write the enable bit.
	*/
	unsigned int timeout = READ_CBUS_REG_BITS(WATCHDOG_TC, 0, 22);
	reset_enable = enable;

	/* fix reboot by robin.zhu */
	if (enable) {
		aml_write_reg32(P_AO_RTI_STATUS_REG0, 0);
	}

	WRITE_CBUS_REG(WATCHDOG_RESET, 0);
	WRITE_CBUS_REG(WATCHDOG_TC, ((enable ? 1 : 0) << WATCHDOG_ENABLE_BIT) | timeout);
}

/*
 * get the watchdog reset status
 */
static inline bool aml_wdt_get_enable(void)
{
	return (READ_CBUS_REG_BITS(WATCHDOG_TC, WATCHDOG_ENABLE_BIT, 1) ? true : false);
}

static void aml_wdt_hw_init(bool enable, unsigned int timeout)
{
	unsigned int val = 0;
	enable = aml_wdt_get_enable();

	/* fix reboot by robin.zhu */
	if (enable) {
		aml_write_reg32(P_AO_RTI_STATUS_REG0, 0);
	}

	val = (enable ? 1 : 0) << WATCHDOG_ENABLE_BIT;
	timeout = READ_CBUS_REG_BITS(WATCHDOG_TC, 0, 22);
	if ((timeout/0x186a0) < AML_WDT_STARTUP_TIMEOUT)
	timeout = 0x186a0 * AML_WDT_STARTUP_TIMEOUT;
	val |= timeout;
	WRITE_CBUS_REG(WATCHDOG_RESET, 0);
	WRITE_CBUS_REG(WATCHDOG_TC, val);
}

/*
 * writing with any value will reset the internal counter
 * of the watchdog timer to zero.
 * this will be used to pet the watchdog.
 */
static inline void aml_wdt_keepalive(void)
{
	WRITE_CBUS_REG(WATCHDOG_RESET, 0);
}


static ssize_t aml_wdt_timeout_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u seconds\n", aml_wdt_gettimeout());
}

static ssize_t aml_wdt_timeout_store(struct device *dev,
                                     struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int val = simple_strtoul(buf, NULL, 10);
	aml_wdt_settimeout(val);
	return count;
}

static DEVICE_ATTR(wdt_timeout, S_IRUGO | S_IWUGO, aml_wdt_timeout_show, aml_wdt_timeout_store);

static ssize_t aml_wdt_reset_enable_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	int len = 0;
	bool en = aml_wdt_get_enable();
	len += sprintf(buf + len, "hardware: %s\n", (en ? "enable" : "disable"));
	len += sprintf(buf + len, "driver:   %s\n", (reset_enable ? "enable" : "disable"));
	return len;
}

static ssize_t aml_wdt_reset_enable_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int val = simple_strtoul(buf, NULL, 10);
	reset_enable = val ? true : false;
	/* 0:false other: true */
	aml_wdt_set_enable(reset_enable);
	return count;
}

static DEVICE_ATTR(reset_enable, S_IRUGO | S_IWUGO, aml_wdt_reset_enable_show, aml_wdt_reset_enable_store);

static ssize_t aml_wdt_ping_timeout_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u seconds\n", ping_timeout);
}

static ssize_t aml_wdt_ping_timeout_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	int val = simple_strtoul(buf, NULL, 10);
	if (val > 0)
		ping_timeout = val;
	else
		ping_timeout = 1;
	user_pet_timer_count_max = user_pet_timeout/ping_timeout + 1;
	return count;
}

static DEVICE_ATTR(ping_timeout, S_IRUGO | S_IWUGO, aml_wdt_ping_timeout_show, aml_wdt_ping_timeout_store);


static ssize_t aml_wdt_ping_enable_show(struct device *dev,
                                        struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", (ping_enable ? "enable" : "disable"));
}

static ssize_t aml_wdt_ping_enable_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	int val = simple_strtol(buf, NULL, 10);
	/* 0:disable other:enable */
	ping_enable = !!val;
	if (!ping_enable)
		del_timer(&ping_timer);
	else
		mod_timer(&ping_timer, jiffies + (ping_timeout * HZ));
	return count;
}

static DEVICE_ATTR(ping_enable, S_IRUGO | S_IWUGO, aml_wdt_ping_enable_show, aml_wdt_ping_enable_store);

static ssize_t aml_wdt_user_pet_timeout_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u seconds\n", user_pet_timeout);
}

static ssize_t aml_wdt_user_pet_timeout_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int val = simple_strtoul(buf, NULL, 10);
	user_pet_timeout = val;
	if (ping_timeout)
		user_pet_timer_count_max = user_pet_timeout/ping_timeout + 1;
	return count;
}

static DEVICE_ATTR(user_pet_timeout, S_IRUGO | S_IWUGO, aml_wdt_user_pet_timeout_show, aml_wdt_user_pet_timeout_store);

/*
 * if user_pet_count is not 0 and does not chagned in 10s,
 * stop pet, and the watchdog timer will reset.
 */
static void aml_wdt_timer_ping(unsigned long data)
{
	if (user_pet) {
		if(user_pet_debug != 0) {
			pr_info("%s, user_pet:%u user_pet_old:%u user_pet_timer_count:%u\n", __func__,
			user_pet, user_pet_old, user_pet_timer_count);
		}

		if (user_pet_old != user_pet) {
			user_pet_old = user_pet;
			user_pet_timer_count = 0;
		} else
			user_pet_timer_count++;

		if (user_pet_timer_count < user_pet_timer_count_max) {
			aml_wdt_keepalive();
			mod_timer(&ping_timer, jiffies + (ping_timeout * HZ));
		} else {
			/* fixed */
			aml_write_reg32(P_AO_RTI_STATUS_REG0, 0);
			aml_write_reg32(P_AO_RTI_STATUS_REG1, 0x8765a5a);

			if(user_pet_reset_enable == 0) {
				user_pet_timer_count = 0;
				aml_wdt_keepalive();
			}
			pr_info("\n*****SOFTWARE WATCHDOG IS Triggered*********************************\n");
			pr_info("%s, user_pet:%u user_pet_old:%u user_pet_timer_count:%u\n", __func__,
			user_pet, user_pet_old, user_pet_timer_count);
			pr_info("*****SOFTWARE WATCHDOG IS Triggered*********************************\n");
			if(user_pet_reset_enable == 0) {
				mod_timer(&ping_timer, jiffies + (ping_timeout * HZ));
			}
		}
	} else {
		aml_wdt_keepalive();
		mod_timer(&ping_timer, jiffies + (ping_timeout * HZ));
	}
}

static int aml_wdt_open(struct inode *inode, struct file *file)
{
	/* @todo */
	return 0;
}

static int aml_wdt_release(struct inode *inode, struct file *file)
{
	/* @todo */
	return 0;
}

static long aml_wdt_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	/* @todo */
	return 0;
}

static const struct file_operations aml_wdt_fops = {
	.owner          = THIS_MODULE,
	.open           = aml_wdt_open,
	.release        = aml_wdt_release,
	.unlocked_ioctl     = aml_wdt_ioctl,
};

static struct miscdevice aml_wdt_miscdev = {
	.minor = WATCHDOG_MINOR,
	.name = "watchdog",
	.fops = &aml_wdt_fops,
};

static int aml_wdt_probe(struct platform_device *pdev)
{
	int ret = 0;

	if (aml_wdt_miscdev.parent)
		return -EBUSY;
	aml_wdt_miscdev.parent = &pdev->dev;

	/*
	* initialize the internal timer.
	* keeping watchdog timer alive will be done in that internal timer.
	*/
	setup_timer(&ping_timer, aml_wdt_timer_ping, 0);
	if (ping_enable)
		mod_timer(&ping_timer, jiffies + (ping_timeout * HZ));

	/* register misc device */
	ret = misc_register(&aml_wdt_miscdev);
	if (ret) {
		pr_err("%s, failed to register misc device\n", __func__);
		goto fail_misc_register;
	}

	/* create device sysfs */
	ret = device_create_file(&pdev->dev, &dev_attr_wdt_timeout);
	ret = device_create_file(&pdev->dev, &dev_attr_reset_enable);
	if (ret)
		goto fail_device_create_file1;
	ret = device_create_file(&pdev->dev, &dev_attr_ping_enable);
	if (ret)
		goto fail_device_create_file2;
	ret = device_create_file(&pdev->dev, &dev_attr_user_pet_timeout);
	if (ret)
		goto fail_device_create_file3;
	ret = device_create_file(&pdev->dev, &dev_attr_ping_timeout);

	user_pet_timer_count_max = user_pet_timeout/ping_timeout + 1;

	/* init && start watchdog timer*/
	aml_wdt_hw_init(true, AML_WDT_STARTUP_TIMEOUT);

	pr_info("%s, driver probe ok\n", __func__);

	return 0;

fail_device_create_file3:
	device_remove_file(&pdev->dev, &dev_attr_ping_enable);
fail_device_create_file2:
	device_remove_file(&pdev->dev, &dev_attr_reset_enable);
fail_device_create_file1:
	misc_deregister(&aml_wdt_miscdev);
fail_misc_register:
	del_timer(&ping_timer);
	return ret;
}

static int aml_wdt_remove(struct platform_device *pdev)
{
	/* stop watchdog timer */
	aml_wdt_set_enable(false);
	device_remove_file(&pdev->dev, &dev_attr_user_pet_timeout);
	device_remove_file(&pdev->dev, &dev_attr_ping_enable);
	device_remove_file(&pdev->dev, &dev_attr_reset_enable);
	misc_deregister(&aml_wdt_miscdev);
	del_timer(&ping_timer);
	pr_info("%s, driver remove ok\n", __func__);
	return 0;
}

#ifdef CONFIG_PM
static unsigned int reg_wdt_ctrl_saved = 0;

static int aml_wdt_suspend(struct platform_device *pdev, pm_message_t state)
{
	reg_wdt_ctrl_saved = READ_CBUS_REG(WATCHDOG_TC);
	aml_wdt_set_enable(false);
	aml_wdt_keepalive(); /* i think it is no usage */
	del_timer(&ping_timer);
	user_pet = 0;
	user_pet_timer_count = 0;
	return 0;
}

static int aml_wdt_resume(struct platform_device *pdev)
{
	if (ping_enable)
		mod_timer(&ping_timer, jiffies + (ping_timeout * HZ));
	WRITE_CBUS_REG(WATCHDOG_RESET, 0);
	WRITE_CBUS_REG(WATCHDOG_TC, reg_wdt_ctrl_saved);
	return 0;
}
#endif

#ifdef CONFIG_USE_OF
static const struct of_device_id aml_wdt_match[] = {
	{
		.compatible = "amlogic,aml_wdt_tv",
	},
	{},
};
#endif

static struct platform_driver aml_wdt_driver = {
	.driver = {
		.name = "aml_wdt",
		.owner  = THIS_MODULE,
#ifdef CONFIG_USE_OF
		.of_match_table = aml_wdt_match,
#endif
	},
	.probe = aml_wdt_probe,
	.remove = aml_wdt_remove,
#ifdef CONFIG_PM
	.suspend = aml_wdt_suspend,
	.resume = aml_wdt_resume,
#endif
};

static int __init aml_wdt_init(void)
{
	pr_info("%s, register platform driver...\n", __func__);
	return platform_driver_register(&aml_wdt_driver);
}

static void __exit aml_wdt_exit(void)
{
	platform_driver_unregister(&aml_wdt_driver);
	pr_info("%s, platform driver unregistered ok\n", __func__);
}

static int __init aml_wdt_reset_enable_setup (char *str)
{
	unsigned int val;
	val = simple_strtoul(str, NULL, 10);
	reset_enable = val ? true : false;
	return 1;
}


__setup ("wdt_reset_en=", aml_wdt_reset_enable_setup);


static int __init aml_wdt_ping_enable_setup (char *str)
{
	unsigned int val;
	val = simple_strtoul(str, NULL, 10);
	ping_enable = val ? true : false;
	return 1;
}


__setup ("wdt_ping_en=", aml_wdt_ping_enable_setup);


module_init(aml_wdt_init);
module_exit(aml_wdt_exit);

module_param(user_pet, uint, S_IRUGO | S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(user_pet, "user space pet entry");

module_param(user_pet_timer_count, uint, S_IRUGO | S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(user_pet_timer_count, "user space pet count");

module_param(user_pet_timer_count_max, uint, S_IRUGO | S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(user_pet_timer_count_max, "user space pet max count");

module_param(user_pet_debug, uint, S_IRUGO | S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(user_pet_debug, "user_pet_debug");

module_param(user_pet_reset_enable, uint, S_IRUGO | S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(user_pet_reset_enable, "user_pet_reset_enable");


MODULE_AUTHOR("Bobby Yang <bo.yang@amlogic.com>");
MODULE_DESCRIPTION("Driver for watchdog timer");
MODULE_LICENSE("GPL");

