/*
 * drivers/amlogic/display/aml_jtag.c
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
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/printk.h>

#include <mach/register.h>
#include <mach/io.h>
#include <plat/io.h>
#include <linux/amlogic/jtag.h>
#include <linux/amlogic/aml_wdt.h>



#undef pr_fmt
#define pr_fmt(fmt) "jtag: "fmt
#define INFO(format, arg...) pr_info(format, ## arg)
#define ERR(format,  arg...)  pr_err(format, ## arg)


#define AML_JTAG_NAME		"aml_jtag"


static struct class jtag_cls;
static unsigned int jtag_select = AML_JTAG_DISABLE;

bool is_jtag_disable(void)
{
	if (jtag_select == AML_JTAG_DISABLE)
		return true;
	else
		return false;
}


static void jtag_clean_other_pinmux_aopad(void)
{
	// clean GPIOAO_8
	aml_clr_reg32_mask(P_AO_RTI_PIN_MUX_REG,
		(1<<17) | (1<<14) | (1<<16) | (1<<28));
	aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_11, (1<<26));
	// clean GPIOAO_9
	aml_clr_reg32_mask(P_AO_RTI_PIN_MUX_REG,
		(1<<27) | (1<<13) | (1<<15) | (1<<27));
	aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_11, (1<<25));
	// clean GPIOAO_11
	aml_clr_reg32_mask(P_AO_RTI_PIN_MUX_REG, (1<<22) | (1<<28));
}


static void jtag_clean_other_pinmux_eepad(void)
{
	// clean CARD_0
	aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_2, (1<<14) | (1<<7));
	// clean CARD_1
	aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_2, (1<<15) | (1<<6));
	// clean CARD_2
	aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_2, (1<<11) | (1<<5));
	// clean CARD_3
	aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_2, (1<<10) | (1<<4));
}


static void jtag_setup(void)
{
	switch (jtag_select) {
	case AML_JTAG_DISABLE:
		// 0x0
		aml_write_reg32(P_AO_SECURE_REG1, 0x0);
		break;
	case AML_JTAG_AOPAD_AOCPU:
		// 0x5
		aml_write_reg32(P_AO_SECURE_REG1, (0x01<<0) | (0x01<<2));
		// clean other pinmux
		jtag_clean_other_pinmux_aopad();
		break;
	case AML_JTAG_AOPAD_SYSCPU:
		// 0x102
		aml_write_reg32(P_AO_SECURE_REG1, (0x02<<0) | (0x01<<8));
		// clean other pinmux
		jtag_clean_other_pinmux_aopad();
		break;
	case AML_JTAG_AOPAD_MEDIACPU:
		// 0x43
		aml_write_reg32(P_AO_SECURE_REG1, (0x03<<0) | (0x01<<6));
		// clean other pinmux
		jtag_clean_other_pinmux_aopad();
		break;
	case AML_JTAG_EEPAD_AOCPU:
		// 0x18
		aml_write_reg32(P_AO_SECURE_REG1, (0x01<<4) | (0x02<<2));
		// clean other pinmux
		jtag_clean_other_pinmux_eepad();
		break;
	case AML_JTAG_EEPAD_SYSCPU:
		// 0x220
		aml_write_reg32(P_AO_SECURE_REG1, (0x02<<4) | (0x02<<8));
		// clean other pinmux
		jtag_clean_other_pinmux_eepad();
		break;
	case AML_JTAG_EEPAD_MEDIACPU:
		// 0xb0
		aml_write_reg32(P_AO_SECURE_REG1, (0x03<<4) | (0x02<<6));
		// clean other pinmux
		jtag_clean_other_pinmux_eepad();
		break;
	default: /* none */
		aml_write_reg32(P_AO_SECURE_REG1, 0x0);
		break;
	}
}

static void jtag_disable_watchdog(void)
{

	switch (jtag_select) {
	case AML_JTAG_AOPAD_AOCPU:
	case AML_JTAG_AOPAD_SYSCPU:
	case AML_JTAG_AOPAD_MEDIACPU:
	case AML_JTAG_EEPAD_AOCPU:
	case AML_JTAG_EEPAD_SYSCPU:
	case AML_JTAG_EEPAD_MEDIACPU:
		disable_watchdog();
		break;
	default:
		break;
	}
}

static int __init setup_jtag(char *p)
{
	INFO("%s\n", p);

	if (!strncmp(p, "disable", 7))
		jtag_select = AML_JTAG_DISABLE;
	else if (!strncmp(p, "aopad_aocpu", 11))
		jtag_select = AML_JTAG_AOPAD_AOCPU;
	else if (!strncmp(p, "aopad_syscpu", 12))
		jtag_select = AML_JTAG_AOPAD_SYSCPU;
	else if (!strncmp(p, "aopad_mediacpu", 14))
		jtag_select = AML_JTAG_AOPAD_MEDIACPU;
	else if (!strncmp(p, "eepad_aocpu", 11))
		jtag_select = AML_JTAG_EEPAD_AOCPU;
	else if (!strncmp(p, "eepad_syscpu", 12))
		jtag_select = AML_JTAG_EEPAD_SYSCPU;
	else if (!strncmp(p, "eepad_mediacpu", 14))
		jtag_select = AML_JTAG_EEPAD_MEDIACPU;
	else
		jtag_select = AML_JTAG_DISABLE;

	return 0;
}
__setup("jtag=", setup_jtag);



static ssize_t jtag_setup_show(struct class *cls,
			struct class_attribute *attr, char *buf)
{
	unsigned int len = 0;
	len += sprintf(buf + len, "jtag select %d\n", jtag_select);
	len += sprintf(buf + len, "\n");
	len += sprintf(buf + len, "usage:\n");
	len += sprintf(buf + len, "    echo <n> > setup\n");
	len += sprintf(buf + len, "\n");
	len += sprintf(buf + len, "the value of <n> is:\n");
	len += sprintf(buf + len, "    0 - disable\n");
	len += sprintf(buf + len, "    1 - aopad_aocpu\n");
	len += sprintf(buf + len, "    2 - aopad_syscpu\n");
	len += sprintf(buf + len, "    3 - aopad_mediacpu\n");
	len += sprintf(buf + len, "    4 - eepad_aocpu\n");
	len += sprintf(buf + len, "    5 - eepad_syscpu\n");
	len += sprintf(buf + len, "    6 - eepad_mediacpu\n");
	return len;
}


static ssize_t jtag_setup_store(struct class *cls,
			 struct class_attribute *attr,
			 const char *buffer, size_t count)
{
	unsigned int sel;

	sel = simple_strtol(buffer, NULL, 10);
	if ((sel < AML_JTAG_DISABLE) || (sel > AML_JTAG_EEPAD_MEDIACPU)) {
		INFO("invalid value\n");
		return count;
	}

	jtag_select = sel;
	/* setup jtag */
	jtag_disable_watchdog();
	jtag_setup();


	return count;
}


static ssize_t jtag_debug_show(struct class *cls,
			struct class_attribute *attr, char *buf)
{
	unsigned int mux_ao, mux_11, mux_2, sec_1;
	unsigned int len = 0;

	mux_ao = aml_read_reg32(P_AO_RTI_PIN_MUX_REG);
	mux_11 = aml_read_reg32(P_PERIPHS_PIN_MUX_11);
	mux_2  = aml_read_reg32(P_PERIPHS_PIN_MUX_2);
	sec_1  = aml_read_reg32(P_AO_SECURE_REG1);

	len += sprintf(buf + len, "P_AO_RTI_PIN_MUX_REG = 0x%x\n", mux_ao);
	len += sprintf(buf + len, "P_PERIPHS_PIN_MUX_11 = 0x%x\n", mux_11);
	len += sprintf(buf + len, "P_PERIPHS_PIN_MUX_2  = 0x%x\n", mux_2);
	len += sprintf(buf + len, "P_AO_SECURE_REG1     = 0x%x\n", sec_1);
	len += sprintf(buf + len, "\n");
	len += sprintf(buf + len, "JTAG_AO_PAD  Secure[1:0] = %u\n",
		(sec_1 & (0x3 << 0)) >> 0);
	len += sprintf(buf + len, "JTAG_EE_PAD  Secure[5:4] = %u\n",
		(sec_1 & (0x3 << 4)) >> 4);
	len += sprintf(buf + len, "JTAG_SYS_CPU Secure[9:8] = %u\n",
		(sec_1 & (0x3 << 8)) >> 8);
	len += sprintf(buf + len, "JTAG_AO_CPU  Secure[3:2] = %u\n",
		(sec_1 & (0x3 << 2)) >> 2);
	len += sprintf(buf + len, "JTAG_MED_CPU Secure[7:6] = %u\n",
		(sec_1 & (0x3 << 6)) >> 6);

	// HDMI_CEC_AO bit[17]
	len += sprintf(buf + len, "GPIOAO_8\n");
	len += sprintf(buf + len, "HDMI_CEC_AO  ao_reg[17] = %u\n",
		(mux_ao & (1 << 17)) >> 17);
	len += sprintf(buf + len, "HDMITX_CEC   ao_reg[14] = %u\n",
		(mux_ao & (1 << 17)) >> 17);
	len += sprintf(buf + len, "HDMITX_CEC   reg11[26]  = %u\n",
		(mux_11 & (1 << 26)) >> 26);
	len += sprintf(buf + len, "HDMIRX_CEC   ao_reg[16] = %u\n",
		(mux_ao & (1 << 16)) >> 16);
	len += sprintf(buf + len, "HDMIRX_CEC   ao_reg[28] = %u\n",
		(mux_ao & (1 << 28)) >> 28);

	len += sprintf(buf + len, "GPIOAO_9\n");
	len += sprintf(buf + len, "HDMI_CEC_AO  ao_reg[27] = %u\n",
		(mux_ao & (1 << 27)) >> 27);
	len += sprintf(buf + len, "HDMITX_CEC   ao_reg[13] = %u\n",
		(mux_ao & (1 << 13)) >> 13);
	len += sprintf(buf + len, "HDMITX_CEC   reg11[25]  = %u\n",
		(mux_11 & (1 << 25)) >> 25);
	len += sprintf(buf + len, "HDMIRX_CEC   ao_reg[15] = %u\n",
		(mux_ao & (1 << 15)) >> 15);
	len += sprintf(buf + len, "HDMIRX_CEC   ao_reg[27] = %u\n",
		(mux_ao & (1 << 27)) >> 27);

	len += sprintf(buf + len, "GPIOAO_10\n");
	len += sprintf(buf + len, "GPIOAO_11\n");
	len += sprintf(buf + len, "PWM_AO_A     ao_reg[22] = %u\n",
		(mux_ao & (1 << 22)) >> 22);
	len += sprintf(buf + len, "PWM_F        ao_reg[28] = %u\n",
		(mux_ao & (1 << 28)) >> 28);

	len += sprintf(buf + len, "CARD_0\n");
	len += sprintf(buf + len, "SD_D1_B      reg2[14]   = %u\n",
		(mux_2 & (1 << 14)) >> 14);
	len += sprintf(buf + len, "SDXC_D1_B    reg2[7]    = %u\n",
		(mux_2 & (1 << 7)) >> 7);

	len += sprintf(buf + len, "CARD_1\n");
	len += sprintf(buf + len, "SD_D0_B      reg2[15]   = %u\n",
		(mux_2 & (1 << 15)) >> 15);
	len += sprintf(buf + len, "SDXC_D0_B    reg2[6]    = %u\n",
		(mux_2 & (1 << 6)) >> 6);

	len += sprintf(buf + len, "CARD_2\n");
	len += sprintf(buf + len, "SD_CLK_B     reg2[11]   = %u\n",
		(mux_2 & (1 << 11)) >> 11);
	len += sprintf(buf + len, "SDXC_CLK_B   reg2[5]    = %u\n",
		(mux_2 & (1 << 5)) >> 5);

	len += sprintf(buf + len, "CARD_3\n");
	len += sprintf(buf + len, "SD_CMD_B     reg2[10]   = %u\n",
		(mux_2 & (1 << 10)) >> 10);
	len += sprintf(buf + len, "SDXC_CMD_B   reg2[4]    = %u\n",
		(mux_2 & (1 << 4)) >> 4);
	return len;
}


static struct class_attribute aml_jtag_attrs[] = {
	__ATTR(debug,  0644, jtag_debug_show, NULL),
	__ATTR(setup,  0644, jtag_setup_show, jtag_setup_store),
	__ATTR_NULL,
};


static int __init aml_jtag_init(void)
{
	/* create class attributes */
	jtag_cls.name = AML_JTAG_NAME;
	jtag_cls.owner = THIS_MODULE;
	jtag_cls.class_attrs = aml_jtag_attrs;
	class_register(&jtag_cls);

	/* setup jtag */
	jtag_disable_watchdog();
	jtag_setup();

	INFO("module init ok\n");
	return 0;
}


static void __exit aml_jtag_exit(void)
{
	class_unregister(&jtag_cls);
	INFO("module exit\n");
}


module_init(aml_jtag_init);
module_exit(aml_jtag_exit);

MODULE_DESCRIPTION("Meson JTAG Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Amlogic, Inc.");

