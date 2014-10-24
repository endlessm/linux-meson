/*
 * arch/arm/mach-meson6tv/cpu.c
 *
 * Copyright (C) 2011-2013 Amlogic, Inc.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <plat/io.h>
#include <plat/regops.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <asm/hardware/cache-l2x0.h>

extern void meson6_l2x0_init(void __iomem *);

#ifdef  CONFIG_MESON_L2CC_OPTIMIZE
static inline u32 __init read_actlr(void)
{
	u32 actlr;

	__asm__("mrc p15, 0, %0, c1, c0, 1\n" : "=r" (actlr));
	printk(KERN_INFO "===actlr=0x%x\n", actlr);
	return actlr;
}

static inline void __init write_actlr(u32 actlr)
{
	__asm__("mcr p15, 0, %0, c1, c0, 1\n" : : "r" (actlr));
}
#endif


unsigned (*get_cpu_temperature_celius)(void) = NULL;
EXPORT_SYMBOL_GPL(get_cpu_temperature_celius);

int get_cpu_temperature(void)
{
	return get_cpu_temperature_celius ? get_cpu_temperature_celius() : -1;
}

int mali_revb_flag = -1;
EXPORT_SYMBOL_GPL(mali_revb_flag);

static int __init maliversion(char *str)
{
	mali_revb_flag=-1;
	if (strncasecmp(str, "a", 1) == 0)
		mali_revb_flag = 0;
	else if(strncasecmp(str, "b", 1) == 0)
		mali_revb_flag = 1;


	return 1;
}
__setup("mali_version=", maliversion);

