/*
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *  Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * Meson secure APIs.
 *
 * Copyright (C) 2013 Amlogic, Inc.
 *
 * Author: Platform-SH@amlogic.com
 *         Platform-BJ@amlogic.com
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <plat/io.h>
#include <plat/regops.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <asm/cacheflush.h>
#include <asm/hardware/cache-l2x0.h>
#include <linux/dma-mapping.h>
#include <mach/io.h>

#include <mach/meson-secure.h>
#include <linux/sched.h>

#define MESON_SECURE_DEBUG 0
#if MESON_SECURE_DEBUG
#define TZDBG(fmt, args...) printk("meson-secure: " fmt, ## args);
#else
#define TZDBG(fmt, args...)
#endif

/*
int meson_secure_memblock(unsigned startaddr, unsigned endaddr, struct secure_memblock_ctrl* pctrl)
{
	int ret;
	struct secure_memblock_info memblock_info;

	if(!pctrl)
		return -1;
	if(((startaddr & 0xffff) != 0) || ((endaddr&0xffff)!=0xffff)){
		printk("secure memory block must be 16 bits align!\n");
		return -1;
	}

	memblock_info.startaddr = startaddr;
	memblock_info.endaddr = endaddr;
	memcpy(&(memblock_info.memblock_ctrl), pctrl, sizeof(memblock_info.memblock_ctrl));
	__cpuc_flush_dcache_area((void*)&memblock_info, sizeof(memblock_info));
	outer_clean_range(__pa(&memblock_info), __pa(&memblock_info+1));

	ret = meson_smc_internal_api(INTERNAL_API_MEMBLOCK_CONFIG, __pa(&memblock_info));
	return ret;
}
*/

struct memconfig memsecure[MEMCONFIG_NUM] = {0};
int meson_trustzone_memconfig(void)
{
	int ret;
	struct memconfig_hal_api_arg arg;
	arg.memconfigbuf_phy_addr = __pa(memsecure);
	arg.memconfigbuf_count = MEMCONFIG_NUM;

	__cpuc_flush_dcache_area(memsecure, sizeof(memsecure));
	outer_clean_range(__pa(memsecure), (__pa(memsecure + MEMCONFIG_NUM)));
	__cpuc_flush_dcache_area(&arg, sizeof(arg));
	outer_clean_range(__pa(&arg), __pa(((struct memconfig_hal_api_arg*)&arg)) + 1);

	ret = meson_smc_hal_api(TRUSTZONE_HAL_API_MEMCONFIG, __pa(&arg));

	outer_inv_range(__pa(&arg), __pa(((struct memconfig_hal_api_arg*)&arg)) + 1);
	dmac_unmap_area(&arg, sizeof(arg), DMA_FROM_DEVICE);
	outer_inv_range(__pa(memsecure), __pa(memsecure + MEMCONFIG_NUM));
	dmac_unmap_area(memsecure, sizeof(memsecure), DMA_FROM_DEVICE);

	return ret;
}

unsigned int meson_trustzone_getmemsecure_size(void)
{
	unsigned int size = 0;
	int i;
	for (i = 0; i < MEMCONFIG_NUM; i++) {
		if ((memsecure[i].start_phy_addr != 0) && (memsecure[i].end_phy_addr != 0)) {
			size += memsecure[i].end_phy_addr - memsecure[i].start_phy_addr + 1;
		}
	}
	return size;
}

int meson_trustzone_getmemconfig(unsigned char* name, unsigned int* startphyaddr, unsigned int* endphyaddr)
{
	unsigned int i;
	for (i = 0; i < MEMCONFIG_NUM; i++) {
		if (strcmp(name, memsecure[i].name) == 0) {
			break;
		}
	}
	if (i == MEMCONFIG_NUM) {
		return -1;
	}
	if ((memsecure[i].start_phy_addr == 0) || (memsecure[i].end_phy_addr == 0)) {
		return -1;
	}

	*startphyaddr = memsecure[i].start_phy_addr;
	*endphyaddr = memsecure[i].end_phy_addr;
	return 0;
}

int meson_trustzone_efuse(struct efuse_hal_api_arg* arg)
{
	int ret;
	if (!arg) {
		return -1;
	}
	set_cpus_allowed_ptr(current, cpumask_of(0));
	__cpuc_flush_dcache_area(__va(arg->buffer_phy), arg->size);
	outer_clean_range((arg->buffer_phy), (arg->buffer_phy + arg->size));

	__cpuc_flush_dcache_area(__va(arg->retcnt_phy), sizeof(unsigned int));
	outer_clean_range(arg->retcnt_phy, (arg->retcnt_phy + sizeof(unsigned int)));

	__cpuc_flush_dcache_area((void*)arg, sizeof(struct efuse_hal_api_arg));
	outer_clean_range(__pa(arg), __pa(arg + 1));

	ret = meson_smc_hal_api(TRUSTZONE_HAL_API_EFUSE, __pa(arg));

	if (arg->cmd == EFUSE_HAL_API_READ) {
		outer_inv_range((arg->buffer_phy), (arg->buffer_phy + arg->size));
		dmac_unmap_area(__va(arg->buffer_phy), arg->size, DMA_FROM_DEVICE);
	}
	outer_inv_range((arg->retcnt_phy), (arg->retcnt_phy + sizeof(unsigned int)));
	dmac_unmap_area(__va(arg->buffer_phy), arg->size, DMA_FROM_DEVICE);
	set_cpus_allowed_ptr(current, cpu_all_mask);

	return ret;
}

uint32_t meson_secure_reg_read(uint32_t addr)
{
	uint32_t ret;
	uint32_t paddr;
	int offset;

	offset = IO_SECBUS_PHY_BASE - IO_SECBUS_BASE;
	paddr = addr + offset;
	ret = meson_smc2(paddr);

	return ret;
}

uint32_t meson_secure_reg_write(uint32_t addr, uint32_t val)
{
	uint32_t ret;
	uint32_t paddr;
	int offset;

	offset = IO_SECBUS_PHY_BASE - IO_SECBUS_BASE;
	paddr = addr + offset;
	ret = meson_smc3(paddr, val);

	return ret;
}

uint32_t meson_secure_mem_size(void)
{
	return MESON_TRUSTZONE_MEM_SIZE;
}

uint32_t meson_secure_mem_end(void)
{
	return (MESON_TRUSTZONE_MEM_START + MESON_TRUSTZONE_MEM_SIZE);
}
