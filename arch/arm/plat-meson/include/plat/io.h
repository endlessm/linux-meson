/*
 *
 * arch/arm/plat-meson/include/plat/regops.h
 *
 *  Copyright (C) 2011 AMLOGIC, INC.
 *
 *	Common interface for aml CBUS/AHB/APB/AXI operation
 *
 * License terms: GNU General Public License (GPL) version 2
 * Basic platform init and mapping functions.
 */
#ifndef __REGOPS_H__
#define __REGOPS_H__
#include <linux/types.h>
#include <linux/io.h>
#ifndef __ASSEMBLY__
#ifdef CONFIG_SMP
#define MESON_SMP_RMB() smp_rmb()
#define MESON_SMP_WMB() smp_wmb()
#define MESON_SMP_MB() smp_mb()
#else
#define MESON_SMP_RMB()
#define MESON_SMP_WMB()
#define MESON_SMP_MB()
#endif
#ifndef REGOPS_DEBUG
static __inline__ uint32_t aml_read_reg32(uint32_t _reg)
{
    MESON_SMP_RMB();
	return readl_relaxed((volatile void *)_reg);
};
static __inline__ void aml_write_reg32( uint32_t _reg, const uint32_t _value)
{
	writel_relaxed( _value,(volatile void *)_reg );
	MESON_SMP_MB();
};
static __inline__ void aml_set_reg32_bits(uint32_t _reg, const uint32_t _value, const uint32_t _start, const uint32_t _len)
{
    MESON_SMP_RMB();
	writel_relaxed(( (readl_relaxed((volatile void *)_reg) & ~((( 1L << (_len) )-1) << (_start))) | ((unsigned)((_value)&((1L<<(_len))-1)) << (_start))), (volatile void *)_reg );
	MESON_SMP_WMB();
}
static __inline__ void aml_clrset_reg32_bits(uint32_t _reg, const uint32_t clr, const uint32_t set)
{
    MESON_SMP_RMB();
	writel_relaxed((readl_relaxed((volatile void *)_reg) & ~(clr)) | (set), (volatile void *)_reg );
	MESON_SMP_WMB();
}

static __inline__ uint32_t aml_get_reg32_bits(uint32_t _reg, const uint32_t _start, const uint32_t _len)
{
    MESON_SMP_RMB();
	return	( (readl_relaxed((volatile void *)_reg) >> (_start)) & (( 1L << (_len) ) - 1) );
}
static __inline__ void aml_set_reg32_mask( uint32_t _reg, const uint32_t _mask )
{
    MESON_SMP_RMB();
		writel_relaxed( (readl_relaxed((volatile void *)_reg) | (_mask) ), (volatile void *)_reg );
		MESON_SMP_WMB();
}
static __inline__ void aml_clr_reg32_mask( uint32_t _reg, const uint32_t _mask)
{
    MESON_SMP_RMB();
		writel_relaxed( (readl_relaxed((volatile void *)_reg) & (~(_mask)) ), (volatile void *)_reg );
		MESON_SMP_WMB();
}
#else
static __inline__ uint32_t aml_read_reg32( uint32_t _reg)
{
	uint32_t _val;

	_val = readl_relaxed((volatile void *)_reg);
	if(g_regops_dbg_lvl)
		printk( KERN_DEBUG "rd:%X = %X\n",_reg,_val);
	return val;
};
static __inline__ void aml_write_reg32( uint32_t _reg, const uint32_t _value)
{
	if(g_regops_dbg_lvl)
		printk( KERN_DEBUG "wr:%X = %X\n",_reg,_value);
	writel_relaxed( _value, (volatile void *)_reg );
};
static __inline__ void aml_set_reg32_bits( uint32_t _reg, const uint32_t _value,const uint32_t _start, const uint32_t _len)
{
	if(g_regops_dbg_lvl)
		printk( KERN_DEBUG "setbit:%X = (%X,%X,%X)\n",_reg,_value,_start,_len);
	writel_relaxed((readl_relaxed((volatile void *)_reg) & ~((( 1L << (_len) )-1) << (_start)) | ((unsigned)((_value)&((1L<<(_len))-1)) << (_start))), (volatile void *)_reg );
}
static __inline__ void aml_clrset_reg32_bits(uint32_t _reg, const uint32_t clr, const uint32_t set)
{
    if(g_regops_dbg_lvl)
		printk( KERN_DEBUG "clrsetbit:%X = (%X,%X)\n",_reg,clr,set);
	writel_relaxed((readl_relaxed((volatile void *)_reg) & ~(clr)) | (set), (volatile void *)_reg );
}

static __inline__ uint32_t aml_get_reg32_bits( uint32_t _reg, const uint32_t _start, const uint32_t _len)
{
	uint32_t _val;

	_val = ( (readl_relaxed((volatile void *)_reg) >> (_start)) & (( 1L << (_len) ) - 1) );

	if(g_regops_dbg_lvl)
		printk( KERN_DEBUG "getbit:%X = (%X,%X,%X)\n",_reg,_val,_start,_len);

	return	_val;
}
static __inline__ void aml_set_reg32_mask( uint32_t _reg, const uint32_t _mask )
{
	uint32_t _val;

	_val = readl_relaxed((volatile void *)_reg) | _mask;

	if(g_regops_dbg_lvl)
		printk( KERN_DEBUG "setmsk:%X |= %X, %X\n",_reg,_mask,_val);

	writel_relaxed( val , (volatile void *)_reg );
}
static __inline__ void aml_clr_reg32_mask( uint32_t _reg, const uint32_t _mask)
{
	uint32_t _val;

	_val = readl_relaxed((volatile void *)_reg) & (~ _mask);

	if(g_regops_dbg_lvl)
		printk( KERN_DEBUG "setmsk:%X |= %X, %X\n",_reg,_mask,_val);

	writel_relaxed( _val , (volatile void *)_reg );
}
#endif
#endif

#endif
