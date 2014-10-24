/*
 * Author: AMLOGIC, Inc.
 * Copyright (C) 2010 Amlogic Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _ETH_PLAT_
#define _ETH_PLAT_
#include <mach/pinmux.h>

struct aml_eth_platdata {
	pinmux_item_t *pinmux_items;
	void (*pinmux_setup)(void);
	void (*pinmux_cleanup)(void);
	void (*clock_enable)(void);
	void (*clock_disable)(void);
	void (*reset)(void);
};

extern struct platform_device meson_device_eth;
extern void meson_eth_set_platdata(struct aml_eth_platdata *pd);
#endif /* _ETH_PLAT_ */

