/*
 * include/linux/amlogic/jtag.h
 *
 * Copyright (C) 2015 Amlogic, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __INCLUDE_AMLOGIC_JTAG_H
#define __INCLUDE_AMLOGIC_JTAG_H

#include <linux/types.h>

enum {
	AML_JTAG_DISABLE = 0,
	AML_JTAG_AOPAD_AOCPU,
	AML_JTAG_AOPAD_SYSCPU,
	AML_JTAG_AOPAD_MEDIACPU,
	AML_JTAG_EEPAD_AOCPU,
	AML_JTAG_EEPAD_SYSCPU,
	AML_JTAG_EEPAD_MEDIACPU,
};

#ifdef CONFIG_AML_JTAG_CONTROL
extern bool is_jtag_disable(void);
#else
static inline bool is_jtag_disable(void) {return true;}
#endif

#endif
