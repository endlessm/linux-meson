/*
 * arch/arm/plat-meson/include/mach/cpu.h
 *
 * MESON cpu type detection
 *
 * Copyright (C) 2012 Amlogic
 *
 * Written by Victor Wan <victor.wan@amlogic.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#ifndef __ASM_ARCH_MESON_CPU_H
#define __ASM_ARCH_MESON_CPU_H

#define MESON_CPU_TYPE_MESON1		0x10
#define MESON_CPU_TYPE_MESON2		0x20
#define MESON_CPU_TYPE_MESON3		0x30
#define MESON_CPU_TYPE_MESON6		0x60
#define MESON_CPU_TYPE_MESON6TV		0x70
#define MESON_CPU_TYPE_MESON6TVD	0x75
#define MESON_CPU_TYPE_MESON8		0x80
#define MESON_CPU_TYPE_MESON8B		0x8B
#define MESON_CPU_TYPE_MESONG9TV	0x90


#define MESON_CPU_VERSION_LVL_MAJOR	0
#define MESON_CPU_VERSION_LVL_MINOR	1
#define MESON_CPU_VERSION_LVL_PACK	2
#define MESON_CPU_VERSION_LVL_MISC	3
#define MESON_CPU_VERSION_LVL_MAX	MESON_CPU_VERSION_LVL_MISC

int  meson_cpu_version_init(void);
int get_meson_cpu_version(int level);
#endif
