/*
 * AMLOGIC Audio/Video streaming port driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the named License,
 * or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 * Author:  Tim Yao <timyao@amlogic.com>
 *
 */

#ifndef VDEC_H
#define VDEC_H
#include <mach/cpu.h>

#include <linux/platform_device.h>

#include <mach/cpu.h>

extern void vdec_set_decinfo(void *p);
extern int vdec_set_resource(struct resource *s, struct device *p);

extern s32 vdec_init(vformat_t vf);
extern s32 vdec_release(vformat_t vf);

s32 vdec_dev_register(void);
s32 vdec_dev_unregister(void);
void vdec_power_mode(int level);

#if MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON6TVD

typedef enum {
    VDEC_1,
    VDEC_HCODEC,
    VDEC_2,
    VDEC_HEVC
} vdec_type_t;

extern void vdec2_power_mode(int level);
extern void vdec_poweron(vdec_type_t core);
extern void vdec_poweroff(vdec_type_t core);
extern bool vdec_on(vdec_type_t core);
#else
#define vdec_poweron(core)
#define vdec_poweroff(core)
#endif

#endif /* VDEC_H */
