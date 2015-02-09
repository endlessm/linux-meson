/*
 * Copyright (C) 2014 Endless Mobile
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 *
 * Written by:
 *     Jasper St. Pierre <jstpierre@mecheye.net>
 */


#ifndef __MESON_MODES_H__
#define __MESON_MODES_H__

#include <drm/drmP.h>
#include <linux/amlogic/vout/vout_notify.h>

enum meson_modes_flags {
	MESON_MODES_HDMI = (1 << 0),
	MESON_MODES_CVBS = (1 << 1),
};

/* For the CVBS hack modes, we multiply by 2, then
 * take off a hardcoded 10% for overscan. */
#define CVBS_HACK_MODE_OVERSCAN_PERCENT (10)
#define CVBS_HACK_MODE_SIZE(w) ((w-(w/CVBS_HACK_MODE_OVERSCAN_PERCENT))*2)

vmode_t drm_mode_to_vmode(const struct drm_display_mode *mode,
			  enum meson_modes_flags flags);

#endif /* __MESON_MODES_H__ */
