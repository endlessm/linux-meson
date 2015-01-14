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

#include <linux/kernel.h>

#ifndef __MESON_CANVAS_H__
#define __MESON_CANVAS_H__

enum meson_canvas_wrap {
	MESON_CANVAS_WRAP_NONE = 0x00,
	MESON_CANVAS_WRAP_X    = 0x01,
	MESON_CANVAS_WRAP_Y    = 0x02,
};

enum meson_canvas_blkmode {
	MESON_CANVAS_BLKMODE_LINEAR = 0x00,
	MESON_CANVAS_BLKMODE_32x32  = 0x01,
	MESON_CANVAS_BLKMODE_64x64  = 0x02,
};

void meson_canvas_setup(uint32_t canvas_index,
                        uint32_t addr,
                        uint32_t stride, uint32_t height,
                        enum meson_canvas_wrap wrap,
                        enum meson_canvas_blkmode blkmode);

#endif

