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

#include "meson_canvas.h"

#include <mach/am_regs.h>

/* Set up a canvas. */
void meson_canvas_setup(uint32_t canvas_index,
                        uint32_t addr,
                        uint32_t stride, uint32_t height,
                        enum meson_canvas_wrap wrap,
                        enum meson_canvas_blkmode blkmode)
{
	CANVAS_WRITE(DC_CAV_LUT_DATAL,
		     (((addr + 7) >> 3)) |
		     (((stride + 7) >> 3) << CANVAS_WIDTH_LBIT));
	CANVAS_WRITE(DC_CAV_LUT_DATAH,
		     ((((stride + 7) >> 3) >> CANVAS_WIDTH_LWID) << CANVAS_WIDTH_HBIT) |
		     (height << CANVAS_HEIGHT_BIT) |
		     (wrap << 22) |
		     (blkmode << CANVAS_BLKMODE_BIT));
	CANVAS_WRITE(DC_CAV_LUT_ADDR, CANVAS_LUT_WR_EN | canvas_index);

	/* Force a read-back to make sure everything is flushed. */
	CANVAS_READ(DC_CAV_LUT_DATAH);
}
