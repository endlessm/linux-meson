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
#include <linux/module.h>
#include <linux/platform_device.h>

#include <drm/drmP.h>
#include <drm/meson_drm.h>

#include <mach/am_regs.h>

#include "meson_canvas.h"
#include "meson_canvas_def.h"

static int meson_color_format_for_fb(struct drm_framebuffer *fb)
{
	WARN_ON(fb->pixel_format != DRM_FORMAT_ARGB8888);

	return ((0 << 7) | /* Big-endian */
		(1 << 3) | /* ARGB8888 */
		(3 << 0)   /* 32-bit */);
}

static int meson_ge2d_bitblt(struct drm_device *dev,
			     struct drm_framebuffer *src_fb,
			     struct drm_framebuffer *dst_fb,
			     uint32_t dst_x, uint32_t dst_y)
{
	meson_canvas_setup_fb(MESON_DRV_CANVAS_BITBLT_SRC, src_fb);
	meson_canvas_setup_fb(MESON_DRV_CANVAS_BITBLT_DST, dst_fb);

	/* It might look like I'm simply resetting the device here, but
	 * I'm actually setting up all the parameters for our blit. It
	 * just so happens that 0 for all these values chooses the sane
	 * defaults that are all we need. I love this hardware. */

	aml_write_reg32(P_GE2D_GEN_CTRL0, 0);
	aml_write_reg32(P_GE2D_GEN_CTRL1, 0);

	/* There's the possibility for a SRC2, but we don't use that
	 * in a bitblt. */
	aml_write_reg32(P_GE2D_GEN_CTRL2,
			((meson_color_format_for_fb(dst_fb) << 16) | /* DST1 */
			 (meson_color_format_for_fb(src_fb) << 0))   /* SRC1 */);

	aml_write_reg32(P_GE2D_SRC1_X_START_END, (0 << 16) | ((src_fb->width + 1) << 0));
	aml_write_reg32(P_GE2D_SRC1_Y_START_END, (0 << 16) | ((src_fb->height + 1) << 0));

	aml_write_reg32(P_GE2D_DST_X_START_END, (dst_x << 16) | ((dst_x + src_fb->width + 1) << 0));
	aml_write_reg32(P_GE2D_DST_X_START_END, (dst_y << 16) | ((dst_y + src_fb->height + 1) << 0));

	aml_write_reg32(P_GE2D_ALU_OP_CTRL,
			((0x51 << 16) | /* DST COLOR = SRC COLOR */
			 (0x51 << 0))  /* DST ALPHA = SRC ALPHA */);

	/* Start 'er up. */
	aml_write_reg32(P_GE2D_CMD_CTRL, 1);

	/* Awful hack to wait until we're done. */
	do {
		msleep(200);
	} while ((aml_read_reg32(P_GE2D_STATUS0) & 1));

	/* ... and there we go! Everything's all done! */
	return 0;
}

int meson_ioctl_ge2d_bitblt(struct drm_device *dev, void *data,
			    struct drm_file *file)
{
	struct drm_meson_ge2d_bitblt *args = data;
	struct drm_framebuffer *src_fb, *dst_fb;

	src_fb = drm_framebuffer_lookup(dev, args->src_fb_id);
	if (unlikely(!src_fb))
		return -ENOENT;

	dst_fb = drm_framebuffer_lookup(dev, args->dst_fb_id);
	if (unlikely(!dst_fb))
		return -ENOENT;

	return meson_ge2d_bitblt(dev, src_fb, dst_fb, args->dst_x, args->dst_y);
}
