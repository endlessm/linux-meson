/*
 * Copyright (C) 2016 Endless Mobile
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
 *     Carlo Caione <carlo@endlessm.com>
 */

#ifndef MESON_FB_H
#define MESON_FB_H

struct meson_framebuffer {
	struct drm_framebuffer		fb;
	struct meson_drm_gem_object	*obj[4];
};
#define drm_fb_to_meson_fb(mfb) container_of(mfb, struct meson_framebuffer, fb)

struct drm_fb_helper *meson_fbdev_init(struct drm_device *dev,
		unsigned int preferred_bpp,
		unsigned int num_crtc,
		unsigned int max_conn_count);
struct drm_framebuffer *meson_fb_create(struct drm_device *dev,
		struct drm_file *file_priv, struct drm_mode_fb_cmd2 *mode_cmd);
struct meson_drm_gem_object *meson_drm_get_gem_obj(struct drm_framebuffer *fb,
		unsigned int plane);
int meson_drm_debugfs_show(struct seq_file *m, void *arg);

#endif
