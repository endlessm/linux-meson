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

#ifndef __MESON_DRM_H__
#define __MESON_DRM_H__

#include <stddef.h>
#include <drm/drm.h>

struct drm_meson_gem_create_with_ump {
	uint64_t size;
	unsigned int flags;
	unsigned int handle;
	uint32_t ump_secure_id;
};

#define DRM_MESON_GEM_CREATE_WITH_UMP    0x00
#define DRM_MESON_NUM_IOCTLS             0x01

/* Use flags */
#define DRM_MESON_GEM_CREATE_WITH_UMP_FLAG_SCANOUT 0x01
#define DRM_MESON_GEM_CREATE_WITH_UMP_FLAG_TEXTURE 0x02

#define DRM_IOCTL_MESON_GEM_CREATE_WITH_UMP  DRM_IOWR(DRM_COMMAND_BASE + DRM_MESON_GEM_CREATE_WITH_UMP, struct drm_meson_gem_create_with_ump)

#endif
