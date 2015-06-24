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

enum drm_meson_msync_op {
	DRM_MESON_MSYNC_CLEAN = 0,
	DRM_MESON_MSYNC_CLEAN_AND_INVALIDATE = 1,
	DRM_MESON_MSYNC_INVALIDATE = 2,
	DRM_MESON_MSYNC_FLUSH_L1   = 3,
	DRM_MESON_MSYNC_READOUT_CACHE_ENABLED = 128,
};

struct drm_meson_msync {
	u32 handle;
	enum drm_meson_msync_op op;
	void *mapping;
	void *address;
	u32 size;
	u32 is_cached;
};

#define DRM_MESON_GEM_DOMAIN_CPU            0x01
#define DRM_MESON_GEM_DOMAIN_MALI           0x02

struct drm_meson_gem_set_domain {
	u32 handle;
	u32 write_domain;
};

enum drm_meson_cache_op_control {
	DRM_MESON_CACHE_OP_START,
	DRM_MESON_CACHE_OP_FINISH,
	DRM_MESON_CACHE_OP_COUNT,
};

struct drm_meson_cache_operations_control {
	enum drm_meson_cache_op_control op;
};

#define DRM_MESON_GEM_CREATE_WITH_UMP       0x00
#define DRM_MESON_MSYNC                     0x01
#define DRM_MESON_GEM_SET_DOMAIN            0x02
#define DRM_MESON_CACHE_OPERATIONS_CONTROL  0x03
#define DRM_MESON_NUM_IOCTLS                0x04

/* Use flags */
#define DRM_MESON_GEM_CREATE_WITH_UMP_FLAG_SCANOUT 0x01
#define DRM_MESON_GEM_CREATE_WITH_UMP_FLAG_TEXTURE 0x02

#define DRM_IOCTL_MESON_GEM_CREATE_WITH_UMP  DRM_IOWR(DRM_COMMAND_BASE + DRM_MESON_GEM_CREATE_WITH_UMP, struct drm_meson_gem_create_with_ump)
#define DRM_IOCTL_MESON_MSYNC  DRM_IOWR(DRM_COMMAND_BASE + DRM_MESON_MSYNC, struct drm_meson_msync)
#define DRM_IOCTL_MESON_GEM_SET_DOMAIN  DRM_IOWR(DRM_COMMAND_BASE + DRM_MESON_GEM_SET_DOMAIN, struct drm_meson_gem_set_domain)
#define DRM_IOCTL_MESON_CACHE_OPERATIONS_CONTROL  DRM_IOWR(DRM_COMMAND_BASE + DRM_MESON_CACHE_OPERATIONS_CONTROL, struct drm_meson_cache_operations_control)

#endif
