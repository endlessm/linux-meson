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

#ifndef __MESON_GEM_PRIME_H__
#define __MESON_GEM_PRIME_H__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <drm/drmP.h>

struct meson_drm_gem_object {
	struct drm_gem_object	base;
	struct dma_attrs	dma_attrs;
	struct page		**pages;
	struct sg_table		*sgt;
	unsigned long		size;
	unsigned int		nr_pages;
	dma_addr_t		paddr;
	void			*vaddr;
	bool			is_scattered;
};

static inline struct meson_drm_gem_object *
to_meson_drm_gem_obj(struct drm_gem_object *gem_obj)
{
	return container_of(gem_obj, struct meson_drm_gem_object, base);
}

struct meson_drm_gem_object *meson_drm_gem_create_with_handle(
		struct drm_device *dev,
		unsigned int size,
		unsigned int *handle,
		struct drm_file *file_priv,
		struct dma_attrs *dma_attrs);
void meson_drm_gem_free_object(struct drm_gem_object *gem_obj);
int meson_drm_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf);
struct sg_table *meson_drm_gem_get_sg_table(struct drm_gem_object *obj);
int meson_drm_gem_mmap(struct file *filp, struct vm_area_struct *vma);
struct meson_drm_gem_object *meson_drm_gem_create_obj(struct drm_device *dev,
		unsigned int size);
int meson_drm_gem_dumb_create(struct drm_file *file_priv,
		struct drm_device *dev,
		struct drm_mode_create_dumb *args);
int meson_drm_gem_dumb_map_offset(struct drm_file *file_priv,
		struct drm_device *drm,
		uint32_t handle,
		uint64_t *offset);

#endif
