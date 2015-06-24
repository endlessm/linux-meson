/*
 * Copyright (C) 2015 Endless Mobile
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
 *     João Paulo Rechi Vita <jprvita@endlessm.com>
 */

#include <asm/cacheflush.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/meson_drm.h>
#include <linux/mutex.h>

#include "meson_priv.h"

#define DBG_MSG(level,args) do {} while(0) /* FIXME: debug properly */

static void level1_cache_flush_all(void)
{
	DBG_MSG(4, ("meson_drm_ump_osk_msync(): Flushing the whole L1 cache\n"));
	__cpuc_flush_kern_all();
}

/* This is a copy of _ump_osk_msync from drivers/amlogic/gpu/ump/linux/ump_osk_low_level_mem.c
 * with adapted parameters */
static void meson_drm_ump_osk_msync(struct drm_gem_cma_object *cma_obj, void *virt, u32 offset, size_t size, enum meson_drm_msync_op op, struct meson_drm_session_data *session_data)
{
	struct drm_gem_object *gem_obj;
	u32 start_p, end_p;

	/* Flush L1 using virtual address, the entire range in one go.
	 * Only flush if user space process has a valid write mapping on given address. */
	if ((cma_obj) && (virt != NULL) && (access_ok(VERIFY_WRITE, virt, size))) {
		__cpuc_flush_dcache_area(virt, size);
		DBG_MSG(3, ("meson_drm_ump_osk_msync(): Flushing CPU L1 Cache. CPU address: %x, size: %x\n", virt, size));
	} else {
		if (session_data) {
			if (op == MESON_DRM_MSYNC_FLUSH_L1) {
				DBG_MSG(4, ("meson_drm_ump_osk_msync(): Pending %d L1 cache flushes\n", session_data->has_pending_level1_cache_flush));
				session_data->has_pending_level1_cache_flush = 0;
				level1_cache_flush_all();
				return;
			} else {
				if (session_data->cache_operations_ongoing) { /* This is set in cache_operations_control(start) */
					session_data->has_pending_level1_cache_flush++;
					DBG_MSG(4, ("meson_drm_ump_osk_msync(): Defering L1 cache flush (%d pending)\n" session_data->has_pending_level1_cache_flush));
				} else {
					/* Flushing the L1 cache for each switch_user() if ump_cache_operations_control(START) is not called */
					level1_cache_flush_all();
				}
			}
		} else {
			DBG_MSG(4, ("Unkown state %s %d\n", __FUNCTION__, __LINE__));
			level1_cache_flush_all();
		}
	}

	if (!cma_obj)
		return;
	gem_obj = &cma_obj->base;

	DBG_MSG(3, ("meson_drm_ump_osk_msync(): Flushing CPU L2 Cache\n"));

	/* Flush L2 using physical addresses
	 * Our allocations are always contiguous (GEM CMA), so we have only one mem block */

	if (offset >= gem_obj->size) {
		offset -= gem_obj->size;
		return;
	}

	if (offset) {
		start_p = (u32)cma_obj->paddr + offset;
		/* We'll zero the offset later, after using it to calculate end_p. */
	} else {
		start_p = (u32)cma_obj->paddr;
	}

	if (size < gem_obj->size - offset) {
		end_p = start_p + size;
		size = 0;
	} else {
		if (offset) {
			end_p = start_p + (gem_obj->size - offset);
			size -= gem_obj->size - offset;
			offset = 0;
		} else {
			end_p = start_p + gem_obj->size;
			size -= gem_obj->size;
		}
	}

	switch (op) {
	case MESON_DRM_MSYNC_CLEAN:
		outer_clean_range(start_p, end_p);
		break;
	case MESON_DRM_MSYNC_CLEAN_AND_INVALIDATE:
		outer_flush_range(start_p, end_p);
		break;
	case MESON_DRM_MSYNC_INVALIDATE:
		outer_inv_range(start_p, end_p);
		break;
	default:
		break;
	}

	return;
}

/* this code was heavily inspired by _ump_ukk_msync() in
 * drivers/amlogic/gpu/ump/common/ump_kernel_api.c */
int meson_ioctl_msync(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_gem_object *gem_obj;
	struct drm_gem_cma_object *cma_obj;
	struct drm_meson_msync *args = data;
	struct meson_drm_session_data *session_data = file->driver_priv;

	void *virtual = NULL;
	u32 size = 0;
	u32 offset = 0;

	if (!args || !session_data)
		return -EINVAL;

	gem_obj = drm_gem_object_lookup(dev, file, args->handle);
	if (NULL == gem_obj) {
		DBG_MSG(1, ("meson_ioctl_msync(): %02u Failed to look up mapping\n", args->handle));
		return -EFAULT;
	}

	cma_obj = to_drm_gem_cma_obj(gem_obj);
	if (NULL == cma_obj) {
		DBG_MSG(1, ("meson_ioctl_msync(): %02u Failed to get gem_cma_obj containing gem_obj\n", args->handle));
		return -EFAULT;
	}

	/* Returns the cache settings back to Userspace */
	args->is_cached = dma_get_attr(DMA_ATTR_NON_CONSISTENT, &cma_obj->dma_attrs);

	DBG_MSG(3, ("meson_ioctl_msync(): %02u cache_enabled %d\n op %d address 0x%08x mapping 0x%08x\n",
		    args->handle, args->is_cached, args->op, args->address, args->mapping));

	/* Nothing to do in these cases */
	if ((MESON_DRM_MSYNC_READOUT_CACHE_ENABLED == args->op) || (!args->is_cached))
		return 0;

	if (args->address) {
		virtual = (void *)((u32)args->address);
		offset = (u32)((args->address) - (args->mapping));
	} else {
		/* Flush entire mapping when no address is specified. */
		virtual = args->mapping;
	}

	if (args->size) {
		size = args->size;
	} else {
		/* Flush entire mapping when no size is specified. */
		size = gem_obj->size - offset;
	}

	if ((offset + size) > gem_obj->size) {
		DBG_MSG(1, ("meson_ioctl_msync(): %02u Trying to flush more than the entire allocation: offset %u + size %u > %u\n", args->handle, offset, size, gem_obj->size));
		return -EINVAL;
	}

	/* No need to lock here since session_data=NULL */
	meson_drm_ump_osk_msync(cma_obj, virtual, offset, gem_obj->size, args->op, NULL);

	return 0;
}

/* this code was heavily inspired by _ump_ukk_switch_hw_usage() in
 * drivers/amlogic/gpu/ump/common/ump_kernel_api.c */
int meson_ioctl_set_domain(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_gem_object *gem_obj;
	struct drm_gem_cma_object *cma_obj;
	struct drm_meson_gem_set_domain *args = data;
	struct meson_drm_session_data *session_data = file->driver_priv;
	enum meson_drm_msync_op cache_op = MESON_DRM_MSYNC_CLEAN_AND_INVALIDATE;

	if (!args || !session_data)
		return -EINVAL;

	gem_obj = drm_gem_object_lookup(dev, file, args->handle);
	if (NULL == gem_obj) {
		DBG_MSG(1, ("meson_ioctl_set_domain(): %02u Failed to look up mapping\n", args->handle));
		return -EFAULT;
	}

	cma_obj = to_drm_gem_cma_obj(gem_obj);
	if (NULL == cma_obj) {
		DBG_MSG(1, ("meson_ioctl_set_domain(): %02u Failed to get gem_cma_obj containing gem_obj\n", args->handle));
		return -EFAULT;
	}

	DBG_MSG(3, ("meson_ioctl_set_domain(): %02u %s -> %s\n",
		    args->handle,
		    gem_obj->write_domain == DRM_MESON_GEM_DOMAIN_CPU ? "CPU" : "MALI",
		    args->write_domain == DRM_MESON_GEM_DOMAIN_CPU ? "CPU" : "MALI"));

	/* If the buffer is not cacheable there is no need to do anything */
	if (!dma_get_attr(DMA_ATTR_NON_CONSISTENT, &cma_obj->dma_attrs)) {
		DBG_MSG(3, ("meson_ioctl_set_domain(): %02u Changing owner of uncached memory, cache flushing not needed.\n",
			    args->handle));
		goto out;
	}

	if (gem_obj->write_domain == args->write_domain) {
		DBG_MSG(4, ("meson_ioctl_set_domain(): %02u New_user equal previous, cache flushing not needed\n",
			    args->handle,));
		goto out;
	}
	if (
		/* Previous AND new is both different from CPU */
		(gem_obj->write_domain != DRM_MESON_GEM_DOMAIN_CPU) && (args->write_domain != DRM_MESON_GEM_DOMAIN_CPU)
	) {
		DBG_MSG(4, ("meson_ioctl_set_domain(): %02u Previous and new user are not CPU, cache flushing not needed\n",
			    args->handle,));
		goto out;
	}

	if ((gem_obj->write_domain != DRM_MESON_GEM_DOMAIN_CPU) && (args->write_domain == DRM_MESON_GEM_DOMAIN_CPU)) {
		cache_op = MESON_DRM_MSYNC_INVALIDATE;
		DBG_MSG(4, ("meson_ioctl_set_domain(): %02u Cache invalidate\n", args->handle));
	} else
		DBG_MSG(4, ("meson_ioctl_set_domain(): %02u Cache clean and invalidate\n", args->handle));

	mutex_lock(&session_data->mutex);
	meson_drm_ump_osk_msync(cma_obj, NULL, 0, gem_obj->size, cache_op, session_data);
	mutex_unlock(&session_data->mutex);

out:
	/* We only care about W accesses, which are actually RW */
	gem_obj->read_domains = gem_obj->write_domain = args->write_domain;

	DBG_MSG(4, ("meson_ioctl_set_domain(): %02u Finish\n", args->handle));
	return PTR_ERR_OR_ZERO(cma_obj);
}
