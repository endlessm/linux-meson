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
#include "meson_gem.h"

#define DBG_MSG(level,args) do {} while(0) /* FIXME: debug properly */

static void cache_control_op(enum drm_meson_msync_op op, u32 start_p, u32 end_p)
{
	switch (op) {
	case DRM_MESON_MSYNC_CLEAN:
		outer_clean_range(start_p, end_p);
		break;
	case DRM_MESON_MSYNC_CLEAN_AND_INVALIDATE:
		outer_flush_range(start_p, end_p);
		break;
	case DRM_MESON_MSYNC_INVALIDATE:
		outer_inv_range(start_p, end_p);
		break;
	default:
		break;
	}
}

static void level1_cache_flush_all(void)
{
	DBG_MSG(4, ("meson_drm_ump_osk_msync(): Flushing the whole L1 cache\n"));
	__cpuc_flush_kern_all();
}

/* This is a copy of _ump_osk_msync from drivers/amlogic/gpu/ump/linux/ump_osk_low_level_mem.c
 * with adapted parameters */
static void meson_drm_ump_osk_msync(struct drm_gem_object *gem_obj, void *virt, u32 offset, size_t size, enum drm_meson_msync_op op, struct meson_drm_session_data *session_data)
{
	struct meson_drm_gem_object *meson_gem_obj;
	u32 start_p, end_p;
	int i;
	struct scatterlist *sgl;


	/* Flush L1 using virtual address, the entire range in one go.
	 * Only flush if user space process has a valid write mapping on given address. */
	if ((gem_obj) && (virt != NULL) && (access_ok(VERIFY_WRITE, virt, size))) {
		__cpuc_flush_dcache_area(virt, size);
		DBG_MSG(3, ("meson_drm_ump_osk_msync(): Flushing CPU L1 Cache. CPU address: %x, size: %x\n", virt, size));
	} else {
		if (session_data) {
			if (op == DRM_MESON_MSYNC_FLUSH_L1) {
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

	if (!gem_obj)
		return;

	DBG_MSG(3, ("meson_drm_ump_osk_msync(): Flushing CPU L2 Cache\n"));

	meson_gem_obj = to_meson_drm_gem_obj(gem_obj);

	for_each_sg(meson_gem_obj->sgt->sgl, sgl, meson_gem_obj->sgt->nents, i) {
		start_p = sg_phys(sgl);
		end_p = start_p + sgl->length;

		cache_control_op(op, start_p, end_p);
	}

	return;
}

/* this code was heavily inspired by _ump_ukk_msync() in
 * drivers/amlogic/gpu/ump/common/ump_kernel_api.c */
int meson_ioctl_msync(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_gem_object *gem_obj;
	struct meson_drm_gem_object *meson_gem_obj;
	struct drm_meson_msync *args = data;
	struct meson_drm_session_data *session_data = file->driver_priv;
	int ret = 0;

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

	meson_gem_obj = to_meson_drm_gem_obj(gem_obj);

	args->is_cached = dma_get_attr(DMA_ATTR_NON_CONSISTENT, &meson_gem_obj->dma_attrs);

	DBG_MSG(3, ("meson_ioctl_msync(): %02u cache_enabled %d\n op %d address 0x%08x mapping 0x%08x\n",
		    args->handle, args->is_cached, args->op, args->address, args->mapping));

	/* Nothing to do in these cases */
	if ((DRM_MESON_MSYNC_READOUT_CACHE_ENABLED == args->op) || (!args->is_cached))
		goto out;

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
		ret = -EINVAL;
		goto out;
	}

	/* No need to lock here since session_data=NULL */
	meson_drm_ump_osk_msync(gem_obj, virtual, offset, gem_obj->size, args->op, NULL);

 out:
	drm_gem_object_unreference_unlocked(gem_obj);
	return ret;
}

/* this code was heavily inspired by _ump_ukk_switch_hw_usage() in
 * drivers/amlogic/gpu/ump/common/ump_kernel_api.c */
int meson_ioctl_set_domain(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_gem_object *gem_obj;
	struct meson_drm_gem_object *meson_gem_obj;
	struct drm_meson_gem_set_domain *args = data;
	struct meson_drm_session_data *session_data = file->driver_priv;
	enum drm_meson_msync_op cache_op = DRM_MESON_MSYNC_CLEAN_AND_INVALIDATE;
	int ret = 0;

	if (!args || !session_data)
		return -EINVAL;

	gem_obj = drm_gem_object_lookup(dev, file, args->handle);
	if (NULL == gem_obj) {
		DBG_MSG(1, ("meson_ioctl_set_domain(): %02u Failed to look up mapping\n", args->handle));
		return -EFAULT;
	}

	meson_gem_obj = to_meson_drm_gem_obj(gem_obj);

	DBG_MSG(3, ("meson_ioctl_set_domain(): %02u %s -> %s\n",
		    args->handle,
		    gem_obj->write_domain == DRM_MESON_GEM_DOMAIN_CPU ? "CPU" : "MALI",
		    args->write_domain == DRM_MESON_GEM_DOMAIN_CPU ? "CPU" : "MALI"));

	/* If the buffer is not cacheable there is no need to do anything */
	if (!dma_get_attr(DMA_ATTR_NON_CONSISTENT, &meson_gem_obj->dma_attrs)) {
		DBG_MSG(3, ("meson_ioctl_set_domain(): %02u Changing owner of uncached memory, cache flushing not needed.\n",
			    args->handle));
		ret = -EINVAL;
		goto out;
	}

	if (gem_obj->write_domain == args->write_domain) {
		DBG_MSG(4, ("meson_ioctl_set_domain(): %02u New_user equal previous, cache flushing not needed\n",
			    args->handle,));
		goto out;
	}

	if ((gem_obj->write_domain != DRM_MESON_GEM_DOMAIN_CPU) && (args->write_domain == DRM_MESON_GEM_DOMAIN_CPU)) {
		cache_op = DRM_MESON_MSYNC_INVALIDATE;
		DBG_MSG(4, ("meson_ioctl_set_domain(): %02u Cache invalidate\n", args->handle));
	} else
		DBG_MSG(4, ("meson_ioctl_set_domain(): %02u Cache clean and invalidate\n", args->handle));

	mutex_lock(&session_data->mutex);
	meson_drm_ump_osk_msync(gem_obj, NULL, 0, gem_obj->size, cache_op, session_data);
	mutex_unlock(&session_data->mutex);

out:
	/* We only care about W accesses, which are actually RW */
	gem_obj->read_domains = gem_obj->write_domain = args->write_domain;

	drm_gem_object_unreference_unlocked(gem_obj);

	DBG_MSG(4, ("meson_ioctl_set_domain(): %02u Finish\n", args->handle));

	return ret;
}

/* this code was heavily inspired by _ump_ukk_cache_operations_control() in
 * drivers/amlogic/gpu/ump/common/ump_kernel_api.c */
int meson_ioctl_cache_operations_control(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_meson_cache_operations_control *args = data;
	struct meson_drm_session_data *session_data = file->driver_priv;

	if (!args || !session_data || args->op >= DRM_MESON_CACHE_OP_COUNT)
		return -EINVAL;

	mutex_lock(&session_data->mutex);

	switch (args->op) {
		case DRM_MESON_CACHE_OP_START:
			session_data->cache_operations_ongoing++;
			DBG_MSG(4, ("meson_ioctl_cache_operations_control(): Cache ops start, %d cache ops ongoing\n", session_data->cache_operations_ongoing));
			break;
		case DRM_MESON_CACHE_OP_FINISH:
			session_data->cache_operations_ongoing--;
			DBG_MSG(4, ("meson_ioctl_cache_operations_control(): Cache ops finish, %d cache ops ongoing\n", session_data->cache_operations_ongoing));
#if 0
			if (session_data->has_pending_level1_cache_flush) {
				/* This function will set has_pending_level1_cache_flush=0 */
				_ump_osk_msync(NULL, NULL, 0, 0, _UMP_UK_MSYNC_FLUSH_L1, session_data);
			}
#endif

			/* to be on the safe side: always flush l1 cache when cache operations are done */
			meson_drm_ump_osk_msync(NULL, NULL, 0, 0, DRM_MESON_MSYNC_FLUSH_L1, session_data);
			DBG_MSG(4, ("meson_ioctl_cache_operations_control(): Cache ops finish end\n"));
			break;
		case DRM_MESON_CACHE_OP_COUNT:
			/* Avoid warning (-Wswitch) */
			break;
	}

	mutex_unlock(&session_data->mutex);

	return 0;
}
