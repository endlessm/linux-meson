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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_vma_manager.h>

#include <drm/meson_drm.h>

#include "meson_priv.h"
#include "meson_gem.h"

static struct meson_drm_gem_object *meson_drm_gem_init(struct drm_device *dev,
						       unsigned long size)
{
	struct meson_drm_gem_object *meson_gem_obj;
	struct drm_gem_object *gem_obj;
	int ret;

	meson_gem_obj = kzalloc(sizeof(*meson_gem_obj), GFP_KERNEL);
	if (!meson_gem_obj)
		return ERR_PTR(-ENOMEM);

	gem_obj = &meson_gem_obj->base;

	ret = drm_gem_object_init(dev, gem_obj, size);
	if (ret) {
		DRM_ERROR("failed to initialize gem object\n");
		goto error;
	}

	ret = drm_gem_create_mmap_offset(gem_obj);
	if (ret < 0) {
		drm_gem_object_release(gem_obj);
		goto error;
	}

	DRM_DEBUG_KMS("created file object = 0x%x\n", (unsigned int)gem_obj->filp);

	return meson_gem_obj;

error:
	kfree(meson_gem_obj);
	return ERR_PTR(ret);
}

static void meson_drm_gem_scattered_free(struct meson_drm_gem_object *meson_gem_obj,
					 int page_count)
{
	while (page_count--)
		if (meson_gem_obj->pages[page_count])
			__free_pages(meson_gem_obj->pages[page_count], 0);
	drm_free_large(meson_gem_obj->pages);
}

static int meson_drm_gem_scattered_alloc_buf(struct meson_drm_gem_object *meson_gem_obj)
{
	gfp_t gfp = GFP_KERNEL | GFP_DMA | __GFP_NOWARN | __GFP_NORETRY;
	int count;
	int ret = -ENOMEM;
	int i = 0;

	count = meson_gem_obj->size >> PAGE_SHIFT;

	meson_gem_obj->pages = drm_calloc_large(count, sizeof(struct page *));
	if (!meson_gem_obj->pages) {
		DRM_ERROR("failed to allocate pages.\n");
		return -ENOMEM;
	}

	while (count) {
		int j, order = __fls(count);

		meson_gem_obj->pages[i] = alloc_pages(gfp, order);
		while (!meson_gem_obj->pages[i] && order)
			meson_gem_obj->pages[i] = alloc_pages(gfp, --order);

		if (!meson_gem_obj->pages[i])
			goto error;

		if (order) {
			split_page(meson_gem_obj->pages[i], order);
			j = 1 << order;
			while (--j)
				meson_gem_obj->pages[i + j] = meson_gem_obj->pages[i] + j;
		}

		i += 1 << order;
		count -= 1 << order;
	}

	meson_gem_obj->nr_pages = i;

	meson_gem_obj->sgt = drm_prime_pages_to_sg(meson_gem_obj->pages, meson_gem_obj->nr_pages);
	if (IS_ERR(meson_gem_obj->sgt)) {
		ret = PTR_ERR(meson_gem_obj->sgt);
		goto error;
	}

	return 0;

error:
	meson_drm_gem_scattered_free(meson_gem_obj, i);
	return ret;
}

static int meson_drm_gem_cma_alloc_buf(struct meson_drm_gem_object *meson_gem_obj,
			 	       struct drm_device *drm)
{
	unsigned int nr_pages;
	int ret = 0;

	nr_pages = meson_gem_obj->size >> PAGE_SHIFT;
	meson_gem_obj->nr_pages = nr_pages;

	meson_gem_obj->pages = drm_calloc_large(nr_pages, sizeof(struct page *));
	if (!meson_gem_obj->pages) {
		DRM_ERROR("failed to allocate pages.\n");
		return -ENOMEM;
	}

	meson_gem_obj->sgt = kmalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!meson_gem_obj->sgt) {
		DRM_ERROR("failed to allocate sgt.\n");
		ret = -ENOMEM;
		goto err_free;
	}

	meson_gem_obj->vaddr = dma_alloc_attrs(drm->dev, meson_gem_obj->size,
					       &meson_gem_obj->paddr,
					       GFP_KERNEL | __GFP_NOWARN,
					       &meson_gem_obj->dma_attrs);
	if (!meson_gem_obj->vaddr) {
		DRM_ERROR("failed to allocate buffer with size %ld\n", meson_gem_obj->size);
		ret = -ENOMEM;
		goto err_sgt_kfree;
	}


	ret = dma_get_sgtable_attrs(drm->dev, meson_gem_obj->sgt, meson_gem_obj->vaddr,
				    meson_gem_obj->paddr, meson_gem_obj->size,
				    &meson_gem_obj->dma_attrs);
	if (ret < 0) {
		DRM_ERROR("failed to get sgtable.\n");
		goto err_dma_free;
	}

	if (drm_prime_sg_to_page_addr_arrays(meson_gem_obj->sgt, meson_gem_obj->pages, NULL,
				nr_pages)) {
		DRM_ERROR("invalid sgtable.\n");
		ret = -EINVAL;
		goto err_sgt_free;
	}

	return 0;

err_sgt_free:
	sg_free_table(meson_gem_obj->sgt);
err_dma_free:
	dma_free_attrs(drm->dev, meson_gem_obj->base.size,
		       meson_gem_obj->vaddr, meson_gem_obj->paddr,
		       &meson_gem_obj->dma_attrs);
err_sgt_kfree:
	kfree(meson_gem_obj->sgt);
err_free:
	drm_free_large(meson_gem_obj->pages);
	return ret;
}

static int meson_drm_gem_alloc_buf(struct meson_drm_gem_object *meson_gem_obj,
				   struct drm_device *drm)
{
	if (meson_gem_obj->is_scattered)
		return meson_drm_gem_scattered_alloc_buf(meson_gem_obj);

	return meson_drm_gem_cma_alloc_buf(meson_gem_obj, drm);
}

static struct meson_drm_gem_object *meson_drm_gem_create(struct drm_device *dev,
							 unsigned long size,
							 struct dma_attrs *dma_attrs)
{
	struct meson_drm_gem_object *meson_gem_obj;
	int ret;

	size = round_up(size, PAGE_SIZE);

	meson_gem_obj = meson_drm_gem_init(dev, size);
	if (IS_ERR(meson_gem_obj))
		return meson_gem_obj;

	meson_gem_obj->size = size;
	meson_gem_obj->dma_attrs = *dma_attrs;
	meson_gem_obj->is_scattered = !dma_get_attr(DMA_ATTR_WRITE_COMBINE, dma_attrs);

	ret = meson_drm_gem_alloc_buf(meson_gem_obj, dev);
	if (ret < 0) {
		drm_gem_object_release(&meson_gem_obj->base);
		kfree(meson_gem_obj);
		return ERR_PTR(ret);
	}

	return meson_gem_obj;
}

static void meson_drm_gem_destroy(struct meson_drm_gem_object *meson_gem_obj)
{
	struct drm_gem_object *gem_obj = &meson_gem_obj->base;

	DRM_DEBUG_KMS("handle count = %d\n", gem_obj->handle_count);

	if (meson_gem_obj->vaddr)
		dma_free_attrs(gem_obj->dev->dev, meson_gem_obj->base.size,
			       meson_gem_obj->vaddr, meson_gem_obj->paddr,
			       &meson_gem_obj->dma_attrs);

	/*
	 * TODO: we do not deal yet with dma_buf imported as a (scattered) gem object
	 */
	else if (!gem_obj->import_attach)
		meson_drm_gem_scattered_free(meson_gem_obj, meson_gem_obj->nr_pages);

	sg_free_table(meson_gem_obj->sgt);
	kfree(meson_gem_obj->sgt);

	drm_gem_object_release(gem_obj);
	kfree(meson_gem_obj);
}

static int meson_drm_gem_handle_create(struct drm_gem_object *obj,
				       struct drm_file *file_priv,
				       unsigned int *handle)
{
	int ret;

	ret = drm_gem_handle_create(file_priv, obj, handle);
	if (ret)
		return ret;

	DRM_DEBUG_KMS("gem handle = 0x%x\n", *handle);

	drm_gem_object_unreference_unlocked(obj);

	return 0;
}

struct meson_drm_gem_object *meson_drm_gem_create_obj(struct drm_device *dev,
						      unsigned int size)
{
	DEFINE_DMA_ATTRS(dma_attrs);

	dma_set_attr(DMA_ATTR_WRITE_COMBINE, &dma_attrs);
	size = PAGE_ALIGN(size);

	return meson_drm_gem_create(dev, size, &dma_attrs);
}

struct meson_drm_gem_object *meson_drm_gem_create_with_handle(struct drm_device *dev,
							      unsigned int size,
							      unsigned int *handle,
							      struct drm_file *file_priv,
							      struct dma_attrs *dma_attrs)
{
	struct meson_drm_gem_object *meson_gem_obj;
	int ret;

	/* UMP requires a page-aligned size for its buffers. */
	size = PAGE_ALIGN (size);

	meson_gem_obj = meson_drm_gem_create(dev, size, dma_attrs);
	if (IS_ERR(meson_gem_obj))
		return meson_gem_obj;

	ret = meson_drm_gem_handle_create(&meson_gem_obj->base, file_priv, handle);
	if (ret) {
		meson_drm_gem_destroy(meson_gem_obj);
		return ERR_PTR(ret);
	}

	return meson_gem_obj;
}

void meson_drm_gem_free_object(struct drm_gem_object *gem_obj)
{
	meson_drm_gem_destroy(to_meson_drm_gem_obj(gem_obj));
}

struct sg_table *meson_drm_gem_get_sg_table(struct drm_gem_object *gem_obj)
{
	struct meson_drm_gem_object *meson_gem_obj;

	meson_gem_obj = to_meson_drm_gem_obj(gem_obj);

	return drm_prime_pages_to_sg(meson_gem_obj->pages, meson_gem_obj->nr_pages);
}

static int meson_drm_gem_scattered_mmap_obj(struct meson_drm_gem_object *meson_gem_obj,
					    struct vm_area_struct *vma)
{

	unsigned long addr = vma->vm_start;
	int i, ret;

	for (i = 0; i < meson_gem_obj->nr_pages; i++) {
		struct page *page = meson_gem_obj->pages[i];

		ret = vm_insert_pfn(vma, addr, page_to_pfn(page));
		if (ret)
			return -EFAULT;

		addr += PAGE_SIZE;
	}

	return 0;
}

static int meson_drm_gem_cma_mmap_obj(struct meson_drm_gem_object *meson_gem_obj,
				      struct vm_area_struct *vma)
{
	int ret;

	/*
	 * Clear the VM_PFNMAP flag that was set by drm_gem_mmap(), and set the
	 * vm_pgoff (used as a fake buffer offset by DRM) to 0 as we want to map
	 * the whole buffer.
	 */
	vma->vm_flags &= ~VM_PFNMAP;
	vma->vm_pgoff = 0;
	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);

	ret = dma_mmap_attrs(meson_gem_obj->base.dev->dev, vma,
			     meson_gem_obj->vaddr, meson_gem_obj->paddr,
			     vma->vm_end - vma->vm_start,
			     &meson_gem_obj->dma_attrs);
	if (ret)
		drm_gem_vm_close(vma);

	return ret;
}

int meson_drm_gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct drm_gem_object *gem_obj;
	struct meson_drm_gem_object *meson_gem_obj;
	int ret;

	ret = drm_gem_mmap(filp, vma);
	if (ret)
		return ret;

	gem_obj = vma->vm_private_data;
	meson_gem_obj = to_meson_drm_gem_obj(gem_obj);

	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);

	if (!meson_gem_obj->is_scattered)
		return meson_drm_gem_cma_mmap_obj(meson_gem_obj, vma);

	return meson_drm_gem_scattered_mmap_obj(meson_gem_obj, vma);
}

int meson_drm_gem_dumb_create(struct drm_file *file_priv,
			      struct drm_device *dev,
			      struct drm_mode_create_dumb *args)
{
	struct meson_drm_gem_object *meson_gem_obj;
	int min_pitch = DIV_ROUND_UP(args->width * args->bpp, 8);
	DEFINE_DMA_ATTRS(dma_attrs);

	if (args->pitch < min_pitch)
		args->pitch = min_pitch;

	if (args->size < args->pitch * args->height)
		args->size = args->pitch * args->height;

	dma_set_attr(DMA_ATTR_WRITE_COMBINE, &dma_attrs);

	meson_gem_obj = meson_drm_gem_create_with_handle(dev, args->size, &args->handle, file_priv, &dma_attrs);

	return PTR_ERR_OR_ZERO(meson_gem_obj);
}

int meson_drm_gem_dumb_map_offset(struct drm_file *file_priv,
				  struct drm_device *drm,
				  uint32_t handle, uint64_t *offset)
{
	struct drm_gem_object *gem_obj;

	mutex_lock(&drm->struct_mutex);

	gem_obj = drm_gem_object_lookup(drm, file_priv, handle);
	if (!gem_obj) {
		DRM_ERROR("failed to lookup gem object\n");
		mutex_unlock(&drm->struct_mutex);
		return -EINVAL;
	}

	*offset = drm_vma_node_offset_addr(&gem_obj->vma_node);

	drm_gem_object_unreference(gem_obj);

	mutex_unlock(&drm->struct_mutex);

	return 0;
}

