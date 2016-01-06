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
#include "meson_gem_prime.h"

const struct vm_operations_struct meson_drm_gem_vm_ops = {
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static struct meson_drm_gem_object *meson_drm_gem_init(struct drm_device *dev,
						       unsigned long size)
{
	struct meson_drm_gem_object *meson_gem;
	struct drm_gem_object *obj;
	int ret;

	meson_gem = kzalloc(sizeof(*meson_gem), GFP_KERNEL);
	if (!meson_gem)
		return ERR_PTR(-ENOMEM);

	meson_gem->size = size;
	obj = &meson_gem->base;

	ret = drm_gem_object_init(dev, obj, size);
	if (ret < 0) {
		DRM_ERROR("failed to initialize gem object\n");
		kfree(meson_gem);
		return ERR_PTR(ret);
	}

	ret = drm_gem_create_mmap_offset(obj);
	if (ret < 0) {
		drm_gem_object_release(obj);
		kfree(meson_gem);
		return ERR_PTR(ret);
	}

	DRM_DEBUG_KMS("created file object = 0x%x\n", (unsigned int)obj->filp);

	return meson_gem;
}

// TODO: check
static void meson_drm_gem_free(struct meson_drm_gem_object *meson_gem, int idx)
{
	while (idx--)
		if (meson_gem->pages[idx])
			__free_pages(meson_gem->pages[idx], 0);
	drm_free_large(meson_gem->pages);
}

static int meson_drm_gem_alloc_buf(struct meson_drm_gem_object *meson_gem)
{
	int count = meson_gem->size >> PAGE_SHIFT;
	gfp_t gfp = GFP_KERNEL | GFP_DMA | __GFP_NOWARN | __GFP_NORETRY;
	int ret = -ENOMEM;
	int i = 0;

	meson_gem->pages = drm_calloc_large(count, sizeof(struct page *));
	if (!meson_gem->pages) {
		DRM_ERROR("failed to allocate pages.\n");
               return -ENOMEM;
	}

	while (count) {
		int j, order = __fls(count);

		meson_gem->pages[i] = alloc_pages(gfp, order);
		while (!meson_gem->pages[i] && order)
			meson_gem->pages[i] = alloc_pages(gfp, --order);

		if (!meson_gem->pages[i])
			goto error;

		if (order) {
			split_page(meson_gem->pages[i], order);
			j = 1 << order;
			while (--j)
				meson_gem->pages[i + j] = meson_gem->pages[i] + j;
		}

		i += 1 << order;
		count -= 1 << order;
	}

	meson_gem->nr_pages = i;

	return 0;

error:
	meson_drm_gem_free(meson_gem, i);
	return ret;
}

static struct meson_drm_gem_object *meson_drm_gem_create(struct drm_device *dev,
							 unsigned int flags,
							 unsigned long size)
{
	struct meson_drm_gem_object *meson_gem;
	int ret;

	size = roundup(size, PAGE_SIZE);

	meson_gem = meson_drm_gem_init(dev, size);
	if (IS_ERR(meson_gem))
		return meson_gem;

	meson_gem->flags = flags;

	ret = meson_drm_gem_alloc_buf(meson_gem);
	if (ret < 0) {
		drm_gem_object_release(&meson_gem->base);
		kfree(meson_gem);
		return ERR_PTR(ret);
	}

	return meson_gem;
}

void meson_drm_gem_destroy(struct meson_drm_gem_object *meson_gem)
{
	struct drm_gem_object *obj = &meson_gem->base;

	DRM_DEBUG_KMS("handle count = %d\n", obj->handle_count);

	if (obj->import_attach)
		// TODO: import_attach not yet supported since we need to implement .gem_prime_import_sg_table
		//drm_prime_gem_destroy(obj, meson_gem->sgt);
		;
	else
		// TODO: check
		meson_drm_gem_free(meson_gem, meson_gem->nr_pages);

	drm_gem_object_release(obj);

	kfree(meson_gem);
}

void meson_drm_gem_free_object(struct drm_gem_object *gem_obj)
{
	meson_drm_gem_destroy(to_meson_drm_gem_obj(gem_obj));
}

struct sg_table *meson_drm_gem_get_sg_table(struct drm_gem_object *gem_obj)
{
	struct meson_drm_gem_object *meson_gem = to_meson_drm_gem_obj(gem_obj);

	return drm_prime_pages_to_sg(meson_gem->pages, meson_gem->nr_pages);
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

struct meson_drm_gem_object *meson_drm_gem_create_with_handle(
		struct drm_device *dev, void *data,
		struct drm_file *file_priv,
		struct dma_attrs *dma_attrs)
{
	struct drm_meson_gem_create_with_ump *args = data;
	struct meson_drm_gem_object *meson_gem;
	int ret;

	meson_gem = meson_drm_gem_create(dev, args->flags, args->size);
	if (IS_ERR(meson_gem))
		return meson_gem;

	meson_gem->dma_attrs = *dma_attrs;

	ret = meson_drm_gem_handle_create(&meson_gem->base, file_priv,
					   &args->handle);
	if (ret) {
		meson_drm_gem_destroy(meson_gem);
		return ERR_PTR(ret);
	}

	return meson_gem;
}

