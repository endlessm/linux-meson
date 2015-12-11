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

static struct meson_drm_gem_object *meson_drm_gem_init(struct drm_device *drm,
						       unsigned int size)
{
	struct meson_drm_gem_object *meson_gem_obj;
	struct drm_gem_object *gem_obj;
	int ret;

	meson_gem_obj = kzalloc(sizeof(*meson_gem_obj), GFP_KERNEL);
	if (!meson_gem_obj)
		return NULL;

	gem_obj = &meson_gem_obj->base;

	ret = drm_gem_object_init(drm, gem_obj, size);
	if (ret < 0) {
		kfree(meson_gem_obj);
		return NULL;
	}

	return meson_gem_obj;
}

static struct meson_drm_gem_buf *meson_drm_gem_init_buf(struct drm_device *drm,
							unsigned int size)
{
	struct meson_drm_gem_buf *buf;

	buf = kzalloc(sizeof(struct meson_drm_gem_buf), GFP_KERNEL);
	if (!buf)
		return NULL;

	return buf;
}

static void meson_drm_gem_free_buf(struct meson_drm_gem_buf *buf, int idx)
{
	while (idx--)
		if (buf->pages[idx])
			__free_pages(buf->pages[idx], 0);
	kfree(buf->pages);
}

static int meson_drm_gem_alloc_buf(struct drm_device *drm,
				   struct meson_drm_gem_buf *buf,
				   unsigned int size)
{
	int count = size >> PAGE_SHIFT;
	gfp_t gfp = GFP_KERNEL | GFP_DMA | __GFP_NOWARN | __GFP_NORETRY;
	int ret = -ENOMEM;
	int i = 0;

	buf->pages = kzalloc(count * sizeof(struct page *), GFP_KERNEL);
	if (!buf->pages)
		return -ENOMEM;

	while (count) {
		int j, order = __fls(count);

		buf->pages[i] = alloc_pages(gfp, order);
		while (!buf->pages[i] && order)
			buf->pages[i] = alloc_pages(gfp, --order);

		if (!buf->pages[i])
			goto error;

		if (order) {
			split_page(buf->pages[i], order);
			j = 1 << order;
			while (--j)
				buf->pages[i + j] = buf->pages[i] + j;
		}

		i += 1 << order;
		count -= 1 << order;
	}

	buf->cnt_pages = i;

	buf->sgt = drm_prime_pages_to_sg(buf->pages, i);
	if (IS_ERR(buf->sgt)) {
		ret = PTR_ERR(buf->sgt);
		goto error;
	}

	return 0;

error:
	meson_drm_gem_free_buf(buf, i);
	return ret;
}


static struct meson_drm_gem_object *meson_drm_gem_create(struct drm_device *drm,
							 unsigned int size,
							 struct dma_attrs *dma_attrs)
{
	struct meson_drm_gem_buf *buf;
	struct meson_drm_gem_object *meson_gem_obj;
	int ret;

	size = round_up(size, PAGE_SIZE);

	buf = meson_drm_gem_init_buf(drm, size);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	meson_gem_obj = meson_drm_gem_init(drm, size);
	if (!meson_gem_obj) {
		ret = -ENOMEM;
		goto err_free_buf;
	}

	meson_gem_obj->buffer = buf;

	ret = meson_drm_gem_alloc_buf(drm, buf, size);
	if (ret < 0)
		goto err_free_gem;

	return meson_gem_obj;

err_free_gem:
	drm_gem_object_release(&meson_gem_obj->base);
	kfree(meson_gem_obj);
err_free_buf:
	kfree(buf);
	return ERR_PTR(ret);
}

void meson_drm_gem_destroy(struct meson_drm_gem_object *meson_gem_obj)
{
	struct drm_gem_object *gem_obj;
	struct meson_drm_gem_buf *buf;

	gem_obj = &meson_gem_obj->base;
	buf = meson_gem_obj->buffer;

	if (gem_obj->import_attach)
		goto next;

	sg_free_table(buf->sgt);
	kfree(buf->sgt);

next:
	meson_drm_gem_free_buf(buf, buf->cnt_pages);
	kfree(buf);

	drm_gem_free_mmap_offset(gem_obj);
	drm_gem_object_release(gem_obj);

	kfree(meson_gem_obj);
}

void meson_drm_gem_free_object(struct drm_gem_object *gem_obj)
{
	struct meson_drm_gem_object *meson_gem_obj;
	struct meson_drm_gem_buf *buf;

	meson_gem_obj = to_meson_drm_gem_object(gem_obj);
	buf = meson_gem_obj->buffer;

	if (gem_obj->import_attach)
		drm_prime_gem_destroy(gem_obj, buf->sgt);

	meson_drm_gem_destroy(meson_gem_obj);
}

struct sg_table *meson_drm_gem_get_sg_table(struct drm_gem_object *gem_obj)
{
	struct meson_drm_gem_object *meson_gem_obj;
	struct meson_drm_gem_buf *buf;

	meson_gem_obj = to_meson_drm_gem_object(gem_obj);
	buf = meson_gem_obj->buffer;

	return buf->sgt;
}

struct meson_drm_gem_object *meson_drm_gem_create_with_handle(
		struct drm_file *file_priv,
		struct drm_device *drm, unsigned int size,
		unsigned int *handle, struct dma_attrs *dma_attrs)
{
	struct meson_drm_gem_object *meson_gem_obj;
	struct drm_gem_object *gem_obj;
	int ret;

	meson_gem_obj = meson_drm_gem_create(drm, size, dma_attrs);
	if (IS_ERR(meson_gem_obj))
		return meson_gem_obj;

	gem_obj = &meson_gem_obj->base;

	ret = drm_gem_handle_create(file_priv, gem_obj, handle);
	if (ret)
		goto err_handle;

	drm_gem_object_unreference_unlocked(gem_obj);

	return meson_gem_obj;

err_handle:
	meson_drm_gem_destroy(meson_gem_obj);
	return ERR_PTR(ret);
}

