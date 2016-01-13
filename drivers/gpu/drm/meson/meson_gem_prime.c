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

static struct meson_drm_gem_scattered_object *meson_drm_gem_init(
		struct drm_device *dev,
		unsigned long size)
{
	struct meson_drm_gem_scattered_object *scattered_obj;
	struct drm_gem_object *obj;
	int ret;

	scattered_obj = kzalloc(sizeof(*scattered_obj), GFP_KERNEL);
	if (!scattered_obj)
		return ERR_PTR(-ENOMEM);

	scattered_obj->size = size;
	obj = &scattered_obj->base;

	ret = drm_gem_object_init(dev, obj, size);
	if (ret < 0) {
		DRM_ERROR("failed to initialize gem object\n");
		kfree(scattered_obj);
		return ERR_PTR(ret);
	}

	ret = drm_gem_create_mmap_offset(obj);
	if (ret < 0) {
		drm_gem_object_release(obj);
		kfree(scattered_obj);
		return ERR_PTR(ret);
	}

	DRM_DEBUG_KMS("created file object = 0x%x\n", (unsigned int)obj->filp);

	return scattered_obj;
}

static void meson_drm_gem_free(struct meson_drm_gem_scattered_object *scattered_obj,
			       int page_count)
{
	while (page_count--)
		if (scattered_obj->pages[page_count])
			__free_pages(scattered_obj->pages[page_count], 0);
	drm_free_large(scattered_obj->pages);
}

static int meson_drm_gem_alloc_buf(struct meson_drm_gem_scattered_object *scattered_obj)
{
	gfp_t gfp = GFP_KERNEL | GFP_DMA | __GFP_NOWARN | __GFP_NORETRY;
	int count;
	int ret = -ENOMEM;
	int i = 0;

	count = scattered_obj->size >> PAGE_SHIFT;

	scattered_obj->pages = drm_calloc_large(count, sizeof(struct page *));
	if (!scattered_obj->pages) {
		DRM_ERROR("failed to allocate pages.\n");
               return -ENOMEM;
	}

	while (count) {
		int j, order = __fls(count);

		scattered_obj->pages[i] = alloc_pages(gfp, order);
		while (!scattered_obj->pages[i] && order)
			scattered_obj->pages[i] = alloc_pages(gfp, --order);

		if (!scattered_obj->pages[i])
			goto error;

		if (order) {
			split_page(scattered_obj->pages[i], order);
			j = 1 << order;
			while (--j)
				scattered_obj->pages[i + j] = scattered_obj->pages[i] + j;
		}

		i += 1 << order;
		count -= 1 << order;
	}

	scattered_obj->nr_pages = i;

	/* sgt for backend lowlevel buffer */
	scattered_obj->sgt = drm_prime_pages_to_sg(scattered_obj->pages, scattered_obj->nr_pages);
	if (IS_ERR(scattered_obj->sgt)) {
		ret = PTR_ERR(scattered_obj->sgt);
		goto error;
	}

	return 0;

error:
	meson_drm_gem_free(scattered_obj, i);
	return ret;
}

static struct meson_drm_gem_scattered_object *meson_drm_gem_create(
		struct drm_device *dev,
		unsigned int flags,
		unsigned long size)
{
	struct meson_drm_gem_scattered_object *scattered_obj;
	int ret;

	size = roundup(size, PAGE_SIZE);

	scattered_obj = meson_drm_gem_init(dev, size);
	if (IS_ERR(scattered_obj))
		return scattered_obj;

	scattered_obj->flags = flags;

	ret = meson_drm_gem_alloc_buf(scattered_obj);
	if (ret < 0) {
		drm_gem_free_mmap_offset(&scattered_obj->base);
		drm_gem_object_release(&scattered_obj->base);
		kfree(scattered_obj);
		return ERR_PTR(ret);
	}

	return scattered_obj;
}

void meson_drm_gem_destroy(struct meson_drm_gem_scattered_object *scattered_obj)
{
	struct drm_gem_object *obj = &scattered_obj->base;

	drm_gem_free_mmap_offset(obj);

	DRM_DEBUG_KMS("handle count = %d\n", obj->handle_count);

	// TODO: we do not deal yet with dma_buf imported as a (scattered) gem object
	if (!obj->import_attach) {
		meson_drm_gem_free(scattered_obj, scattered_obj->nr_pages);
		sg_free_table(scattered_obj->sgt);
		kfree(scattered_obj->sgt);
	}

	drm_gem_object_release(obj);

	kfree(scattered_obj);
}

void meson_drm_gem_scattered_free_object(struct drm_gem_object *gem_obj)
{
	meson_drm_gem_destroy(to_meson_drm_gem_scattered_obj(gem_obj));
}

struct sg_table *meson_drm_gem_scattered_prime_get_sg_table(struct drm_gem_object *gem_obj)
{
	struct meson_drm_gem_scattered_object *scattered_obj;

	scattered_obj = to_meson_drm_gem_scattered_obj(gem_obj);

	/* the returned sgt will be deallocated by the buffer-user when detaching */
	return drm_prime_pages_to_sg(scattered_obj->pages, scattered_obj->nr_pages);
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

struct meson_drm_gem_scattered_object *meson_drm_gem_scattered_create_with_handle(
		struct drm_device *dev, void *data,
		struct drm_file *file_priv,
		struct dma_attrs *dma_attrs)
{
	struct drm_meson_gem_create_with_ump *args = data;
	struct meson_drm_gem_scattered_object *scattered_obj;
	int ret;

	scattered_obj = meson_drm_gem_create(dev, args->flags, args->size);
	if (IS_ERR(scattered_obj))
		return scattered_obj;

	scattered_obj->dma_attrs = *dma_attrs;
	scattered_obj->base.is_scattered = true;

	ret = meson_drm_gem_handle_create(&scattered_obj->base, file_priv,
					  &args->handle);
	if (ret) {
		meson_drm_gem_destroy(scattered_obj);
		return ERR_PTR(ret);
	}

	return scattered_obj;
}

int meson_drm_gem_scattered_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct drm_gem_object *obj = vma->vm_private_data;
	struct meson_drm_gem_scattered_object *meson_gem = to_meson_drm_gem_scattered_obj(obj);
	unsigned long pfn;
	pgoff_t page_offset;
	int ret;

	page_offset = ((unsigned long)vmf->virtual_address - vma->vm_start) >> PAGE_SHIFT;
	if (page_offset >= (meson_gem->size >> PAGE_SHIFT)) {
		DRM_ERROR("invalid page offset\n");
		ret = -EINVAL;
		goto out;
	}

	pfn = page_to_pfn(meson_gem->pages[page_offset]);
	ret = vm_insert_pfn(vma, (unsigned long)vmf->virtual_address, pfn);

out:
	switch (ret) {
	case 0:
	case -ERESTARTSYS:
	case -EINTR:
		return VM_FAULT_NOPAGE;
	case -ENOMEM:
		return VM_FAULT_OOM;
	default:
		return VM_FAULT_SIGBUS;
	}
}

