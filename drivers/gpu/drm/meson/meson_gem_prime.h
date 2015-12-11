#ifndef __MESON_GEM_PRIME_H__
#define __MESON_GEM_PRIME_H__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <drm/drmP.h>

struct meson_drm_gem_object {
	struct drm_gem_object base;
	struct dma_attrs dma_attrs;
	struct page **pages;

	unsigned long size;
	unsigned int flags;
	unsigned int nr_pages;

	dma_addr_t paddr;
	void *vaddr;

	bool is_scattered;
};

static inline struct meson_drm_gem_object *
to_meson_drm_gem_obj(struct drm_gem_object *gem_obj)
{
	return container_of(gem_obj, struct meson_drm_gem_object, base);
}

struct meson_drm_gem_object *meson_drm_gem_create_with_handle(
		struct drm_device *dev, void *data,
		struct drm_file *file_priv,
		struct dma_attrs *dma_attrs);
void meson_drm_gem_free_object(struct drm_gem_object *gem_obj);
int meson_drm_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf);
struct sg_table *meson_drm_gem_get_sg_table(struct drm_gem_object *obj);
int meson_drm_gem_mmap(struct file *filp, struct vm_area_struct *vma);

#endif
