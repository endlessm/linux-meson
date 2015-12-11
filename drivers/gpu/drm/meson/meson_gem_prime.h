#ifndef __MESON_GEM_PRIME_H__
#define __MESON_GEM_PRIME_H__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <drm/drmP.h>

struct meson_drm_gem_scattered_object {
	unsigned long size;
	unsigned int flags;
	unsigned int nr_pages;
	struct drm_gem_object base;
	struct page **pages;
	struct dma_attrs dma_attrs;
};

static inline struct meson_drm_gem_scattered_object *
to_meson_drm_gem_scattered_obj(struct drm_gem_object *gem_obj)
{
	return container_of(gem_obj, struct meson_drm_gem_scattered_object, base);
}

struct meson_drm_gem_scattered_object *meson_drm_gem_scattered_create_with_handle(
		struct drm_device *dev, void *data,
		struct drm_file *file_priv,
		struct dma_attrs *dma_attrs);
struct sg_table *meson_drm_gem_scattered_prime_get_sg_table(struct drm_gem_object *obj);
void meson_drm_gem_scattered_free_object(struct drm_gem_object *gem_obj);
int meson_drm_gem_scattered_fault(struct vm_area_struct *vma, struct vm_fault *vmf);

#endif
