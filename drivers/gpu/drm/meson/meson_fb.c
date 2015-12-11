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
#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_flip_work.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_rect.h>
#include <drm/meson_drm.h>

#include "meson_fb.h"
#include "meson_cvbs.h"
#include "meson_hdmi.h"
#include "meson_modes.h"
#include "meson_priv.h"
#include "meson_gem.h"

static void meson_fb_destroy(struct drm_framebuffer *fb)
{
	struct meson_framebuffer *mfb = drm_fb_to_meson_fb(fb);
	int i;

	for (i = 0; i < 4; i++)
		if (mfb->obj[i])
		drm_gem_object_unreference_unlocked(&mfb->obj[i]->base);

	drm_framebuffer_cleanup(&mfb->fb);
	kfree(mfb);
}

static int meson_fb_create_handle(struct drm_framebuffer *fb,
				  struct drm_file *dfile,
				  unsigned int *handle)
{
	struct meson_framebuffer *mfb = drm_fb_to_meson_fb(fb);

	return drm_gem_handle_create(dfile, &mfb->obj[0]->base, handle);
}

static const struct drm_framebuffer_funcs meson_fb_funcs = {
	.destroy	= meson_fb_destroy,
	.create_handle	= meson_fb_create_handle,
};

static struct meson_framebuffer *meson_framebuffer_create(struct drm_device *dev,
							  struct drm_mode_fb_cmd2 *mode,
							  struct meson_drm_gem_object **obj,
							  unsigned int num_planes)
{
	struct meson_framebuffer *mfb;
	int ret;
	int i;

	mfb = kzalloc(sizeof(*mfb), GFP_KERNEL);
	if (!mfb) {
		DRM_ERROR("failed to allocate Meson fb object\n");
		return ERR_PTR(-ENOMEM);
	}

	for (i = 0; i < num_planes; i++)
		mfb->obj[i] = obj[i];

	drm_helper_mode_fill_fb_struct(&mfb->fb, mode);

	ret = drm_framebuffer_init(dev, &mfb->fb, &meson_fb_funcs);
	if (ret) {
		kfree(mfb);
		return ERR_PTR(ret);
	}

	return mfb;
}

static struct fb_ops meson_fbdev_ops = {
	.owner		= THIS_MODULE,
	.fb_fillrect    = sys_fillrect,
	.fb_copyarea    = sys_copyarea,
	.fb_imageblit   = sys_imageblit,
	.fb_check_var   = drm_fb_helper_check_var,
	.fb_set_par     = drm_fb_helper_set_par,
	.fb_blank       = drm_fb_helper_blank,
	.fb_pan_display = drm_fb_helper_pan_display,
	.fb_setcmap     = drm_fb_helper_setcmap,
};

static int meson_fbdev_create(struct drm_fb_helper *fbh,
			      struct drm_fb_helper_surface_size *sizes)
{
	struct drm_device *dev = fbh->dev;
	struct drm_mode_fb_cmd2 mode;
	struct meson_drm_gem_object *obj;
	struct fb_info *fbi;
	struct meson_framebuffer *mfb;
	struct drm_framebuffer *fb;
	unsigned int bytes_per_pixel;
	unsigned long offset;
	size_t size;
	int ret;

	bytes_per_pixel = DIV_ROUND_UP(sizes->surface_bpp, 8);

	memset(&mode, 0, sizeof(mode));
	mode.width = sizes->surface_width;
	mode.height = sizes->surface_height;
	mode.pitches[0] = sizes->surface_width * bytes_per_pixel;
	mode.pixel_format = drm_mode_legacy_fb_format(sizes->surface_bpp, sizes->surface_depth);

	size = mode.pitches[0] * mode.height;
	obj = meson_drm_gem_create_obj(dev, size);
	if (!obj) {
		DRM_ERROR("failed to allocate fb memory\n");
		return -ENOMEM;
	}

	fbi = framebuffer_alloc(0, dev->dev);
	if (!fbi) {
		dev_err(dev->dev, "Failed to allocate framebuffer info.\n");
		ret = -ENOMEM;
		goto err_drm_gem_free_object;
	}

	mfb = meson_framebuffer_create(dev, &mode, &obj, 1);
	if (IS_ERR(mfb)) {
		dev_err(dev->dev, "Failed to allocate DRM framebuffer.\n");
		ret = PTR_ERR(mfb);
		goto err_framebuffer_release;
	}

	fb = &mfb->fb;
	fbh->fb = fb;
	fbh->fbdev = fbi;

	fbi->par = fbh;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	fbi->fbops = &meson_fbdev_ops;

	ret = fb_alloc_cmap(&fbi->cmap, 256, 0);
	if (ret) {
		dev_err(dev->dev, "Failed to allocate color map.\n");
		goto err_drm_fb_destroy;
	}

	drm_fb_helper_fill_fix(fbi, mfb->fb.pitches[0], mfb->fb.depth);
	drm_fb_helper_fill_var(fbi, fbh, sizes->fb_width, sizes->fb_height);

	offset = fbi->var.xoffset * bytes_per_pixel;
	offset += fbi->var.yoffset * fb->pitches[0];

	dev->mode_config.fb_base = (resource_size_t)obj->paddr;
	fbi->screen_base = obj->vaddr + offset;
	fbi->fix.smem_start = (unsigned long)(obj->paddr + offset);
	fbi->screen_size = size;
	fbi->fix.smem_len = size;

	return 0;

err_drm_fb_destroy:
	drm_framebuffer_unregister_private(fb);
	meson_fb_destroy(fb);
err_framebuffer_release:
	framebuffer_release(fbi);
err_drm_gem_free_object:
	meson_drm_gem_free_object(&obj->base);
	return ret;
}

static int meson_fbdev_probe(struct drm_fb_helper *fbh,
		struct drm_fb_helper_surface_size *sizes)
{
	int ret = 0;

	if (!fbh->fb) {
		ret = meson_fbdev_create(fbh, sizes);
		if (ret == 0)
			ret = 1;
	}

	return ret;
}

static const struct drm_fb_helper_funcs meson_fbdev_helper_funcs = {
	.fb_probe = meson_fbdev_probe,
};

struct drm_fb_helper *meson_fbdev_init(struct drm_device *dev,
				       unsigned int preferred_bpp,
				       unsigned int num_crtc,
				       unsigned int max_conn_count)
{
	struct drm_fb_helper *fbh;
	int ret;

	fbh = devm_kzalloc(dev->dev, sizeof(*fbh), GFP_KERNEL);
	if (!fbh)
		return ERR_PTR(-ENOMEM);

	drm_fb_helper_prepare(dev, fbh, &meson_fbdev_helper_funcs);

	ret = drm_fb_helper_init(dev, fbh, num_crtc, max_conn_count);
	if (ret) {
		DRM_ERROR("failed to initialize drm fb helper\n");
		goto err_fb_helper;
	}

	ret = drm_fb_helper_single_add_all_connectors(fbh);
	if (ret) {
		DRM_ERROR("failed to add fb connectors\n");
		goto err_fb_setup;
	}

	ret = drm_fb_helper_initial_config(fbh, preferred_bpp);
	if (ret) {
		DRM_ERROR("failed to set initial config\n");
		goto err_fb_setup;
	}

	return fbh;

err_fb_setup:
	drm_fb_helper_fini(fbh);
err_fb_helper:
	return ERR_PTR(ret);
}

struct drm_framebuffer *meson_fb_create(struct drm_device *dev,
					struct drm_file *file_priv,
					struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct drm_gem_object *obj;
	struct meson_drm_gem_object *objs[4];
	struct meson_framebuffer *mfb;
	unsigned int hsub;
	unsigned int vsub;
	int ret;
	int i;

	hsub = drm_format_horz_chroma_subsampling(mode_cmd->pixel_format);
	vsub = drm_format_vert_chroma_subsampling(mode_cmd->pixel_format);

	for (i = 0; i < drm_format_num_planes(mode_cmd->pixel_format); i++) {
		unsigned int width = mode_cmd->width / (i ? hsub : 1);
		unsigned int height = mode_cmd->height / (i ? vsub : 1);
		unsigned int min_size;

		obj = drm_gem_object_lookup(dev, file_priv, mode_cmd->handles[i]);
		if (!obj) {
			dev_err(dev->dev, "Failed to lookup GEM object\n");
			ret = -ENXIO;
			goto err_gem_object_unreference;
		}

		min_size = (height - 1) * mode_cmd->pitches[i]
			 + width * drm_format_plane_cpp(mode_cmd->pixel_format, i)
			 + mode_cmd->offsets[i];

		if (obj->size < min_size) {
			drm_gem_object_unreference_unlocked(obj);
			ret = -EINVAL;
			goto err_gem_object_unreference;
		}
		objs[i] = to_meson_drm_gem_obj(obj);
	}

	mfb = meson_framebuffer_create(dev, mode_cmd, objs, i);
	if (IS_ERR(mfb)) {
		ret = PTR_ERR(mfb);
		goto err_gem_object_unreference;
	}

	return &mfb->fb;

err_gem_object_unreference:
	for (i--; i >= 0; i--)
		drm_gem_object_unreference_unlocked(&objs[i]->base);
	return ERR_PTR(ret);
}

struct meson_drm_gem_object *meson_drm_get_gem_obj(struct drm_framebuffer *fb,
						   unsigned int plane)
{
	struct meson_framebuffer *mfb = drm_fb_to_meson_fb(fb);

	if (plane >= 4)
		return NULL;

	return mfb->obj[plane];
}

static void meson_drm_fb_describe(struct drm_framebuffer *fb, struct seq_file *m)
{
	struct meson_framebuffer *mfb = drm_fb_to_meson_fb(fb);
	struct meson_drm_gem_object *meson_obj;
	struct drm_gem_object *obj;
	int i, n = drm_format_num_planes(fb->pixel_format);

	seq_printf(m, "fb: %p %dx%d@%4.4s\n", fb, fb->width, fb->height,
			(char *)&fb->pixel_format);

	for (i = 0; i < n; i++) {
		meson_obj = mfb->obj[i];
		obj = &meson_obj->base;

		seq_printf(m, "   %d: offset=%d pitch=%d, obj: ",
				i, fb->offsets[i], fb->pitches[i]);
		seq_printf(m, "%2d (%2d) %pad %p %d\n",
				obj->name, obj->refcount.refcount.counter,
				&meson_obj->paddr, meson_obj->vaddr, obj->size);
	}
}

int meson_drm_debugfs_show(struct seq_file *m, void *arg)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_framebuffer *fb;
	int ret;

	ret = mutex_lock_interruptible(&dev->mode_config.mutex);
	if (ret)
		return ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret) {
		mutex_unlock(&dev->mode_config.mutex);
		return ret;
	}

	list_for_each_entry(fb, &dev->mode_config.fb_list, head)
		meson_drm_fb_describe(fb, m);

	mutex_unlock(&dev->struct_mutex);
	mutex_unlock(&dev->mode_config.mutex);

	return 0;
}

