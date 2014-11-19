/*
 * Copyright (C) 2014 Endless Mobile
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
 *     Jasper St. Pierre <jstpierre@mecheye.net>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <video/videomode.h>

#include <mach/am_regs.h>
#include <mach/irqs.h>
#include <mach/canvas.h>
#include <linux/amlogic/vout/vout_notify.h>

#define DRIVER_NAME "meson"
#define DRIVER_DESC "Amlogic Meson DRM driver"

/* Canvas configuration. */

enum meson_canvas_wrap {
	MESON_CANVAS_WRAP_NONE = 0x00,
	MESON_CANVAS_WRAP_X    = 0x01,
	MESON_CANVAS_WRAP_Y    = 0x02,
};

enum meson_canvas_blkmode {
	MESON_CANVAS_BLKMODE_LINEAR = 0x00,
	MESON_CANVAS_BLKMODE_32x32  = 0x01,
	MESON_CANVAS_BLKMODE_64x64  = 0x02,
};

/* Set up a canvas. */
static void canvas_setup(uint32_t canvas_index,
			 uint32_t addr,
			 uint32_t stride, uint32_t height,
			 enum meson_canvas_wrap wrap,
			 enum meson_canvas_blkmode blkmode)
{
	CANVAS_WRITE(DC_CAV_LUT_DATAL,
		     (((addr + 7) >> 3)) |
		     (((stride + 7) >> 3) << CANVAS_WIDTH_LBIT));
	CANVAS_WRITE(DC_CAV_LUT_DATAH,
		     ((((stride + 7) >> 3) >> CANVAS_WIDTH_LWID) << CANVAS_WIDTH_HBIT) |
		     (height << CANVAS_HEIGHT_BIT) |
		     (wrap << 22) |
		     (blkmode << CANVAS_BLKMODE_BIT));
	CANVAS_WRITE(DC_CAV_LUT_ADDR, CANVAS_LUT_WR_EN | canvas_index);

	/* Force a read-back to make sure everything is flushed. */
	CANVAS_READ(DC_CAV_LUT_DATAH);
}

/* CRTC */

struct meson_crtc {
	struct drm_crtc base;
};
#define to_meson_crtc(x) container_of(x, struct meson_crtc, base)

static void meson_crtc_destroy(struct drm_crtc *crtc)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);
	drm_crtc_cleanup(crtc);
	kfree(meson_crtc);
}

enum osd_w0_bitflags {
	OSD_ENDIANNESS_BE = (0x00 << 15),
	OSD_ENDIANNESS_LE = (0x01 << 15),

	OSD_BLK_MODE_422 = (0x03 << 8),
	OSD_BLK_MODE_16  = (0x04 << 8),
	OSD_BLK_MODE_32  = (0x05 << 8),
	OSD_BLK_MODE_24  = (0x07 << 8),

	OSD_OUTPUT_COLOR_YUV = (0x00 << 7),
	OSD_OUTPUT_COLOR_RGB = (0x01 << 7),

	OSD_COLOR_MATRIX_32_RGBA = (0x00 << 2),
	OSD_COLOR_MATRIX_32_ARGB = (0x01 << 2),
	OSD_COLOR_MATRIX_32_ABGR = (0x02 << 2),
	OSD_COLOR_MATRIX_32_BGRA = (0x03 << 2),
};

/* How this driver assigns the planes. */
enum osd_plane_idx {
	OSD_PLANE_FB = 0,
	OSD_PLANE_CURSOR = 1,
};

/* Dumb metaprogramming, should replace with something better. */
#define OSD_REGISTERS				\
	M(CTRL_STAT)				\
	M(BLK0_CFG_W0)				\
	M(BLK0_CFG_W1)				\
	M(BLK0_CFG_W2)				\
	M(BLK0_CFG_W3)				\
	M(BLK0_CFG_W4)

struct osd_plane {
	enum osd_plane_idx osd_index;
	uint32_t canvas_index;
	struct {
#define M(n) uint32_t n;
OSD_REGISTERS
#undef M
	} reg;

	uint32_t width, height;
};

/* Pick two canvases in the "user canvas" space that aren't
 * likely to compete. */
static struct osd_plane osd_planes[] = {
	{ OSD_PLANE_FB, 0x4e,
	  {
#define M(n) P_VIU_OSD1_##n ,
OSD_REGISTERS
#undef M
	  }
	},
	{ OSD_PLANE_CURSOR, 0x4f,
	  {
#define M(n) P_VIU_OSD2_##n ,
OSD_REGISTERS
#undef M
	  }
	},
};

#define OSD_REGISTER(n, idx) osd_planes[(idx)].reg.n

static void osd_plane_set(enum osd_plane_idx idx,
			  struct drm_gem_cma_object *cma_bo,
			  uint32_t width, uint32_t height)
{
	osd_planes[idx].width = width;
	osd_planes[idx].height = height;

	/* Swap out the OSD canvas with the new addr. */
	canvas_setup(osd_planes[idx].canvas_index,
		     cma_bo->paddr,
		     osd_planes[idx].width * 4,
		     osd_planes[idx].height,
		     MESON_CANVAS_WRAP_NONE,
		     MESON_CANVAS_BLKMODE_LINEAR);

	/* Set up BLK0 to point to the right canvas */
	aml_write_reg32(OSD_REGISTER(BLK0_CFG_W0, idx),
			(osd_planes[idx].canvas_index << 16) |
			OSD_ENDIANNESS_LE | OSD_BLK_MODE_32 | OSD_OUTPUT_COLOR_RGB | OSD_COLOR_MATRIX_32_ARGB);

	/* Enable OSD and BLK0. */
	aml_write_reg32(OSD_REGISTER(CTRL_STAT, idx),
			(1 << 21) |    /* Enable OSD */
			(0xFF << 12) | /* Alpha is 0xFF */
			(1 << 0)       /* Enable BLK0 */);
}

static void osd_plane_move(enum osd_plane_idx idx, int x, int y)
{
	uint32_t width, height;
	uint32_t pan_x, pan_y;
	uint32_t disp_x, disp_y;

	/* Move the OSD. */

	width = osd_planes[idx].width - 1;
	height = osd_planes[idx].height - 1;

	/* If we're off the edge of the screen negatively, use the pan
	 * feature to crop the image, as the display X/Y values are unsigned. */
	if (x > 0) {
		disp_x = x;
		pan_x = 0;
	} else {
		disp_x = 0;
		pan_x = -x;
		width -= pan_x;
	}

	if (y > 0) {
		pan_y = 0;
		disp_y = y;
	} else {
		disp_y = 0;
		pan_y = -y;
		height -= pan_y;
	}

	aml_write_reg32(OSD_REGISTER(BLK0_CFG_W1, idx),
			(pan_x + width) << 16 | pan_x);
	aml_write_reg32(OSD_REGISTER(BLK0_CFG_W2, idx),
			(pan_y + height) << 16 | pan_y);
	aml_write_reg32(OSD_REGISTER(BLK0_CFG_W3, idx),
			(disp_x + width) << 16 | disp_x);
	aml_write_reg32(OSD_REGISTER(BLK0_CFG_W4, idx),
			(disp_y + height) << 16 | disp_y);
}

static void update_fb_plane(struct drm_crtc *crtc)
{
	if (crtc->fb) {
		struct drm_gem_cma_object *cma_bo;
		cma_bo = drm_fb_cma_get_gem_obj(crtc->fb, 0);
		osd_plane_set(OSD_PLANE_FB, cma_bo,
			      crtc->mode.hdisplay,
			      crtc->mode.vdisplay);
		aml_set_reg32_mask(P_VPP_MISC, VPP_OSD1_POSTBLEND);
	} else {
		aml_clr_reg32_mask(P_VPP_MISC, VPP_OSD1_POSTBLEND);
	}
}

#if 0
static int meson_crtc_page_flip(struct drm_crtc *crtc,
				struct drm_framebuffer *fb,
				struct drm_pending_vblank_event *event)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);

	if (meson_crtc->event)
		return -EBUSY;

	crtc->fb = fb;
	meson_crtc->event = event;

	update_fb_plane(crtc);
	return 0;
}
#endif

static int meson_crtc_cursor_set(struct drm_crtc *crtc,
				 struct drm_file *file_priv, uint32_t handle,
				 uint32_t width, uint32_t height)
{
	struct drm_device *dev = crtc->dev;
	struct drm_gem_cma_object *cma_bo;

	if (handle) {
		struct drm_gem_object *bo;
		bo = drm_gem_object_lookup(dev, file_priv, handle);
		if (!bo)
			return -ENOENT;
		cma_bo = (struct drm_gem_cma_object *) bo;
	} else {
		cma_bo = NULL;
	}

	if (cma_bo) {
		osd_plane_set(OSD_PLANE_CURSOR, cma_bo, width, height);
		aml_set_reg32_mask(P_VPP_MISC, VPP_OSD2_POSTBLEND);
	} else {
		aml_clr_reg32_mask(P_VPP_MISC, VPP_OSD2_POSTBLEND);
	}

	return 0;
}

static int meson_crtc_cursor_move(struct drm_crtc *crtc, int x, int y)
{
	osd_plane_move(OSD_PLANE_CURSOR, x, y);
	return 0;
}

static const struct drm_crtc_funcs meson_crtc_funcs = {
	.set_config     = drm_crtc_helper_set_config,
	.destroy        = meson_crtc_destroy,
	/* Disable page flip until we have it done. */
#if 0
	.page_flip      = meson_crtc_page_flip,
#endif
	.cursor_set     = meson_crtc_cursor_set,
	.cursor_move    = meson_crtc_cursor_move,
};

static void meson_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	/* TODO: Implement DPMS */
}

static void meson_crtc_prepare(struct drm_crtc *crtc)
{
}

static void meson_crtc_commit(struct drm_crtc *crtc)
{
}

static bool meson_crtc_mode_fixup(struct drm_crtc *crtc,
				  const struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	return true;
}

/* XXX: Investigate supporting user-supplied modes, and separating
 * the primary plane mode from the connector's mode. */
static const struct {
	vmode_t vmode;
	struct videomode timing;
} supported_modes[] = {
	{ VMODE_VGA,   { 25200,    640,  16,  48,  96,  480, 10, 33, 2, DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH } }, /* CEA Mode 1 */
	{ VMODE_480P,  { 27027,    720,  16,  60,  62,  480,  9, 30, 6, DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH } }, /* CEA Mode 2 */
	{ VMODE_720P,  { 74250,   1280, 110, 220,  40,  720,  5, 20, 5, DISPLAY_FLAGS_HSYNC_LOW  | DISPLAY_FLAGS_VSYNC_LOW  } }, /* CEA Mode 4 */
	{ VMODE_1080P, { 148500,  1920,  88, 148,  44,  1080, 4, 36, 5, DISPLAY_FLAGS_HSYNC_LOW  | DISPLAY_FLAGS_VSYNC_LOW  } }, /* CEA Mode 16 */
	{ VMODE_576P,  { 27000,    720,  12,  68,  64,  576,  5, 39, 5, DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH } }, /* CEA Mode 17 */
};

static vmode_t drm_mode_to_vmode(struct drm_display_mode *mode)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		struct drm_display_mode supported_mode = {};

		drm_display_mode_from_videomode(&supported_modes[i].timing, &supported_mode);

		if (drm_mode_equal(mode, &supported_mode))
			return supported_modes[i].vmode;
	}

	return -1;
}

static void set_vmode(vmode_t mode)
{
	/* Call into aml's vout driver. */

	/* AML's vout or HDMI driver really does not like when
	 * you change the mode to the same thing it already is
	 * for currently unknown reasons.
	 *
	 * Double-check that it hasn't changed before calling
	 * set_current_vmode and notifying the HDMI stack. */

	/* XXX: Replace aml's vout driver with something sensible. */

	if (mode == get_current_vmode())
		return;

	set_current_vmode(mode);
	vout_notifier_call_chain(VOUT_EVENT_MODE_CHANGE, &mode);
}

static int meson_crtc_mode_set(struct drm_crtc *crtc,
			       struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode,
			       int x, int y,
			       struct drm_framebuffer *old_fb)
{
	vmode_t vmode;

	vmode = drm_mode_to_vmode(mode);
	if (vmode < 0)
		return -EINVAL;

	set_vmode(vmode);

	update_fb_plane(crtc);
	osd_plane_move(OSD_PLANE_FB, -x, -y);
	return 0;
}

static int meson_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
				    struct drm_framebuffer *old_fb)
{
	update_fb_plane(crtc);
	osd_plane_move(OSD_PLANE_FB, -x, -y);
	return 0;
}

static void meson_crtc_load_lut(struct drm_crtc *crtc)
{
}

static const struct drm_crtc_helper_funcs meson_crtc_helper_funcs = {
	.dpms           = meson_crtc_dpms,
	.prepare        = meson_crtc_prepare,
	.commit         = meson_crtc_commit,
	.mode_fixup     = meson_crtc_mode_fixup,
	.mode_set       = meson_crtc_mode_set,
	.mode_set_base  = meson_crtc_mode_set_base,
	.load_lut       = meson_crtc_load_lut,
};

struct drm_crtc *meson_crtc_create(struct drm_device *dev)
{
	struct meson_crtc *meson_crtc;
	struct drm_crtc *crtc;
	int ret;

	meson_crtc = kzalloc(sizeof(*meson_crtc), GFP_KERNEL);
	if (!meson_crtc)
		return NULL;

	crtc = &meson_crtc->base;
	ret = drm_crtc_init(dev, crtc, &meson_crtc_funcs);
	if (ret < 0)
		goto fail;

	drm_crtc_helper_add(crtc, &meson_crtc_helper_funcs);

	return crtc;

fail:
	meson_crtc_destroy(crtc);
	return NULL;
}

/* Encoder */

static void meson_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs meson_encoder_funcs = {
	.destroy        = meson_encoder_destroy,
};

static void meson_encoder_dpms(struct drm_encoder *encoder, int mode)
{
}

static bool meson_encoder_mode_fixup(struct drm_encoder *encoder,
				     const struct drm_display_mode *mode,
				     struct drm_display_mode *adjusted_mode)
{
	/* nothing needed */
	return true;
}

static void meson_encoder_prepare(struct drm_encoder *encoder)
{
}

static void meson_encoder_commit(struct drm_encoder *encoder)
{
}

static void meson_encoder_mode_set(struct drm_encoder *encoder,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	/* nothing needed */
}

static const struct drm_encoder_helper_funcs meson_encoder_helper_funcs = {
	.dpms           = meson_encoder_dpms,
	.mode_fixup     = meson_encoder_mode_fixup,
	.prepare        = meson_encoder_prepare,
	.commit         = meson_encoder_commit,
	.mode_set       = meson_encoder_mode_set,
};

static struct drm_encoder *meson_encoder_create(struct drm_device *dev)
{
	struct drm_encoder *encoder;
	int ret;

	encoder = kzalloc(sizeof(*encoder), GFP_KERNEL);
	if (!encoder)
		return NULL;

	encoder->possible_crtcs = 1;
	ret = drm_encoder_init(dev, encoder, &meson_encoder_funcs, DRM_MODE_ENCODER_DAC);
	if (ret < 0)
		goto fail;

	drm_encoder_helper_add(encoder, &meson_encoder_helper_funcs);
	return encoder;

fail:
	meson_encoder_destroy(encoder);
	return NULL;
}

/* Connector */

struct meson_connector {
	struct drm_connector base;
	struct drm_encoder *encoder;
};
#define to_meson_connector(x) container_of(x, struct meson_connector, base)

static void meson_connector_destroy(struct drm_connector *connector)
{
	struct meson_connector *meson_connector = to_meson_connector(connector);
	drm_connector_cleanup(connector);
	kfree(meson_connector);
}

static enum drm_connector_status meson_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static int meson_connector_get_modes(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		struct drm_display_mode *mode = drm_mode_create(dev);

		drm_display_mode_from_videomode(&supported_modes[i].timing, mode);

		mode->type = DRM_MODE_TYPE_DRIVER;

		/* Default to 1080P. XXX TODO: Check EDID. */
		if (supported_modes[i].vmode == VMODE_1080P)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	return i;
}

static int meson_connector_mode_valid(struct drm_connector *connector, struct drm_display_mode *mode)
{
	return MODE_OK;
}

static struct drm_encoder *meson_connector_best_encoder(struct drm_connector *connector)
{
	struct meson_connector *meson_connector = to_meson_connector(connector);
	return meson_connector->encoder;
}

static const struct drm_connector_funcs meson_connector_funcs = {
	.destroy            = meson_connector_destroy,
	.detect             = meson_connector_detect,
	.dpms               = drm_helper_connector_dpms,
	.fill_modes         = drm_helper_probe_single_connector_modes,
};

static const struct drm_connector_helper_funcs meson_connector_helper_funcs = {
	.get_modes          = meson_connector_get_modes,
	.mode_valid         = meson_connector_mode_valid,
	.best_encoder       = meson_connector_best_encoder,
};

static struct drm_connector *meson_connector_create(struct drm_device *dev, struct drm_encoder *encoder)
{
	struct meson_connector *meson_connector;
	struct drm_connector *connector;
	int ret;

	meson_connector = kzalloc(sizeof(*meson_connector), GFP_KERNEL);
	if (!meson_connector)
		return NULL;

	connector = &meson_connector->base;
	meson_connector->encoder = encoder;

	drm_connector_init(dev, connector, &meson_connector_funcs, DRM_MODE_CONNECTOR_HDMIA);
	drm_connector_helper_add(connector, &meson_connector_helper_funcs);

	connector->interlace_allowed = 0;
	connector->doublescan_allowed = 0;

	ret = drm_mode_connector_attach_encoder(connector, encoder);
	if (ret)
		goto fail;

	drm_sysfs_connector_add(connector);
	return connector;

fail:
	meson_connector_destroy(connector);
	return NULL;
}

/* DRM Driver */

struct meson_drm_private {
	struct drm_crtc *crtc;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	struct drm_fbdev_cma *fbdev;
};

static void meson_fb_output_poll_changed(struct drm_device *dev)
{
	struct meson_drm_private *priv = dev->dev_private;
	drm_fbdev_cma_hotplug_event(priv->fbdev);
}

static const struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create           = drm_fb_cma_create,
	.output_poll_changed = meson_fb_output_poll_changed,
};

/* Configure the VPP to act like how we expect it to. Other drivers,
 * like the ones included in U-Boot, might turn on weird features
 * like the HW scaler or special planes. Reset the VPP to a sane mode
 * that expects like we behave.
 */
static void reset_vpp(void)
{
	/* Turn off the HW scalers -- U-Boot turns these on and we
	 * need to clear them to make things work. */
	aml_clr_reg32_mask(P_VPP_OSD_SC_CTRL0, 1 << 3);
	aml_clr_reg32_mask(P_VPP_OSD_VSC_CTRL0, 1 << 24);
	aml_clr_reg32_mask(P_VPP_OSD_HSC_CTRL0, 1 << 22);

	/* Force all planes off -- U-Boot might configure them and
	 * we shouldn't have any stale planes. */
	aml_clr_reg32_mask(P_VPP_MISC, VPP_OSD1_POSTBLEND | VPP_OSD2_POSTBLEND);

	/* Turn on POSTBLEND. */
	aml_set_reg32_mask(P_VPP_MISC, VPP_POSTBLEND_EN);

	/* Put OSD2 (cursor) on top of OSD1. */
	aml_set_reg32_mask(P_VPP_MISC, VPP_POST_FG_OSD2 | VPP_PRE_FG_OSD2);
}

static int meson_load(struct drm_device *dev, unsigned long flags)
{
	struct platform_device *pdev = dev->platformdev;
	struct meson_drm_private *priv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev->dev_private = priv;

	drm_mode_config_init(dev);
	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;
	/* Just a guess for now */
	dev->mode_config.max_width = 9999;
	dev->mode_config.max_height = 9999;
	dev->mode_config.funcs = &mode_config_funcs;

	priv->crtc = meson_crtc_create(dev);
	priv->encoder = meson_encoder_create(dev);
	priv->connector = meson_connector_create(dev, priv->encoder);

	priv->fbdev = drm_fbdev_cma_init(dev, 32,
					 dev->mode_config.num_crtc,
					 dev->mode_config.num_connector);

	drm_kms_helper_poll_init(dev);

	platform_set_drvdata(pdev, dev);

	/* set vout mode at startup to prevent the rest of
	 * amlogic's drivers from crashing... */
	set_vmode(VMODE_1080P);

	reset_vpp();

	return 0;
}

static int meson_unload(struct drm_device *dev)
{
	drm_kms_helper_poll_fini(dev);
	drm_mode_config_cleanup(dev);

	kfree(dev->dev_private);
	dev->dev_private = NULL;

	return 0;
}

static void meson_lastclose(struct drm_device *dev)
{
	struct meson_drm_private *priv = dev->dev_private;
	drm_fbdev_cma_restore_mode(priv->fbdev);
}

static const struct file_operations fops = {
	.owner              = THIS_MODULE,
	.open               = drm_open,
	.release            = drm_release,
	.unlocked_ioctl     = drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl       = drm_compat_ioctl,
#endif
	.poll               = drm_poll,
	.read               = drm_read,
	.llseek             = no_llseek,
	.mmap               = drm_gem_cma_mmap,
};

static struct drm_driver meson_driver = {
	.driver_features    = DRIVER_GEM | DRIVER_MODESET,
	.load               = meson_load,
	.unload             = meson_unload,
	.lastclose          = meson_lastclose,
	.fops               = &fops,
	.name               = DRIVER_NAME,
	.desc               = DRIVER_DESC,
	.date               = "20141113",
	.major              = 1,
	.minor              = 0,
	.get_vblank_counter = drm_vblank_count,
	.gem_free_object    = drm_gem_cma_free_object,
	.gem_vm_ops         = &drm_gem_cma_vm_ops,
	.dumb_create        = drm_gem_cma_dumb_create,
	.dumb_map_offset    = drm_gem_cma_dumb_map_offset,
	.dumb_destroy       = drm_gem_cma_dumb_destroy,
};

static int meson_pdev_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;

	/* No DT configuration? Bail. */
	if (!node)
		return -ENXIO;

	return drm_platform_init(&meson_driver, pdev);
}

static int meson_pdev_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id dt_match[] = {
	{ .compatible = "amlogic,meson8b" },
	{}
};
MODULE_DEVICE_TABLE(of, dt_match);

static struct platform_driver meson_drm_platform_driver = {
	.probe      = meson_pdev_probe,
	.remove     = meson_pdev_remove,
	.driver     = {
		.owner  = THIS_MODULE,
		.name   = DRIVER_NAME,
		.of_match_table = dt_match,
	},
};

module_platform_driver(meson_drm_platform_driver);

MODULE_AUTHOR("Jasper St. Pierre <jstpierre@mecheye.net>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
