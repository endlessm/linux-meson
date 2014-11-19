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
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_rect.h>
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

/* Dumb metaprogramming, should replace with something better. */
#define OSD_REGISTERS				\
	M(CTRL_STAT)				\
	M(BLK0_CFG_W0)				\
	M(BLK0_CFG_W1)				\
	M(BLK0_CFG_W2)				\
	M(BLK0_CFG_W3)				\
	M(BLK0_CFG_W4)

struct osd_plane_def {
	uint32_t canvas_index;

	uint32_t vpp_misc_postblend;

	struct {
#define M(n) uint32_t n;
OSD_REGISTERS
#undef M
	} reg;
};

struct meson_plane {
	struct drm_plane base;
	struct osd_plane_def *def;
};
#define to_meson_plane(x) container_of(x, struct meson_plane, base)

static int meson_plane_atomic_check(struct drm_plane *plane,
				    struct drm_plane_state *state)
{
	struct drm_rect src = {
		.x1 = state->src_x,
		.y1 = state->src_y,
		.x2 = state->src_x + state->src_w,
		.y2 = state->src_y + state->src_h,
	};
	struct drm_rect dest = {
		.x1 = state->crtc_x,
		.y1 = state->crtc_y,
		.x2 = state->crtc_x + state->crtc_w,
		.y2 = state->crtc_y + state->crtc_h,
	};


	if (state->fb) {
		int ret;

		ret = drm_rect_calc_hscale(&src, &dest, DRM_PLANE_HELPER_NO_SCALING, DRM_PLANE_HELPER_NO_SCALING);
		if (ret < 0)
			return ret;

		ret = drm_rect_calc_vscale(&src, &dest, DRM_PLANE_HELPER_NO_SCALING, DRM_PLANE_HELPER_NO_SCALING);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static void meson_plane_atomic_update(struct drm_plane *plane)
{
	struct meson_plane *meson_plane = to_meson_plane(plane);
	struct drm_plane_state *state = plane->state;
	struct drm_rect src = {
		.x1 = (state->src_x) >> 16,
		.y1 = (state->src_y) >> 16,
		.x2 = (state->src_x + state->src_w) >> 16,
		.y2 = (state->src_y + state->src_h) >> 16,
	};
	struct drm_rect dest = {
		.x1 = state->crtc_x,
		.y1 = state->crtc_y,
		.x2 = state->crtc_x + state->crtc_w,
		.y2 = state->crtc_y + state->crtc_h,
	};
	const struct drm_rect clip = {
		.x2 = INT_MAX,
		.y2 = INT_MAX,
	};
	bool visible;

	if (state->fb) {
		visible = drm_rect_clip_scaled(&src, &dest, &clip,
					       DRM_PLANE_HELPER_NO_SCALING,
					       DRM_PLANE_HELPER_NO_SCALING);
	} else {
		visible = false;
	}

	if (visible) {
		struct drm_gem_cma_object *cma_bo;

		aml_set_reg32_mask(P_VPP_MISC, meson_plane->def->vpp_misc_postblend);

		cma_bo = drm_fb_cma_get_gem_obj(state->fb, 0);

		/* Swap out the OSD canvas with the new addr. */
		canvas_setup(meson_plane->def->canvas_index,
			     cma_bo->paddr,
			     (state->src_w >> 16) * 4,
			     (state->src_h >> 16),
			     MESON_CANVAS_WRAP_NONE,
			     MESON_CANVAS_BLKMODE_LINEAR);

		/* Set up BLK0 to point to the right canvas */
		aml_write_reg32(meson_plane->def->reg.BLK0_CFG_W0,
				(meson_plane->def->canvas_index << 16) |
				OSD_ENDIANNESS_LE | OSD_BLK_MODE_32 | OSD_OUTPUT_COLOR_RGB | OSD_COLOR_MATRIX_32_ARGB);

		/* Enable OSD and BLK0. */
		aml_write_reg32(meson_plane->def->reg.CTRL_STAT,
				(1 << 21) |    /* Enable OSD */
				(0xFF << 12) | /* Alpha is 0xFF */
				(1 << 0)       /* Enable BLK0 */);

		aml_write_reg32(meson_plane->def->reg.BLK0_CFG_W1,
				((src.x2 - 1) << 16) | src.x1);
		aml_write_reg32(meson_plane->def->reg.BLK0_CFG_W2,
				((src.y2 - 1) << 16) | src.y1);
		aml_write_reg32(meson_plane->def->reg.BLK0_CFG_W3,
				((dest.x2 - 1) << 16) | dest.x1);
		aml_write_reg32(meson_plane->def->reg.BLK0_CFG_W4,
				((dest.y2 - 1) << 16) | dest.y1);
	} else {
		aml_clr_reg32_mask(P_VPP_MISC, meson_plane->def->vpp_misc_postblend);
	}
}

static const struct drm_plane_helper_funcs meson_plane_helper_funcs = {
	.atomic_check = meson_plane_atomic_check,
	.atomic_update = meson_plane_atomic_update,
};

static void meson_plane_destroy(struct drm_plane *plane)
{
	drm_plane_helper_disable(plane);
	drm_plane_cleanup(plane);
	kfree(plane);
}

static const struct drm_plane_funcs meson_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= meson_plane_destroy,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

static const uint32_t supported_drm_formats[] = {
	DRM_FORMAT_ARGB8888,
};

static struct drm_plane *meson_plane_create(struct drm_device *dev,
					    enum drm_plane_type type,
					    struct osd_plane_def *osd_plane_def)
{
	struct meson_plane *meson_plane;
	struct drm_plane *plane;
	int ret;

	meson_plane = kzalloc(sizeof(*meson_plane), GFP_KERNEL);
	if (!meson_plane) {
		ret = -ENOMEM;
		goto fail;
	}

	plane = &meson_plane->base;

	meson_plane->def = osd_plane_def;

	drm_universal_plane_init(dev, plane, 0xFF,
				 &meson_plane_funcs,
				 supported_drm_formats,
				 ARRAY_SIZE(supported_drm_formats),
				 type);
	drm_plane_helper_add(plane, &meson_plane_helper_funcs);
	return plane;

fail:
	return ERR_PTR(ret);
}

static const struct drm_crtc_funcs meson_crtc_funcs = {
	.set_config             = drm_atomic_helper_set_config,
	.destroy		= meson_crtc_destroy,
	.reset			= drm_atomic_helper_crtc_reset,
	.page_flip		= drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
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

static vmode_t drm_mode_to_vmode(const struct drm_display_mode *mode)
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

static bool meson_crtc_mode_fixup(struct drm_crtc *crtc,
				  const struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	vmode_t vmode;

	vmode = drm_mode_to_vmode(mode);

	/* Invalid mode. */
	if (vmode < 0)
		return false;

	return true;
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

static void meson_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	vmode_t vmode;
	vmode = drm_mode_to_vmode(&crtc->state->adjusted_mode);
	set_vmode(vmode);
}

static void meson_crtc_load_lut(struct drm_crtc *crtc)
{
}

static const struct drm_crtc_helper_funcs meson_crtc_helper_funcs = {
	.dpms           = meson_crtc_dpms,
	.prepare        = meson_crtc_prepare,
	.commit         = meson_crtc_commit,
	.mode_fixup     = meson_crtc_mode_fixup,
	.mode_set       = drm_helper_crtc_mode_set,
	.mode_set_base  = drm_helper_crtc_mode_set_base,
	.mode_set_nofb  = meson_crtc_mode_set_nofb,
	.load_lut       = meson_crtc_load_lut,
};

/* Pick two canvases in the "user canvas" space that aren't
 * likely to compete. */
static struct osd_plane_def osd_plane_defs[] = {
	{
		.canvas_index = 0x4e,
		.vpp_misc_postblend = VPP_OSD1_POSTBLEND,
		{
#define M(n) .n = P_VIU_OSD1_##n ,
			OSD_REGISTERS
#undef M
		}
	},
	{
		.canvas_index = 0x4f,
		.vpp_misc_postblend = VPP_OSD2_POSTBLEND,
		{
#define M(n) .n = P_VIU_OSD2_##n ,
			OSD_REGISTERS
#undef M
		}
	},
};

struct drm_crtc *meson_crtc_create(struct drm_device *dev)
{
	struct meson_crtc *meson_crtc;
	struct drm_crtc *crtc;
	int ret;
	struct drm_plane *primary_plane, *cursor_plane;

	meson_crtc = kzalloc(sizeof(*meson_crtc), GFP_KERNEL);
	if (!meson_crtc)
		return NULL;

	primary_plane = meson_plane_create(dev,
					   DRM_PLANE_TYPE_PRIMARY,
					   &osd_plane_defs[0]);
	cursor_plane = meson_plane_create(dev,
					  DRM_PLANE_TYPE_CURSOR,
					  &osd_plane_defs[1]);

	crtc = &meson_crtc->base;
	ret = drm_crtc_init_with_planes(dev, crtc,
					primary_plane, cursor_plane,
					&meson_crtc_funcs);
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
	.destroy		= meson_connector_destroy,
	.detect			= meson_connector_detect,
	.dpms			= drm_helper_connector_dpms,
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.reset			= drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
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

	drm_connector_register(connector);
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
	.atomic_check        = drm_atomic_helper_check,
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
	int ret;

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

	ret = drm_vblank_init(dev, dev->mode_config.num_crtc);
	if (ret < 0) {
		/* XXX: Don't leak memory. */
		return ret;
	}

	drm_mode_config_reset(dev);

	drm_kms_helper_poll_init(dev);

	platform_set_drvdata(pdev, dev);

	/* set vout mode at startup to prevent the rest of
	 * amlogic's drivers from crashing... */
	set_vmode(VMODE_1080P);

	reset_vpp();

	drm_irq_install(dev, INT_VIU_VSYNC);

	priv->fbdev = drm_fbdev_cma_init(dev, 32,
					 dev->mode_config.num_crtc,
					 dev->mode_config.num_connector);

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

static int meson_enable_vblank(struct drm_device *dev, int crtc)
{
	return 0;
}

static void meson_disable_vblank(struct drm_device *dev, int crtc)
{
}

static irqreturn_t meson_irq(int irq, void *arg)
{
	struct drm_device *dev = arg;
	drm_handle_vblank(dev, 0);
	return IRQ_HANDLED;
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
	.driver_features    = DRIVER_HAVE_IRQ | DRIVER_GEM | DRIVER_MODESET,
	.load               = meson_load,
	.unload             = meson_unload,
	.lastclose          = meson_lastclose,
	.enable_vblank      = meson_enable_vblank,
	.disable_vblank     = meson_disable_vblank,
	.get_vblank_counter = drm_vblank_count,
	.irq_handler        = meson_irq,
	.set_busid          = drm_platform_set_busid,
	.fops               = &fops,
	.name               = DRIVER_NAME,
	.desc               = DRIVER_DESC,
	.date               = "20141113",
	.major              = 1,
	.minor              = 0,
	.gem_free_object    = drm_gem_cma_free_object,
	.gem_vm_ops         = &drm_gem_cma_vm_ops,
	.dumb_create        = drm_gem_cma_dumb_create,
	.dumb_map_offset    = drm_gem_cma_dumb_map_offset,
	.dumb_destroy       = drm_gem_dumb_destroy,
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
