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
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>
#include <video/videomode.h>

#include <mach/am_regs.h>
#include <mach/irqs.h>
#include <mach/hdmi_tx_reg.h>
#include <linux/amlogic/vout/vout_notify.h>

/* XXX: This is for EDID. Figure out how to do it better. */
#include <linux/amlogic/hdmi_tx/hdmi_tx_module.h>

/* XXX: Use standard EDID system */
static const struct {
	HDMI_Video_Codes_t hdmi_vic;
	struct videomode timing;
} supported_modes[] = {
	{ HDMI_640x480p60, { 25200,    640,  16,  48,  96,  480, 10, 33, 2, DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH } }, /* CEA Mode 1 */
	{ HDMI_480p60,     { 27027,    720,  16,  60,  62,  480,  9, 30, 6, DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH } }, /* CEA Mode 2 */
	{ HDMI_720p60,     { 74250,   1280, 110, 220,  40,  720,  5, 20, 5, DISPLAY_FLAGS_HSYNC_LOW  | DISPLAY_FLAGS_VSYNC_LOW  } }, /* CEA Mode 4 */
	{ HDMI_1080p60,    { 148500,  1920,  88, 148,  44,  1080, 4, 36, 5, DISPLAY_FLAGS_HSYNC_LOW  | DISPLAY_FLAGS_VSYNC_LOW  } }, /* CEA Mode 16 */
};

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
	ret = drm_encoder_init(dev, encoder, &meson_encoder_funcs, DRM_MODE_ENCODER_TMDS);
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

static bool read_hpd_gpio(void)
{
	return !!(aml_read_reg32(P_PREG_PAD_GPIO3_I) & (1 << 19));
}

static enum drm_connector_status meson_connector_detect(struct drm_connector *connector, bool force)
{
	return read_hpd_gpio() ? connector_status_connected : connector_status_disconnected;
}

static bool get_mode_type_from_edid(int *mode_type, hdmitx_dev_t *hdmitx, HDMI_Video_Codes_t hdmi_vic)
{
	int i;

	if (hdmitx->tv_no_edid)
		return false;

	if (hdmitx->RXCap.native_VIC == hdmi_vic) {
		*mode_type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		return true;
	}

	for (i = 0; i < hdmitx->RXCap.VIC_count; i++) {
		if (hdmitx->RXCap.VIC[i] == hdmi_vic) {
			*mode_type = DRM_MODE_TYPE_DRIVER;
			return true;
		}
	}

	return false;
}

static int meson_connector_get_modes(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	hdmitx_dev_t *hdmitx = get_hdmitx_device();
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		struct drm_display_mode *mode = drm_mode_create(dev);

		if (!get_mode_type_from_edid(&mode->type, hdmitx, supported_modes[i].hdmi_vic))
			continue;

		drm_display_mode_from_videomode(&supported_modes[i].timing, mode);

		/* Default to 1080P. XXX TODO: Check EDID. */
		if (supported_modes[i].hdmi_vic == HDMI_1080p60)
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

static irqreturn_t meson_hdmi_intr_handler(int irq, void *user_data)
{
	struct drm_connector *connector = user_data;
	struct drm_device *dev = connector->dev;

	/* Clear interrupt status flags. We don't actually care what
	 * the INTR was about. */
	hdmi_wr_reg(OTHER_BASE_ADDR + HDMI_OTHER_INTR_STAT_CLR, 0xF);

	/* This interrupt means one of three things: HPD rose, HPD fell,
	 * or EDID has changed. For all three, emit a hotplug event. */
	drm_kms_helper_hotplug_event(dev);

	return IRQ_HANDLED;
}

struct drm_connector *meson_hdmi_connector_create(struct drm_device *dev)
{
	struct meson_connector *meson_connector;
	struct drm_connector *connector;
        struct drm_encoder *encoder;
	int ret;

	encoder = meson_encoder_create(dev);
	if (!encoder)
		return NULL;

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

	ret = devm_request_irq(dev->dev, INT_HDMI_TX, meson_hdmi_intr_handler,
			       0, dev_name(dev->dev), connector);
	if (ret < 0)
		goto fail;

	drm_connector_register(connector);
	return connector;

fail:
	meson_connector_destroy(connector);
	return NULL;
}
