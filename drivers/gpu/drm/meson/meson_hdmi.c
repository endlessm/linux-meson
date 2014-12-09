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
#include <drm/drm_edid.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>

#include "meson_modes.h"

#include <mach/am_regs.h>
#include <mach/irqs.h>
#include <mach/hdmi_tx_reg.h>
#include <linux/amlogic/vout/vout_notify.h>
#include <linux/amlogic/hdmi_tx/hdmi_tx_module.h>

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

#define EDID_BLOCKS 4
#define EDID_BUF_LENGTH (EDID_LENGTH * EDID_BLOCKS)

static bool read_edid(uint8_t *edid_buf)
{
	hdmitx_dev_t *hdmitx = get_hdmitx_device();

	if (hdmitx->tv_no_edid)
		return false;

	memcpy(edid_buf, hdmitx->EDID_buf, EDID_BUF_LENGTH);

	/* TODO: Check extension block validity. */
	if (!drm_edid_block_valid(edid_buf, 0, true))
		return false;

	return true;
}

static int meson_connector_get_modes(struct drm_connector *connector)
{
	char edid_buf[EDID_BUF_LENGTH];
	struct edid *edid = (struct edid *) edid_buf;

	if (!read_edid(edid_buf))
		return 0;

	drm_mode_connector_update_edid_property(connector, edid);
	return drm_add_edid_modes(connector, edid);
}

static int meson_connector_mode_valid(struct drm_connector *connector, struct drm_display_mode *mode)
{
	return (drm_mode_to_vmode(mode) >= 0) ? MODE_OK : MODE_BAD;
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
