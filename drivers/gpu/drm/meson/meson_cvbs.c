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

static bool read_hpd_gpio(void)
{
	return !!(aml_read_reg32(P_PREG_PAD_GPIO3_I) & (1 << 19));
}

static enum drm_connector_status meson_connector_detect(struct drm_connector *connector, bool force)
{
	/* Userspace can get really confused sometimes if we give it
	 * two connectors that are connected. Detect if HDMI is plugged in,
	 * and if so, disconnect our connector. */

	return read_hpd_gpio() ? connector_status_disconnected : connector_status_connected;
}

static int meson_connector_get_modes(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct drm_display_mode *mode;

	mode = drm_cvt_mode(dev, 720, 576, 50, false, true, false);
	mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	mode = drm_cvt_mode(dev, 720, 480, 60, false, true, false);
	mode->type |= DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode);

	return 2;
}

static int meson_connector_mode_valid(struct drm_connector *connector, struct drm_display_mode *mode)
{
	return (drm_mode_to_vmode(mode, MESON_MODES_CVBS) < VMODE_MAX) ? MODE_OK : MODE_BAD;
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

struct drm_connector *meson_cvbs_connector_create(struct drm_device *dev)
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

	drm_connector_init(dev, connector, &meson_connector_funcs, DRM_MODE_CONNECTOR_Composite);
	drm_connector_helper_add(connector, &meson_connector_helper_funcs);

	connector->interlace_allowed = 1;
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
