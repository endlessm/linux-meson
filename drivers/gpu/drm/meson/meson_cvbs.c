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

#include "meson_priv.h"
#include "meson_modes.h"
#include "meson_cvbs.h"

#include <mach/am_regs.h>
#include <mach/irqs.h>
#include <linux/amlogic/aml_gpio_consumer.h>

/* Encoder */

enum meson_cvbs_switch_state {
	MESON_CVBS_SWITCH_PAL,
	MESON_CVBS_SWITCH_NTSC,
	MESON_CVBS_SWITCH_UNDEFINED,
};

struct meson_cvbs_work {
	struct delayed_work work;
	struct drm_device *dev;
};

static int gpio_pal = -1;
static int gpio_ntsc = -1;
static struct meson_cvbs_work meson_cvbs_work;

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
	vmode_t vmode;
	vmode = drm_mode_to_vmode(adjusted_mode, MESON_MODES_CVBS);
	return (vmode != VMODE_MAX);
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
	vmode_t vmode;
	vmode = drm_mode_to_vmode(adjusted_mode, MESON_MODES_CVBS);
	meson_drm_set_vmode(vmode);

	/* A write to this badly-named register is also needed to unblank
	 * the CVBS output.
	 */
	aml_write_reg32(P_VPU_HDMI_DATA_OVR, 0);
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
	bool enabled;
	struct drm_display_mode *mode;
};
#define to_meson_connector(x) container_of(x, struct meson_connector, base)

static void meson_connector_destroy(struct drm_connector *connector)
{
	struct meson_connector *meson_connector = to_meson_connector(connector);
	drm_mode_destroy(connector->dev, meson_connector->mode);
	drm_connector_cleanup(connector);
	kfree(meson_connector);
}

static bool read_hpd_gpio(void)
{
	return !!(aml_read_reg32(P_PREG_PAD_GPIO3_I) & (1 << 19));
}

static enum meson_cvbs_switch_state meson_cvbs_get_switch_state(void)
{
	if (gpio_pal > 0 && amlogic_get_value(gpio_pal, "mesondrm pal"))
		return MESON_CVBS_SWITCH_PAL;
	if (gpio_ntsc > 0 && amlogic_get_value(gpio_ntsc, "mesondrm ntsc"))
		return MESON_CVBS_SWITCH_NTSC;
	return MESON_CVBS_SWITCH_UNDEFINED;
}

static enum drm_connector_status meson_connector_detect(struct drm_connector *connector, bool force)
{
	struct meson_connector *meson_connector = to_meson_connector(connector);
	int vrefresh = drm_mode_vrefresh(meson_connector->mode);
	enum meson_cvbs_switch_state s = meson_cvbs_get_switch_state();
	struct device *d = connector->dev->dev;

	if (!meson_connector->enabled)
		return connector_status_disconnected;

	/* PAL connector */
	if (vrefresh == 100 && s != MESON_CVBS_SWITCH_PAL) {
		dev_info(d, "CVBS/NTSC selected from switch\n");
		return connector_status_disconnected;
	}

	/* NTSC connector */
	if (vrefresh == 120 && s != MESON_CVBS_SWITCH_NTSC) {
		dev_info(d, "CVBS/PAL selected from switch\n");
		return connector_status_disconnected;
	}

	/* use the opposite from HPD -- HDMI connected means composite disconnected,
	 * HDMI disconnected means composite connected. */
	return read_hpd_gpio() ? connector_status_disconnected : connector_status_connected;
}

static int meson_connector_get_modes(struct drm_connector *connector)
{
	struct meson_connector *meson_connector = to_meson_connector(connector);
	struct drm_device *dev = connector->dev;
	drm_mode_probed_add(connector, drm_mode_duplicate(dev, meson_connector->mode));
	return 1;
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

struct drm_connector *meson_cvbs_connector_create(struct drm_device *dev,
						  bool enabled,
						  struct drm_display_mode *mode)
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
	meson_connector->enabled = enabled;
	meson_connector->mode = mode;

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

static void cvbs_switch_work_func(struct work_struct *work)
{
	struct meson_cvbs_work *meson_cvbs_work =
		container_of(work, struct meson_cvbs_work, work.work);

	/* This interrupt means the CVBS color enconding (PAL/NTSC) switch
	 * position has changed. */
	drm_kms_helper_hotplug_event(meson_cvbs_work->dev);
}

static irqreturn_t cvbs_switch_intr_handler(int irq, void *user_data)
{
	struct meson_cvbs_work *meson_cvbs_work = user_data;

	mod_delayed_work(system_wq, &meson_cvbs_work->work,
			 msecs_to_jiffies(200));
	return IRQ_HANDLED;
}

int meson_cvbs_init(struct drm_device *dev)
{
	struct device *d = dev->dev;
	const char *str;
	int ret;

	ret = of_property_read_string(d->of_node, "cvbs_pal_gpio", &str);
	if (ret < 0) {
		dev_warn(d, "Failed to read property \"cvbs_pal_gpio\"\n");
		goto out;
	}
	gpio_pal = amlogic_gpio_name_map_num(str);
	if (gpio_pal < 0) {
		dev_warn(d, "Failed to map cvbs_pal_gpio to a GPIO number\n");
		goto out;
	}
	ret = amlogic_gpio_request_one(gpio_pal, GPIOF_IN,
			"mesondrm pal");
	if (ret < 0) {
		dev_warn(d, "Failed to request cvbs_pal_gpio\n");
		goto out;
	}
	ret = amlogic_set_pull_up_down(gpio_pal, 1, "mesondrm pal");
	if (ret < 0) {
		dev_warn(d, "Failed to up-down cvbs_pal_gpio\n");
		goto out;
	}

	ret = of_property_read_string(d->of_node, "cvbs_ntsc_gpio", &str);
	if (ret < 0) {
		dev_warn(d, "Failed to read property \"cvbs_ntsc_gpio\"\n");
		goto out;
	}
	gpio_ntsc = amlogic_gpio_name_map_num(str);
	if (gpio_pal < 0) {
		dev_warn(d, "Failed to map cvbs_ntsc_gpio to a GPIO number\n");
		goto out;
	}
	ret = amlogic_gpio_request_one(gpio_ntsc, GPIOF_IN,
				       "mesondrm ntsc");
	if (ret < 0) {
		dev_warn(d, "Failed to request cvbs_ntsc_gpio\n");
		goto out;
	}
	ret = amlogic_set_pull_up_down(gpio_ntsc, 1, "mesondrm ntsc");
	if (ret < 0) {
		dev_warn(d, "Failed to up-down cvbs_ntsc_gpio\n");
		goto out;
	}

	ret = amlogic_gpio_to_irq(gpio_pal, "mesondrm pal",
				  AML_GPIO_IRQ(2, FILTER_NUM7,
					       GPIO_IRQ_RISING));
	if (ret < 0) {
		dev_warn(d,
			 "Failed to get IRQ line for cvbs_pal_gpio rising\n");
		goto out;
	}
	ret = amlogic_gpio_to_irq(gpio_pal, "mesondrm pal",
				  AML_GPIO_IRQ(1, FILTER_NUM7,
					       GPIO_IRQ_FALLING));
	if (ret < 0) {
		dev_warn(d,
			 "Failed to get IRQ line for cvbs_pal_gpio falling\n");
		goto out;
	}

	meson_cvbs_work.dev = dev;
	INIT_DELAYED_WORK(&meson_cvbs_work.work, cvbs_switch_work_func);

	ret = devm_request_threaded_irq(d, INT_GPIO_2, NULL,
					cvbs_switch_intr_handler, IRQF_ONESHOT,
					"mesondrm pal", &meson_cvbs_work);
	if (ret < 0) {
		dev_warn(d, "Failed to claim cvbs_pal_gpio rising IRQ\n");
		goto out;
	}
	ret = devm_request_threaded_irq(d, INT_GPIO_1, NULL,
					cvbs_switch_intr_handler, IRQF_ONESHOT,
					"mesondrm pal", &meson_cvbs_work);
	if (ret < 0) {
		dev_warn(d, "Failed to claim cvbs_pal_gpio falling IRQ\n");
		goto out;
	}

out:
	return ret;
}
