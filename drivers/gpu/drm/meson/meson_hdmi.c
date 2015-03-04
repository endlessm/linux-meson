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
#include "meson_priv.h"

#include <mach/am_regs.h>
#include <mach/irqs.h>
#include <mach/hdmi_tx_reg.h>

#include <linux/amlogic/hdmi_tx/hdmi_info_global.h>
#include <linux/amlogic/hdmi_tx/hdmi_tx_module.h>

static void meson_set_hdmi_audio(void)
{
	hdmitx_dev_t *hdmitx_device = get_hdmitx_device();

	if (hdmitx_device->HWOp.Cntl) {
		static int st = 0;
		st = hdmitx_device->HWOp.CntlMisc(hdmitx_device, MISC_HPD_GPI_ST, 0);

		if ((st == 1) && (hdmitx_device->hpd_state == 0))
			hdmitx_device->hpd_event = 1;

		if ((hdmitx_device->cur_VIC != HDMI_Unkown) &&
		   (!(hdmitx_device->HWOp.GetState(hdmitx_device, STAT_AUDIO_PACK, 0))))
			hdmitx_device->HWOp.CntlConfig(hdmitx_device, CONF_AUDIO_MUTE_OP, AUDIO_UNMUTE);
	}

	if (hdmitx_device->hpd_event == 1) {
		hdmitx_device->hpd_event = 0;
		hdmitx_device->hpd_state = 1;

		/* TODO: 2ch only for now */
		hdmitx_set_audio(hdmitx_device, &(hdmitx_device->cur_audio_param), 1);
	}
}

/* Encoder */

static void meson_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs meson_encoder_funcs = {
	.destroy        = meson_encoder_destroy,
};

static bool meson_encoder_mode_fixup(struct drm_encoder *encoder,
				     const struct drm_display_mode *mode,
				     struct drm_display_mode *adjusted_mode)
{
	vmode_t vmode;
	vmode = drm_mode_to_vmode(adjusted_mode, MESON_MODES_HDMI);
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
	vmode = drm_mode_to_vmode(adjusted_mode, MESON_MODES_HDMI);
	meson_drm_set_vmode(vmode);
	meson_set_hdmi_audio();

	/* Make sure to unblank our display */
	aml_write_reg32(P_VPU_HDMI_DATA_OVR, 0);
}

static void meson_encoder_disable(struct drm_encoder *encoder)
{
}

static const struct drm_encoder_helper_funcs meson_encoder_helper_funcs = {
	.mode_fixup     = meson_encoder_mode_fixup,
	.prepare        = meson_encoder_prepare,
	.commit         = meson_encoder_commit,
	.mode_set       = meson_encoder_mode_set,
	.disable	= meson_encoder_disable,
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
	struct delayed_work hotplug_work;
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

static int fetch_edid(void)
{
	/* Turn on the EDID power. */
	aml_set_reg32_bits(P_HHI_MEM_PD_REG0, 0, 8, 2);

	/* Ask for EDID by setting sys_config_trigger high. */
	hdmi_set_reg_bits(TX_HDCP_EDID_CONFIG, 1, 6, 1);

	/* XXX: Figure out how to turn on the EDID interrupt */
	msleep(200);
	if (!(hdmi_rd_reg(TX_HDCP_ST_EDID_STATUS) & (1 << 4))) {
		BUG();
		return -1;
	}

	return 0;
}

#define EDID_BLOCKS 4
#define EDID_BUF_LENGTH (EDID_LENGTH * EDID_BLOCKS)

static bool read_edid(uint8_t *edid_buf)
{
	int i;

	if (!(hdmi_rd_reg(TX_HDCP_ST_EDID_STATUS) & (1 << 4))) {
		/* If we don't have EDID, then request it from the HW. */
		int ret;

		ret = fetch_edid();
		if (ret < 0)
			return false;
	}

	for (i = 0; i < EDID_BUF_LENGTH; i++)
		edid_buf[i] = hdmi_rd_reg(TX_RX_EDID_OFFSET + i);

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

	if (drm_detect_hdmi_monitor(edid)) {
		hdmi_set_reg_bits(TX_TMDS_MODE, 0x3, 6, 2);
	} else {
		hdmi_set_reg_bits(TX_VIDEO_DTV_OPTION_L, 0x0, 6, 2);
		hdmi_set_reg_bits(TX_TMDS_MODE, 0x2, 6, 2);
	}

	drm_mode_connector_update_edid_property(connector, edid);
	return drm_add_edid_modes(connector, edid);
}

static int meson_connector_mode_valid(struct drm_connector *connector, struct drm_display_mode *mode)
{
	return (drm_mode_to_vmode(mode, MESON_MODES_HDMI) < VMODE_MAX) ? MODE_OK : MODE_BAD;
}

static struct drm_encoder *meson_connector_best_encoder(struct drm_connector *connector)
{
	struct meson_connector *meson_connector = to_meson_connector(connector);
	return meson_connector->encoder;
}

static const struct drm_connector_funcs meson_connector_funcs = {
	.destroy		= meson_connector_destroy,
	.detect			= meson_connector_detect,
	.dpms			= drm_atomic_helper_connector_dpms,
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

static void hdmi_hotplug_work_func(struct work_struct *work)
{
	struct meson_connector *meson_connector =
		container_of(work, struct meson_connector, hotplug_work.work);
	struct drm_connector *connector = &meson_connector->base;
	struct drm_device *dev = connector->dev;

	/* Clear interrupt status flags. We don't actually care what
	 * the INTR was about. */
	hdmi_wr_reg(OTHER_BASE_ADDR + HDMI_OTHER_INTR_STAT_CLR, 0xF);

	/* This interrupt means one of three things: HPD rose, HPD fell,
	 * or EDID has changed. For all three, emit a hotplug event. */
	drm_kms_helper_hotplug_event(dev);
}

static irqreturn_t meson_hdmi_intr_handler(int irq, void *user_data)
{
	struct drm_connector *connector = user_data;
	struct meson_connector *meson_connector = to_meson_connector(connector);

	mod_delayed_work(system_wq, &meson_connector->hotplug_work,
			 msecs_to_jiffies(200));

	return IRQ_HANDLED;
}

struct drm_connector *meson_hdmi_connector_create(struct drm_device *dev)
{
	struct meson_connector *meson_connector;
	struct drm_connector *connector;
        struct drm_encoder *encoder;
	int ret;

	/* Clear the VIC field of the AVI InfoFrame, which the boot loader
	 * might have configured. This has been seen to cause EDID read
	 * failures and "No signal" reported by the output.
	 *
	 * I'm not sure if the presence of a value here has some hardware-level
	 * effect, or just changes the behaviour of the hdmitx code, but I
	 * suspect the latter.
	 */
	hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR + 0x04, 0);

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

	INIT_DELAYED_WORK(&meson_connector->hotplug_work, hdmi_hotplug_work_func);

	ret = devm_request_threaded_irq(dev->dev, INT_HDMI_TX, NULL, meson_hdmi_intr_handler,
					IRQF_ONESHOT, dev_name(dev->dev), connector);
	if (ret < 0)
		goto fail;

	drm_connector_register(connector);
	return connector;

fail:
	meson_connector_destroy(connector);
	return NULL;
}
