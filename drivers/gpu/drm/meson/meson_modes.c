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

#include "meson_modes.h"

/* XXX: This table is a horrible hack. Basically, the amlogic HDMI drivers
 * have its own set of output display modes in its code, in an enum called
 * vmode_t, and they are things like `VMODE_480P`, `VMODE_1080P`. We need
 * to map the standard modes that KMS supplies to
 *
 * The obvious way to do this would be to match on the mode name that
 * drm_edid.c assigns to each mode, but for some reason, under very
 * specific circumstances like hotplug, Xorg seems to hand us a mode with
 * a blank name and I'm not sure why. So just cheat and only match on the
 * hdisplay/vdisplay/vrefresh.
 */

/* XXX: Replace this with our own HDMI driver eventually? */

/* In order to add modes to this table, you need to match up four pieces
 * of information:
 *
 *  1. drm_edid.c keeps a giant list of standard HDMI modes in a table
 *     called "edid_cea_modes". Each mode contains above it a comment with the
 *     standard mode name and a number, the "VIC" (Video Identification Code).
 *
 *  2. include/linux/amlogic/hdmi_tx/hdmi_info_global.h has a giant list of
 *     VICs, "HDMI_Video_Codes_t", which match up to the ones that DRM has
 *     (the values are from the same specification, CEA-861).
 *
 *  3. drivers/amlogic/hdmi/hdmi_tx/hdmi_tx_edid.c has a table of mode names
 *     to HDMI VICs, "dispmode_VIC_tab". Find the mode name for the VIC you
 *     got in step 2.
 *
 *  4. drivers/amlogic/display/vout/tvconf.c has a table of mode names that
 *     match up to vmode_t's, "tv_info". Find the mode name from step 3 and
 *     match it with the vmode_t.
 *
 *  5. Steal the vdisplay/hdisplay/vrefresh attributes from DRM's mode table
 *     and the vmode_t from Step 4 and insert them into the table. It's just
 *     that simple!
 *
 * Alternatively, you can just guess at the correct values. That works too.
 */
static const struct {
	int hdisplay, vdisplay, vrefresh;
	vmode_t vmode;
	enum meson_modes_flags lookup_flags;
} supported_modes[] = {
	/* HDMI modes */
	{ 640,  480,  60, VMODE_640X480P_60HZ,   MESON_MODES_HDMI },
	{ 640,  480,  75, VMODE_640X480P_75HZ,   MESON_MODES_HDMI },
	{ 720,  480,  60, VMODE_480P,            MESON_MODES_HDMI },
	{ 720,  576,  50, VMODE_576P,            MESON_MODES_HDMI },
	{ 800,  600,  60, VMODE_800X600P_60HZ,   MESON_MODES_HDMI },
	{ 800,  600,  75, VMODE_800X600P_75HZ,   MESON_MODES_HDMI },
	{ 1024, 768,  60, VMODE_1024X768P_60HZ,  MESON_MODES_HDMI },
	{ 1024, 768,  75, VMODE_1024X768P_75HZ,  MESON_MODES_HDMI },
	{ 1280, 720,  50, VMODE_720P_50HZ,       MESON_MODES_HDMI },
	{ 1280, 720,  60, VMODE_720P,            MESON_MODES_HDMI },
	{ 1280, 800,  60, VMODE_1280X800P_60HZ,  MESON_MODES_HDMI },
	{ 1280, 1024, 60, VMODE_1280X1024P_60HZ, MESON_MODES_HDMI },
	{ 1280, 1024, 75, VMODE_1280X1024P_75HZ, MESON_MODES_HDMI },
	{ 1360, 768,  60, VMODE_1360X768P_60HZ,	 MESON_MODES_HDMI },
	{ 1366, 768,  60, VMODE_1366X768P_60HZ,	 MESON_MODES_HDMI },
	{ 1440, 900,  60, VMODE_1440X900P_60HZ,  MESON_MODES_HDMI },
	{ 1600, 900,  60, VMODE_1600X900P_60HZ,	 MESON_MODES_HDMI },
	{ 1920, 1080, 24, VMODE_1080P_24HZ,      MESON_MODES_HDMI },
	{ 1920, 1080, 50, VMODE_1080P_50HZ,      MESON_MODES_HDMI },
	{ 1920, 1080, 60, VMODE_1080P,           MESON_MODES_HDMI },

	/* CVBS modes */

	/* XXX: For some reason, calling drm_mode_vrefresh on these modes gives us
	 * incorrect refresh rates. The original algorithm comes from an Excel
	 * spreadsheet from 2003, which I really don't want to debug. */

	{ CVBS_HACK_MODE_SIZE(720),
	  CVBS_HACK_MODE_SIZE(480),
	  120, VMODE_480CVBS, MESON_MODES_CVBS },
};

vmode_t drm_mode_to_vmode(const struct drm_display_mode *mode,
			  enum meson_modes_flags flags)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if (supported_modes[i].hdisplay == mode->hdisplay &&
		    supported_modes[i].vdisplay == mode->vdisplay &&
		    supported_modes[i].vrefresh == drm_mode_vrefresh(mode) &&
		    (supported_modes[i].lookup_flags & flags) != 0)
			return supported_modes[i].vmode;
	}

	return VMODE_MAX;
}
