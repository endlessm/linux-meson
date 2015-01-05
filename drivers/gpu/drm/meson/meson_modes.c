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
static const struct {
	int hdisplay, vdisplay, vrefresh;
	vmode_t vmode;
	enum meson_modes_flags lookup_flags;
} supported_modes[] = {
	/* HDMI modes */
	{ 640,   480, 60, VMODE_VGA,        MESON_MODES_HDMI },
	{ 720,   480, 60, VMODE_480P,       MESON_MODES_HDMI },
	{ 1280,  720, 60, VMODE_720P,       MESON_MODES_HDMI },
	{ 1920, 1080, 60, VMODE_1080P,      MESON_MODES_HDMI },
	{ 1920, 1080, 50, VMODE_1080P_50HZ, MESON_MODES_HDMI },

	/* CVBS modes */
	{ 720,   576, 50, VMODE_576CVBS,    MESON_MODES_CVBS },
	{ 720,   480, 60, VMODE_480CVBS,    MESON_MODES_CVBS },
};

vmode_t drm_mode_to_vmode(const struct drm_display_mode *mode,
			  enum meson_modes_flags flags)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if (supported_modes[i].hdisplay == mode->hdisplay &&
		    supported_modes[i].vdisplay == mode->vdisplay &&
		    supported_modes[i].vrefresh == mode->vrefresh &&
		    (supported_modes[i].lookup_flags & flags) != 0)
			return supported_modes[i].vmode;
	}

	return VMODE_MAX;
}
