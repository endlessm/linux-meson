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

/* XXX: Replace this with our own HDMI driver eventually? */
static const struct {
	const char *drm_mode_name;
	vmode_t vmode;
	enum meson_modes_flags lookup_flags;
} supported_modes[] = {
	/* HDMI modes */
	{ "640x480",   VMODE_VGA,     MESON_MODES_HDMI },
	{ "720x480",   VMODE_480P,    MESON_MODES_HDMI },
	{ "1280x720",  VMODE_720P,    MESON_MODES_HDMI },
	{ "1920x1080", VMODE_1080P,   MESON_MODES_HDMI },

	/* CVBS modes */
	{ "720x576i",  VMODE_576CVBS, MESON_MODES_CVBS },
	{ "720x480i",  VMODE_480CVBS, MESON_MODES_CVBS },
};

vmode_t drm_mode_to_vmode(const struct drm_display_mode *mode,
			  enum meson_modes_flags flags)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if (strcmp(mode->name, supported_modes[i].drm_mode_name) == 0 &&
		    (supported_modes[i].lookup_flags & flags) != 0)
			return supported_modes[i].vmode;
	}

	return VMODE_MAX;
}
