/*************************************************************
 * Amlogic
 * vout  serve program
 *
 * Copyright (C) 2010 Amlogic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the named License,
 * or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 * Author:   evoke.zhang@amlogic
 *
 *
 **************************************************************/
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/amlogic/vout/vinfo.h>

static const char *vmode_name[]={
	"480i",
	"480irpt",
	"480cvbs",
	"480p",
#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	"480p59hz",
#endif
	"480prpt",
	"576i",
	"576irpt",
	"576cvbs",
	"576p",
	"576prpt",
	"720p",
#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	"720p59hz",
#endif
	"720p50hz",
	"768p60hz",
	"768p50hz",
	"1080i",
#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	"1080i59hz",
#endif
	"1080i50hz",
	"1080p",
#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	"1080p59hz",
#endif
	"1080p50hz",
	"1080p24hz",
#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	"1080p23hz",
#endif
	"4k2k30hz",
#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	"4k2k29hz",
#endif
	"4k2k25hz",
	"4k2k24hz",
#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	"4k2k23hz",
#endif
	"4k2ksmpte",
	"4k2kfake5g",
	"4k2k60hz",
	"4k2k60hz420",
	"4k2k50hz",
	"4k2k50hz420",
	"4k2k5g",
	"4k1k120hz",
	"4k1k120hz420",
	"4k1k100hz",
	"4k1k100hz420",
	"4k05k240hz",
	"4k05k240hz420",
	"4k05k200hz",
	"4k05k200hz420",
	"vga",
	"svga",
	"xga",
	"sxga",
	"wsxga",
	"fhdvga",
	"panel",
	"lvds1080p",
	"lvds1080p50hz",
	"lvds768p",
	"vbyone4k2k60hz",
	"invalid",
};

static const vinfo_t vinfo_invalid = {
	.name              = "invalid",
	.mode              = VMODE_INIT_NULL,
	.width             = 1920,
	.height            = 1080,
	.field_height      = 1080,
	.aspect_ratio_num  = 16,
	.aspect_ratio_den  = 9,
	.sync_duration_num = 60,
	.sync_duration_den = 1,
	.viu_color_fmt = TVIN_RGB444,
};

vmode_t vmode_name_to_mode(char *str)
{
	int i;
	vmode_t vmode;

	for (i = 0; i < VMODE_MAX; i++) {
		if (strcmp(vmode_name[i], str) == 0)
			break;
	}
	vmode = VMODE_480I + i;
	return vmode;
}

const vinfo_t *get_invalid_vinfo(void)
{
	printk("[error]: invalid vinfo. current vmode is not supported!\n");
	return &vinfo_invalid;
}

