/*
 * rt5640_ioctl.h  --  RT5640 ALSA SoC audio driver IO control
 *
 * Copyright 2012 Realtek Microelectronics
 * Author: Bard <bardliao@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/spi/spi.h>
#include <sound/soc.h>
#include "rt56xx_ioctl.h"
#include "rt5640_ioctl.h"
#include "rt5640.h"
#if (CONFIG_SND_SOC_RT5642_MODULE | CONFIG_SND_SOC_RT5642)
#include "rt5640-dsp.h"
#endif

hweq_t hweq_param[] = {
	{/* NORMAL */
		{0},
		{0},
		0x0000,
	},
	{/* SPK */
		{0xa0,	0xa1,	0xa2,	0xa3,	0xa4,	0xa5,	0xa6,	0xa7,	0xa8,	0xa9,	0xaa,	0xab,	0xac,	0xad,	0xae,	0xaf,	0xb0,	0xb1,	0xb2},
		{0x1c10,0x01f4,	0xc5e9,	0x1a98,	0x1d2c,	0xc882,	0x1c10,	0x01f4,	0xe904,	0x1c10,	0x01f4, 0xe904,	0x1c10,	0x01f4,	0x1c10,	0x01f4,	0x2000,	0x0000,	0x2000},
		0x0000,
	},
	{/* HP */
		{0xa0,	0xa1,	0xa2,	0xa3,	0xa4,	0xa5,	0xa6,	0xa7,	0xa8,	0xa9,	0xaa,	0xab,	0xac,	0xad,	0xae,	0xaf,	0xb0,	0xb1,	0xb2},
		{0x1c10,0x01f4,	0xc5e9,	0x1a98,	0x1d2c,	0xc882,	0x1c10,	0x01f4,	0xe904,	0x1c10,	0x01f4, 0xe904,	0x1c10,	0x01f4,	0x1c10,	0x01f4,	0x2000,	0x0000,	0x2000},
		0x0000,
	},
	{/* Vocal */
		{0xa0,	0xa1,	0xa2,	0xa3,	0xa4,	0xa5,	0xa6,	0xa7,	0xa8,	0xa9,	0xaa,	0xab,	0xac,	0xad,	0xae,	0xaf,	0xb0,	0xb1,	0xb2,	0xb3,	0xb4},
		{0x1ee1,0x0e23,	0xc129,	0x1ee1,	0x0699,	0xc339,	0x1cee,	0xfb54,	0xd653,	0x1180,	0x0000, 0xefb5,	0x0701,	0x0000,	0x1ebb,	0xfb54,	0x1fce,	0x0031,	0x1fce,	0x0287,	0x194c},
		0x007f,
	},
	{/* Powerful */
		{0xa0,	0xa1,	0xa2,	0xa3,	0xa4,	0xa5,	0xa6,	0xa7,	0xa8,	0xa9,	0xaa,	0xab,	0xac,	0xad,	0xae,	0xaf,	0xb0,	0xb1,	0xb2,	0xb3,	0xb4},
		{0x1f6f,0x1830,	0xc2c8,	0x1d42,	0xf65f,	0xc559,	0x1b41,	0xfe43,	0xd772,	0x102b,	0xfe43, 0xefb5,	0x0701,	0x0000,	0x17f8,	0x2298,	0x1fca,	0x0035,	0x1fca,	0x0287,	0x194c},
		0x003f,
	},
	{/* Party */
		{0xa0,	0xa1,	0xa2,	0xa3,	0xa4,	0xa5,	0xa6,	0xa7,	0xa8,	0xa9,	0xaa,	0xab,	0xac,	0xad,	0xae,	0xaf,	0xb0,	0xb1,	0xb2,	0xb3,	0xb4},
		{0x1f8c,0x0e23,	0xc0ea,	0x1f19,	0xfc00,	0xc4df,	0x1bbc,	0x0000,	0xcaaf,	0x178e,	0xff1b, 0xd9e3,	0x0d44,	0xfb54,	0x1180,	0x0e23,	0x1fca,	0x0035,	0x1fca,	0x0287,	0x194c},
		0x003f,
	},
	{/* Classical */
		{0xa0,	0xa1,	0xa2,	0xa3,	0xa4,	0xa5,	0xa6,	0xa7,	0xa8,	0xa9,	0xaa,	0xab,	0xac,	0xad,	0xae,	0xaf,	0xb0,	0xb1,	0xb2,	0xb3,	0xb4},
		{0xe835,0x0000,	0xc25c,	0x1dcc,	0x00f2,	0xc4df,	0x1bbc,	0x0699,	0xcb87,	0x16ad,	0xfd77, 0xefb5,	0x0701,	0x0000,	0x1d87,	0xf900,	0x1fca,	0x0035,	0x1fca,	0x0287,	0x194c},
		0x00bf,
	},
	{/* Jazz */
		{0xa0,	0xa1,	0xa2,	0xa3,	0xa4,	0xa5,	0xa6,	0xa7,	0xa8,	0xa9,	0xaa,	0xab,	0xac,	0xad,	0xae,	0xaf,	0xb0,	0xb1,	0xb2,	0xb3,	0xb4},
		{0x1dcc,0x11d0,	0xc25c,	0x1dcc,	0x0000,	0xca4a,	0x17f8,	0x0556,	0xd941,	0x0e05,	0xfc00, 0x06f2,	0xe84a,	0xf805,	0x1dcc,	0x11d0,	0x1f08,	0x00f0,	0x1f0c,	0x0287,	0x194c},
		0x007f,
	},
	{/* Treble */
		{0xa0,	0xa1,	0xa2,	0xa3,	0xa4,	0xa5,	0xa6,	0xa7,	0xa8,	0xa9,	0xaa,	0xab,	0xac,	0xad,	0xae,	0xaf,	0xb0,	0xb1,	0xb2,	0xb3,	0xb4},
		{0x1d42,0xf65f,	0xc13a,	0x1ed0,	0xff1b,	0xc25c,	0x1dcc,	0x00f2,	0xc628,	0x1a70,	0xfc00, 0xefb5,	0x0701,	0x0000,	0x1bbc,	0x1d18,	0x1fca,	0x0035,	0x1fca,	0x0287,	0x194c},
		0x003f,
	},
	{/* Club */
		{0xa0,	0xa1,	0xa2,	0xa3,	0xa4,	0xa5,	0xa6,	0xa7,	0xa8,	0xa9,	0xaa,	0xab,	0xac,	0xad,	0xae,	0xaf,	0xb0,	0xb1,	0xb2,	0xb3,	0xb4},
		{0xf429,0x0000,	0xc129,	0x1ee1,	0x07f0,	0xc899,	0x17f8,	0x0adc,	0xd772,	0x102b,	0xfe43, 0xf795,	0xf429,	0x0000,	0x1f6f,	0x0000,	0x1fc2,	0x003d,	0x1fc2,	0x0287,	0x194c},
		0x00ff,
	},
};
#define RT5640_HWEQ_LEN ARRAY_SIZE(hweq_param)

int rt5640_update_eqmode(
	struct snd_soc_codec *codec, int mode, int set_custom_eqmode)
{
	struct rt56xx_ops *ioctl_ops = rt56xx_get_ioctl_ops();
	int i;
	static int current_eqmode = NORMAL;
	static int custom_eqmode = NORMAL;

	if(codec == NULL ||  mode >= RT5640_HWEQ_LEN)
		return -EINVAL;

	dev_dbg(codec->dev, "%s(): mode=%d set_custom_eqmode=%d\n", __func__, mode, set_custom_eqmode);

	if (set_custom_eqmode) {
		custom_eqmode = mode;
		if (!(snd_soc_read(codec, RT5640_PWR_DIG1) & (RT5640_PWR_DAC_L1 | RT5640_PWR_DAC_R1)))
			return 0;
	}

	if (mode == SPK || mode == HP)
		if (custom_eqmode != NORMAL)
			mode = custom_eqmode;

	if(mode == current_eqmode)
		return 0;

	for(i = 0; i < EQ_REG_NUM; i++) {
		if(hweq_param[mode].reg[i])
			ioctl_ops->index_write(codec, hweq_param[mode].reg[i],
					hweq_param[mode].value[i]);
		else
			break;
	}
	snd_soc_update_bits(codec, RT5640_EQ_CTRL2, RT5640_EQ_CTRL_MASK,
					hweq_param[mode].ctrl);
	snd_soc_update_bits(codec, RT5640_EQ_CTRL1,
		RT5640_EQ_UPD, RT5640_EQ_UPD);
	snd_soc_update_bits(codec, RT5640_EQ_CTRL1, RT5640_EQ_UPD, 0);
	
	current_eqmode = mode;

	return 0;
}

void set_drc_agc_enable(struct snd_soc_codec *codec, int enable, int path)
{
	snd_soc_update_bits(codec, RT5640_DRC_AGC_1, RT5640_DRC_AGC_P_MASK |
		RT5640_DRC_AGC_MASK | RT5640_DRC_AGC_UPD,
		enable << RT5640_DRC_AGC_SFT | path << RT5640_DRC_AGC_P_SFT |
		1 << RT5640_DRC_AGC_UPD_BIT);
}

void set_drc_agc_parameters(struct snd_soc_codec *codec, int attack_rate,
			int sample_rate, int recovery_rate, int limit_level)
{
	snd_soc_update_bits(codec, RT5640_DRC_AGC_3, RT5640_DRC_AGC_TAR_MASK,
				limit_level << RT5640_DRC_AGC_TAR_SFT);
	snd_soc_update_bits(codec, RT5640_DRC_AGC_1, RT5640_DRC_AGC_AR_MASK |
		RT5640_DRC_AGC_R_MASK | RT5640_DRC_AGC_UPD |
		RT5640_DRC_AGC_RC_MASK, attack_rate << RT5640_DRC_AGC_AR_SFT |
		sample_rate << RT5640_DRC_AGC_R_SFT |
		recovery_rate << RT5640_DRC_AGC_RC_SFT |
		0x1 << RT5640_DRC_AGC_UPD_BIT);
}

void set_digital_boost_gain(struct snd_soc_codec *codec,
			int post_gain, int pre_gain)
{
	snd_soc_update_bits(codec, RT5640_DRC_AGC_2,
		RT5640_DRC_AGC_POB_MASK | RT5640_DRC_AGC_PRB_MASK,
		post_gain << RT5640_DRC_AGC_POB_SFT |
		pre_gain << RT5640_DRC_AGC_PRB_SFT);
	snd_soc_update_bits(codec, RT5640_DRC_AGC_1, 
		RT5640_DRC_AGC_UPD, 1 << RT5640_DRC_AGC_UPD_BIT);
}

void set_noise_gate(struct snd_soc_codec *codec, int noise_gate_en,
	int noise_gate_hold_en, int compression_gain, int noise_gate_th)
{
	snd_soc_update_bits(codec, RT5640_DRC_AGC_3,
		RT5640_DRC_AGC_NGB_MASK | RT5640_DRC_AGC_NG_MASK |
		RT5640_DRC_AGC_NGH_MASK | RT5640_DRC_AGC_NGT_MASK,
		noise_gate_en << RT5640_DRC_AGC_NG_SFT |
		noise_gate_hold_en << RT5640_DRC_AGC_NGH_SFT |
		compression_gain << RT5640_DRC_AGC_NGB_SFT |
		noise_gate_th << RT5640_DRC_AGC_NGT_SFT);
	snd_soc_update_bits(codec, RT5640_DRC_AGC_1,
		RT5640_DRC_AGC_UPD, 1 << RT5640_DRC_AGC_UPD_BIT);
}

void set_drc_agc_compression(struct snd_soc_codec *codec,
		int compression_en, int compression_ratio)
{
	snd_soc_update_bits(codec, RT5640_DRC_AGC_2,
		RT5640_DRC_AGC_CP_MASK | RT5640_DRC_AGC_CPR_MASK,
		compression_en << RT5640_DRC_AGC_CP_SFT |
		compression_ratio << RT5640_DRC_AGC_CPR_SFT);
	snd_soc_update_bits(codec, RT5640_DRC_AGC_1,
		RT5640_DRC_AGC_UPD, 1 << RT5640_DRC_AGC_UPD_BIT);
}

void get_drc_agc_enable(struct snd_soc_codec *codec, int *enable, int *path)
{
	unsigned int reg = snd_soc_read(codec, RT5640_DRC_AGC_1);

	*enable = (reg & RT5640_DRC_AGC_MASK) >> RT5640_DRC_AGC_SFT;
	*path = (reg & RT5640_DRC_AGC_P_MASK) >> RT5640_DRC_AGC_P_SFT;
}

void get_drc_agc_parameters(struct snd_soc_codec *codec, int *attack_rate,
		int *sample_rate, int *recovery_rate, int *limit_level)
{
	unsigned int reg = snd_soc_read(codec, RT5640_DRC_AGC_3);

	*limit_level = (reg & RT5640_DRC_AGC_TAR_MASK) >>
			RT5640_DRC_AGC_TAR_SFT;
	reg = snd_soc_read(codec, RT5640_DRC_AGC_1);
	*attack_rate = (reg & RT5640_DRC_AGC_AR_MASK) >> RT5640_DRC_AGC_AR_SFT;
	*sample_rate = (reg & RT5640_DRC_AGC_R_MASK) >> RT5640_DRC_AGC_R_SFT;
	*recovery_rate = (reg & RT5640_DRC_AGC_RC_MASK) >>
				RT5640_DRC_AGC_RC_SFT;
}

void get_digital_boost_gain(struct snd_soc_codec *codec,
			int *post_gain, int *pre_gain)
{
	unsigned int reg = snd_soc_read(codec, RT5640_DRC_AGC_2);

	*post_gain = (reg & RT5640_DRC_AGC_POB_MASK) >> RT5640_DRC_AGC_POB_SFT;
	*pre_gain = (reg & RT5640_DRC_AGC_PRB_MASK) >> RT5640_DRC_AGC_PRB_SFT;
}

void get_noise_gate(struct snd_soc_codec *codec, int *noise_gate_en,
	int *noise_gate_hold_en, int *compression_gain, int *noise_gate_th)
{
	unsigned int reg = snd_soc_read(codec, RT5640_DRC_AGC_3);

	printk("get_noise_gate reg=0x%04x\n",reg);
	*noise_gate_en = (reg & RT5640_DRC_AGC_NG_MASK) >>
				RT5640_DRC_AGC_NG_SFT;
	*noise_gate_hold_en = (reg & RT5640_DRC_AGC_NGH_MASK) >>
				RT5640_DRC_AGC_NGH_SFT;
	*compression_gain = (reg & RT5640_DRC_AGC_NGB_MASK) >>
				RT5640_DRC_AGC_NGB_SFT;
	*noise_gate_th = (reg & RT5640_DRC_AGC_NGT_MASK) >>
				RT5640_DRC_AGC_NGT_SFT;
}

void get_drc_agc_compression(struct snd_soc_codec *codec,
		int *compression_en, int *compression_ratio)
{
	unsigned int reg = snd_soc_read(codec, RT5640_DRC_AGC_2);

	*compression_en = (reg & RT5640_DRC_AGC_CP_MASK) >>
				RT5640_DRC_AGC_CP_SFT;
	*compression_ratio = (reg & RT5640_DRC_AGC_CPR_MASK) >>
				RT5640_DRC_AGC_CPR_SFT;
}

int rt5640_ioctl_common(struct snd_hwdep *hw, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct snd_soc_codec *codec = hw->private_data;
	struct rt56xx_cmd __user *_rt56xx = (struct rt56xx_cmd *)arg;
	struct rt56xx_cmd rt56xx;
	struct rt56xx_ops *ioctl_ops = rt56xx_get_ioctl_ops();
	int *buf, mask1 = 0, mask2 = 0;
	static int eq_mode;

	if (copy_from_user(&rt56xx, _rt56xx, sizeof(rt56xx))) {
		dev_err(codec->dev,"copy_from_user faild\n");
		return -EFAULT;
	}
	dev_dbg(codec->dev, "%s(): rt56xx.number=%d, cmd=%d\n",
			__func__, rt56xx.number, cmd);
	buf = kmalloc(sizeof(*buf) * rt56xx.number, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;
	if (copy_from_user(buf, rt56xx.buf, sizeof(*buf) * rt56xx.number)) {
		goto err;
	}
	
	switch (cmd) {
	case RT_SET_CODEC_HWEQ_IOCTL:
		if (eq_mode == *buf)
			break;
		eq_mode = *buf;
		rt5640_update_eqmode(codec, eq_mode, 1);
		break;

	case RT_GET_CODEC_ID:
		*buf = snd_soc_read(codec, RT5640_VENDOR_ID2);
		if (copy_to_user(rt56xx.buf, buf, sizeof(*buf) * rt56xx.number))
			goto err;
		break;

	case RT_SET_CODEC_SPK_VOL_IOCTL:
		if(*(buf) <= 0x27) {
			snd_soc_update_bits(codec, RT5640_SPK_VOL,
				RT5640_L_VOL_MASK | RT5640_R_VOL_MASK,
				*(buf) << RT5640_L_VOL_SFT |
				*(buf) << RT5640_R_VOL_SFT);
		}
		break;

	case RT_SET_CODEC_MIC_GAIN_IOCTL:
		if(*(buf) <= 0x8) {
			snd_soc_update_bits(codec, RT5640_IN1_IN2,
				RT5640_BST_MASK1, *(buf) << RT5640_BST_SFT1);
			snd_soc_update_bits(codec, RT5640_IN3_IN4,
				RT5640_BST_MASK2, *(buf) << RT5640_BST_SFT2);
		}
		break;

	case RT_SET_CODEC_3D_SPK_IOCTL:
		if(rt56xx.number < 4)
			break;
		if (NULL == ioctl_ops->index_update_bits)
			break;

		mask1 = 0;
		if(*buf != -1)
			mask1 |= RT5640_3D_SPK_MASK;
		if(*(buf + 1) != -1)
			mask1 |= RT5640_3D_SPK_M_MASK;
		if(*(buf + 2) != -1)
			mask1 |= RT5640_3D_SPK_CG_MASK;
		if(*(buf + 3) != -1)
			mask1 |= RT5640_3D_SPK_SG_MASK;
		ioctl_ops->index_update_bits(codec, RT5640_3D_SPK, mask1,
			*(buf) << RT5640_3D_SPK_SFT |
			*(buf + 1) << RT5640_3D_SPK_M_SFT |
			*(buf + 2) << RT5640_3D_SPK_CG_SFT |
			*(buf + 3) << RT5640_3D_SPK_SG_SFT);
		break;

	case RT_SET_CODEC_MP3PLUS_IOCTL:
		if(rt56xx.number < 5)
			break;
		mask1 = mask2 = 0;
		if(*buf != -1)
			mask1 |= RT5640_M_MP3_MASK;
		if(*(buf + 1) != -1)
			mask1 |= RT5640_EG_MP3_MASK;
		if(*(buf + 2) != -1)
			mask2 |= RT5640_OG_MP3_MASK;
		if(*(buf + 3) != -1)
			mask2 |= RT5640_HG_MP3_MASK;
		if(*(buf + 4) != -1)
			mask2 |= RT5640_MP3_WT_MASK;
		
		snd_soc_update_bits(codec, RT5640_MP3_PLUS1, mask1,
			*(buf) << RT5640_M_MP3_SFT |
			*(buf + 1) << RT5640_EG_MP3_SFT);
		snd_soc_update_bits(codec, RT5640_MP3_PLUS2, mask2,
			*(buf + 2) << RT5640_OG_MP3_SFT |
			*(buf + 3) << RT5640_HG_MP3_SFT |
			*(buf + 4) << RT5640_MP3_WT_SFT);
		break;
	case RT_SET_CODEC_3D_HEADPHONE_IOCTL:
		if(rt56xx.number < 4)
			break;
		if (NULL == ioctl_ops->index_update_bits)
			break;

		mask1 = 0;
		if(*buf != -1)
			mask1 |= RT5640_3D_HP_MASK;
		if(*(buf + 1) != -1)
			mask1 |= RT5640_3D_BT_MASK;
		if(*(buf + 2) != -1)
			mask1 |= RT5640_3D_1F_MIX_MASK;
		if(*(buf + 3) != -1)
			mask1 |= RT5640_3D_HP_M_MASK;

		snd_soc_update_bits(codec, RT5640_3D_HP, mask1,
			*(buf)<<RT5640_3D_HP_SFT |
			*(buf + 1) << RT5640_3D_BT_SFT |
			*(buf + 2) << RT5640_3D_1F_MIX_SFT |
			*(buf + 3) << RT5640_3D_HP_M_SFT);
		if(*(buf + 4) != -1)
			ioctl_ops->index_update_bits(codec,
					0x59, 0x1f, *(buf+4));
		break;

	case RT_SET_CODEC_BASS_BACK_IOCTL:
		if(rt56xx.number < 3)
			break;
		mask1 = 0;
		if(*buf != -1)
			mask1 |= RT5640_BB_MASK;
		if(*(buf + 1) != -1)
			mask1 |= RT5640_BB_CT_MASK;
		if(*(buf + 2) != -1)
			mask1 |= RT5640_G_BB_BST_MASK;
		
		snd_soc_update_bits(codec, RT5640_BASE_BACK, mask1,
			*(buf) << RT5640_BB_SFT |
			*(buf + 1) << RT5640_BB_CT_SFT |
			*(buf + 2) << RT5640_G_BB_BST_SFT);
		break;

	case RT_SET_CODEC_DIPOLE_SPK_IOCTL:
		if(rt56xx.number < 2)
			break;
		if (NULL == ioctl_ops->index_update_bits)
			break;

		mask1 = 0;
		if(*buf != -1)
			mask1 |= RT5640_DP_SPK_MASK;
		if(*(buf + 1) != -1)
			mask1 |= RT5640_DP_ATT_MASK;
		
		ioctl_ops->index_update_bits(codec, RT5640_DIP_SPK_INF,
			mask1, *(buf) << RT5640_DP_SPK_SFT |
			*(buf + 1) << RT5640_DP_ATT_SFT );
		break;

	case RT_SET_CODEC_DRC_AGC_ENABLE_IOCTL:
		if(rt56xx.number < 2)
			break;
		set_drc_agc_enable(codec, *(buf), *(buf + 1));
		break;

	case RT_SET_CODEC_DRC_AGC_PAR_IOCTL:
		if(rt56xx.number < 4)
			break;
		set_drc_agc_parameters(codec, *(buf), *(buf + 1),
				*(buf + 2), *(buf + 3));
		break;

	case RT_SET_CODEC_DIGI_BOOST_GAIN_IOCTL:
		if(rt56xx.number < 2)
			break;
		set_digital_boost_gain(codec, *(buf), *(buf + 1));
		break;

	case RT_SET_CODEC_NOISE_GATE_IOCTL:
		if(rt56xx.number < 4)
			break;
		set_noise_gate(codec, *(buf), *(buf + 1),
				*(buf + 2), *(buf + 3));
		break;

	case RT_SET_CODEC_DRC_AGC_COMP_IOCTL:
		if(rt56xx.number < 2)
			break;
		set_drc_agc_compression(codec, *(buf), *(buf + 1));
		break;

	case RT_SET_CODEC_WNR_ENABLE_IOCTL:
		if (NULL == ioctl_ops->index_update_bits)
			break;

		ioctl_ops->index_update_bits(codec, RT5640_WND_1,
			RT5640_WND_MASK, *(buf) << RT5640_WND_SFT );
		break;

	case RT_GET_CODEC_DRC_AGC_ENABLE_IOCTL:
		if(rt56xx.number < 2)
			break;
		get_drc_agc_enable(codec, (buf), (buf + 1));
		if (copy_to_user(rt56xx.buf, buf, sizeof(*buf) * rt56xx.number))
			goto err;
		break;

	case RT_GET_CODEC_DRC_AGC_PAR_IOCTL:
		if(rt56xx.number < 4)
			break;
		get_drc_agc_parameters(codec, (buf), (buf + 1),
				(buf + 2), (buf + 3));
		if (copy_to_user(rt56xx.buf, buf,
			sizeof(*buf) * rt56xx.number))
			goto err;
		break;

	case RT_GET_CODEC_DIGI_BOOST_GAIN_IOCTL:
		if(rt56xx.number < 2)
			break;
		get_digital_boost_gain(codec, (buf), (buf + 1));
		if (copy_to_user(rt56xx.buf, buf,
			sizeof(*buf) * rt56xx.number))
			goto err;
		break;

	case RT_GET_CODEC_NOISE_GATE_IOCTL:
		if(rt56xx.number < 4)
			break;
		get_noise_gate(codec, (buf), (buf + 1), (buf + 2), (buf + 3));
		if (copy_to_user(rt56xx.buf, buf,
			sizeof(*buf) * rt56xx.number))
			goto err;
		break;

	case RT_GET_CODEC_DRC_AGC_COMP_IOCTL:
		if(rt56xx.number < 2)
			break;
		get_drc_agc_compression(codec, (buf), (buf + 1));
		if (copy_to_user(rt56xx.buf, buf,
			sizeof(*buf) * rt56xx.number))
			goto err;
		break;

	case RT_GET_CODEC_SPK_VOL_IOCTL:
		*buf = (snd_soc_read(codec, RT5640_SPK_VOL) & RT5640_L_VOL_MASK)
			>> RT5640_L_VOL_SFT;
		if (copy_to_user(rt56xx.buf, buf, sizeof(*buf) * rt56xx.number))
			goto err;
		break;

	case RT_GET_CODEC_MIC_GAIN_IOCTL:
		*buf = (snd_soc_read(codec, RT5640_IN1_IN2) & RT5640_BST_MASK1)
			>> RT5640_BST_SFT1;
		if (copy_to_user(rt56xx.buf, buf, sizeof(*buf) * rt56xx.number))
			goto err;
		break;
#if (CONFIG_SND_SOC_RT5642_MODULE | CONFIG_SND_SOC_RT5642)
	case RT_READ_CODEC_DSP_IOCTL:
	case RT_WRITE_CODEC_DSP_IOCTL:
	case RT_GET_CODEC_DSP_MODE_IOCTL:
		return rt56xx_dsp_ioctl_common(hw, file, cmd, arg);
#endif
	case RT_GET_CODEC_HWEQ_IOCTL:
	case RT_GET_CODEC_3D_SPK_IOCTL:
	case RT_GET_CODEC_MP3PLUS_IOCTL:
	case RT_GET_CODEC_3D_HEADPHONE_IOCTL:
	case RT_GET_CODEC_BASS_BACK_IOCTL:
	case RT_GET_CODEC_DIPOLE_SPK_IOCTL:
	default:
		break;
	}

	kfree(buf);
	return 0;

err:
	kfree(buf);
	return -EFAULT;
}
EXPORT_SYMBOL_GPL(rt5640_ioctl_common);
