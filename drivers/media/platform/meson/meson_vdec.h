/*
 * Meson video decoder driver
 *
 * Copyright (C) 2015 Endless Mobile, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#ifndef __MESON_VDEC_H__
#define __MESON_VDEC_H__ 

#include <linux/mutex.h>

#include <linux/amlogic/amports/vframe_receiver.h>
#include <linux/amlogic/ge2d/ge2d.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

struct vdec_dev {
	struct v4l2_device	v4l2_dev;
	struct vframe_receiver_s video_vf_receiver;

	struct video_device	*vfd;
	struct v4l2_m2m_dev	*m2m_dev;
	void			*vb_alloc_ctx;
	struct mutex		dev_mutex;

	void			*decoder_buf;
	phys_addr_t		decoder_buf_phys;

	ge2d_context_t		*ge2d_context;
	config_para_ex_t		ge2d_config;
};

int vdec_image_init(struct vdec_dev *dev);
void vdec_image_exit(struct vdec_dev *dev);

int vdec_process_image(struct vdec_dev *dev, struct vframe_s *vf,
		       struct vb2_buffer *dst, u32 bytesperline);

#endif
