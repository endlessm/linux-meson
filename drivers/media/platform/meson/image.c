/*
 * Meson video decoder image processing
 *
 * Copyright (C) 2015 Endless Mobile, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#include <linux/amlogic/amports/canvas.h>
#include <linux/amlogic/ge2d/ge2d.h>
#include <linux/amlogic/amports/vframe.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include <mach/mod_gate.h>

#include "meson_vdec.h"

#define DST_CANVAS_INDEX 0x70 /* ionvideo would use this */

static void src_config(struct vframe_s *vf, config_para_ex_t *ge2d_config)
{
	ge2d_config->alu_const_color = 0;
	ge2d_config->bitmask_en = 0;
	ge2d_config->src1_gb_alpha = 0;

	ge2d_config->src_key.key_enable = 0;
	ge2d_config->src_key.key_mask = 0;
	ge2d_config->src_key.key_mode = 0;
	ge2d_config->src_para.canvas_index = vf->canvas0Addr;
	ge2d_config->src_para.mem_type = CANVAS_TYPE_INVALID;
	ge2d_config->src_para.format = GE2D_FORMAT_M24_NV21; // FIXME interlace
	ge2d_config->src_para.fill_color_en = 0;
	ge2d_config->src_para.fill_mode = 0;
	ge2d_config->src_para.x_rev = 0;
	ge2d_config->src_para.y_rev = 0;
	ge2d_config->src_para.color = 0xffffffff;
	ge2d_config->src_para.top = 0;
	ge2d_config->src_para.left = 0;
	ge2d_config->src_para.height = vf->height;
	ge2d_config->src_para.width = vf->width;

	ge2d_config->src2_para.mem_type = CANVAS_TYPE_INVALID;
}

static int paint(ge2d_context_t *context, config_para_ex_t *ge2d_config, int dst_pixel_format, int dst_canvas_id, int* src_position, int* dst_paint_position, int* dst_plane_position)
{
	ge2d_config->dst_para.mem_type = CANVAS_TYPE_INVALID;
	ge2d_config->dst_para.fill_color_en = 0;
	ge2d_config->dst_para.fill_mode = 0;
	ge2d_config->dst_para.color = 0;
	ge2d_config->dst_para.top = dst_plane_position[0];
	ge2d_config->dst_para.left = dst_plane_position[1];
	ge2d_config->dst_para.width = dst_plane_position[2];
	ge2d_config->dst_para.height = dst_plane_position[3];
	ge2d_config->dst_para.x_rev = 0;
	ge2d_config->dst_para.y_rev = 0;
	ge2d_config->dst_xy_swap = 0;

	ge2d_config->dst_para.format = dst_pixel_format | GE2D_LITTLE_ENDIAN;
	ge2d_config->dst_para.canvas_index = dst_canvas_id;

	if (ge2d_context_config_ex(context, ge2d_config) < 0) {
		pr_err("Ge2d configing error.\n");
		return -1;
	}
	stretchblt_noalpha(context, src_position[0], src_position[1], src_position[2], src_position[3], dst_paint_position[0], dst_paint_position[1], dst_paint_position[2], dst_paint_position[3]);

	return 0;
}

static int dst_canvas_config(struct vframe_s *vf, struct vb2_buffer *buf,
			      u32 pixelformat, unsigned int plane0stride)
{
	if (pixelformat == V4L2_PIX_FMT_NV12M) {
		canvas_config(DST_CANVAS_INDEX,
			      vb2_dma_contig_plane_dma_addr(buf, 0),
			      plane0stride, vf->height,
			      CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_LINEAR);
		canvas_config(DST_CANVAS_INDEX + 1,
			      vb2_dma_contig_plane_dma_addr(buf, 1),
			      plane0stride, vf->height >> 1,
			      CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_LINEAR);
		return DST_CANVAS_INDEX | ((DST_CANVAS_INDEX + 1) << 8);
	} else if (pixelformat == V4L2_PIX_FMT_BGR32) {
		canvas_config(DST_CANVAS_INDEX,
			      vb2_dma_contig_plane_dma_addr(buf, 0),
			      plane0stride, vf->height,
			      CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_LINEAR);
		return DST_CANVAS_INDEX;
	}
	return 0;
}

int vdec_process_image(struct vdec_dev *dev, struct vframe_s *vf,
		       struct vb2_buffer *dst, u32 pixelformat,
		       unsigned int plane0stride)
{
	int src_position[4];
	int dst_paint_position[4];
	int dst_plane_position[4];
	int dst_pixel_format;
	int dst_canvas_id;

	dev_dbg(dev->v4l2_dev.dev, "Processing vf %d %p into vb2 buf %d\n",
		  vf->index, vf, dst->v4l2_buf.index);

	switch (pixelformat) {
	case V4L2_PIX_FMT_BGR32:
		dst_pixel_format = GE2D_FORMAT_S32_ARGB;
		break;
	case V4L2_PIX_FMT_NV12M:
		dst_pixel_format = GE2D_FORMAT_M24_NV12;
		break;
	default:
		v4l2_err(&dev->v4l2_dev, "Unrecognised format %x\n",
			 pixelformat);
		return -EINVAL;
	}

	src_position[0] = 0;
	src_position[1] = 0;
	src_position[2] = vf->width;
	src_position[3] = vf->height;
	if (src_position[2] == 0 || src_position[3] == 0) {
		pr_err("bad src position params\n");
		return -1;
	}

	dst_plane_position[0] = 0;
	dst_plane_position[1] = 0;
	dst_plane_position[2] = vf->width;
	dst_plane_position[3] = vf->height;
	dst_paint_position[0] = dst_plane_position[0];
	dst_paint_position[1] = dst_plane_position[1];
	dst_paint_position[2] = dst_plane_position[2];
	dst_paint_position[3] = dst_plane_position[3];

	src_config(vf, &dev->ge2d_config);
	dst_canvas_id = dst_canvas_config(vf, dst, pixelformat, plane0stride);

	paint(dev->ge2d_context, &dev->ge2d_config, dst_pixel_format,
	      dst_canvas_id, src_position, dst_paint_position,
	      dst_plane_position);

	return 0;
}

int vdec_image_init(struct vdec_dev *dev)
{
	dev->ge2d_context = create_ge2d_work_queue();
	if (!dev->ge2d_context)
		return -EINVAL;

	switch_mod_gate_by_name("ge2d", 1);
	return 0;
}

void vdec_image_exit(struct vdec_dev *dev)
{
	switch_mod_gate_by_name("ge2d", 0);
	destroy_ge2d_work_queue(dev->ge2d_context);
}
