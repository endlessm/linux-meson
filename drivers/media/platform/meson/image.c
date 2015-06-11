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
	canvas_t src_cs0, src_cs1, src_cs2;

	ge2d_config->alu_const_color = 0;
	ge2d_config->bitmask_en = 0;
	ge2d_config->src1_gb_alpha = 0;

	canvas_read(vf->canvas0Addr & 0xff, &src_cs0);
	canvas_read(vf->canvas0Addr >> 8 & 0xff, &src_cs1);
	canvas_read(vf->canvas0Addr >> 16 & 0xff, &src_cs2);
	ge2d_config->src_planes[0].addr = src_cs0.addr;
	ge2d_config->src_planes[0].w = src_cs0.width;
	ge2d_config->src_planes[0].h = src_cs0.height;
	ge2d_config->src_planes[1].addr = src_cs1.addr;
	ge2d_config->src_planes[1].w = src_cs1.width;
	ge2d_config->src_planes[1].h = src_cs1.height;
	ge2d_config->src_planes[2].addr = src_cs2.addr;
	ge2d_config->src_planes[2].w = src_cs2.width;
	ge2d_config->src_planes[2].h = src_cs2.height;

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

static int paint(ge2d_context_t *context, config_para_ex_t *ge2d_config, int dst_pixel_format, int* src_position, int* dst_paint_position, int* dst_plane_position)
{
	canvas_t dst_cd;

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

	canvas_read(DST_CANVAS_INDEX, &dst_cd);
	ge2d_config->dst_planes[0].addr = dst_cd.addr;
	ge2d_config->dst_planes[0].w = dst_cd.width;
	ge2d_config->dst_planes[0].h = dst_cd.height;
	ge2d_config->dst_para.format = dst_pixel_format | GE2D_LITTLE_ENDIAN;
	ge2d_config->dst_para.canvas_index = DST_CANVAS_INDEX;

	if (ge2d_context_config_ex(context, ge2d_config) < 0) {
		pr_err("Ge2d configing error.\n");
		return -1;
	}
	stretchblt_noalpha(context, src_position[0], src_position[1], src_position[2], src_position[3], dst_paint_position[0], dst_paint_position[1], dst_paint_position[2], dst_paint_position[3]);

	return 0;
}

void dst_canvas_config(struct vframe_s *vf, struct vb2_buffer *buf)
{
	dma_addr_t phys_addr = vb2_dma_contig_plane_dma_addr(buf, 0);

	canvas_config(DST_CANVAS_INDEX, phys_addr,
		      round_up(vf->width, WIDTH_ALIGN) * 4, vf->height,
		      CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_LINEAR);
}

int vdec_process_image(struct vdec_dev *dev, struct vframe_s *vf,
		       struct vb2_buffer *dst)
{
	int src_position[4];
	int dst_paint_position[4];
	int dst_plane_position[4];
	int dst_pixel_format = GE2D_FORMAT_S32_ABGR; // FIXME allow fmt selection

	v4l2_info(&dev->v4l2_dev, "Processing vf %d %p into vb2 buf %d\n",
		  vf->index, vf, dst->v4l2_buf.index);

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
	dst_canvas_config(vf, dst);

	paint(dev->ge2d_context, &dev->ge2d_config, dst_pixel_format, src_position, dst_paint_position, dst_plane_position);
	vb2_set_plane_payload(dst, 0, vf->height * vf->width * 4); // FIXME 32bpp
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
