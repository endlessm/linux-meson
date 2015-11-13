/*
 * Meson video decoder driver
 *
 * Copyright (C) 2015 Endless Mobile, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <linux/amlogic/amstream.h>
#include <linux/amlogic/amports/ptsserv.h>
#include <linux/amlogic/amports/vformat.h>
#include <linux/amlogic/amports/vframe_provider.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "meson_vdec.h"
#include "amlglue.h"

#define DRIVER_NAME "meson-vdec"
#define RECEIVER_NAME "m2m"

// FIXME tweak or make dynamic?
#define VDEC_MAX_BUFFERS		32

/* This is used to store the encoded data */
#define VDEC_ST_FIFO_SIZE	(3*1024*1024)

/* This is the default encoded v4l2_buffer size */
#define VDEC_ENCODED_BUF_SIZE   (1*1024*1024)

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-1)");

struct vdec_buf {
	struct list_head list;
	struct vb2_buffer *buf;
};

#define EOS_TAIL_BUF_SIZE 1024
static const u8 eos_tail_data[] = {
	0x00, 0x00, 0x00, 0x01, 0x06, 0x05, 0xff, 0xe4, 0xdc, 0x45, 0xe9, 0xbd, 0xe6, 0xd9, 0x48, 0xb7,
	0x96, 0x2c, 0xd8, 0x20, 0xd9, 0x23, 0xee, 0xef, 0x78, 0x32, 0x36, 0x34, 0x20, 0x2d, 0x20, 0x63,
	0x6f, 0x72, 0x65, 0x20, 0x36, 0x37, 0x20, 0x72, 0x31, 0x31, 0x33, 0x30, 0x20, 0x38, 0x34, 0x37,
	0x35, 0x39, 0x37, 0x37, 0x20, 0x2d, 0x20, 0x48, 0x2e, 0x32, 0x36, 0x34, 0x2f, 0x4d, 0x50, 0x45,
	0x47, 0x2d, 0x34, 0x20, 0x41, 0x56, 0x43, 0x20, 0x63, 0x6f, 0x64, 0x65, 0x63, 0x20, 0x2d, 0x20,
	0x43, 0x6f, 0x70, 0x79, 0x6c, 0x65, 0x66, 0x74, 0x20, 0x32, 0x30, 0x30, 0x33, 0x2d, 0x32, 0x30,
	0x30, 0x39, 0x20, 0x2d, 0x20, 0x68, 0x74, 0x74, 0x70, 0x3a, 0x2f, 0x2f, 0x77, 0x77, 0x77, 0x2e,
	0x76, 0x69, 0x64, 0x65, 0x6f, 0x6c, 0x61, 0x6e, 0x2e, 0x6f, 0x72, 0x67, 0x2f, 0x78, 0x32, 0x36,
	0x34, 0x2e, 0x68, 0x74, 0x6d, 0x6c, 0x20, 0x2d, 0x20, 0x6f, 0x70, 0x74, 0x69, 0x6f, 0x6e, 0x73,
	0x3a, 0x20, 0x63, 0x61, 0x62, 0x61, 0x63, 0x3d, 0x31, 0x20, 0x72, 0x65, 0x66, 0x3d, 0x31, 0x20,
	0x64, 0x65, 0x62, 0x6c, 0x6f, 0x63, 0x6b, 0x3d, 0x31, 0x3a, 0x30, 0x3a, 0x30, 0x20, 0x61, 0x6e,
	0x61, 0x6c, 0x79, 0x73, 0x65, 0x3d, 0x30, 0x78, 0x31, 0x3a, 0x30, 0x78, 0x31, 0x31, 0x31, 0x20,
	0x6d, 0x65, 0x3d, 0x68, 0x65, 0x78, 0x20, 0x73, 0x75, 0x62, 0x6d, 0x65, 0x3d, 0x36, 0x20, 0x70,
	0x73, 0x79, 0x5f, 0x72, 0x64, 0x3d, 0x31, 0x2e, 0x30, 0x3a, 0x30, 0x2e, 0x30, 0x20, 0x6d, 0x69,
	0x78, 0x65, 0x64, 0x5f, 0x72, 0x65, 0x66, 0x3d, 0x30, 0x20, 0x6d, 0x65, 0x5f, 0x72, 0x61, 0x6e,
	0x67, 0x65, 0x3d, 0x31, 0x36, 0x20, 0x63, 0x68, 0x72, 0x6f, 0x6d, 0x61, 0x5f, 0x6d, 0x65, 0x3d,
	0x31, 0x20, 0x74, 0x72, 0x65, 0x6c, 0x6c, 0x69, 0x73, 0x3d, 0x30, 0x20, 0x38, 0x78, 0x38, 0x64,
	0x63, 0x74, 0x3d, 0x30, 0x20, 0x63, 0x71, 0x6d, 0x3d, 0x30, 0x20, 0x64, 0x65, 0x61, 0x64, 0x7a,
	0x6f, 0x6e, 0x65, 0x3d, 0x32, 0x31, 0x2c, 0x31, 0x31, 0x20, 0x63, 0x68, 0x72, 0x6f, 0x6d, 0x61,
	0x5f, 0x71, 0x70, 0x5f, 0x6f, 0x66, 0x66, 0x73, 0x65, 0x74, 0x3d, 0x2d, 0x32, 0x20, 0x74, 0x68,
	0x72, 0x65, 0x61, 0x64, 0x73, 0x3d, 0x31, 0x20, 0x6e, 0x72, 0x3d, 0x30, 0x20, 0x64, 0x65, 0x63,
	0x69, 0x6d, 0x61, 0x74, 0x65, 0x3d, 0x31, 0x20, 0x6d, 0x62, 0x61, 0x66, 0x66, 0x3d, 0x30, 0x20,
	0x62, 0x66, 0x72, 0x61, 0x6d, 0x65, 0x73, 0x3d, 0x30, 0x20, 0x6b, 0x65, 0x79, 0x69, 0x6e, 0x74,
	0x3d, 0x32, 0x35, 0x30, 0x20, 0x6b, 0x65, 0x79, 0x69, 0x6e, 0x74, 0x5f, 0x6d, 0x69, 0x6e, 0x3d,
	0x32, 0x35, 0x20, 0x73, 0x63, 0x65, 0x6e, 0x65, 0x63, 0x75, 0x74, 0x3d, 0x34, 0x30, 0x20, 0x72,
	0x63, 0x3d, 0x61, 0x62, 0x72, 0x20, 0x62, 0x69, 0x74, 0x72, 0x61, 0x74, 0x65, 0x3d, 0x31, 0x30,
	0x20, 0x72, 0x61, 0x74, 0x65, 0x74, 0x6f, 0x6c, 0x3d, 0x31, 0x2e, 0x30, 0x20, 0x71, 0x63, 0x6f,
	0x6d, 0x70, 0x3d, 0x30, 0x2e, 0x36, 0x30, 0x20, 0x71, 0x70, 0x6d, 0x69, 0x6e, 0x3d, 0x31, 0x30,
	0x20, 0x71, 0x70, 0x6d, 0x61, 0x78, 0x3d, 0x35, 0x31, 0x20, 0x71, 0x70, 0x73, 0x74, 0x65, 0x70,
	0x3d, 0x34, 0x20, 0x69, 0x70, 0x5f, 0x72, 0x61, 0x74, 0x69, 0x6f, 0x3d, 0x31, 0x2e, 0x34, 0x30,
	0x20, 0x61, 0x71, 0x3d, 0x31, 0x3a, 0x31, 0x2e, 0x30, 0x30, 0x00, 0x80, 0x00, 0x00, 0x00, 0x01,
	0x67, 0x4d, 0x40, 0x0a, 0x9a, 0x74, 0xf4, 0x20, 0x00, 0x00, 0x03, 0x00, 0x20, 0x00, 0x00, 0x06,
	0x51, 0xe2, 0x44, 0xd4, 0x00, 0x00, 0x00, 0x01, 0x68, 0xee, 0x32, 0xc8, 0x00, 0x00, 0x00, 0x01,
	0x65, 0x88, 0x80, 0x20, 0x00, 0x08, 0x7f, 0xea, 0x6a, 0xe2, 0x99, 0xb6, 0x57, 0xae, 0x49, 0x30,
	0xf5, 0xfe, 0x5e, 0x46, 0x0b, 0x72, 0x44, 0xc4, 0xe1, 0xfc, 0x62, 0xda, 0xf1, 0xfb, 0xa2, 0xdb,
	0xd6, 0xbe, 0x5c, 0xd7, 0x24, 0xa3, 0xf5, 0xb9, 0x2f, 0x57, 0x16, 0x49, 0x75, 0x47, 0x77, 0x09,
	0x5c, 0xa1, 0xb4, 0xc3, 0x4f, 0x60, 0x2b, 0xb0, 0x0c, 0xc8, 0xd6, 0x66, 0xba, 0x9b, 0x82, 0x29,
	0x33, 0x92, 0x26, 0x99, 0x31, 0x1c, 0x7f, 0x9b
};

enum eos_state {
	EOS_NONE,
	EOS_TAIL_WAITING,
	EOS_TAIL_SENT,
	EOS_WAIT_IDLE,
	EOS_DETECTED,
};

static struct vdec_fmt {
	const char *name;
	u32 pixelformat;
} formats[] = {
	{
		.name = "NV12",
		.pixelformat = V4L2_PIX_FMT_NV12M,
	},
	{
		.name = "BGRA",
		.pixelformat = V4L2_PIX_FMT_BGR32,
	},
};

struct vdec_ctx {
	struct v4l2_fh		fh;
	struct vdec_dev	*dev;
	struct v4l2_m2m_ctx	*m2m_ctx;
	struct task_struct *image_thread;
	struct vdec_fmt *fmt;
	struct timer_list eos_idle_timer;
	struct vframe_receiver_s vf_receiver;

	void *buf_vaddr;

	/* Protects concurrent access to:
	 * eos_state and its relationship with v4l2_m2m source buf queue
	 * esparser_busy
	 */
	spinlock_t data_lock;

	int src_bufs_size;
	int src_bufs_cnt;
	int dst_bufs_cnt;
	struct vdec_buf src_bufs[VDEC_MAX_BUFFERS];
	struct vdec_buf dst_bufs[VDEC_MAX_BUFFERS];

	/* EOS handling */
	enum eos_state eos_state;
	void *eos_tail_buf;
	phys_addr_t eos_tail_buf_phys;

	/* Source and destination queue data */
	bool esparser_busy;
	size_t parsed_len;
	bool src_streaming;
	bool dst_streaming;
	u32 frame_width;
	u32 frame_height;
	wait_queue_head_t esparser_queue;
};

struct buffer_size_info {
	unsigned int num_planes;
	unsigned int plane_stride[2];
	unsigned int buffer_size[2];
};

static void get_buffer_size_info(struct vdec_fmt *fmt, u32 width, u32 height,
			struct buffer_size_info *i)
{
	/* GE2D can only work with output buffers with certain aligned widths */

	switch (fmt->pixelformat) {
	case V4L2_PIX_FMT_BGR32:
		i->num_planes = 1;
		i->plane_stride[0] = round_up(width, 8) * 4;
		i->buffer_size[0] = i->plane_stride[0] * height;
		break;
	case V4L2_PIX_FMT_NV12M:
		i->num_planes = 2;
		i->plane_stride[0] = i->plane_stride[1] = round_up(width, 32);
		i->buffer_size[0] = i->plane_stride[0] * height;
		i->buffer_size[1] = i->plane_stride[1] *
				    (round_up(height, 2) / 2);
		break;
	default:
		memset(i, 0, sizeof(*i));
	}
}

static void configure_v4l2_plane_fmt(struct vdec_ctx *ctx,
				     struct v4l2_pix_format_mplane *mp)
{
	struct buffer_size_info buf_info;

	mp->pixelformat = ctx->fmt->pixelformat;
	if (ctx->frame_width)
		mp->width = ctx->frame_width;
        if (ctx->frame_height)
		mp->height = ctx->frame_height;
	mp->field = V4L2_FIELD_NONE;

	get_buffer_size_info(ctx->fmt, mp->width, mp->height, &buf_info);
	mp->num_planes = buf_info.num_planes;
	mp->plane_fmt[0].bytesperline = buf_info.plane_stride[0];
	mp->plane_fmt[1].bytesperline = buf_info.plane_stride[1];
	mp->plane_fmt[0].sizeimage = buf_info.buffer_size[0];
	mp->plane_fmt[1].sizeimage = buf_info.buffer_size[1];
}

static inline struct vdec_ctx *file2ctx(struct file *file)
{
	return container_of(file->private_data, struct vdec_ctx, fh);
}

static void mark_all_dst_done(struct vdec_ctx *ctx)
{
	struct vb2_buffer *buf;

	while ((buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx))) {
		vb2_set_plane_payload(buf, 0, 0);
		vb2_set_plane_payload(buf, 1, 0);
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_DONE);
	}
}

/*
 * EOS handling
 *
 * The hardware doesn't make this easy for us. First, the decoder seems to
 * hold onto the final few frames. When userspace drains the decoder at EOF
 * on the OUTPUT side via V4L2_DEC_CMD_STOP, we send a special tail
 * sequence just like Amlogic's libplayer does. Then the final frames arrive.
 *
 * Now we are tasked with detecting which of the frames is the last one,
 * of which we get no strong indication. The VIFIFO becomes empty several
 * frames before the final one is presented by the decoder. We resort to
 * time-based polling to try to understand when the decoder has truly
 * finished. We try to find a 100ms period where these conditions are met:
 * 1. VIFIFO is empty
 * 2. Decoder is not paused because it was starved of output buffers
 * 3. No new output buffers were made available during this time
 * 4. No new frames were presented during this time
 *
 * The caller should hold data_lock.
 */
static void send_eos_tail(struct vdec_ctx *ctx)
{
	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev,
		 "sending EOS tail to esparser\n");
	ctx->eos_state = EOS_TAIL_SENT;
	ctx->esparser_busy = true;
	esparser_start_search(PARSER_VIDEO, ctx->eos_tail_buf_phys,
			      EOS_TAIL_BUF_SIZE);
}

static void run_eos_idle_timer(struct vdec_ctx *ctx)
{
	mod_timer(&ctx->eos_idle_timer, jiffies + (HZ / 10));
}

static void eos_check_idle(unsigned long arg)
{
	struct vdec_ctx *ctx = (struct vdec_ctx *) arg;

	if (!ctx->src_streaming)
		return;

	/* The VIFIFO level does not reach 0 at the end of playback, it
	 * always seems to have a small amount of data there which does not
	 * get flushed. So we use a low threshold to detect VIFIFO empty.
	 * FIXME: experiment and check that 256 bytes is the upper limit here
	 * before the data is actually flushed. */
	if (vf_peek(RECEIVER_NAME) ||
	    READ_VREG(VLD_MEM_VIFIFO_LEVEL) > 256 ||
	    vh264_output_is_starved()) {
		run_eos_idle_timer(ctx);
		return;
	}

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "EOS detected\n");
	ctx->eos_state = EOS_DETECTED;
	mark_all_dst_done(ctx);
}

/* Try to send the next source buffer to the parser.
 * We first check that there is a source buffer available, and that the
 * parser is not already busy.
 *
 * Caller should hold data_lock.
 */
static void parse_next_buffer(struct vdec_ctx *ctx)
{
	struct vb2_buffer *buf;
	dma_addr_t phys_addr;
	unsigned long size;
	uint64_t ts;

	if (ctx->esparser_busy || !ctx->src_streaming)
		return;

	buf = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	if (!buf)
		return;

	phys_addr = vb2_dma_contig_plane_dma_addr(buf, 0);
	size = vb2_get_plane_payload(buf, 0);

	/* Legacy draining was triggered passing an empty buffer */
	if (size == 0) {
		if (ctx->eos_state >= EOS_TAIL_WAITING) {
			v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_DONE);
		} else {
			send_eos_tail(ctx);
		}
		return;
	}

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev,
		 "send src buffer %d to parser, phys addr %x size %ld\n",
		 buf->v4l2_buf.index, phys_addr, size);

	ctx->esparser_busy = true;

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev,
		 "parsing at pts %li.%lis offset %x\n",
		 buf->v4l2_buf.timestamp.tv_sec,
		 buf->v4l2_buf.timestamp.tv_usec, ctx->parsed_len);

	ts = (uint64_t)buf->v4l2_buf.timestamp.tv_sec * 1000000 +
		(uint64_t)buf->v4l2_buf.timestamp.tv_usec;

	pts_checkin_offset_us64(PTS_TYPE_VIDEO, ctx->parsed_len, ts);
	ctx->parsed_len += size;
	esparser_start_search(PARSER_VIDEO, phys_addr, size);
}

/*
 * Amlogic callbacks
 */
static void parser_cb(void *data)
{
	struct vdec_ctx *ctx = data;
	struct vb2_buffer *src_buf;

	/* Compressed video has been parsed into the decoder FIFO, so we can
	 * return this buffer to userspace. */
	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev,
		 "esparser reports completion\n");

	spin_lock(&ctx->data_lock);
	ctx->esparser_busy = false;
	src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	if (src_buf)
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);

	if (ctx->eos_state == EOS_TAIL_WAITING &&
	    v4l2_m2m_num_src_bufs_ready(ctx->m2m_ctx) == 0) {
		send_eos_tail(ctx);
	} else if (ctx->eos_state == EOS_TAIL_SENT) {
		ctx->eos_state = EOS_WAIT_IDLE;
		run_eos_idle_timer(ctx);
	} else {
		parse_next_buffer(ctx);
	}
	spin_unlock(&ctx->data_lock);
}

// FIXME should be a VF message
static void h264_params_cb(void *data, int status, u32 width, u32 height)
{
	struct vdec_ctx *ctx = data;
	const struct v4l2_event ev = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev,
		 "h264_params_cb status=%d w=%d h=%d\n",
		 status, width, height);

	if (status) {
		v4l2_err(&ctx->dev->v4l2_dev, "H264 params error.\n");
		return;
	}

	ctx->frame_width = width;
	ctx->frame_height = height;
	v4l2_event_queue_fh(&ctx->fh, &ev);
	wake_up(&ctx->esparser_queue);
}

/*
 * m2m operations
 */
static void device_run(void *priv)
{
	/* No-op. This mechanism requires both source and dest queues to be
	 * active, but in order to parse the header, we need to feed data to
	 * the device when the source queue is active, before the dest queue
	 * is set up.
	 * Instead of using device_run, we feed data to the hardware whenever
	 * the source queue is active. */
}

static void job_abort(void *priv)
{
	struct vdec_ctx *ctx = priv;
	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "job_abort\n");
	v4l2_m2m_job_finish(ctx->dev->m2m_dev, ctx->m2m_ctx);
}

static void vdec_lock(void *priv)
{
	struct vdec_ctx *ctx = priv;
	struct vdec_dev *dev = ctx->dev;
	mutex_lock(&dev->dev_mutex);
}

static void vdec_unlock(void *priv)
{
	struct vdec_ctx *ctx = priv;
	struct vdec_dev *dev = ctx->dev;
	mutex_unlock(&dev->dev_mutex);
}

static struct v4l2_m2m_ops m2m_ops = {
	.device_run	= device_run,
	.job_abort	= job_abort,
	.lock		= vdec_lock,
	.unlock		= vdec_unlock,
};

/*
 * video ioctls
 */
static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct vdec_ctx *ctx = file2ctx(file);
	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "ioc_querycap\n");

	strncpy(cap->driver, DRIVER_NAME, sizeof(cap->driver) - 1);
	strncpy(cap->card, DRIVER_NAME, sizeof(cap->card) - 1);
	snprintf(cap->bus_info, sizeof(cap->bus_info),
			"platform:%s", DRIVER_NAME);
	cap->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	struct vdec_ctx *ctx = file2ctx(file);
	struct vdec_fmt *fmt;

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "enum_fmt_vid_cap\n");

	if (f->index >= ARRAY_SIZE(formats))
		return -EINVAL;

	fmt = &formats[f->index];
	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->pixelformat;
	return 0;
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	struct vdec_ctx *ctx = file2ctx(file);
	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "enum_fmt_vid_out\n");

        if (f->index != 0)
		return -EINVAL;

	snprintf(f->description, sizeof(f->description), "H264");
	f->pixelformat = V4L2_PIX_FMT_H264;
	return 0;
}

static int vidioc_g_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct vdec_ctx *ctx = file2ctx(file);
	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "g_fmt_vid_out\n");

	f->fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	f->fmt.pix_mp.width = f->fmt.pix_mp.height = 0;
	f->fmt.pix_mp.field = V4L2_FIELD_NONE;
	f->fmt.pix_mp.num_planes = 1;
	f->fmt.pix_mp.plane_fmt[0].bytesperline = 0; // FIXME should be buffer size
	f->fmt.pix_mp.plane_fmt[0].sizeimage = 0; // FIXME should be buffer size
	return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct vdec_ctx *ctx = file2ctx(file);

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "g_fmt_vid_cap\n");

	wait_event_interruptible_timeout(ctx->esparser_queue,
		ctx->frame_width != 0, HZ/5);

        if (ctx->frame_width == 0 || ctx->frame_height == 0)
		return -EINVAL;

	configure_v4l2_plane_fmt(ctx, &f->fmt.pix_mp);
	return 0;
}

static int try_fmt_vid_cap(struct vdec_ctx *ctx, struct v4l2_format *f,
			   bool set)
{
	struct vdec_fmt *fmt = &formats[0];
	int i;

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "ioc_try_fmt_vid_cap\n");

	/* V4L2 specification suggests the driver corrects the format struct
	* if any of the dimensions is unsupported */

	for (i = 0; i < ARRAY_SIZE(formats); i++) {
		if (formats[i].pixelformat == f->fmt.pix_mp.pixelformat)
			fmt = &formats[i];
	}

	if (set)
		ctx->fmt = fmt;

	configure_v4l2_plane_fmt(ctx, &f->fmt.pix_mp);
	return 0;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct vdec_ctx *ctx = file2ctx(file);

	v4l2_info(&ctx->dev->v4l2_dev, "ioc_try_fmt_vid_cap\n");

	return try_fmt_vid_cap(ctx, f, false);
}

static int vidioc_try_fmt_vid_out(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct vdec_ctx *ctx = file2ctx(file);

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "ioc_try_fmt_vid_out\n");

	f->fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	f->fmt.pix_mp.num_planes = 1;
	f->fmt.pix_mp.plane_fmt[0].bytesperline = 0;

        if (f->fmt.pix_mp.plane_fmt[0].sizeimage == 0)
		f->fmt.pix_mp.plane_fmt[0].sizeimage = VDEC_ENCODED_BUF_SIZE;

	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct vdec_ctx *ctx = file2ctx(file);
	struct vb2_queue *vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev,
		 "ioc_s_fmt_vid_cap type=%d\n", f->type);

	if (!vq)
		return -EINVAL;

	if (vb2_is_busy(vq)) {
		v4l2_err(&ctx->dev->v4l2_dev, "%s queue busy\n", __func__);
		return -EBUSY;
	}

	return try_fmt_vid_cap(ctx, f, true);
}

static int vidioc_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct vdec_ctx *ctx = file2ctx(file);
	int ret;

	ret = vidioc_try_fmt_vid_out (file, priv, f);

	if (ret == 0)
		ctx->src_bufs_size = f->fmt.pix_mp.plane_fmt[0].sizeimage;

        return ret;
}

static int vidioc_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *reqbufs)
{
	struct vdec_ctx *ctx = file2ctx(file);

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev,
		 "reqbufs type %d\n", reqbufs->type);

	if (reqbufs->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		ctx->src_bufs_cnt = 0;
	else
		ctx->dst_bufs_cnt = 0;

	return v4l2_m2m_reqbufs(file, ctx->m2m_ctx, reqbufs);
}

static int vidioc_querybuf(struct file *file, void *priv,
			   struct v4l2_buffer *buf)
{
	struct vdec_ctx *ctx = file2ctx(file);
	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev,
		 "ioc_querybuf %d\n", buf->index);

	return v4l2_m2m_querybuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_expbuf(struct file *file, void *priv,
			   struct v4l2_exportbuffer *buf)
{
	struct vdec_ctx *ctx = file2ctx(file);
	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "ioc_expbuf %d\n", buf->index);

	return v4l2_m2m_expbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct vdec_ctx *ctx = file2ctx(file);
	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev,
		 "ioc_qbuf %d %d\n", buf->index, buf->m.planes[0].bytesused);
	return v4l2_m2m_qbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct vdec_ctx *ctx = file2ctx(file);
	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev,
		 "ioc_dqbuf %d\n", buf->index);

	return v4l2_m2m_dqbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_streamon(struct file *file, void *priv,
			   enum v4l2_buf_type type)
{
	struct vdec_ctx *ctx = file2ctx(file);
	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "streamon type=%d\n", type);

	return v4l2_m2m_streamon(file, ctx->m2m_ctx, type);
}

static int vidioc_streamoff(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	struct vdec_ctx *ctx = file2ctx(file);
	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "streamoff type=%d\n", type);

	return v4l2_m2m_streamoff(file, ctx->m2m_ctx, type);
}

static int vidioc_subscribe_event(struct v4l2_fh *fh,
				  const struct  v4l2_event_subscription *sub)
{
	v4l2_dbg(1, debug, fh->vdev->v4l2_dev,
		 "subscribe event %d\n", sub->type);

	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	default:
		v4l2_err(fh->vdev->v4l2_dev, "unknown event subscribe %d\n",
			 sub->type);
		return -EINVAL;
	}
}

static int vidioc_decoder_cmd(struct file *file, void *priv,
			      struct v4l2_decoder_cmd *cmd)
{
	struct vdec_ctx *ctx = file2ctx(file);

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "decoder cmd %d\n", cmd->cmd);

	if (cmd->cmd != V4L2_DEC_CMD_STOP)
		return -EINVAL;

	/* Already stopping? */
	if (ctx->eos_state >= EOS_TAIL_WAITING)
		return -EINVAL;

	spin_lock_irq(&ctx->data_lock);
	if (v4l2_m2m_num_src_bufs_ready(ctx->m2m_ctx) > 0)
		ctx->eos_state = EOS_TAIL_WAITING;
	else
		send_eos_tail(ctx);
	spin_unlock_irq(&ctx->data_lock);

	return 0;
}

static const struct v4l2_ioctl_ops vdec_ioctl_ops = {
	.vidioc_querycap	= vidioc_querycap,

	.vidioc_enum_fmt_vid_cap_mplane = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap_mplane	= vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap_mplane	= vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap_mplane	= vidioc_s_fmt_vid_cap,

	.vidioc_enum_fmt_vid_out_mplane = vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out_mplane	= vidioc_g_fmt_vid_out,
	.vidioc_try_fmt_vid_out_mplane	= vidioc_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out_mplane	= vidioc_s_fmt_vid_out,

	.vidioc_reqbufs		= vidioc_reqbufs,
	.vidioc_querybuf	= vidioc_querybuf,
	.vidioc_expbuf		= vidioc_expbuf,

	.vidioc_qbuf		= vidioc_qbuf,
	.vidioc_dqbuf		= vidioc_dqbuf,

	.vidioc_streamon	= vidioc_streamon,
	.vidioc_streamoff	= vidioc_streamoff,
	.vidioc_subscribe_event = vidioc_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_decoder_cmd = vidioc_decoder_cmd,
};

/*
 * Queue operations
 */
static int vdec_src_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct vdec_ctx *ctx = vb2_get_drv_priv(vq);
	stream_port_t *port = amstream_find_port("amstream_vbuf");
	stream_buf_t *sbuf = get_buf_by_type(BUF_TYPE_VIDEO);
	unsigned long flags;
	int ret;

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "src_start_streaming\n");

	amstream_port_open(port);
	/* enable sync_outside and pts_outside */
	amstream_dec_info.param = (void *) 0x3;

	port->vformat = VFORMAT_H264;
	port->flag |= PORT_FLAG_VFORMAT;
	ret = video_port_init(port, sbuf);
	if (ret) {
		amstream_port_release(amstream_find_port("amstream_vbuf"));
		return ret;
	}

	ctx->frame_width = ctx->frame_height = 0;
	ctx->parsed_len = 0;
	kthread_unpark(ctx->image_thread);
	ctx->src_streaming = true;

	/* We stream source buffers regardless of dest queue state. */
	spin_lock_irqsave(&ctx->data_lock, flags);
	parse_next_buffer(ctx);
	spin_unlock_irqrestore(&ctx->data_lock, flags);

	return 0;
}

static int vdec_src_stop_streaming(struct vb2_queue *vq)
{
	struct vdec_ctx *ctx = vb2_get_drv_priv(vq);
	ctx->src_streaming = false;
	kthread_park(ctx->image_thread);
	del_timer_sync(&ctx->eos_idle_timer);

	/* FIXME: calls video_port_release internally, which means the API
	 * is not really symmetrical with the init routines. */
	amstream_port_release(amstream_find_port("amstream_vbuf"));
	ctx->esparser_busy = false;
	ctx->eos_state = EOS_NONE;

	return 0;
}

static int vdec_dst_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct vdec_ctx *ctx = vb2_get_drv_priv(vq);
	ctx->dst_streaming = true;
	return 0;
}

static int vdec_dst_stop_streaming(struct vb2_queue *vq)
{
	struct vdec_ctx *ctx = vb2_get_drv_priv(vq);
	ctx->dst_streaming = false;
	mark_all_dst_done(ctx);
	return 0;
}

static int vdec_src_queue_setup(struct vb2_queue *vq,
				const struct v4l2_format *fmt,
				unsigned int *nbuffers,
				unsigned int *nplanes,
				unsigned int sizes[], void *alloc_ctxs[])
{
	struct vdec_ctx *ctx = vb2_get_drv_priv(vq);

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "queue_setup_output\n");

	*nplanes = 1;

	sizes[0] = ctx->src_bufs_size;

	alloc_ctxs[0] = ctx->dev->vb_alloc_ctx;

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev,
		  "get %d buffer(s) of size %d each.\n", *nplanes, sizes[0]);

	return 0;
}

static int vdec_dst_queue_setup(struct vb2_queue *vq,
				const struct v4l2_format *fmt,
				unsigned int *nbuffers,
				unsigned int *nplanes,
				unsigned int sizes[], void *alloc_ctxs[])
{
	struct vdec_ctx *ctx = vb2_get_drv_priv(vq);
	struct buffer_size_info buf_info;

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "queue_setup_capture\n");

	get_buffer_size_info(ctx->fmt, ctx->frame_width, ctx->frame_height,
			     &buf_info);

	*nplanes = buf_info.num_planes;
	sizes[0] = buf_info.buffer_size[0];
	sizes[1] = buf_info.buffer_size[1];
	alloc_ctxs[0] = ctx->dev->vb_alloc_ctx;
	alloc_ctxs[1] = ctx->dev->vb_alloc_ctx;

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev,
		  "get %d capture buffer(s).\n", *nbuffers);

	return 0;
}

static int vdec_src_buf_init(struct vb2_buffer *vb)
{
	struct vdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	int i = vb->v4l2_buf.index;
	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "src_buf_init %d\n", i);

	ctx->src_bufs[i].buf = vb;
	ctx->src_bufs_cnt++;
	return 0;
}

static int vdec_dst_buf_init(struct vb2_buffer *vb)
{
	struct vdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	int i = vb->v4l2_buf.index;
	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "dst_buf_init %d\n", i);

	ctx->dst_bufs[i].buf = vb;
	ctx->dst_bufs_cnt++;
	return 0;
}

// user space queues the buffer
static int vdec_dst_buf_prepare(struct vb2_buffer *vb)
{
	struct vdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev,
		 "buf_prepare %d\n", vb->v4l2_buf.index);
	return 0;
}

// pass buffer ownership to driver, ready for IO
static void vdec_src_buf_queue(struct vb2_buffer *vb)
{
	struct vdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long flags;

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev,
		 "src_buf_queue %d\n", vb->v4l2_buf.index);
	v4l2_m2m_buf_queue(ctx->m2m_ctx, vb);

	spin_lock_irqsave(&ctx->data_lock, flags);
	parse_next_buffer(ctx);
	spin_unlock_irqrestore(&ctx->data_lock, flags);
}

static void vdec_dst_buf_queue(struct vb2_buffer *vb)
{
	struct vdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev,
		 "dst_buf_queue %d\n", vb->v4l2_buf.index);

	if (ctx->eos_state == EOS_DETECTED) {
		vb2_set_plane_payload(vb, 0, 0);
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_DONE);
		return;
	}

	v4l2_m2m_buf_queue(ctx->m2m_ctx, vb);

	if (ctx->eos_state == EOS_WAIT_IDLE)
		run_eos_idle_timer(ctx);

	wake_up_process(ctx->image_thread);
}

static void vdec_wait_prepare(struct vb2_queue *q)
{
	struct vdec_ctx *ctx = vb2_get_drv_priv(q);
	vdec_unlock(ctx);
}

static void vdec_wait_finish(struct vb2_queue *q)
{
	struct vdec_ctx *ctx = vb2_get_drv_priv(q);
	vdec_lock(ctx);
}


static struct vb2_ops vdec_src_qops = {
	.start_streaming = vdec_src_start_streaming,
	.stop_streaming = vdec_src_stop_streaming,
	.queue_setup	 = vdec_src_queue_setup,
	.buf_init	 = vdec_src_buf_init,
	.buf_queue	 = vdec_src_buf_queue,
	.wait_prepare	 = vdec_wait_prepare,
	.wait_finish	 = vdec_wait_finish,
};

static struct vb2_ops vdec_dst_qops = {
	.start_streaming = vdec_dst_start_streaming,
	.stop_streaming = vdec_dst_stop_streaming,
	.queue_setup	 = vdec_dst_queue_setup,
	.buf_init	 = vdec_dst_buf_init,
	.buf_prepare	 = vdec_dst_buf_prepare,
	.buf_queue	 = vdec_dst_buf_queue,
	.wait_prepare	 = vdec_wait_prepare,
	.wait_finish	 = vdec_wait_finish,
};

static int queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
	struct vdec_ctx *ctx = priv;
	int ret;

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "queue_init\n");

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &vdec_src_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_COPY;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &vdec_dst_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_COPY;

	return vb2_queue_init(dst_vq);
}

static bool image_buffers_ready(struct vdec_ctx *ctx) {
	return vf_peek(RECEIVER_NAME) &&
		v4l2_m2m_num_dst_bufs_ready(ctx->m2m_ctx) > 0;
}

// FIXME: make this async (interrupt-driven)
static int image_thread(void *data) {
	struct vdec_ctx *ctx = data;
	struct vframe_s *vf;
	struct vb2_buffer *dst;
	struct buffer_size_info buf_info;

	set_freezable();
	for (;;) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (!kthread_should_stop() && !kthread_should_park() &&
		    !image_buffers_ready(ctx))
			schedule();

		set_current_state(TASK_RUNNING);
		try_to_freeze();
		if (kthread_should_stop())
			break;

		if (kthread_should_park())
			kthread_parkme();

		if (!image_buffers_ready(ctx))
			continue;

		v4l2_dbg(1, debug, &ctx->dev->v4l2_dev,
			 "image thread wakeup\n");

		vf = vf_get(RECEIVER_NAME);
		if (!vf) {
			v4l2_err(&ctx->dev->v4l2_dev, "no frame?\n");
			continue;
		}

		dst = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
		if (!dst) {
			v4l2_err(&ctx->dev->v4l2_dev, "no dst buffer?\n");
			vf_put(vf, RECEIVER_NAME);
			continue;
		}

		dst->v4l2_buf.timestamp.tv_sec = (__kernel_time_t)(div64_u64(vf->pts_us64, 1000000));
		dst->v4l2_buf.timestamp.tv_usec = ( __kernel_suseconds_t)
			(vf->pts_us64 - (dst->v4l2_buf.timestamp.tv_sec * 1000000));

		v4l2_dbg(1, debug, &ctx->dev->v4l2_dev,
			 "got decoded frame at pts %li.%lis\n",
			 dst->v4l2_buf.timestamp.tv_sec,
			 dst->v4l2_buf.timestamp.tv_usec);

		get_buffer_size_info(ctx->fmt, ctx->frame_width,
				     ctx->frame_height, &buf_info);
		vdec_process_image(ctx->dev, vf, dst, ctx->fmt->pixelformat,
				   buf_info.plane_stride[0]);
		vb2_set_plane_payload(dst, 0, buf_info.buffer_size[0]);
		vb2_set_plane_payload(dst, 1, buf_info.buffer_size[1]);
		vf_put(vf, RECEIVER_NAME);
		v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
	}

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "image thread exit\n");
	return 0;
}

static int vf_receiver_event(int type, void *data, void *user_data)
{
	struct vdec_ctx *ctx = user_data;
	if (type != VFRAME_EVENT_PROVIDER_QUREY_STATE)
		v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "vf event %d\n", type);

	switch (type) {
	case VFRAME_EVENT_PROVIDER_QUREY_STATE:
		return (ctx->src_streaming && ctx->dst_streaming)
			? RECEIVER_ACTIVE : RECEIVER_INACTIVE;
	case VFRAME_EVENT_PROVIDER_VFRAME_READY:
		if (ctx->eos_state == EOS_WAIT_IDLE)
			run_eos_idle_timer(ctx);

		wake_up_process(ctx->image_thread);
		break;
	}
	return 0;
}


static const struct vframe_receiver_op_s vf_receiver =
	{ .event_cb = vf_receiver_event };

/*
 * File operations
 */
static int meson_vdec_open(struct file *file)
{
	struct vdec_dev *dev = video_drvdata(file);
	struct vdec_ctx *ctx = NULL;
	int ret = 0;
	stream_port_t *port = amstream_find_port("amstream_vbuf");
	stream_buf_t *sbuf = get_buf_by_type(BUF_TYPE_VIDEO);

	v4l2_dbg(1, debug, &dev->v4l2_dev, "vdec_open\n");
	if (!port)
		return -ENOENT;

	if (mutex_lock_interruptible(&dev->dev_mutex))
		return -ERESTARTSYS;

	if (dev->open) {
		ret = -EBUSY;
		goto open_unlock;
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto open_unlock;
	}

	esparser_set_search_done_cb(ctx, parser_cb);
	vh264_set_params_cb(ctx, h264_params_cb);
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	ctx->dev = dev;
	ctx->esparser_busy = false;
	ctx->parsed_len = 0;
	ctx->fmt = &formats[0];
	spin_lock_init(&ctx->data_lock);
	init_waitqueue_head(&ctx->esparser_queue);

	init_timer(&ctx->eos_idle_timer);
	ctx->eos_idle_timer.function = eos_check_idle;
	ctx->eos_idle_timer.data = (unsigned long) ctx;

	ctx->buf_vaddr = dma_alloc_coherent(NULL, VDEC_ST_FIFO_SIZE,
					    &sbuf->buf_start, GFP_KERNEL);
	if (!ctx->buf_vaddr) {
		ret = -ENOMEM;
		goto err_free_ctx;
	}

	ctx->eos_tail_buf = dma_alloc_coherent(NULL, EOS_TAIL_BUF_SIZE,
					       &ctx->eos_tail_buf_phys,
					       GFP_KERNEL);
	if (!ctx->eos_tail_buf) {
		ret = -ENOMEM;
		goto err_free_buf;
	}

	memset(ctx->eos_tail_buf, 0, EOS_TAIL_BUF_SIZE);
	memcpy(ctx->eos_tail_buf, eos_tail_data, sizeof(eos_tail_data));

	sbuf->buf_size = sbuf->default_buf_size = VDEC_ST_FIFO_SIZE;
	sbuf->flag = BUF_FLAG_IOMEM;

	ctx->m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx, &queue_init);
	if (IS_ERR(ctx->m2m_ctx)) {
		ret = PTR_ERR(ctx->m2m_ctx);
		goto err_free_buf2;
	}

	v4l2_fh_add(&ctx->fh);

	vf_receiver_init(&ctx->vf_receiver, RECEIVER_NAME, &vf_receiver, ctx);
	vf_reg_receiver(&ctx->vf_receiver);

	ctx->image_thread = kthread_run(image_thread, ctx, DRIVER_NAME);
	if (IS_ERR(ctx->image_thread)) {
		ret = PTR_ERR(ctx->image_thread);
		goto err_del_fh;
	}

	kthread_park(ctx->image_thread);

	dev->open = true;

open_unlock:
	mutex_unlock(&dev->dev_mutex);
	return ret;

err_del_fh:
	v4l2_fh_del(&ctx->fh);

err_free_buf2:
	dma_free_coherent(NULL, EOS_TAIL_BUF_SIZE, ctx->eos_tail_buf,
			  ctx->eos_tail_buf_phys);
err_free_buf:
	dma_free_coherent(NULL, VDEC_ST_FIFO_SIZE, ctx->buf_vaddr,
			  sbuf->buf_start);
err_free_ctx:
	kfree(ctx);
	goto open_unlock;
}

static int meson_vdec_release(struct file *file)
{
	struct vdec_dev *dev = video_drvdata(file);
	struct vdec_ctx *ctx = file2ctx(file);
	stream_buf_t *sbuf = get_buf_by_type(BUF_TYPE_VIDEO);

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "vdec_release\n");

	kthread_stop(ctx->image_thread);
	vf_unreg_receiver(&ctx->vf_receiver);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	mutex_lock(&dev->dev_mutex);
	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	dma_free_coherent(NULL, VDEC_ST_FIFO_SIZE, ctx->buf_vaddr,
		          sbuf->buf_start);

	dma_free_coherent(NULL, EOS_TAIL_BUF_SIZE, ctx->eos_tail_buf,
		ctx->eos_tail_buf_phys);

	dev->open = false;
	mutex_unlock(&dev->dev_mutex);
	kfree(ctx);

	return 0;
}

static unsigned int vdec_poll(struct file *file,
				 struct poll_table_struct *wait)
{
	struct vdec_ctx *ctx = file2ctx(file);
	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "vdec_poll\n");
	return v4l2_m2m_poll(file, ctx->m2m_ctx, wait);
}

static int vdec_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct vdec_dev *dev = video_drvdata(file);
	struct vdec_ctx *ctx = file2ctx(file);
	int ret;

	v4l2_dbg(1, debug, &ctx->dev->v4l2_dev, "vdec_mmap\n");

	if (mutex_lock_interruptible(&dev->dev_mutex))
		return -ERESTARTSYS;
	ret = v4l2_m2m_mmap(file, ctx->m2m_ctx, vma);
	mutex_unlock(&dev->dev_mutex);
	return ret;
}

static const struct v4l2_file_operations vdec_fops = {
	.owner		= THIS_MODULE,
	.open		= meson_vdec_open,
	.release	= meson_vdec_release,
	.poll		= vdec_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vdec_mmap,
};

static struct video_device vdec_videodev = {
	.name		= DRIVER_NAME,
	.vfl_dir	= VFL_DIR_M2M,
	.fops		= &vdec_fops,
	.ioctl_ops	= &vdec_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,
};

static int meson_vdec_probe(struct platform_device *pdev)
{
	struct vdec_dev *dev;
	struct video_device *vfd;
	int ret;

	dev_info(&pdev->dev, "probe\n");

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	ret = vdec_image_init(dev);
	if (ret)
		return ret;

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret)
		goto image_exit;

	mutex_init(&dev->dev_mutex);

	vfd = video_device_alloc();
	if (!vfd) {
		v4l2_err(&dev->v4l2_dev, "Failed to allocate video device\n");
		ret = -ENOMEM;
		goto unreg_dev;
	}

	*vfd = vdec_videodev;
	vfd->lock = &dev->dev_mutex;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, 0);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "Failed to register video device\n");
		goto rel_vdev;
	}

	video_set_drvdata(vfd, dev);
	snprintf(vfd->name, sizeof(vfd->name), DRIVER_NAME);
	dev->vfd = vfd;
	v4l2_info(&dev->v4l2_dev, DRIVER_NAME
			"Device registered as /dev/video%d\n", vfd->num);

	platform_set_drvdata(pdev, dev);
	dev->m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR(dev->m2m_dev)) {
		v4l2_err(&dev->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(dev->m2m_dev);
		goto err_m2m;
	}

	dev->vb_alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);

	return 0;

err_m2m:
	v4l2_m2m_release(dev->m2m_dev);
	video_unregister_device(dev->vfd);
rel_vdev:
	video_device_release(vfd);
unreg_dev:
	v4l2_device_unregister(&dev->v4l2_dev);
image_exit:
	vdec_image_exit(dev);

	return ret;
}

static int meson_vdec_remove(struct platform_device *pdev)
{
	struct vdec_dev *dev = (struct vdec_dev *)platform_get_drvdata(pdev);

	v4l2_info(&dev->v4l2_dev, "Removing " DRIVER_NAME);
	vdec_image_exit(dev);
	vb2_dma_contig_cleanup_ctx(dev->vb_alloc_ctx);
	v4l2_m2m_release(dev->m2m_dev);
	video_unregister_device(dev->vfd);
	v4l2_device_unregister(&dev->v4l2_dev);
	return 0;
}

static const struct of_device_id meson_vdec_of_ids[] = {
	{
		.compatible = "amlogic,meson8b-codec",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_mfc_match);

static struct platform_driver meson_vdec_driver = {
	.probe		= meson_vdec_probe,
	.remove		= meson_vdec_remove,
	.driver	= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = meson_vdec_of_ids,
	},
};

module_platform_driver(meson_vdec_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Drake <drake@endlessm.com>");
MODULE_DESCRIPTION("Amlogic Meson video decoder driver");
