/*
 * Meson video decoder driver
 *
 * Copyright (C) 2015 Endless Mobile, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <media/v4l2-mem2mem.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#define DRIVER_NAME "meson-vdec"

struct vdec_dev {
	struct v4l2_device	v4l2_dev;
	struct video_device	*vfd;
	struct v4l2_m2m_dev	*m2m_dev;
	struct mutex		dev_mutex;
};

struct vdec_ctx {
	struct v4l2_fh		fh;
	struct vdec_dev	*dev;
	struct v4l2_m2m_ctx	*m2m_ctx;
};

/*
 * video ioctls
 */
static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strncpy(cap->driver, MEM2MEM_NAME, sizeof(cap->driver) - 1);
	strncpy(cap->card, MEM2MEM_NAME, sizeof(cap->card) - 1);
	snprintf(cap->bus_info, sizeof(cap->bus_info),
			"platform:%s", MEM2MEM_NAME);
	cap->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int enum_fmt(struct v4l2_fmtdesc *f, u32 type)
{
	int i, num;
	struct m2mtest_fmt *fmt;

	num = 0;

	for (i = 0; i < NUM_FORMATS; ++i) {
		if (formats[i].types & type) {
			/* index-th format of type type found ? */
			if (num == f->index)
				break;
			/* Correct type but haven't reached our index yet,
			 * just increment per-type index */
			++num;
		}
	}

	if (i < NUM_FORMATS) {
		/* Format found */
		fmt = &formats[i];
		strncpy(f->description, fmt->name, sizeof(f->description) - 1);
		f->pixelformat = fmt->fourcc;
		return 0;
	}

	/* Format not found */
	return -EINVAL;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return enum_fmt(f, MEM2MEM_CAPTURE);
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return enum_fmt(f, MEM2MEM_OUTPUT);
}

static int vidioc_g_fmt(struct m2mtest_ctx *ctx, struct v4l2_format *f)
{
	struct vb2_queue *vq;
	struct m2mtest_q_data *q_data;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(ctx, f->type);

	f->fmt.pix.width	= q_data->width;
	f->fmt.pix.height	= q_data->height;
	f->fmt.pix.field	= V4L2_FIELD_NONE;
	f->fmt.pix.pixelformat	= q_data->fmt->fourcc;
	f->fmt.pix.bytesperline	= (q_data->width * q_data->fmt->depth) >> 3;
	f->fmt.pix.sizeimage	= q_data->sizeimage;
	f->fmt.pix.colorspace	= ctx->colorspace;

	return 0;
}

static int vidioc_g_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	return vidioc_g_fmt(file2ctx(file), f);
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	return vidioc_g_fmt(file2ctx(file), f);
}

static int vidioc_try_fmt(struct v4l2_format *f, struct m2mtest_fmt *fmt)
{
	enum v4l2_field field;

	field = f->fmt.pix.field;

	if (field == V4L2_FIELD_ANY)
		field = V4L2_FIELD_NONE;
	else if (V4L2_FIELD_NONE != field)
		return -EINVAL;

	/* V4L2 specification suggests the driver corrects the format struct
	 * if any of the dimensions is unsupported */
	f->fmt.pix.field = field;

	if (f->fmt.pix.height < MIN_H)
		f->fmt.pix.height = MIN_H;
	else if (f->fmt.pix.height > MAX_H)
		f->fmt.pix.height = MAX_H;

	if (f->fmt.pix.width < MIN_W)
		f->fmt.pix.width = MIN_W;
	else if (f->fmt.pix.width > MAX_W)
		f->fmt.pix.width = MAX_W;

	f->fmt.pix.width &= ~DIM_ALIGN_MASK;
	f->fmt.pix.bytesperline = (f->fmt.pix.width * fmt->depth) >> 3;
	f->fmt.pix.sizeimage = f->fmt.pix.height * f->fmt.pix.bytesperline;

	return 0;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct m2mtest_fmt *fmt;
	struct m2mtest_ctx *ctx = file2ctx(file);

	fmt = find_format(f);
	if (!fmt || !(fmt->types & MEM2MEM_CAPTURE)) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}
	f->fmt.pix.colorspace = ctx->colorspace;

	return vidioc_try_fmt(f, fmt);
}

static int vidioc_try_fmt_vid_out(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct m2mtest_fmt *fmt;
	struct m2mtest_ctx *ctx = file2ctx(file);

	fmt = find_format(f);
	if (!fmt || !(fmt->types & MEM2MEM_OUTPUT)) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}
	if (!f->fmt.pix.colorspace)
		f->fmt.pix.colorspace = V4L2_COLORSPACE_REC709;

	return vidioc_try_fmt(f, fmt);
}

static int vidioc_s_fmt(struct m2mtest_ctx *ctx, struct v4l2_format *f)
{
	struct m2mtest_q_data *q_data;
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(ctx, f->type);
	if (!q_data)
		return -EINVAL;

	if (vb2_is_busy(vq)) {
		v4l2_err(&ctx->dev->v4l2_dev, "%s queue busy\n", __func__);
		return -EBUSY;
	}

	q_data->fmt		= find_format(f);
	q_data->width		= f->fmt.pix.width;
	q_data->height		= f->fmt.pix.height;
	q_data->sizeimage	= q_data->width * q_data->height
				* q_data->fmt->depth >> 3;

	dprintk(ctx->dev,
		"Setting format for type %d, wxh: %dx%d, fmt: %d\n",
		f->type, q_data->width, q_data->height, q_data->fmt->fourcc);

	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	int ret;

	ret = vidioc_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	return vidioc_s_fmt(file2ctx(file), f);
}

static int vidioc_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct m2mtest_ctx *ctx = file2ctx(file);
	int ret;

	ret = vidioc_try_fmt_vid_out(file, priv, f);
	if (ret)
		return ret;

	ret = vidioc_s_fmt(file2ctx(file), f);
	if (!ret)
		ctx->colorspace = f->fmt.pix.colorspace;
	return ret;
}

static int vidioc_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *reqbufs)
{
	struct m2mtest_ctx *ctx = file2ctx(file);

	return v4l2_m2m_reqbufs(file, ctx->m2m_ctx, reqbufs);
}

static int vidioc_querybuf(struct file *file, void *priv,
			   struct v4l2_buffer *buf)
{
	struct m2mtest_ctx *ctx = file2ctx(file);

	return v4l2_m2m_querybuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct m2mtest_ctx *ctx = file2ctx(file);

	return v4l2_m2m_qbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct m2mtest_ctx *ctx = file2ctx(file);

	return v4l2_m2m_dqbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_streamon(struct file *file, void *priv,
			   enum v4l2_buf_type type)
{
	struct m2mtest_ctx *ctx = file2ctx(file);

	return v4l2_m2m_streamon(file, ctx->m2m_ctx, type);
}

static int vidioc_streamoff(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	struct m2mtest_ctx *ctx = file2ctx(file);

	return v4l2_m2m_streamoff(file, ctx->m2m_ctx, type);
}

static int m2mtest_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct m2mtest_ctx *ctx =
		container_of(ctrl->handler, struct m2mtest_ctx, hdl);

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		if (ctrl->val)
			ctx->mode |= MEM2MEM_HFLIP;
		else
			ctx->mode &= ~MEM2MEM_HFLIP;
		break;

	case V4L2_CID_VFLIP:
		if (ctrl->val)
			ctx->mode |= MEM2MEM_VFLIP;
		else
			ctx->mode &= ~MEM2MEM_VFLIP;
		break;

	case V4L2_CID_TRANS_TIME_MSEC:
		ctx->transtime = ctrl->val;
		break;

	case V4L2_CID_TRANS_NUM_BUFS:
		ctx->translen = ctrl->val;
		break;

	default:
		v4l2_err(&ctx->dev->v4l2_dev, "Invalid control\n");
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops m2mtest_ctrl_ops = {
	.s_ctrl = m2mtest_s_ctrl,
};

static const struct v4l2_ioctl_ops m2mtest_ioctl_ops = {
	.vidioc_querycap	= vidioc_querycap,

	.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap	= vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap	= vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap	= vidioc_s_fmt_vid_cap,

	.vidioc_enum_fmt_vid_out = vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out	= vidioc_g_fmt_vid_out,
	.vidioc_try_fmt_vid_out	= vidioc_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out	= vidioc_s_fmt_vid_out,

	.vidioc_reqbufs		= vidioc_reqbufs,
	.vidioc_querybuf	= vidioc_querybuf,

	.vidioc_qbuf		= vidioc_qbuf,
	.vidioc_dqbuf		= vidioc_dqbuf,

	.vidioc_streamon	= vidioc_streamon,
	.vidioc_streamoff	= vidioc_streamoff,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/*
 * File operations
 */
static int vdec_open(struct file *file)
{
	struct vdec_dev *dev = video_drvdata(file);
	struct vdec_ctx *ctx = NULL;
	int rc = 0;

	if (mutex_lock_interruptible(&dev->dev_mutex))
		return -ERESTARTSYS;
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		rc = -ENOMEM;
		goto open_unlock;
	}

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	ctx->dev = dev;

	ctx->m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx, &queue_init);
	if (IS_ERR(ctx->m2m_ctx)) {
		ret = PTR_ERR(ctx->m2m_ctx);
		kfree(ctx);
		goto open_unlock;
	}

	v4l2_fh_add(&ctx->fh);

open_unlock:
	mutex_unlock(&dev->dev_mutex);
	return rc;
}

static int vdec_release(struct file *file)
{
	struct vdec_dev *dev = video_drvdata(file);
	struct vdec_ctx *ctx = file2ctx(file);

	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	mutex_lock(&dev->dev_mutex);
	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	mutex_unlock(&dev->dev_mutex);
	kfree(ctx);

	return 0;
}

static unsigned int vdec_poll(struct file *file,
				 struct poll_table_struct *wait)
{
	struct vdec_ctx *ctx = file2ctx(file);

	return v4l2_m2m_poll(file, ctx->m2m_ctx, wait);
}

static int vdec_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct vdec_dev *dev = video_drvdata(file);
	struct vdec_ctx *ctx = file2ctx(file);
	int ret;

	if (mutex_lock_interruptible(&dev->dev_mutex))
		return -ERESTARTSYS;
	ret = v4l2_m2m_mmap(file, ctx->m2m_ctx, vma);
	mutex_unlock(&dev->dev_mutex);
	return ret;
}

static const struct v4l2_file_operations vdec_fops = {
	.owner		= THIS_MODULE,
	.open		= vdec_open,
	.release	= vdec_release,
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

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret)
		return ret;

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
	snprintf(vfd->name, sizeof(vfd->name), "%s", m2mtest_videodev.name);
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

	return 0;

err_m2m:
	v4l2_m2m_release(dev->m2m_dev);
	video_unregister_device(dev->vfd);
rel_vdev:
	video_device_release(vfd);
unreg_dev:
	v4l2_device_unregister(&dev->v4l2_dev);

	return ret;
}

static int meson_vdec_remove(struct platform_device *pdev)
{
	struct vdec_dev *dev = (struct vdec_dev *)platform_get_drvdata(pdev);

	v4l2_info(&dev->v4l2_dev, "Removing " DRIVER_NAME);
	v4l2_m2m_release(dev->m2m_dev);
	video_unregister_device(dev->vfd);
	v4l2_device_unregister(&dev->v4l2_dev);
	return 0;
}

static const struct of_device_id meson_vdec_of_ids[] = {
	{
		.compatible = "amlogic,meson8b-vdec",
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
