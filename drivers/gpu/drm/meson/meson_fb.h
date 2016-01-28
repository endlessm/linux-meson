#ifndef MESON_FB_H
#define MESON_FB_H

struct meson_framebuffer {
	struct drm_framebuffer fb;
	struct meson_drm_gem_object *obj[4];
};
#define drm_fb_to_meson_fb(mfb) container_of(mfb, struct meson_framebuffer, fb)

struct drm_fb_helper *meson_fbdev_init(struct drm_device *dev,
		unsigned int preferred_bpp,
		unsigned int num_crtc,
		unsigned int max_conn_count);
struct drm_framebuffer *meson_fb_create(struct drm_device *dev,
		struct drm_file *file_priv, struct drm_mode_fb_cmd2 *mode_cmd);
struct meson_drm_gem_object *meson_drm_get_gem_obj(struct drm_framebuffer *fb,
		unsigned int plane);

#endif
