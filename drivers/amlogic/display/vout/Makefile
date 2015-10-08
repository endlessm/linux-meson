obj-y += vout_notify.o  vout_serve.o vinfo.o

obj-$(CONFIG_AML_VDAC_HW_SWITCH)	+= vdac_switch.o

obj-$(CONFIG_AML_TVOUT)				+= tvout.o
tvout-objs := tvoutc.o tvconf.o
ifeq ($(CONFIG_ARCH_MESONG9TV),y)
else
ifeq ($(CONFIG_ARCH_MESONG9BB),y)
else
tvout-objs += enc_clk_config.o
endif
endif
obj-$(CONFIG_AML_LCD_TABLET)		+= lcdoutc.o

#EXTRA_CFLAGS += -DCONFIG_PM
