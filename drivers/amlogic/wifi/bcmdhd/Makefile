# bcmdhd
DHDCFLAGS = -Wall -Wstrict-prototypes -Dlinux -DBCMDRIVER                     \
	-DDHD_FIRSTREAD=128 -DMAX_HDR_READ=128 -DWLP2P                        \
	-DPROP_TXSTATUS_VSDB -DDHDTCPACK_SUPPRESS                             \
	-DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DWLBTAMP -DBCMFILEIMAGE  \
	-DDHDTHREAD -DDHD_DEBUG -DSDTEST -DBDC -DTOE                          \
	-DDHD_BCMEVENTS -DSHOW_EVENTS -DPROP_TXSTATUS -DBCMDBG                \
        -DWL_CFG80211_P2P_DEV_IF -DWL_IFACE_COMB_NUM_CHANNELS                 \
	-DMMC_SDIO_ABORT -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS             \
	-DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT                                \
	-DKEEP_ALIVE -DPKT_FILTER_SUPPORT                                     \
	-DEMBEDDED_PLATFORM -DENABLE_INSMOD_NO_FW_LOAD -DPNO_SUPPORT          \
	-DDHD_USE_IDLECOUNT -DSET_RANDOM_MAC_SOFTAP -DROAM_ENABLE -DVSDB      \
	-DWL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST -DSDIO_CRC_ERROR_FIX       \
	-DESCAN_RESULT_PATCH -DHT40_GO -DPASS_ARP_PACKET                      \
	-DDHD_DONOT_FORWARD_BCMEVENT_AS_NETWORK_PKT -DAMPDU_HOSTREORDER       \
	-DCUSTOM_SDIO_F2_BLKSIZE=128 -DSUPPORT_PM2_ONLY -DWL_SDO -DWLTDLS     \
	-DWL_SUPPORT_BACKPORTED_KPATCHES -DUSE_SDIOFIFO_IOVAR                 \
	-I$(src) -I$(src)/include

DHDOFILES = aiutils.o bcmsdh_sdmmc_linux.o dhd_linux.o siutils.o bcmutils.o   \
	dhd_linux_sched.o dhd_sdio.o bcmwifi_channels.o bcmevent.o hndpmu.o dhd_bta.o  \
	bcmsdh.o dhd_cdc.o bcmsdh_linux.o dhd_common.o dhd_wlfc.o dhd_ip.o linux_osl.o \
	bcmsdh_sdmmc.o dhd_custom_gpio.o sbutils.o wldev_common.o wl_android.o dhd_pno.o \
	dhd_config.o


DHDCFLAGS += -DCUSTOMER_HW_AMLOGIC

ifeq ($(CONFIG_BCMDHD_POWER_OFF_IN_SUSPEND),y)
DHDCFLAGS += -DPOWER_OFF_IN_SUSPEND
endif

ifeq ($(CONFIG_BCMDHD_HW_OOB),y)
DHDCFLAGS += -DCUSTOM_OOB_GPIO_NUM=INT_GPIO_4 -DOOB_INTR_ONLY -DHW_OOB -DMMC_SDIO_ABORT
else
DHDCFLAGS += -DSDIO_ISR_THREAD
endif

obj-$(CONFIG_BCMDHD) += bcmdhd.o
ifeq ($(KBUILD_SRC),)
DHD_SRC = $(src)
else
DHD_SRC = $(KBUILD_SRC)/$(src)
endif
ifneq ($(wildcard $(DHD_SRC)/dhd_nvram.obj),)
include $(src)/makefile.my
endif
bcmdhd-objs += $(DHDOFILES)
ifneq ($(CONFIG_WIRELESS_EXT),)
bcmdhd-objs += wl_iw.o
DHDCFLAGS += -DSOFTAP
endif
ifneq ($(CONFIG_CFG80211),)
bcmdhd-objs += wl_cfg80211.o wl_cfgp2p.o wl_linux_mon.o dhd_cfg80211.o
DHDCFLAGS += -DWL_CFG80211 -DWL_CFG80211_STA_EVENT 
DHDCFLAGS += -DCUSTOM_ROAM_TRIGGER_SETTING=-85
DHDCFLAGS += -DCUSTOM_ROAM_DELTA_SETTING=15
DHDCFLAGS += -DCUSTOM_KEEP_ALIVE_SETTING=28000
DHDCFLAGS += -DCUSTOM_PNO_EVENT_LOCK_xTIME=7
endif
ifneq ($(CONFIG_DHD_USE_SCHED_SCAN),)
DHDCFLAGS += -DWL_SCHED_SCAN
endif
EXTRA_CFLAGS = $(DHDCFLAGS)
ifeq ($(CONFIG_BCMDHD),m)
EXTRA_LDFLAGS += --strip-debug
endif
