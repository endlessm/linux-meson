
#ifndef _dhd_config_
#define _dhd_config_

#include <bcmdevs.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <wlioctl.h>
#ifdef POWER_OFF_IN_SUSPEND
#include <wl_android.h>
#include <bcmsdbus.h>
#include <linux/mmc/sdio_func.h>
#endif

#ifdef POWER_OFF_IN_SUSPEND
extern bool wifi_ready;
#endif
extern void *bcmsdh_get_drvdata(void);

#ifdef POWER_OFF_IN_SUSPEND
extern struct net_device *g_netdev;
#if defined(CONFIG_HAS_EARLYSUSPEND)
extern int g_wifi_on;
void wl_cfg80211_stop(void);
void wl_cfg80211_send_disconnect(void);
void wl_cfg80211_user_sync(bool lock);
#endif
void dhd_conf_wifi_suspend(struct sdio_func *func);
void dhd_conf_register_wifi_suspend(struct sdio_func *func);
void dhd_conf_unregister_wifi_suspend(struct sdio_func *func);
#endif

#endif /* _dhd_config_ */
