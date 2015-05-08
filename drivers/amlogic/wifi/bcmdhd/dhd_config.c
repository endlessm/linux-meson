
#include <typedefs.h>
#include <osl.h>

#include <bcmutils.h>
#if defined(HW_OOB)
#include <bcmdefs.h>
#include <bcmsdh.h>
#include <hndsoc.h>
#include <sdio.h>
#include <sbchipc.h>
#endif
#include <dhd_config.h>
#include <dhd_dbg.h>
#include <wl_cfg80211.h>

/* message levels */
#define CONFIG_ERROR_LEVEL	0x0001
#define CONFIG_TRACE_LEVEL	0x0002

uint config_msg_level = CONFIG_ERROR_LEVEL;

#define CONFIG_ERROR(x) \
	do { \
		if (config_msg_level & CONFIG_ERROR_LEVEL) { \
			printk(KERN_ERR "CONFIG-ERROR) ");	\
			printk x; \
		} \
	} while (0)
#define CONFIG_TRACE(x) \
	do { \
		if (config_msg_level & CONFIG_TRACE_LEVEL) { \
			printk(KERN_ERR "CONFIG-TRACE) ");	\
			printk x; \
		} \
	} while (0)
#ifdef POWER_OFF_IN_SUSPEND
struct net_device *g_netdev;
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
struct sdio_early_suspend_info {
	struct sdio_func *func;
	struct early_suspend sdio_early_suspend;
	struct work_struct	tqueue;
	int do_late_resume;
};
struct sdio_early_suspend_info sdioinfo[4];

void
dhd_conf_wifi_stop(struct net_device *dev)
{
	if (!dev) {
		CONFIG_ERROR(("%s: dev is null\n", __FUNCTION__));
		return;
	}

	printk("%s in 1\n", __FUNCTION__);
	dhd_net_if_lock(dev);
	printk("%s in 2: g_wifi_on=%d, name=%s\n", __FUNCTION__, g_wifi_on, dev->name);
	if (g_wifi_on) {
		wl_cfg80211_user_sync(true);
		wl_cfg80211_stop();
		dhd_bus_devreset(bcmsdh_get_drvdata(), true);
		sdioh_stop(NULL);
		dhd_customer_gpio_wlan_ctrl(WLAN_POWER_OFF);
		g_wifi_on = FALSE;
		wl_cfg80211_user_sync(false);
	}
	printk("%s out\n", __FUNCTION__);
	dhd_net_if_unlock(dev);

}

bool wifi_ready = true;

void
dhd_conf_wifi_power(bool on)
{
	extern struct wl_priv *wlcfg_drv_priv;
	printk("%s: Enter %d\n", __FUNCTION__, on);
	if (on) {
		wl_cfg80211_user_sync(true);
		wl_android_wifi_on(g_netdev);
		wl_cfg80211_send_disconnect();
		if (wlcfg_drv_priv && wlcfg_drv_priv->p2p)
		wl_cfgp2p_start_p2p_device(NULL, NULL);
		else		
			printk("======= ON : no p2p ======\n");
        //wl_cfg80211_up(NULL);
		wl_cfg80211_user_sync(false);
		wifi_ready = true;
	} else {
		wifi_ready = false;
        //wl_cfg80211_down(NULL);
		if (wlcfg_drv_priv && wlcfg_drv_priv->p2p) {
		wl_cfgp2p_clear_management_ie(wlcfg_drv_priv, 0);
		wl_cfgp2p_clear_management_ie(wlcfg_drv_priv, 1);
		wl_cfgp2p_stop_p2p_device(NULL, wlcfg_drv_priv->p2p_wdev);
		} else 
			printk("======= OFF : no p2p ======\n");
		dhd_conf_wifi_stop(g_netdev);
	}
	printk("%s: Exit %d\n", __FUNCTION__, on);
}

void
dhd_conf_probe_workqueue(struct work_struct *work)
{
    //dhd_conf_wifi_power(false);
}

void
dhd_conf_early_suspend(struct early_suspend *h)
{
	struct sdio_early_suspend_info *sdioinfo = container_of(h, struct sdio_early_suspend_info, sdio_early_suspend);

	printk("%s: Enter\n", __FUNCTION__);
	if(sdioinfo->func->num == 2)
		sdioinfo->do_late_resume = 0;
}

void
dhd_conf_late_resume(struct early_suspend *h)
{
	struct sdio_early_suspend_info *sdioinfo = container_of(h, struct sdio_early_suspend_info, sdio_early_suspend);

	printk("%s: Enter\n", __FUNCTION__);
	if(sdioinfo->func->num == 2 && sdioinfo->do_late_resume ){
		sdioinfo->do_late_resume = 0;
		schedule_work(&sdioinfo->tqueue);
	}
}
#endif /* defined(CONFIG_HAS_EARLYSUSPEND) */

void
dhd_conf_wifi_suspend(struct sdio_func *func)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	if (!sdioinfo[func->num].do_late_resume) {
		dhd_conf_wifi_power(false);
		sdioinfo[func->num].do_late_resume = 1;
	}
#endif
}

void
dhd_conf_register_wifi_suspend(struct sdio_func *func)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	if (func->num == 2) {
		sdioinfo[func->num].func = func;
		sdioinfo[func->num].do_late_resume = 0;
		sdioinfo[func->num].sdio_early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 30;
		sdioinfo[func->num].sdio_early_suspend.suspend = dhd_conf_early_suspend;
		sdioinfo[func->num].sdio_early_suspend.resume = dhd_conf_late_resume;
		register_early_suspend(&sdioinfo[func->num].sdio_early_suspend);
		INIT_WORK(&sdioinfo[func->num].tqueue, dhd_conf_probe_workqueue);
	}
#endif
}

void
dhd_conf_unregister_wifi_suspend(struct sdio_func *func)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	if (func->num == 2) {
		if (sdioinfo[func->num].sdio_early_suspend.suspend) {
			unregister_early_suspend(&sdioinfo[func->num].sdio_early_suspend);
			sdioinfo[func->num].sdio_early_suspend.suspend = NULL;
		}
	}
#endif
}
#endif

