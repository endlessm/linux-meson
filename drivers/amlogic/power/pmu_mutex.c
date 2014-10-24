
#include <linux/mutex.h>
#include <linux/lockdep.h>
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/gfp.h>
#include <linux/rtc.h>
#include <linux/err.h>

/*
 * some data-struct is depend on kernel's menuconfig
 * so make these calls in a single file with open source.
 * This can help compatibility of PMU driver liberary.
 */
static int mutex_cnt = 0;
const  char mutex_name[20] = {};
static struct rtc_device *rtc_dev = NULL;

void *pmu_alloc_mutex(void)
{
    struct mutex *pmutex = NULL;
    struct lock_class_key *key = NULL;

    pmutex = kzalloc(sizeof(struct mutex), GFP_KERNEL);
    if (!pmutex) {
        printk("%s, alloc mutex failed\n", __func__);
        return NULL;
    }
    key = kzalloc(sizeof(struct lock_class_key), GFP_KERNEL);
    if (!key) {
        printk("%s, alloc key failed\n", __func__);
        return NULL;
    }
    sprintf((char *)mutex_name, "pmu_mutex%d", mutex_cnt++); 
    __mutex_init(pmutex, mutex_name, key);
    return (void *)pmutex;
}
EXPORT_SYMBOL(pmu_alloc_mutex);

void pmu_mutex_lock(void *mutex)
{
    mutex_lock((struct mutex *)mutex);    
}
EXPORT_SYMBOL(pmu_mutex_lock);

void pmu_mutex_unlock(void *mutex)
{
    mutex_unlock((struct mutex *)mutex);    
}
EXPORT_SYMBOL(pmu_mutex_unlock);

int pmu_rtc_device_init(void)
{
    if (rtc_dev) {
        return 0;    
    }
    rtc_dev = rtc_class_open("rtc0");
    if (IS_ERR_OR_NULL(rtc_dev)) {
        printk("%s, can't open aml_rtc class\n", __func__);
        return -EINVAL;
    }
    return 0;
}
EXPORT_SYMBOL(pmu_rtc_device_init);

int pmu_rtc_set_alarm(unsigned long seconds) 
{
    struct rtc_wkalrm tmp;
    unsigned long time;
    int ret;

    if (!rtc_dev) {
        printk("%s, NO rtc dev found\n", __func__);
        return -ENODEV;    
    }
    ret = rtc_read_time(rtc_dev, &tmp.time);
    if (ret < 0) {
        printk("%s, read %s alarm failed, %d\n", __func__, rtc_dev->name, ret);
        return ret;    
    }
    tmp.enabled = 1;
    rtc_tm_to_time(&tmp.time, &time);
    time += seconds;
    rtc_time_to_tm(time, &tmp.time);
    ret = rtc_set_alarm(rtc_dev, &tmp);
    printk("%s, set wake up alarm in %ld seconds, ret:%d\n", __func__, seconds, ret);
    return ret;
}
EXPORT_SYMBOL(pmu_rtc_set_alarm);
