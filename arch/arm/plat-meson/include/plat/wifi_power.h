#ifndef _WIFI_POWER_H_
#define _WIFI_POWER_H_
struct wifi_power_platform_data{
    int power_gpio;
    int power_gpio2;
    int (*set_power)(int val);
    int (*set_reset)(int val);
    int (*set_carddetect)(int val);
    void *(*mem_prealloc)(int section, unsigned long size);
    int (*get_mac_addr)(unsigned char *buf);
    void *(*get_country_code)(char *ccode);
    int (*usb_set_power)(int val);
};


#endif

