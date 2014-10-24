#ifndef _HDMI_CONFIG_H_
#define _HDMI_CONFIG_H_

struct hdmi_phy_set_data{
    unsigned long freq;
    unsigned long addr;
    unsigned long data;
};

struct vendor_info_data{
    unsigned char *vendor_name;     // Max Chars: 8
    unsigned int  vendor_id;        // 3 Bytes, Refer to http://standards.ieee.org/develop/regauth/oui/oui.txt
    unsigned char *product_desc;    // Max Chars: 16
    unsigned char *cec_osd_string;  // Max Chars: 14
};

enum pwr_type {
    NONE = 0,
    CPU_GPO = 1,
    AXP202,
};

struct pwr_cpu_gpo {
    unsigned int pin;
    unsigned int val;
};

struct pwr_axp202 {
    unsigned int pin;
    unsigned int val;
};

struct pwr_ctl_var {
    enum pwr_type type;
    union {
        struct pwr_cpu_gpo gpo;
        struct pwr_axp202 axp202;
    }var;
};

struct hdmi_pwr_ctl{
    struct pwr_ctl_var pwr_5v_on;
    struct pwr_ctl_var pwr_5v_off;
    struct pwr_ctl_var pwr_3v3_on;
    struct pwr_ctl_var pwr_3v3_off;
    struct pwr_ctl_var pwr_hpll_vdd_on;
    struct pwr_ctl_var pwr_hpll_vdd_off;
    int pwr_level;
};

struct hdmi_config_platform_data{
    void (*hdmi_sspll_ctrl)(unsigned int level);    // SSPLL control level
    struct hdmi_phy_set_data *phy_data;             // For some boards, HDMI PHY setting may diff from ref board.
    struct vendor_info_data *vend_data;
    struct hdmi_pwr_ctl *pwr_ctl;
};

#endif

