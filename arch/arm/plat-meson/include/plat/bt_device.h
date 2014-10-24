/*
 *
 * arch/arm/mach-meson/bcm-bt.c
 *
 *  Copyright (C) 2010 AMLOGIC, INC.
 *
 * License terms: GNU General Public License (GPL) version 2
 * Platform machine definition.
 */

#ifndef __BT_DEVICE_H
#define __BT_DEVICE_H

struct bt_dev_data {
    int gpio_reset;
    int gpio_en;
    int gpio_host_wake;
    int gpio_wake;
};

#endif  