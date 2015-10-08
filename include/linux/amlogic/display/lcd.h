/*
 * include/linux/amlogic/display/lcd.h
 *
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
*/

#ifndef __AMLOGIC_DISPLAY_LCD_H
#define __AMLOGIC_DISPLAY_LCD_H

#include <linux/notifier.h>


/* lcd display mode occurred */
#define LCD_EVENT_MODE_CHANGE		0x01
/* lcd suspend occurred */
#define LCD_EVENT_SUSPEND		0x02
/* lcd resume occurred */
#define LCD_EVENT_RESUME		0x03
/* lcd register occurred */
#define LCD_EVENT_REGISTERED		0x04
/* lcd unregister occurred */
#define LCD_EVENT_UNREGISTERED		0x05
/* lcd power on occurred */
#define LCD_EVENT_POWERON		0x06
/* lcd power off occurred */
#define LCD_EVENT_POWEROFF		0x07
/* lcd blank change occurred */
#define LCD_EVENT_BLANK			0x06
/* lcd unblank change occurred */
#define LCD_EVENT_UNBLANK		0x07
/* lcd bl pwm_vs vfreq change occurred */
#define LCD_EVENT_BL_UPDATE		0x10


extern int lcd_register_client(struct notifier_block *nb);
extern int lcd_unregister_client(struct notifier_block *nb);
extern int lcd_notifier_call_chain(unsigned long val, void *v);


#endif
