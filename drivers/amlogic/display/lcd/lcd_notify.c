/*
 * drivers/amlogic/display/lcd/lcd_notifier.c
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

#include <linux/amlogic/display/lcd.h>
#include <linux/notifier.h>
#include <linux/export.h>

static BLOCKING_NOTIFIER_HEAD(lcd_notifier_list);

/**
 * lcd_register_client - register a client notifier
 * @nb: notifier block to callback on events
 */
int lcd_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&lcd_notifier_list, nb);
}
EXPORT_SYMBOL(lcd_register_client);

/**
 * lcd_unregister_client - unregister a client notifier
 * @nb: notifier block to callback on events
 */
int lcd_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&lcd_notifier_list, nb);
}
EXPORT_SYMBOL(lcd_unregister_client);

/**
 * lcd_notifier_call_chain - notify clients of lcd events
 *
 */
int lcd_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&lcd_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(lcd_notifier_call_chain);
