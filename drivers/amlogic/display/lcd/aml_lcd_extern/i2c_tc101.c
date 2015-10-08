/*
 * AMLOGIC lcd external driver.
 *
 * Communication protocol:
 * I2C
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/i2c-aml.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <asm/uaccess.h>
#include <mach/pinmux.h>
#include <mach/gpio.h>
#include <linux/amlogic/vout/aml_lcd_extern.h>
#include "lcd_extern.h"

#define LCD_EXTERN_NAME			"i2c_tc101"

static struct i2c_client *aml_tc101_i2c_client;
static struct lcd_extern_config_t *ext_config;

#define INIT_LEN        3
static unsigned char i2c_init_table[][INIT_LEN] = {
	//{0xff, 0xff, 20},//delay mark(20ms)
	{0xf8, 0x30, 0xb2},
	{0xf8, 0x33, 0xc2},
	{0xf8, 0x31, 0xf0},
	{0xf8, 0x40, 0x80},
	{0xf8, 0x81, 0xec},
	{0xff, 0xff, 0xff},//end mark
};

static int lcd_extern_i2c_write(struct i2c_client *i2client,unsigned char *buff, unsigned len)
{
	int ret = 0;
	struct i2c_msg msg[] = {
		{
			.addr = i2client->addr,
			.flags = 0,
			.len = len,
			.buf = buff,
		}
	};

	ret = i2c_transfer(i2client->adapter, msg, 1);
	if (ret < 0)
		LCD_EXT_PR("i2c write failed [addr 0x%02x]\n", i2client->addr);

	return ret;
}
#if 0
static int lcd_extern_i2c_read(struct i2c_client *i2client,unsigned char *buff, unsigned len)
{
	int ret = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = i2client->addr,
			.flags = 0,
			.len = 1,
			.buf = buff,
		},
		{
			.addr = i2client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buff,
		}
	};

	ret = i2c_transfer(i2client->adapter, msgs, 2);
	if (ret < 0)
		LCD_EXT_PR("i2c read failed [addr 0x%02x]\n", i2client->addr);

	return ret;
}
#endif

static int lcd_extern_reg_read(unsigned char reg, unsigned char *buf)
{
	int ret = 0;

	return ret;
}

static int lcd_extern_reg_write(unsigned char reg, unsigned char value)
{
	int ret = 0;

	return ret;
}

static int lcd_extern_i2c_init(void)
{
	int i = 0, ending_flag = 0;
	int ret = 0;

	while (ending_flag == 0) {
		if ((i2c_init_table[i][0] == 0xff) && (i2c_init_table[i][1] == 0xff)) { //special mark
			if (i2c_init_table[i][2] == 0xff) //ending flag
				ending_flag = 1;
			else //delay flag
				mdelay(i2c_init_table[i][2]);
		} else {
			lcd_extern_i2c_write(aml_tc101_i2c_client, i2c_init_table[i], INIT_LEN);
		}
		i++;
	}
	LCD_EXT_PR("%s: %s\n", __func__, ext_config->name);
	return ret;
}

static int lcd_extern_i2c_remove(void)
{
	int ret = 0;

	return ret;
}

static int lcd_extern_power_on(void)
{
	int ret;

	ret = lcd_extern_i2c_init();
	return ret;
}

static int lcd_extern_power_off(void)
{
	int ret;

	ret = lcd_extern_i2c_remove();
	return ret;
}

static int lcd_extern_driver_update(struct aml_lcd_extern_driver_t *ext_drv)
{
	int ret = 0;

	if (ext_drv) {
		ext_drv->reg_read  = lcd_extern_reg_read;
		ext_drv->reg_write = lcd_extern_reg_write;
		ext_drv->power_on  = lcd_extern_power_on;
		ext_drv->power_off = lcd_extern_power_off;
	} else {
		LCD_EXT_PR("%s driver is null\n", LCD_EXTERN_NAME);
		ret = -1;
	}

	return ret;
}

static int aml_tc101_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		LCD_EXT_PR("[error] %s: functionality check failed\n", __func__);
		return -ENODEV;
	} else {
		aml_tc101_i2c_client = client;
	}

	LCD_EXT_PR("%s OK\n", __func__);
	return 0;
}

static int aml_tc101_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id aml_tc101_i2c_id[] = {
	{LCD_EXTERN_NAME, 0},
	{ }
};
// MODULE_DEVICE_TABLE(i2c, aml_tc101_id);

static struct i2c_driver aml_tc101_i2c_driver = {
	.probe    = aml_tc101_i2c_probe,
	.remove   = aml_tc101_i2c_remove,
	.id_table = aml_tc101_i2c_id,
	.driver = {
		.name = LCD_EXTERN_NAME,
		.owner =THIS_MODULE,
	},
};

int aml_lcd_extern_i2c_tc101_probe(struct aml_lcd_extern_driver_t *ext_drv)
{
	struct i2c_board_info i2c_info;
	struct i2c_adapter *adapter;
	struct i2c_client *i2c_client;
	int ret = 0;

	ext_config = &ext_drv->config;
	memset(&i2c_info, 0, sizeof(i2c_info));

	adapter = i2c_get_adapter(ext_drv->config.i2c_bus);
	if (!adapter) {
		LCD_EXT_PR("%s failed to get i2c adapter\n", ext_drv->config.name);
		return -1;
	}

	strncpy(i2c_info.type, ext_drv->config.name, I2C_NAME_SIZE);
	i2c_info.addr = ext_drv->config.i2c_addr;
	i2c_info.platform_data = &ext_drv->config;
	i2c_info.flags = 0;
	if (i2c_info.addr > 0x7f) {
		LCD_EXT_PR("%s invalid i2c address: 0x%02x\n", ext_drv->config.name, ext_drv->config.i2c_addr);
		return -1;
	}
	i2c_client = i2c_new_device(adapter, &i2c_info);
	if (!i2c_client) {
		LCD_EXT_PR("%s failed to new i2c device\n", ext_drv->config.name);
		return -1;
	} else {
		DBG_PRINT("%s new i2c device succeed\n", ext_drv->config.name);
	}

	if (!aml_tc101_i2c_client) {
		ret = i2c_add_driver(&aml_tc101_i2c_driver);
		if (ret) {
			LCD_EXT_PR("%s add i2c_driver failed\n", ext_drv->config.name);
			return -1;
		}
	}

	ret = lcd_extern_driver_update(ext_drv);

	DBG_PRINT("%s: %d\n", __func__, ret);
	return ret;
}
