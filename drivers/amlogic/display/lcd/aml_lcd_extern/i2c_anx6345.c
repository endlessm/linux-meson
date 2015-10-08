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
#include "i2c_anx6345.h"

#define LCD_EXTERN_NAME			"lcd_anx6345"

static struct i2c_client *aml_anx6345_70_client;
static struct i2c_client *aml_anx6345_72_client;
static struct lcd_extern_config_t *ext_config;

const char *anx_addr_name[2] = {"anx6345_70","anx6345_72"};

struct lcd_extern_edp_config {
	int lane_num;
	int bits;
	int link_rate;
};

static struct lcd_extern_edp_config edp_parameter = {
	.lane_num = 1, // 1/2/4
	.bits = 0x00,  // 6bit: 0x00   8bit: 0x10
	.link_rate = 0x0a, //1.62G: 0X06, 2.7G: 0x0a, 5.4G: 0x14
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
static int SP_TX_Write_Reg(unsigned char addr,unsigned char reg ,unsigned char data)
{
	struct i2c_client *client = aml_anx6345_70_client;
	unsigned char buff[2];
	int ret;

	buff[0] = reg;
	buff[1] = data;
	if (addr == 0x70)
		client = aml_anx6345_70_client;
	else if (addr == 0x72)
		client = aml_anx6345_72_client;

	ret = lcd_extern_i2c_write(client, buff, 1);
	if (ret < 0)
		return -1;
	else
		return 0;
}

static int SP_TX_Read_Reg(unsigned char addr,unsigned char reg ,unsigned char* data)
{
	struct i2c_client *client = aml_anx6345_70_client;
	int ret;

	*data = reg;
	if (addr == 0x70)
		client = aml_anx6345_70_client;
	else if (addr == 0x72)
		client = aml_anx6345_72_client;

	ret = lcd_extern_i2c_read(client, data, 1);
	if (ret < 0)
		return -1;
	else
		return 0;
}

static int SP_TX_Wait_AUX_Finished(void)
{
	unsigned char c;
	unsigned char cCnt = 0;

	SP_TX_Read_Reg(0x70, SP_TX_AUX_STATUS, &c);
	while (c & 0x10) {//aux busy
		cCnt++;
		SP_TX_Read_Reg(0x70, SP_TX_AUX_STATUS, &c);

		if (cCnt > 100)
			return 0; //aux fail
	}

	return 1;//aux ok
}

static int SP_TX_AUX_DPCDRead_Bytes(unsigned char addrh, unsigned char addrm, unsigned char addrl,unsigned char cCount,unsigned char *pBuf)
{
	unsigned char c,i;

	//clr buffer
	SP_TX_Write_Reg(0x70, SP_TX_BUF_DATA_COUNT_REG, 0x80);

	//set read cmd and count
	SP_TX_Write_Reg(0x70, SP_TX_AUX_CTRL_REG, ((cCount-1) <<4)|0x09);

	//set aux address15:0
	SP_TX_Write_Reg(0x70, SP_TX_AUX_ADDR_7_0_REG, addrl);
	SP_TX_Write_Reg(0x70, SP_TX_AUX_ADDR_15_8_REG, addrm);

	//set address19:16 and enable aux
	SP_TX_Read_Reg(0x70, SP_TX_AUX_ADDR_19_16_REG, &c);
	SP_TX_Write_Reg(0x70, SP_TX_AUX_ADDR_19_16_REG, (c & 0xf0) | addrh);

	//Enable Aux
	SP_TX_Read_Reg(0x70, SP_TX_AUX_CTRL_REG2, &c);
	SP_TX_Write_Reg(0x70, SP_TX_AUX_CTRL_REG2, c|0x01);

	mdelay(5);
	if (!SP_TX_Wait_AUX_Finished())
		return 0;

	for (i = 0; i < cCount; i++) {
		SP_TX_Read_Reg(0x70, SP_TX_BUF_DATA_0_REG+i, &c);

		*(pBuf+i) = c;

		if (i >= MAX_BUF_CNT)
			break;
	}

	return 1;//aux ok
}

static int lcd_extern_i2c_init(void)
{
	unsigned lane_num;
	unsigned link_rate;
	unsigned char bits;
	unsigned char device_id;
	unsigned char temp;
	unsigned char temp1;
	unsigned count = 0;
	unsigned count1 = 0;

	lane_num = edp_parameter.lane_num;
	link_rate = edp_parameter.link_rate;
	bits = edp_parameter.bits;

	SP_TX_Write_Reg (0x72, 0x05, 0x00);

	SP_TX_Read_Reg(0x72, 0x01, &device_id);

	if (device_id == 0xaa) {
		LCD_EXT_PR("ANX6345 Chip found\n\n");
	} else {
		LCD_EXT_PR("ANX6345 Chip not found\n\n");
		return 1;
	}
	temp = device_id;
	//if aux read fail, do h/w reset,
	while ((!SP_TX_AUX_DPCDRead_Bytes(0x00, 0x00, 0x00, 1, &temp1)) && (count < 200)) {
		//read fail, h/w reset
		SP_TX_Write_Reg (0x72, 0x06, 0x01);
		SP_TX_Write_Reg (0x72, 0x06, 0x00);
		SP_TX_Write_Reg (0x72, 0x05, 0x00);
		mdelay(10);
		count++;
	}

	//software reset
	SP_TX_Read_Reg(0x72, SP_TX_RST_CTRL_REG, &temp);
	SP_TX_Write_Reg(0x72, SP_TX_RST_CTRL_REG, temp | SP_TX_RST_SW_RST);
	SP_TX_Write_Reg(0x72, SP_TX_RST_CTRL_REG, temp & ~SP_TX_RST_SW_RST);

	SP_TX_Write_Reg(0x70, SP_TX_EXTRA_ADDR_REG, 0x50);//EDID address for AUX access
	SP_TX_Write_Reg(0x70, SP_TX_HDCP_CTRL, 0x00); //disable HDCP polling mode.
	//SP_TX_Write_Reg(0x70, SP_TX_HDCP_CTRL, 0x02); //Enable HDCP polling mode.
	SP_TX_Write_Reg(0x70, SP_TX_LINK_DEBUG_REG, 0x30);//enable M value read out

	//SP_TX_Read_Reg(0x70, SP_TX_DEBUG_REG1, &temp);
	SP_TX_Write_Reg(0x70, SP_TX_DEBUG_REG1, 0x00);//disable polling HPD

	SP_TX_Read_Reg(0x70, SP_TX_HDCP_CONTROL_0_REG, &temp);
	SP_TX_Write_Reg(0x70, SP_TX_HDCP_CONTROL_0_REG, temp | 0x03);//set KSV valid

	SP_TX_Read_Reg(0x70, SP_TX_AUX_CTRL_REG2, &temp);
	SP_TX_Write_Reg(0x70, SP_TX_AUX_CTRL_REG2, temp|0x08);//set double AUX output

	SP_TX_Write_Reg(0x72, SP_COMMON_INT_MASK1, 0xbf);//unmask pll change int
	SP_TX_Write_Reg(0x72, SP_COMMON_INT_MASK2, 0xff);//mask all int
	SP_TX_Write_Reg(0x72, SP_COMMON_INT_MASK3, 0xff);//mask all int
	SP_TX_Write_Reg(0x72, SP_COMMON_INT_MASK4, 0xff);//mask all int

	//reset AUX
	SP_TX_Read_Reg(0x72, SP_TX_RST_CTRL2_REG, &temp);
	SP_TX_Write_Reg(0x72, SP_TX_RST_CTRL2_REG, temp |SP_TX_AUX_RST);
	SP_TX_Write_Reg(0x72, SP_TX_RST_CTRL2_REG, temp & (~SP_TX_AUX_RST));

	//Chip initialization

	SP_TX_Write_Reg(0x70, SP_TX_SYS_CTRL1_REG, 0x00);
	mdelay(10);

	SP_TX_Write_Reg(0x72, SP_TX_VID_CTRL2_REG, bits);

	//ANX6345 chip analog setting
	SP_TX_Write_Reg(0x70, SP_TX_PLL_CTRL_REG, 0x00);  //UPDATE: FROM 0X07 TO 0X00

	//ANX chip analog setting
	//SP_TX_Write_Reg(0x72, ANALOG_DEBUG_REG1, 0x70);  //UPDATE: FROM 0XF0 TO 0X70
	SP_TX_Write_Reg(0x70, SP_TX_LINK_DEBUG_REG, 0x30);

	//force HPD
	SP_TX_Write_Reg(0x70, SP_TX_SYS_CTRL3_REG, 0x30);

	/* enable ssc function */
	SP_TX_Write_Reg(0x70, 0xA7, 0x00);          // disable SSC first
	SP_TX_Write_Reg(0x70, 0xD0, 0x5f);         // ssc d  0.4%, f0/4 mode
	SP_TX_Write_Reg(0x70, 0xD1, 0x00);
	SP_TX_Write_Reg(0x70, 0xD2, 0x75);         // ctrl_th
	SP_TX_Read_Reg(0x70, 0xA7, &temp);
	SP_TX_Write_Reg(0x70, 0xA7, temp | 0x10);  // enable SSC
	SP_TX_Read_Reg(0x72, 0x07, &temp);         //reset SSC
	SP_TX_Write_Reg(0x72, 0x07, temp | 0x80);
	SP_TX_Write_Reg(0x72, 0x07, temp & (~0x80));

	//Select 2.7G
	SP_TX_Write_Reg(0x70, SP_TX_LINK_BW_SET_REG, link_rate); //2.7g:0x0a;1.62g:0x06
	//Select 2 lanes
	SP_TX_Write_Reg(0x70, 0xa1, lane_num);

	SP_TX_Write_Reg(0x70, SP_TX_LINK_TRAINING_CTRL_REG, SP_TX_LINK_TRAINING_CTRL_EN);
	mdelay(5);
	SP_TX_Read_Reg(0x70, SP_TX_LINK_TRAINING_CTRL_REG, &temp);
	/* UPDATE: FROM 0X01 TO 0X80 */
	while ((temp & 0x80) != 0) {
		//debug_puts("Waiting...\n");
		mdelay(5);
		count1++;
		if (count1 > 100) {
			LCD_EXT_PR("ANX6345 Link training fail\n");
			break;
		}
		SP_TX_Read_Reg(0x70, SP_TX_LINK_TRAINING_CTRL_REG, &temp);
	}

	SP_TX_Write_Reg(0x72, 0x12, 0x2c);
	SP_TX_Write_Reg(0x72, 0x13, 0x06);
	SP_TX_Write_Reg(0x72, 0x14, 0x00);
	SP_TX_Write_Reg(0x72, 0x15, 0x06);
	SP_TX_Write_Reg(0x72, 0x16, 0x02);
	SP_TX_Write_Reg(0x72, 0x17, 0x04);
	SP_TX_Write_Reg(0x72, 0x18, 0x26);
	SP_TX_Write_Reg(0x72, 0x19, 0x50);
	SP_TX_Write_Reg(0x72, 0x1a, 0x04);
	SP_TX_Write_Reg(0x72, 0x1b, 0x00);
	SP_TX_Write_Reg(0x72, 0x1c, 0x04);
	SP_TX_Write_Reg(0x72, 0x1d, 0x18);
	SP_TX_Write_Reg(0x72, 0x1e, 0x00);
	SP_TX_Write_Reg(0x72, 0x1f, 0x10);
	SP_TX_Write_Reg(0x72, 0x20, 0x00);
	SP_TX_Write_Reg(0x72, 0x21, 0x28);

	SP_TX_Write_Reg(0x72, 0x11, 0x03);
	//enable BIST. In normal mode, don't need to config this reg
	//if want to open BIST,must setting right dat 0x11-0x21 base lcd timing.
	//SP_TX_Write_Reg(0x72, 0x0b, 0x09);//colorbar:08,graystep:09
	SP_TX_Write_Reg(0x72, 0x08, 0x81); //SDR:0x81;DDR:0x8f

	//force HPD and stream valid
	SP_TX_Write_Reg(0x70, 0x82, 0x33);

	LCD_EXT_PR("%s: %s\n", __func__, ext_config->name);
	return 0;
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

#ifdef CONFIG_USE_OF
static int aml_lcd_extern_get_dt_config(int index)
{
	int err;
	struct device_node* child;

	child = aml_lcd_extern_get_dt_child(index);
	if (child == NULL) {
		LCD_EXT_PR("can't get dt child %d\n", index);
		return -1;
	}

	err = of_property_read_u32(child, "lane_num", &edp_parameter.lane_num);
	if (err)
		LCD_EXT_PR("get index failed, use default setting\n");
	DBG_PRINT ("lane_num = %d\n", edp_parameter.lane_num);

	err = of_property_read_u32(child, "bits", &edp_parameter.bits);
	if (err)
		LCD_EXT_PR("get bits failed, use default setting\n");
	DBG_PRINT ("bits = %d\n", edp_parameter.bits);

	err = of_property_read_u32(child, "link_rate", &edp_parameter.link_rate);
	if (err)
		LCD_EXT_PR("get link_rate failed, use default setting\n");
	DBG_PRINT ("link_rate = %d\n", edp_parameter.link_rate);

	return 0;
}
#endif

static int aml_anx6345_70_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		LCD_EXT_PR("[error] %s: functionality check failed\n", __FUNCTION__);
		return -ENODEV;
	} else {
		aml_anx6345_70_client = client;
	}

	LCD_EXT_PR("%s OK\n", __func__);
	return 0;
}

static int aml_anx6345_72_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		LCD_EXT_PR("[error] %s: functionality check failed\n", __func__);
		return -ENODEV;
	} else {
		aml_anx6345_72_client = client;
	}

	LCD_EXT_PR("%s OK\n", __func__);
	return 0;
}

static int aml_anx6345_70_remove(struct i2c_client *client)
{
	return 0;
}
static int aml_anx6345_72_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id aml_anx6345_70_id[] = {
	{"anx6345_70", 0},
	{ }
};
static const struct i2c_device_id aml_anx6345_72_id[] = {
	{"anx6345_72", 0},
	{ }
};

// MODULE_DEVICE_TABLE(i2c, aml_tc101_id);

static struct i2c_driver aml_anx6345_70_driver = {
	.probe    = aml_anx6345_70_probe,
	.remove   = aml_anx6345_70_remove,
	.id_table = aml_anx6345_70_id,
	.driver = {
		.name = "anx6345_70",
		.owner =THIS_MODULE,
	},
};
static struct i2c_driver aml_anx6345_72_driver = {
	.probe    = aml_anx6345_72_probe,
	.remove   = aml_anx6345_72_remove,
	.id_table = aml_anx6345_72_id,
	.driver = {
		.name = "anx6345_72",
		.owner =THIS_MODULE,
	},
};


int aml_lcd_extern_i2c_anx6345_probe(struct aml_lcd_extern_driver_t *ext_drv)
{
	struct i2c_board_info i2c_info[2];
	struct i2c_adapter *adapter;
	struct i2c_client *i2c_client;
	int i = 0;
	int ret = 0;

	ext_config = &ext_drv->config;
	aml_lcd_extern_get_dt_config(ext_drv->config.index);

	for (i = 0; i < 2; i++)
		memset(&i2c_info[i], 0, sizeof(i2c_info[i]));

	adapter = i2c_get_adapter(ext_drv->config.i2c_bus);
	if (!adapter) {
		LCD_EXT_PR("%s failed to get i2c adapter\n", ext_drv->config.name);
		return -1;
	}
	for (i = 0; i < 2; i++)
		strncpy(i2c_info[i].type, anx_addr_name[i], I2C_NAME_SIZE);
	i2c_info[0].addr = (0x70 >> 1);
	i2c_info[1].addr = (0x72 >> 1);
	for (i = 0; i < 2; i++) {
		i2c_info[i].flags = 0;
		if (i2c_info[i].addr > 0x7f) {
			LCD_EXT_PR("%s invalid i2c address: 0x%02x\n", ext_drv->config.name, i2c_info[i].addr);
			return -1;
		}
		i2c_client = i2c_new_device(adapter, &i2c_info[i]);
		if (!i2c_client) {
			LCD_EXT_PR("%s(0x%02x) failed to new i2c device\n", ext_drv->config.name, i2c_info[i].addr);
			return -1;
		} else {
			DBG_PRINT("%s(0x%02x) new i2c device succeed\n", ext_drv->config.name, i2c_info[i].addr);
		}
	}

	if (!aml_anx6345_70_client) {
		ret = i2c_add_driver(&aml_anx6345_70_driver);
		if (ret) {
			LCD_EXT_PR("%s add i2c_driver_70 failed\n", ext_drv->config.name);
			return -1;
		}
	}

	if (!aml_anx6345_72_client) {
		ret = i2c_add_driver(&aml_anx6345_72_driver);
		if (ret) {
			LCD_EXT_PR("%s add i2c_driver_72 failed\n", ext_drv->config.name);
			return -1;
		}
	}

	ret = lcd_extern_driver_update(ext_drv);

	DBG_PRINT("%s: %d\n", __func__, ret);
	return ret;
}

