
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/sysfs.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/i2c-aml.h>
#include <linux/amlogic/aml_gpio_consumer.h>

#define _I2C_ADDR 0x1A
#define MODULE_NAME "aml_projector"
static struct i2c_adapter *gp_adapter = NULL;
static char g_power = 0;
static char g_enable = 0;

struct pro_data_t {
	int i2c_bus_nr;
	char i2c_addr;
	const char *gpio_name;
	int gpio_num;

	struct class *config_class;
	struct device *config_dev;
}g_pro_data_t;	

#define PWR_REG 0x87
#define CONTRAST_REG (0x80 + 0x24) // RGB Percentage from 0% to 200% with 100% as normal 
#define BRIGHTNESS_REG (0x80 + 0x23) //RGB channel offset adjustment range: -256 to 255.75 inclusive.
#define YFLIP_REG (0x80 + 0x8) // long axis 
#define XFLIP_REG (0x80 + 0x9) // short axis
#define CCI_REG (0x80 + 0x57) // Color Correction Interface, val bits[7:2]:reserved, bit[1]:0-CCA Adjust mode,1-Current Adjust mode; bit[0]:0-disable,1-enable
/* Brilliant Color, The processing is: Disable CCI -> change the BC look -> enable CCI */
#define BRILLIANTCOLOR_REG (0x80 + 0x32) 
#define TEMPERATURE_REG (0x80 + 0x64)
#define ELECTRAP_REG  (0xE0)

#define X_AXIS 0 
#define Y_AXIS 1 
/**
 * @i2c read function
 *
 */
static int aml_i2c_read_buff(struct i2c_adapter *adapter, 
		unsigned short dev_addr, char *buf, int addr_len, char *dest, int data_len)
{
	int  i2c_flag = -1;
	struct i2c_msg msgs[] = {
		{
			.addr	= dev_addr,
			.flags	= 0,
			.len	= addr_len,
			.buf	= buf,
		},{
			.addr	= dev_addr,
			.flags	= I2C_M_RD,
			.len	= data_len,
			.buf	= dest,
		}
	};

	i2c_flag = i2c_transfer(adapter, msgs, 2);

	return i2c_flag;
}

/**
 * @i2c write function
 *
 */
static int aml_i2c_write_buff(struct i2c_adapter *adapter, 
				unsigned short dev_addr, char *buf, int len)
{
	struct i2c_msg msg[] = {
		{
			.addr	= dev_addr,
			.flags	= 0,    //|I2C_M_TEN,
			.len	= len,
			.buf	= buf,
		}
	};

	if (i2c_transfer(adapter, msg, 1) < 0) {
		return -1;
	} else
		return 0;
}

static int amlP_read(char *reg, int reglen, char *buf, int len)
{
	return aml_i2c_read_buff(gp_adapter, g_pro_data_t.i2c_addr, reg, reglen, buf, len);
}

static int amlP_write(char *buf, int len)
{
	return aml_i2c_write_buff(gp_adapter, g_pro_data_t.i2c_addr, buf, len);
}

int CCI_read(void)
{
	char reg = CCI_REG;
	char sbuf[1] = {0};

	amlP_read(&reg, 1, sbuf, sizeof(sbuf));
	pr_info("%s called, val:%d \n", __func__, sbuf[0]);
	return sbuf[0];
}

int CCI_set(int val)
{
	char sbuf[2] = {0};
	
	sbuf[0] = CCI_REG;
	sbuf[1] = val;
	pr_info("%s called, val:%d \n", __func__, sbuf[1]);
	
	return amlP_write(sbuf, sizeof(sbuf));
}
int flip_read(int axis)
{
	char reg = axis == X_AXIS ? XFLIP_REG : YFLIP_REG;
	char sbuf[1] = {0};

	amlP_read(&reg, 1, sbuf, sizeof(sbuf));
	pr_info("%s called, val:%d \n", __func__, sbuf[0]);
	return sbuf[0];
}

int flip_set(int axis, int val)
{
	char sbuf[2] = {0};
	
	sbuf[0] = axis==X_AXIS? XFLIP_REG : YFLIP_REG;
	sbuf[1] = val;
	pr_info("%s called, val:%d \n", __func__, sbuf[1]);
	
	return amlP_write(sbuf, sizeof(sbuf));
}

static ssize_t Xflip_show(struct class *dev, struct class_attribute *attr,
			   char *buf)
{
	int val = flip_read(X_AXIS);
	/* Format the output string and return # of bytes */
	return sprintf(buf, "%d\n", val);
}

static ssize_t Xflip_store(struct class *dev, struct class_attribute *attr,
			    const char *buf, size_t count)
{
	int ret=0;
	long val = 0;
	
	ret = kstrtoul(buf, 10, &val);
	pr_info("%s called, val:%ld \n", __func__, val);
	flip_set(X_AXIS, val);

	return count;
}

static ssize_t Yflip_show(struct class *dev, struct class_attribute *attr,
			   char *buf)
{
	int val = flip_read(Y_AXIS);
	/* Format the output string and return # of bytes */
	return sprintf(buf, "%d\n", val);
}

static ssize_t Yflip_store(struct class *dev, struct class_attribute *attr,
			    const char *buf, size_t count)
{
	int ret=0;
	long val = 0;
	
	ret = kstrtoul(buf, 10, &val);
	pr_info("%s called, val:%ld \n", __func__, val);
	flip_set(Y_AXIS, val);

	return count;
}

static ssize_t CCI_show(struct class *dev, struct class_attribute *attr,
			   char *buf)
{
	char reg = CCI_REG;
	char sbuf[2] = {0};
	amlP_read(&reg, 1, sbuf, 1);
	pr_info("%s calledm buf:0x%02x \n", __func__, sbuf[0]);

	/* Format the output string and return # of bytes */
	return sprintf(buf, "0x%02x\n", sbuf[0]);
}

static ssize_t CCI_store(struct class *dev, struct class_attribute *attr,
			    const char *buf, size_t count)
{
	char val = 0;
	long tmp_l = 0;
	char sbuf[2] = {CCI_REG, 0};
	int ret = 0;

	ret = kstrtol(buf, 10, &tmp_l);
	val = tmp_l;
	pr_info("%s called, val:%d \n", __func__, val);

	sbuf[1] = val;
	amlP_write(sbuf, sizeof(sbuf));

	return count;
}
static ssize_t brilliantColor_show(struct class *dev, struct class_attribute *attr,
			   char *buf)
{
	char reg = BRILLIANTCOLOR_REG;
	char sbuf[2] = {0};
	amlP_read(&reg, 1, sbuf, 1);
	pr_info("%s calledm buf:0x%02x \n", __func__, sbuf[0]);

	/* Format the output string and return # of bytes */
	return sprintf(buf, "0x%02x\n", sbuf[0]);
}

static ssize_t brilliantColor_store(struct class *dev, struct class_attribute *attr,
			    const char *buf, size_t count)
{
	char val = 0;
	long tmp_l = 0;
	int ret = 0;
	char sbuf[2] = {BRILLIANTCOLOR_REG, 0};

	ret = kstrtol(buf, 16, &tmp_l);
	val = tmp_l;
	pr_info("%s called, val:%d \n", __func__, val);

	//disable CCI first
	//tmp_l = CCI_read();
	tmp_l = 0;
	CCI_set(tmp_l);
	msleep(10);
	//change BC look
	sbuf[1] = val ; // 0-basic, 1-6500k, 2-7500k, 3-7250k, 4-Overlap, 5-3D
	//sbuf[1] |= val & 0x80; // 1-enable or 0-disable
	amlP_write(sbuf, sizeof(sbuf));
	msleep(10);
	//enable CCI
	tmp_l |= 1; 
	CCI_set(tmp_l);
	//for stability
	msleep(500);

	return count;
}
static ssize_t brightness_show(struct class *dev, struct class_attribute *attr,
			   char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	char *rgb = "RGB";
	unsigned char reg = BRIGHTNESS_REG;
	char sbuf[6] = {0};
	char tmp_buf[128] = {0};
	int ret = 0;

	amlP_read((char *)&reg, 1, sbuf, 6);
	pr_info("%s called, brightness reg val[0-5]:", __func__);
	for(reg=0; reg<6; reg++){
		pr_info("0x%02x ", sbuf[reg]);
	}
	pr_info(",");
	for(reg=0; reg<3; reg++){
		ret += sprintf(tmp_buf + ret, "%c:", rgb[reg]);
		ret += sprintf(tmp_buf + ret, "%d ", sbuf[reg*2+1]|(sbuf[reg*2]<<8));
	}
	pr_info("%s\n", tmp_buf);

	/* Format the output string and return # of bytes */
	return sprintf(buf, "%s\n", tmp_buf);
}

static ssize_t brightness_store(struct class *dev, struct class_attribute *attr,
			    const char *buf, size_t count)
{
	int val[4] = {0};
	char sbuf[7] = {BRIGHTNESS_REG, 0};
	int ret=0, i=0;

	pr_info("%s origin buf:%s\n", __func__, buf);
	ret = sscanf(buf, "R%dG%dB%d", val, val+1, val+2);
	pr_info("%s called, val[0-2]:%d, %d, %d \n", __func__, val[0], val[1], val[2]);
	for(i=0; i<3; i++){
		//R G B
		sbuf[i+1] = (val[i]>>8) & 0x3;
		sbuf[i+2] = val[i] & 0xff;
	}
	
	amlP_write(sbuf, sizeof(sbuf));

	return count;
}

static ssize_t temperature_show(struct class *dev, struct class_attribute *attr,
			   char *buf)
{
	char reg = TEMPERATURE_REG;
	short val = 0;
	amlP_read(&reg, 1, (char *)&val, sizeof(val));
	pr_info("%s calledm buf:%d \n", __func__, val/10);

	/* Format the output string and return # of bytes */
	return sprintf(buf, "%d\n", val/10);
}

static ssize_t contrast_show(struct class *dev, struct class_attribute *attr,
			   char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	char reg = CONTRAST_REG;
	char sbuf[2] = {0};
	amlP_read(&reg, 1, sbuf, 1);
	pr_info("%s calledm buf:0x%02x \n", __func__, sbuf[0]);

	/* Format the output string and return # of bytes */
	return sprintf(buf, "0x%02x\n", sbuf[0]);
}

static ssize_t contrast_store(struct class *dev, struct class_attribute *attr,
			    const char *buf, size_t count)
{
	//struct i2c_client *client = to_i2c_client(dev);
	char val = 0;
	long tmp_l = 0;
	char sbuf[2] = {CONTRAST_REG, 0};
	int ret = 0;

	ret = kstrtol(buf, 10, &tmp_l);
	val = tmp_l;
	pr_info("%s called, val:%d \n", __func__, val);

	sbuf[1] = val;
	amlP_write(sbuf, sizeof(sbuf));

	return count;
}

static ssize_t elecTrap_store(struct class *dev, struct class_attribute *attr,
			    const char *buf, size_t count)
{
	short val = 0;
	int tmp_l = 0;
	char sbuf[7] = {ELECTRAP_REG, 0};

	sscanf(buf, "%d", &tmp_l);
	val = tmp_l;
	pr_info("%s called, val:%d, should be in {-40,40} \n", __func__, val);

	if(val >= 0){
		sbuf[1] = val*200/256;
		sbuf[2] = val*200%256;
	}else{
		val = -val;
		sbuf[1] = ((~(val*200) +1) & 0xff00)/256;
		sbuf[2] = (~(val*200) + 1)%256;
	}
	sbuf[3] = 0x01;
	sbuf[4] = 0x66;
	sbuf[5] = 0x01;
	sbuf[6] = 0x00;
	
	amlP_write(sbuf, sizeof(sbuf));
	//for stability
	msleep(500);

	return count;
}
static ssize_t enable_show(struct class *dev, struct class_attribute *attr,
			   char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	char reg = PWR_REG;
	char sbuf[2] = {0};
	amlP_read(&reg, 1, sbuf, 1);
	pr_info("%s calledm buf:0x%02x \n", __func__, sbuf[0]);

	/* Format the output string and return # of bytes */
	return sprintf(buf, "%d\n", g_enable);
}

static ssize_t enable_store(struct class *dev, struct class_attribute *attr,
			    const char *buf, size_t count)
{
	//struct i2c_client *client = to_i2c_client(dev);
	char val;
	char sbuf[2] = {PWR_REG, 0};

	val = !!(buf[0]-'0');

	pr_info("%s called \n", __func__);

	if ( val ) {
		//power on
		amlP_write(sbuf, sizeof(sbuf));
	}else if(!val){
		//power off
		sbuf[1] = 1;
		amlP_write(sbuf, sizeof(sbuf));
	}
	g_enable = val;

	return count;
}

static ssize_t power_show(struct class *dev, struct class_attribute *attr,
			   char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);

	pr_info("%s called \n", __func__);

	/* Format the output string and return # of bytes */
	return sprintf(buf, "%d\n", g_power);
}

static ssize_t power_store(struct class *dev, struct class_attribute *attr,
			    const char *buf, size_t count)
{
	//struct i2c_client *client = to_i2c_client(dev);
	char val;
	char sbuf[2] = {PWR_REG, 0};

	val = !!(buf[0]-'0');

	pr_info("%s called \n", __func__);

	if (!g_power && val ) {
		//power on
		amlogic_gpio_direction_output(g_pro_data_t.gpio_num, 1, MODULE_NAME);
		msleep(150);
		//enable
		amlP_write(sbuf, sizeof(sbuf));
	}else if(g_power && !val){
		//power off
		sbuf[1] = 1;
		amlP_write(sbuf, sizeof(sbuf));

		msleep(350);
		amlogic_gpio_direction_output(g_pro_data_t.gpio_num, 0, MODULE_NAME);
	}
	g_power = val;

	return count;
}

int init_powerctl_gpio(struct pro_data_t *pro_data)
{
	int ret = 0;
	//use gpio GPIOAO_10
	aml_clr_reg32_mask(P_AO_SECURE_REG1, ((1<<8) | (1<<1)));
	ret = amlogic_gpio_name_map_num(pro_data->gpio_name);
	if (ret < 0) {
		printk("aml projector: faild to map gpio:%s !\n", pro_data->gpio_name);
		return -1;
	}
	pro_data->gpio_num = ret;

	amlogic_gpio_request(pro_data->gpio_num, MODULE_NAME);
	//amlogic_gpio_direction_output(pro_data->gpio_num, 1, MODULE_NAME);

	return 0;
}

static int projector_init(struct device_node *node, struct pro_data_t *pro_data)
{
	int ret = 0;
	const char *str, *status;
	u32 addr;
	ret = of_property_read_string(node, "status", &status);
	if(ret < 0){
		pr_info("%s: Failed to read status from device tree for dev \n", __func__);
		return -1;
	}
	if(strncmp("ok", status, 2) != 0){
		pr_info("%s,status is not OK, ignore ....\n", __func__);
		return -1;
	}
	ret = of_property_read_u32(node, "address",&addr);
	if(ret < 0)
	{
		printk("%s: faild to get i2c address for dev \n", __func__);
		return -1;
	}

	pro_data->i2c_addr = addr;

	ret = of_property_read_string(node, "i2c_bus", &str);
	if (ret) {
		pr_err("%s: faild to get i2c_bus str for dev\n", __func__);
		pro_data->i2c_bus_nr = AML_I2C_MASTER_A;
	} else {
		if (!strncmp(str, "i2c_bus_a", 9))
			pro_data->i2c_bus_nr = AML_I2C_MASTER_A;
		else if (!strncmp(str, "i2c_bus_b", 9))
			pro_data->i2c_bus_nr = AML_I2C_MASTER_B;
		else if (!strncmp(str, "i2c_bus_ao", 9))
			pro_data->i2c_bus_nr = AML_I2C_MASTER_AO;
		else
			pro_data->i2c_bus_nr = AML_I2C_MASTER_B;
	}

	gp_adapter = i2c_get_adapter(pro_data->i2c_bus_nr);

	ret = of_property_read_string(node, "gpio_pwr", &str);
	if (ret) {
		pr_err("%s: faild to get gpio name str for dev\n", __func__);
		return -1;
	} else {
		pro_data->gpio_name = str;
		init_powerctl_gpio(pro_data);
	}

	pr_info("aml projector info: i2c bus num:%d, i2c addr:0x%02x, gpio name:%s \n", pro_data->i2c_bus_nr, pro_data->i2c_addr, pro_data->gpio_name);

	return 0;
}
#define __ATTR_RW(_name) __ATTR(_name,0666,_name##_show,_name##_store)
static struct class_attribute amlP_class_attrs[] = {
    __ATTR_RW(power),
    __ATTR_RW(enable),
    __ATTR_RW(Xflip),
    __ATTR_RW(Yflip),
    __ATTR_RW(brilliantColor),
    __ATTR_RW(brightness),
    __ATTR_RW(CCI),
    __ATTR_RW(contrast),
    __ATTR_RO(temperature),
    __ATTR(elecTrap, 0666, NULL, elecTrap_store),
    __ATTR_NULL
};
static struct class amlP_class = {
    .name = "aml_projector",
    .class_attrs = amlP_class_attrs,
};
static int amlP_probe(struct platform_device *pdev)
{
	struct device_node* node = pdev->dev.of_node;
	int rc;
	
	pr_info("%s ... \n", __func__);

	if(node == NULL){
		pr_err("no dev tree node for %s\n", MODULE_NAME);
		return 0;
	}

	memset(&g_pro_data_t, 0, sizeof(g_pro_data_t));

	if (0 > projector_init(node, &g_pro_data_t)) {
		return 0;
	}
	rc = class_register(&amlP_class);
	//rc = sysfs_create_group(&client->dev.kobj, &amlP_group);
	if (rc)
		goto exit;

	return 0;

 exit:
	return rc;
}

static int amlP_remove(struct platform_device *pdev)
{
	class_unregister(&amlP_class);
	return 0;
}

static const struct of_device_id amlP_dt_match[]={
	{	
		.compatible = "amlogic,projector",
	},
	{},
};

static  struct platform_driver aml_projector_driver = {
	.probe		= amlP_probe,
	.remove		= amlP_remove,
	.driver		= {
		.name	= "aml_cams_prober",
		.owner	= THIS_MODULE,
		.of_match_table = amlP_dt_match,
	},
};

static int __init amlP_init(void)
{
	pr_info("%s ....\n", __func__);
	if (platform_driver_register(&aml_projector_driver)){
		pr_err("aml_projector_driver register failed\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit amlP_exit(void)
{
	platform_driver_unregister(&aml_projector_driver);
}

module_init(amlP_init);
module_exit(amlP_exit);

MODULE_AUTHOR("AML AE team SZ");
MODULE_DESCRIPTION("aml_projector driver");
MODULE_LICENSE("GPL");
