#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c-aml.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <mach/am_regs.h>
#include <mach/gpio.h>
#include <linux/amlogic/vout/aml_lcd_extern.h>
#include "lcd_extern.h"

#ifdef CONFIG_USE_OF
static struct device_node *lcd_extern_node;
#endif
static int lcd_ext_driver_num;
static struct aml_lcd_extern_driver_t *lcd_ext_driver[LCD_EXT_DRIVER_MAX];

struct aml_lcd_extern_driver_t *aml_lcd_extern_get_driver(int index)
{
	int i;
	struct aml_lcd_extern_driver_t *ext_driver = NULL;

	if (index >= LCD_EXTERN_INDEX_INVALID) {
		LCD_EXT_PR("invalid driver index %d\n", index);
		return NULL;
	}
	for (i = 0; i < lcd_ext_driver_num; i++) {
		if (lcd_ext_driver[i]->config.index == index) {
			ext_driver = lcd_ext_driver[i];
			break;
		}
	}
	if (ext_driver == NULL)
		LCD_EXT_PR("invalid driver index %d\n", index);
	return ext_driver;
}

#if 0
static struct aml_lcd_extern_driver_t *aml_lcd_extern_get_driver_by_name(char *name)
{
	int i;
	struct aml_lcd_extern_driver_t *ext_driver = NULL;

	for (i = 0; i < lcd_ext_driver_num; i++) {
		if (strcmp(lcd_ext_driver[i]->config.name, name) == 0) {
			ext_driver = lcd_ext_driver[i];
			break;
		}
	}
	if (ext_driver == NULL)
		LCD_EXT_PR("invalid driver index %d\n", index);
	return ext_driver;
}
#endif

#ifdef CONFIG_USE_OF
struct device_node *aml_lcd_extern_get_dt_child(int index)
{
	char propname[30];
	struct device_node* child;

	sprintf(propname, "extern_%d", index);
	child = of_get_child_by_name(lcd_extern_node, propname);
	return child;
}

static int aml_lcd_extern_get_dt_config(struct device_node *of_node, struct lcd_extern_config_t *econfig)
{
	int err;
	int val;
	const char *str;

	err = of_property_read_u32(of_node, "index", &econfig->index);
	if (err) {
		LCD_EXT_PR("get index failed, exit\n");
		return -1;
	}

	err = of_property_read_string(of_node, "extern_name", &str);
	if (err) {
		str = "none";
		LCD_EXT_PR("get extern_name failed\n");
	}
	strcpy(econfig->name, str);
	LCD_EXT_PR("load config in dtb: %s[%d]\n", econfig->name, econfig->index);

	err = of_property_read_u32(of_node, "type", &econfig->type);
	if (err) {
		econfig->type = LCD_EXTERN_MAX;
		LCD_EXT_PR("error: get type failed, exit\n");
		return -1;
	}

	switch (econfig->type) {
	case LCD_EXTERN_I2C:
		err = of_property_read_u32(of_node, "i2c_address", &econfig->i2c_addr);
		if (err) {
			LCD_EXT_PR("get %s i2c_address failed, exit\n", econfig->name);
			econfig->i2c_addr = 0;
			return -1;
		}
		DBG_PRINT("%s i2c_address=0x%02x\n", econfig->name, econfig->i2c_addr);

		err = of_property_read_string(of_node, "i2c_bus", &str);
		if (err) {
			LCD_EXT_PR("get %s i2c_bus failed, exit\n", econfig->name);
			econfig->i2c_bus = AML_I2C_MASTER_A;
			return -1;
		} else {
			if (strncmp(str, "i2c_bus_a", 9) == 0)
				econfig->i2c_bus = AML_I2C_MASTER_A;
			else if (strncmp(str, "i2c_bus_b", 9) == 0)
				econfig->i2c_bus = AML_I2C_MASTER_B;
			else if (strncmp(str, "i2c_bus_c", 9) == 0)
				econfig->i2c_bus = AML_I2C_MASTER_C;
			else if (strncmp(str, "i2c_bus_d", 9) == 0)
				econfig->i2c_bus = AML_I2C_MASTER_D;
			else if (strncmp(str, "i2c_bus_ao", 10) == 0)
				econfig->i2c_bus = AML_I2C_MASTER_AO;
			else
				econfig->i2c_bus = AML_I2C_MASTER_A;
		}
		DBG_PRINT("%s i2c_bus=%s[%d]\n", econfig->name, str, econfig->i2c_bus);
		break;
	case LCD_EXTERN_SPI:
		err = of_property_read_string(of_node, "gpio_spi_cs", &str);
		if (err) {
			LCD_EXT_PR("get %s gpio_spi_cs failed, exit\n", econfig->name);
			econfig->spi_cs = -1;
			return -1;
		} else {
			val = amlogic_gpio_name_map_num(str);
			if (val > 0) {
				err = lcd_ext_gpio_request(val);
				if (err)
					LCD_EXT_PR("faild to alloc spi_cs gpio (%s)\n", str);
				econfig->spi_cs = val;
				DBG_PRINT("spi_cs gpio = %s(%d)\n", str, econfig->spi_cs);
			} else {
				econfig->spi_cs = -1;
			}
		}
		err = of_property_read_string(of_node, "gpio_spi_clk", &str);
		if (err) {
			LCD_EXT_PR("get %s gpio_spi_clk failed, exit\n", econfig->name);
			econfig->spi_clk = -1;
			return -1;
		} else {
			val = amlogic_gpio_name_map_num(str);
			if (val > 0) {
				err = lcd_ext_gpio_request(val);
				if (err)
					LCD_EXT_PR("faild to alloc spi_clk gpio (%s)\n", str);
				econfig->spi_clk = val;
				DBG_PRINT("spi_clk gpio = %s(%d)\n", str, econfig->spi_clk);
			} else {
				econfig->spi_clk = -1;
			}
		}
		err = of_property_read_string(of_node, "gpio_spi_data", &str);
		if (err) {
			LCD_EXT_PR("get %s gpio_spi_data failed, exit\n", econfig->name);
			econfig->spi_data = -1;
			return -1;
		} else {
			val = amlogic_gpio_name_map_num(str);
			if (val > 0) {
				err = lcd_ext_gpio_request(val);
				if (err)
					LCD_EXT_PR("faild to alloc spi_data gpio (%s)\n", str);
				econfig->spi_data = val;
				DBG_PRINT("spi_data gpio = %s(%d)\n", str, econfig->spi_data);
			} else {
				econfig->spi_data = -1;
			}
		}
		break;
	case LCD_EXTERN_MIPI:
		break;
	default:
		break;
	}

	return 0;
}
#endif

static int aml_lcd_extern_add_i2c(struct aml_lcd_extern_driver_t *ext_drv)
{
	int ret = 0;

	if (strcmp(ext_drv->config.name, "i2c_T5800Q") == 0) {
#ifdef CONFIG_AML_LCD_EXTERN_I2C_T5800Q
		ret = aml_lcd_extern_i2c_T5800Q_probe(ext_drv);
#endif
	} else if (strcmp(ext_drv->config.name, "i2c_tc101") == 0) {
#ifdef CONFIG_AML_LCD_EXTERN_I2C_TC101
		ret = aml_lcd_extern_i2c_tc101_probe(ext_drv);
#endif
	} else if (strcmp(ext_drv->config.name, "i2c_anx6345") == 0) {
#ifdef CONFIG_AML_LCD_EXTERN_I2C_ANX6345
		ret = aml_lcd_extern_i2c_anx6345_probe(ext_drv);
#endif
	} else {
		LCD_EXT_PR("invalid driver name: %s\n", ext_drv->config.name);
		ret = -1;
	}
	return ret;
}

static int aml_lcd_extern_add_spi(struct aml_lcd_extern_driver_t *ext_drv)
{
	int ret = 0;

	if (strcmp(ext_drv->config.name, "spi_LD070WS2") == 0) {
#ifdef CONFIG_AML_LCD_EXTERN_SPI_LD070WS2
		ret = aml_lcd_extern_spi_LD070WS2_probe(ext_drv);
#endif
	} else {
		LCD_EXT_PR("invalid driver name: %s\n", ext_drv->config.name);
		ret = -1;
	}
	return ret;
}

static int aml_lcd_extern_add_mipi(struct aml_lcd_extern_driver_t *ext_drv)
{
	int ret = 0;

	if (strcmp(ext_drv->config.name, "mipi_N070ICN") == 0) {
#ifdef CONFIG_AML_LCD_EXTERN_MIPI_N070ICN
		ret = aml_lcd_extern_mipi_N070ICN_probe(ext_drv);
#endif
	} else if (strcmp(ext_drv->config.name, "mipi_KD080D13") == 0) {
#ifdef CONFIG_AML_LCD_EXTERN_MIPI_KD080D13
		ret = aml_lcd_extern_mipi_KD080D13_probe(ext_drv);
#endif
	} else {
		LCD_EXT_PR("invalid driver name: %s\n", ext_drv->config.name);
		ret = -1;
	}
	return ret;
}

static int aml_lcd_extern_add_invalid(struct aml_lcd_extern_driver_t *ext_drv)
{
	return -1;
}

static int aml_lcd_extern_add_driver(struct lcd_extern_config_t *econfig)
{
	struct aml_lcd_extern_driver_t *ext_drv;
	int ret = 0;

	if (lcd_ext_driver_num >= LCD_EXT_DRIVER_MAX) {
		LCD_EXT_PR("driver num is too much\n");
		return -1;
	}
	lcd_ext_driver[lcd_ext_driver_num] = kmalloc(sizeof(struct aml_lcd_extern_driver_t), GFP_KERNEL);
	if (lcd_ext_driver[lcd_ext_driver_num] == NULL) {
		LCD_EXT_PR("failed to alloc driver %s[%d], not enough memory\n", econfig->name, econfig->index);
		return -1;
	}

	ext_drv = lcd_ext_driver[lcd_ext_driver_num];
	/* fill config parameters */
	ext_drv->config.index = econfig->index;
	strcpy(ext_drv->config.name, econfig->name);
	ext_drv->config.type = econfig->type;

	/* fill config parameters by different type */
	switch (ext_drv->config.type) {
	case LCD_EXTERN_I2C:
		ext_drv->config.i2c_addr = econfig->i2c_addr;
		ext_drv->config.i2c_bus = econfig->i2c_bus;
		ret = aml_lcd_extern_add_i2c(ext_drv);
		break;
	case LCD_EXTERN_SPI:
		ext_drv->config.spi_cs = econfig->spi_cs;
		ext_drv->config.spi_clk = econfig->spi_clk;
		ext_drv->config.spi_data = econfig->spi_data;
		ret = aml_lcd_extern_add_spi(ext_drv);
		break;
	case LCD_EXTERN_MIPI:
		ret = aml_lcd_extern_add_mipi(ext_drv);
		break;
	default:
		ret = aml_lcd_extern_add_invalid(ext_drv);
		LCD_EXT_PR("don't support type %d\n", ext_drv->config.type);
		break;
	}
	if (ret) {
		LCD_EXT_PR("add driver failed\n");
		kfree(lcd_ext_driver[lcd_ext_driver_num]);
		lcd_ext_driver[lcd_ext_driver_num] = NULL;
		return -1;
	}
	lcd_ext_driver_num++;
	LCD_EXT_PR("add driver %s(%d)\n", ext_drv->config.name, ext_drv->config.index);
	return 0;
}

//*********************************************************
//debug function
//*********************************************************
static void aml_lcd_extern_config_dump(struct aml_lcd_extern_driver_t *ext_drv)
{
	if (ext_drv == NULL)
		return;

	printk("    index:       %d\n"
		"    name:        %s\n",
		ext_drv->config.index, ext_drv->config.name);
	switch (ext_drv->config.type) {
	case LCD_EXTERN_I2C:
		printk("    type:        i2c(%d)\n", ext_drv->config.type);
		printk("    i2c_addr:    0x%02x\n"
			"    i2c_bus:     %d\n",
			ext_drv->config.i2c_addr, ext_drv->config.i2c_bus);
		break;
	case LCD_EXTERN_SPI:
		printk("    type:        spi(%d)\n", ext_drv->config.type);
		printk("    spi_cs:      %d\n"
			"    spi_clk:     %d\n"
			"    spi_data:    %d\n",
			ext_drv->config.spi_cs, ext_drv->config.spi_clk, ext_drv->config.spi_data);
		break;
	case LCD_EXTERN_MIPI:
		printk("    type:        mipi(%d)\n", ext_drv->config.type);
		break;
	default:
		break;
	}
	printk("\n");
}

static const char * lcd_extern_debug_usage_str = {
"Usage:\n"
"    echo index <n> > info ; dump specified index driver config\n"
"    echo all > info ; dump all driver config\n"
};

static ssize_t lcd_extern_debug_help(struct class *class, struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", lcd_extern_debug_usage_str);
}

static ssize_t lcd_extern_info_dump(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	unsigned int ret;
	int i, index;
	struct aml_lcd_extern_driver_t *ext_drv;

	index = LCD_EXTERN_INDEX_INVALID;
	switch (buf[0]) {
	case 'i':
		ret = sscanf(buf, "index %d", &index);
		ext_drv = aml_lcd_extern_get_driver(index);
		aml_lcd_extern_config_dump(ext_drv);
		break;
	case 'a':
		for (i = 0; i < lcd_ext_driver_num; i++)
			aml_lcd_extern_config_dump(lcd_ext_driver[i]);
		break;
	default:
		LCD_EXT_PR("don't support command\n");
		break;
	}

	if (ret != 1 || ret !=2)
		return -EINVAL;

	return count;
}

static struct class_attribute lcd_extern_class_attrs[] = {
	__ATTR(info, S_IRUGO | S_IWUSR, lcd_extern_debug_help, lcd_extern_info_dump),
};

static struct class *debug_class;
static int creat_lcd_extern_class(void)
{
	int i;

	debug_class = class_create(THIS_MODULE, "lcd_ext");
	if (IS_ERR(debug_class)) {
		LCD_EXT_PR("create debug class failed\n");
		return -1;
	}

	for (i = 0; i < ARRAY_SIZE(lcd_extern_class_attrs); i++) {
		if (class_create_file(debug_class, &lcd_extern_class_attrs[i]))
			LCD_EXT_PR("create debug attribute %s failed\n", lcd_extern_class_attrs[i].attr.name);
	}

	return 0;
}

static int remove_lcd_extern_class(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(lcd_extern_class_attrs); i++)
		class_remove_file(debug_class, &lcd_extern_class_attrs[i]);

	class_destroy(debug_class);
	debug_class = NULL;

	return 0;
}
//*********************************************************

static int aml_lcd_extern_probe(struct platform_device *pdev)
{
	struct device_node *child;
	struct lcd_extern_config_t ext_config;
	int ret = 0;

	lcd_ext_driver_num = 0;
#ifdef CONFIG_USE_OF
	lcd_extern_node = pdev->dev.of_node;
	for_each_child_of_node(lcd_extern_node, child) {
		ret = aml_lcd_extern_get_dt_config(child, &ext_config);
		if (ret == 0)
			aml_lcd_extern_add_driver(&ext_config);
	}
#endif
	creat_lcd_extern_class();

	LCD_EXT_PR("%s ok\n", __func__);
	return ret;
}

static int aml_lcd_extern_remove(struct platform_device *pdev)
{
	int i;

	remove_lcd_extern_class();
	for (i = 0; i < lcd_ext_driver_num; i++) {
		kfree(lcd_ext_driver[i]);
		lcd_ext_driver[i] = NULL;
	}
	return 0;
}

#ifdef CONFIG_USE_OF
static const struct of_device_id aml_lcd_extern_dt_match[] = {
	{
		.compatible = "amlogic,lcd_extern",
	},
	{},
};
#endif

static struct platform_driver aml_lcd_extern_driver = {
	.probe  = aml_lcd_extern_probe,
	.remove = aml_lcd_extern_remove,
	.driver = {
		.name  = "lcd_extern",
		.owner = THIS_MODULE,
#ifdef CONFIG_USE_OF
		.of_match_table = aml_lcd_extern_dt_match,
#endif
	},
};

static int __init aml_lcd_extern_init(void)
{
	int ret;
	DBG_PRINT("%s\n", __func__);

	ret = platform_driver_register(&aml_lcd_extern_driver);
	if (ret) {
		LCD_EXT_PR("lcd_extern: [error] lcd_extern_driver register failed\n");
		return -ENODEV;
	}
	return ret;
}

static void __exit aml_lcd_extern_exit(void)
{
	platform_driver_unregister(&aml_lcd_extern_driver);
}

module_init(aml_lcd_extern_init);
module_exit(aml_lcd_extern_exit);

MODULE_AUTHOR("AMLOGIC");
MODULE_DESCRIPTION("LCD extern driver");
MODULE_LICENSE("GPL");

