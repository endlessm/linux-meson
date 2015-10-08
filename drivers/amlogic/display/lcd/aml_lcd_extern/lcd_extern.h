
#ifndef _LCD_EXTERN_H_
#define _LCD_EXTERN_H_
#include <linux/of.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/pinctrl/consumer.h>

//#define LCD_EXT_DEBUG_INFO
#ifdef LCD_EXT_DEBUG_INFO
#define DBG_PRINT(format, arg...)        printk("lcd extern: " format, ## arg)
#else
#define DBG_PRINT(format, arg...)
#endif
#define LCD_EXT_PR(format, arg...)        printk("lcd extern: " format, ## arg)

#define LCD_EXTERN_DRIVER		"lcd_extern"

#define lcd_ext_gpio_request(gpio)			amlogic_gpio_request(gpio, LCD_EXTERN_DRIVER)
#define lcd_ext_gpio_free(gpio)				amlogic_gpio_free(gpio, LCD_EXTERN_DRIVER)
#define lcd_ext_gpio_direction_input(gpio)		amlogic_gpio_direction_input(gpio, LCD_EXTERN_DRIVER)
#define lcd_ext_gpio_direction_output(gpio, val)	amlogic_gpio_direction_output(gpio, val, LCD_EXTERN_DRIVER)
#define lcd_ext_gpio_get_value(gpio)			amlogic_get_value(gpio, LCD_EXTERN_DRIVER)
#define lcd_ext_gpio_set_value(gpio,val)		amlogic_set_value(gpio, val, LCD_EXTERN_DRIVER)

#ifdef CONFIG_USE_OF
extern struct device_node *aml_lcd_extern_get_dt_child(int index);
#endif

#ifdef CONFIG_AML_LCD_EXTERN_I2C_T5800Q
extern int aml_lcd_extern_i2c_T5800Q_probe(struct aml_lcd_extern_driver_t *ext_drv);
#endif
#ifdef CONFIG_AML_LCD_EXTERN_I2C_TC101
extern int aml_lcd_extern_i2c_tc101_probe(struct aml_lcd_extern_driver_t *ext_drv);
#endif
#ifdef CONFIG_AML_LCD_EXTERN_I2C_ANX6345
extern int aml_lcd_extern_i2c_anx6345_probe(struct aml_lcd_extern_driver_t *ext_drv);
#endif
#ifdef CONFIG_AML_LCD_EXTERN_SPI_LD070WS2
extern int aml_lcd_extern_spi_LD070WS2_probe(struct aml_lcd_extern_driver_t *ext_drv);
#endif
#ifdef CONFIG_AML_LCD_EXTERN_MIPI_N070ICN
extern int aml_lcd_extern_mipi_N070ICN_probe(struct aml_lcd_extern_driver_t *ext_drv);
#endif
#ifdef CONFIG_AML_LCD_EXTERN_MIPI_KD080D13
extern int aml_lcd_extern_mipi_KD080D13_probe(struct aml_lcd_extern_driver_t *ext_drv);
#endif

#endif

