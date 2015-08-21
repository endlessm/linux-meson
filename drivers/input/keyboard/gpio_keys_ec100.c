/*
 * Driver for Endless EC-100 GPIO power button
 * We get a single interrupt pulse on a rising edge (button press)
 *
 * Based on gpio_keys:
 *
 * Copyright 2005 Phil Blundell
 * Copyright 2010, 2011 David Jander <david@protonic.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/amlogic/aml_gpio_consumer.h>

struct gpio_button_data {
	const struct gpio_keys_button *button;
	struct input_dev *input;
};

struct gpio_keys_drvdata {
	const struct gpio_keys_platform_data *pdata;
	struct input_dev *input;
	struct gpio_button_data data[0];
};

static irqreturn_t gpio_keys_irq_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;

	/* Report key press followed by key release */
	input_event(input, EV_KEY, button->code, 1);
	input_sync(input);
	input_event(input, EV_KEY, button->code, 0);
	input_sync(input);

	return IRQ_HANDLED;
}

static int gpio_keys_setup_key(struct platform_device *pdev,
				struct input_dev *input,
				struct gpio_button_data *bdata,
				struct gpio_keys_button *button)
{
	struct device *dev = &pdev->dev;
	int error;

	bdata->input = input;
	bdata->button = button;
	input_set_capability(input, EV_KEY, button->code);

	amlogic_gpio_request(button->gpio, "gpio_keys");
	amlogic_gpio_direction_input(button->gpio, "gpio_keys");
	amlogic_set_pull_up_down(button->gpio, 1, "gpio_keys");

	amlogic_gpio_to_irq(button->gpio, "gpio_keys",
                    AML_GPIO_IRQ(6, FILTER_NUM7, GPIO_IRQ_RISING));
	button->irq = INT_GPIO_6;

	error = request_any_context_irq(button->irq, gpio_keys_irq_isr, IRQF_SHARED, "gpio_keys", bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			button->irq, error);
		return error;
	}

	return 0;
}

static struct gpio_keys_platform_data *
gpio_keys_get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	struct gpio_keys_platform_data *pdata;
	struct gpio_keys_button *button;
	int error;
	int nbuttons;
	int i;

	node = dev->of_node;
	if (!node)
		return ERR_PTR(-ENODEV);

	nbuttons = of_get_child_count(node);
	if (nbuttons == 0)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev, sizeof(*pdata) + nbuttons * (sizeof *button),
			GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->buttons = (struct gpio_keys_button *)(pdata + 1);
	pdata->nbuttons = nbuttons;

	i = 0;
	for_each_child_of_node(node, pp) {
		const char *gpio_name;
		int gpio;

		error = of_property_read_string(pp, "gpio", &gpio_name);
		if (error) {
			pdata->nbuttons--;
			dev_warn(dev, "Found button without gpio\n");
			continue;
		}

		gpio = amlogic_gpio_name_map_num(gpio_name);
		if (gpio < 0)
			return ERR_PTR(gpio);

		button = &pdata->buttons[i++];

		button->gpio = gpio;

		if (of_property_read_u32(pp, "linux,code", &button->code)) {
			dev_err(dev, "Button without keycode: 0x%x\n",
				button->gpio);
			return ERR_PTR(-EINVAL);
		}
	}

	if (pdata->nbuttons == 0)
		return ERR_PTR(-EINVAL);

	return pdata;
}

static struct of_device_id gpio_keys_of_match[] = {
	{ .compatible = "endless,ec100-gpio-keys", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_keys_of_match);

static void gpio_remove_key(struct gpio_button_data *bdata)
{
	free_irq(bdata->button->irq, bdata);
}

static int gpio_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct gpio_keys_platform_data *pdata = dev_get_platdata(dev);
	struct gpio_keys_drvdata *ddata;
	struct input_dev *input;
	int i, error;

	if (!pdata) {
		pdata = gpio_keys_get_devtree_pdata(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	ddata = devm_kzalloc(dev, sizeof(struct gpio_keys_drvdata) +
			     pdata->nbuttons * sizeof(struct gpio_button_data),
			     GFP_KERNEL);
	input = devm_input_allocate_device(dev);
	if (!ddata || !input) {
		dev_err(dev, "failed to allocate state\n");
		return -ENOMEM;
	}

	ddata->pdata = pdata;
	ddata->input = input;

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdata->name ? : pdev->name;
	input->phys = "ec100-gpio-keys/input0";

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];

		error = gpio_keys_setup_key(pdev, input, bdata, button);
		if (error)
			goto fail2;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto fail2;
	}

	return 0;

 fail2:
	while (--i >= 0)
		gpio_remove_key(&ddata->data[i]);

	platform_set_drvdata(pdev, NULL);
	return error;
}

static int gpio_keys_remove(struct platform_device *pdev)
{
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < ddata->pdata->nbuttons; i++)
		gpio_remove_key(&ddata->data[i]);

	return 0;
}

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= gpio_keys_remove,
	.driver		= {
		.name	= "ec100-gpio-keys",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(gpio_keys_of_match),
	}
};

static int __init gpio_keys_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

late_initcall(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Drake <drake@endlessm.com>");
MODULE_DESCRIPTION("EC100 GPIO key driver");
