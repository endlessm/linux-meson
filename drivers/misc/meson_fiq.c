#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/amlogic/aml_gpio_consumer.h>

#include <asm/fiq.h>
#include <asm/pgtable.h>
#include <asm/mach/irq.h>
#include <mach/irqs.h>
#include <mach/am_regs.h>

#define FIQ

struct meson_fiq_data {
	void __iomem	*base;
	unsigned int	irq;
	int gpio;
};

static struct meson_fiq_data *meson_fiq_data;
extern unsigned char meson_fiq_handler, meson_fiq_handler_end;

#ifdef FIQ
static u8 fiq_stack[4096];
static struct fiq_handler meson_fh = {
	.name	= "meson_fiq_handler"
};
#endif

#ifndef FIQ
static irqreturn_t fiq_handler(int irq, void *dev_id)
{
	asm volatile (
		"ldr	r8, =0xfe108058		\n"
		"ldr	r9, [r8]		\n"
                "tst    r9, #1 << 27            \n"
                "orreq  r9, r9, #1 << 27        \n"
		"streq  r9, [r8]		\n"
                "beq    out                     \n"
                "bic    r9, r9, #1 << 27        \n"
		"str	r9, [r8]		\n"
                "out:                           \n"
                ::: "memory", "cc", "r8", "r9"
	);

	return IRQ_HANDLED;
}
#else
static void __attribute__((naked)) fiq_vector(void)
{
	asm __volatile__("mov pc, r8 ;");
}

static void __attribute__((naked)) fiq_isr(void)
{
	int i;
	unsigned int fiq;

	asm __volatile__(
		"mov    ip, sp;\n"
		"stmfd  sp!, {r0-r12, lr};\n"
		"sub    sp, sp, #256;\n"
		"sub    fp, sp, #256;\n");

	printk(KERN_EMERG "---->>> %s\n", __func__);

	asm __volatile__(
		"add    sp, sp, #256 ;\n"
		"ldmia  sp!, {r0-r12, lr};\n"
		"subs   pc, lr, #4;\n");
}
#endif

#if 0
static irqreturn_t fiq_handler(int irq, void *dev_id)
{
	struct meson_fiq_data *meson_fiq_data = dev_id;
	int cur_val, bit;

	cur_val = amlogic_get_value(meson_fiq_data->gpio, "fiq-gpio");
	amlogic_set_value(meson_fiq_data->gpio, !cur_val, "fiq-gpio");

	return IRQ_HANDLED; 
}
#endif

static int meson_fiq_probe(struct platform_device *pdev)
{
	struct device_node *np;
	int ret;
	u32 reg;
#ifdef FIQ
	struct pt_regs regs;
#endif

	np = pdev->dev.of_node;
	if (!np) {
		dev_err(&pdev->dev, "No device tree data available\n");
		return -EINVAL;
	}

	meson_fiq_data = devm_kzalloc(&pdev->dev, sizeof(*meson_fiq_data),
				      GFP_KERNEL);
	if (!meson_fiq_data)
		return -ENOMEM;

	meson_fiq_data->base = of_iomap(np, 0);
	if (!meson_fiq_data->base) {
		dev_err(&pdev->dev, "Couldn't map base\n");
		return -EINVAL;
	}

	meson_fiq_data->irq = irq_of_parse_and_map(np, 0);
	if (meson_fiq_data->irq < 0) {
		dev_err(&pdev->dev, "Couldn't register given IRQ\n");
		return -EINVAL;
	}

	meson_fiq_data->gpio = amlogic_gpio_name_map_num("GPIOH_8");
	amlogic_gpio_request_one(meson_fiq_data->gpio, GPIOF_OUT_INIT_LOW, "fiq-gpio");
	amlogic_set_pull_up_down(meson_fiq_data->gpio, 1, "fiq-gpio");

	/* Timer A: 1mS timebase */
	reg = readl(meson_fiq_data->base);
	reg &= ~(0x3 << 0);
	reg |= (0x3 << 0);
	writel(reg, meson_fiq_data->base);

	/* Timer A: stop */
	reg = readl(meson_fiq_data->base);
	reg &= ~BIT(16);
	writel(reg, meson_fiq_data->base);

	/* Timer A: 500 units */
	reg = readl(meson_fiq_data->base + (0x0001 << 2));
	reg &= ~(0xff);
	reg |= 500;
	writel(reg, meson_fiq_data->base + (0x0001 << 2));

	/* Timer A: periodic */
	reg = readl(meson_fiq_data->base);
	reg |= BIT(12);
	writel(reg, meson_fiq_data->base);

	/* Timer A: start */
	reg = readl(meson_fiq_data->base);
	reg |= BIT(16);
	writel(reg, meson_fiq_data->base);

#ifndef FIQ
	ret = request_irq(meson_fiq_data->irq, fiq_handler, 0,
			  pdev->dev.driver->name, meson_fiq_data);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't request irq\n");
		return -EINVAL;
	}
#else
	memset(&regs, 0, sizeof(regs));
	regs.ARM_r8 = (unsigned long)fiq_isr;
	regs.ARM_sp = (unsigned long)fiq_stack + sizeof(fiq_stack) - 4;

	set_fiq_regs(&regs);
	set_fiq_handler(fiq_vector, 8);

	request_fiq(meson_fiq_data->irq, NULL);
#endif

	return 0;
}

static int meson_fiq_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id meson_fiq_of_match[] = {
	{ .compatible = "amlogic,fiq" },
	{}
};

static struct platform_driver meson_fiq_driver = {
	.probe	= meson_fiq_probe,
	.remove	= meson_fiq_remove,
	.driver = {
		.name = "meson_fiq",
		.of_match_table = of_match_ptr(meson_fiq_of_match),
		.owner = THIS_MODULE,
	},
};

module_platform_driver(meson_fiq_driver);

MODULE_DESCRIPTION("FIQ handler");
MODULE_AUTHOR("Carlo Caione <carlo@endlessm.com>");
MODULE_LICENSE("GPL");
