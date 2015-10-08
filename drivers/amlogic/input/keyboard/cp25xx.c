/*
 * linux/drivers/input/keypad/cp25xx.c
 *
 * cp25xx touch keypad driver
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#define DRIVER_NAME "cp25xx"
#define cp25xx_dbg(fmt, args...)  { if(1) \
					printk("[cp25xx]: " fmt, ## args); }

/* periodic polling delay and period */
#define KP_POLL_DELAY       (1 * 1000000)
#define KP_POLL_PERIOD      (10 * 1000000)

struct cp25xx_platform_data {
	int key_num;
	const char **key_name;
	int *key_code;
};

struct cp25xx {
	spinlock_t lock;
	struct i2c_client *client;
	struct input_dev *input;
	struct hrtimer timer;
	struct work_struct work;
	struct workqueue_struct *workqueue;
	int last_key_map;
	struct cp25xx_platform_data *pdata;
};

int cp25xx_key_code_list[] = {
	158, 14, 79, 80, 81, 78
};

static int cp25xx_write_reg(struct i2c_client *client, u8 addr, int val)
{
	u8 buf[3];
	int ret;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = 3,
		.buf = buf,
	};
	
	buf[0] = addr;
	buf[1] = (val >> 8) & 0xff;
	buf[2] = val & 0xff;
	ret = i2c_transfer(client->adapter, &msg, 1);
	return (ret == 1) ? 0 : -EINVAL;
}

static int cp25xx_read_reg(struct i2c_client *client, u8 addr)
{
	u8 buf[2];
	int ret;
	struct i2c_msg msg[2] = {
		[0] = {
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		[1] = {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 2,
			.buf = buf,
		},
	};

	ret = i2c_transfer(client->adapter, msg, 2);
	return (ret == 2) ? ((buf[0] << 8) | buf[1]) : -EINVAL;
}

static int cp25xx_initialize(struct cp25xx *kp)
{
	
	if (cp25xx_write_reg(kp->client, 0x01, 0x8000))
		return -EIO;
	if (cp25xx_write_reg(kp->client, 0x02, 0x1000))
		return -EIO;
	if (cp25xx_write_reg(kp->client, 0x21, 0x00ff))
		return -EIO;
	if (cp25xx_write_reg(kp->client, 0x20, 0x00ff))
		return -EIO;
	if (cp25xx_write_reg(kp->client, 0x03, 0x003f))
		return -EIO;
 	return 0;
}

static void cp25xx_key_proc(struct cp25xx *kp)
{
	int key_map, change;
	int key_code;
	const char *key_name;
	int i;
	
	key_map = cp25xx_read_reg(kp->client, 0);
	if(key_map != 0x2528) {
    cp25xx_dbg("no 2528! (0x%x)\n", key_map);
		return;
	}
	key_map = cp25xx_read_reg(kp->client, 0x31);
	if (key_map < 0)
		return;

	change = kp->last_key_map ^ key_map;
	kp->last_key_map = key_map;
	if (!change)
		return;
	
	for(i=0; i<kp->pdata->key_num; i++) {
		if (change & 1) {
			key_name = kp->pdata->key_name[i];
			key_code = kp->pdata->key_code[i];
			if (key_map & 1) {
				input_report_key(kp->input, key_code, 1);
				cp25xx_dbg("key_%s(%d) pressed\n", key_name, key_code);
			}
			else {
				input_report_key(kp->input, key_code, 0);
				cp25xx_dbg("key_%s(%d) released\n", key_name, key_code);
			}
			input_sync(kp->input);
		}
		change >>= 1;
		key_map >>= 1;
	}
}

#ifdef CONFIG_OF
static struct cp25xx_platform_data *
cp25xx_get_platform_data(struct device *dev)
{
	struct cp25xx_platform_data *pdata;
	int key_num;
	const char ** key_name;
	int *key_code;
	struct device_node *np;
	int i;
	
	np = dev->of_node;
	if (of_property_read_u32(np, "key_num", &key_num)) {
		dev_err(dev, "match key_num failed!\n");
		return 0;
	}

	pdata = kzalloc(sizeof(struct cp25xx_platform_data), GFP_KERNEL);
	key_name =kzalloc(sizeof(char *) * key_num, GFP_KERNEL);
	key_code =kzalloc(sizeof(int) * key_num, GFP_KERNEL);
	if (!pdata || !key_name || !key_code) {
		dev_err(dev, "alloc platform data failed!\n");
		return 0;
	}
	pdata->key_name = key_name;
	pdata->key_code = key_code;

	for (i=0; i<key_num; i++) {
		if (of_property_read_string_index(np, "key_name", i, key_name))
			break;
		if (of_property_read_u32_index(np, "key_code", i, key_code))
			break;
		key_name++;
		key_code++;
	}
	pdata->key_num = i;
	return pdata;
}
#endif

static struct input_dev * 
cp25xx_register_input(struct cp25xx_platform_data *pdata)
{
	struct input_dev *input;
	int i;
    
	input = input_allocate_device();
	if (input) {
		set_bit(EV_KEY, input->evbit);
		set_bit(EV_REP, input->evbit);
		for (i=0; i<pdata->key_num; i++) {
			set_bit(pdata->key_code[i], input->keybit);
			cp25xx_dbg("key_%s(%d) registed.\n",pdata->key_name[i],
			           pdata->key_code[i]);
		}
		input->name = DRIVER_NAME;
		input->phys = "cp25xx/input0";
		input->id.bustype = BUS_I2C;
		input->id.vendor = 0x0001;
		input->id.product = 0x0001;
		input->id.version = 0x0100; 
		input->rep[REP_DELAY]=0xffffffff;
		input->rep[REP_PERIOD]=0xffffffff;
		input->keycodesize = sizeof(unsigned short);
		input->keycodemax = 0x1ff;
		if (input_register_device(input) < 0) {
			input_free_device(input);
			input = 0;
		}
	}
  return input;
}

static void cp25xx_free(struct cp25xx *kp)
{
	if (kp) {
		if (kp->input) {
			input_unregister_device(kp->input);
			input_free_device(kp->input);
		}
#ifdef CONFIG_OF
		if (kp->pdata) {
			if (kp->pdata->key_name)
				kfree(kp->pdata->key_name);
			if (kp->pdata->key_code)
				kfree(kp->pdata->key_code);
			kfree(kp->pdata);
		}
#endif
		kfree(kp);
	}
}

static void cp25xx_work(struct work_struct *work)
{
	struct cp25xx *kp;

	kp = (struct cp25xx *)container_of(work, struct cp25xx, work);
	cp25xx_key_proc(kp);
	hrtimer_start(&kp->timer, ktime_set(0, KP_POLL_PERIOD),
			HRTIMER_MODE_REL);
}

static enum hrtimer_restart cp25xx_timer(struct hrtimer *timer)
{
	struct cp25xx *kp;
	unsigned long flags = 0;

	kp = (struct cp25xx*)container_of(timer, struct cp25xx, timer);
	spin_lock_irqsave(&kp->lock, flags);
	queue_work(kp->workqueue, &kp->work);
	spin_unlock_irqrestore(&kp->lock, flags);
	return HRTIMER_NORESTART;
}

static int cp25xx_probe(struct i2c_client *client, 
		const struct i2c_device_id *id)
{
	int err = 0;
	struct cp25xx *kp = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "check i2c failed\n");
		return -ENODEV;
	}
	
	kp = kzalloc(sizeof(struct cp25xx), GFP_KERNEL);
	if (!kp) {
		dev_err(&client->dev, "alloc data failed\n");
		return -ENOMEM;
	}

#ifdef CONFIG_OF
	kp->pdata = cp25xx_get_platform_data(&client->dev);
#else
	kp->pdata = client->platform_data;
#endif

	kp->client = client;
	err = cp25xx_initialize(kp);
	if (err) {
		dev_err(&client->dev, "hw initialize failed\n");
		cp25xx_free(kp);
		return err;
	}
	
	kp->input = cp25xx_register_input(kp->pdata);
	if (!kp->input) {
		dev_err(&client->dev, "register input device failed\n");
		cp25xx_free(kp);
		return -ENOMEM;
	}

	hrtimer_init(&kp->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	kp->timer.function = cp25xx_timer;
	INIT_WORK(&kp->work, cp25xx_work);
	kp->workqueue = create_singlethread_workqueue(DRIVER_NAME);
	if (kp->workqueue == NULL) {
		dev_err(&client->dev, "create work queue failed\n");
		cp25xx_free(kp);
		return -ENOMEM;
	}
	
	i2c_set_clientdata(client, kp);
	hrtimer_start(&kp->timer, ktime_set(0, KP_POLL_PERIOD),
			HRTIMER_MODE_REL);
	
	return 0;
}

static int cp25xx_remove(struct i2c_client *client)
{
	struct cp25xx *kp = i2c_get_clientdata(client);

	i2c_set_clientdata(client, NULL);
	cp25xx_free(kp);
	return 0;
}

static const struct i2c_device_id cp25xx_ids[] = {
	{DRIVER_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, cp25xx_ids);

static struct i2c_driver cp25xx_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = cp25xx_probe,
	.remove = cp25xx_remove,
	//.suspend = cp25xx_suspend,
	//.resume = cp25xx_resume,
	.id_table = cp25xx_ids,
};

static int __init cp25xx_init(void)
{
	cp25xx_dbg("cp25xx init\n");
	return i2c_add_driver(&cp25xx_driver);
}

static void __exit cp25xx_exit(void)
{
	cp25xx_dbg("cp25xx exit\n");
	i2c_del_driver(&cp25xx_driver);
}

module_init(cp25xx_init);
module_exit(cp25xx_exit);

MODULE_AUTHOR("aml");
MODULE_DESCRIPTION("cp2508/12 touch key driver");
MODULE_LICENSE("GPL");
