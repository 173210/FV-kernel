/*
 * Driver for EETI eGalax Multiple Touch Controller
 *
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
 *
 * based on max11801_ts.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/* EETI eGalax serial touch screen controller is a I2C based multiple
 * touch screen controller, it can supports 5 pointer multiple
 * touch. */

/* TODO:
  - auto idle mode support
  - early suspend support for android
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/bitops.h>

#define REPORT_MODE_SINGLE		0x1
#define REPORT_MODE_VENDOR		0x3
#define REPORT_MODE_MTTOUCH		0x4

#define MAX_SUPPORT_POINTS		2

#define EVENT_MODE		0
#define EVENT_STATUS		1
#define EVENT_VALID_OFFSET	7
#define EVENT_VAILD_MASK	(0x1 << EVENT_VALID_OFFSET)
#define EVENT_ID_OFFSET		2
#define EVENT_ID_MASK		(0xf << EVENT_ID_OFFSET)
#define EVENT_IN_RANGE		(0x1 << 1)
#define EVENT_DOWN_UP		(0X1 << 0)
#define MAX_I2C_DATA_LEN	10
#if 1
#define MAX_X_VALUE		800
#define MAX_Y_VALUE		480
#else
#define MAX_X_VALUE		10240
#define MAX_Y_VALUE		6144
#define X_SPLIT
#endif
struct tp050rcz_pointer {
	bool valid;
	bool status;
	u16 x;
	u16 y;
};

struct tp050rcz_ts {
	struct i2c_client		*client;
	struct input_dev		*input_dev;
	struct tp050rcz_pointer		events[MAX_SUPPORT_POINTS];
	struct delayed_work work;
	spinlock_t lock;

};

static void tp050rcz_ts_handler(struct work_struct *work)
{
	struct tp050rcz_ts *data =  container_of(work,
				struct tp050rcz_ts, work.work);
	struct input_dev *input_dev = data->input_dev;
	struct i2c_client *client = data->client;
	u8 buf[MAX_I2C_DATA_LEN];
	int i, x1, y1,x2,y2;
	int touching,oldtouching;
	int z=50;
	int w=15;
//retry:
	for(i=0;i<10;i++)
	{
		buf[i] = i2c_smbus_read_byte_data(client,i);
	}
#if 0
	if(buf[0]==1)
		printk("old:%d,x1:%d,y1:%d\n\n",buf[1],(buf[2]|(buf[3]<<8)), (buf[4]|(buf[5]<<8)));
	if(buf[0]==2)
		printk("old:%d,x1:%d,y1:%d,x2:%d,y2:%d\n\n",buf[1],(buf[2]|(buf[3]<<8)), (buf[4]|(buf[5]<<8)), (buf[6]|(buf[7]<<8)), (buf[8]|(buf[9]<<8)));
#endif
	touching = buf[0];
	oldtouching = buf[1];
	x1 = (buf[3] << 8) | buf[2];
	y1 = (buf[5] << 8) | buf[4];
	x2 = (buf[7] << 8) | buf[6];
	y2 = (buf[9] << 8) | buf[8];
#ifdef X_SPLIT
	x1 = MAX_X_VALUE-x1;
	x2 = MAX_X_VALUE-x2;
#endif


	if (touching) {
		input_report_abs(input_dev, ABS_X, x1);
		input_report_abs(input_dev, ABS_Y, y1);
		input_report_key(input_dev, BTN_TOUCH, 1);
		input_report_abs(input_dev, ABS_PRESSURE, 1);
	}else
	{
		input_report_key(input_dev, BTN_TOUCH, 0);
		input_report_abs(input_dev, ABS_PRESSURE, 0);
		z = 0;
		w = 0;
	}
#ifndef FORCE_SINGLE_POINTER_SUPPORT
	input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, z);
	input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, w);
	input_report_abs(input_dev, ABS_MT_POSITION_X, x1);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y1);
	input_mt_sync(input_dev);
	if (touching==2)
	{
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, z);
		input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, w);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x2);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y2);
		input_mt_sync(input_dev);
	}
#endif
	input_sync(input_dev);

}
static irqreturn_t tp050rcz_ts_interrupt(int irq, void *dev_id)
{
	//printk("___%s___\n",__FUNCTION__);
	struct tp050rcz_ts *data = dev_id;
	schedule_delayed_work(&data->work,msecs_to_jiffies(10));
	return IRQ_HANDLED;
}

static int tp050rcz_wake_up_device(struct i2c_client *client)
{
	return 0;
	int gpio = irq_to_gpio(client->irq);
	int ret;

	ret = gpio_request(gpio, "tp050rcz_irq");
	if (ret < 0) {
		dev_err(&client->dev, "request gpio failed:%d\n", ret);
		return ret;
	}
	/* wake up controller via an falling edge on IRQ. */
	gpio_direction_output(gpio, 0);
	gpio_set_value(gpio, 1);
	/* controller should be waken up, return irq.  */
	gpio_direction_input(gpio);
	gpio_free(gpio);
	return 0;
}

static int tp050rcz_7200_firmware_version(struct i2c_client *client)
{
	static const u8 cmd[MAX_I2C_DATA_LEN] = { 0x37, 0x03, };
	i2c_master_send(client, cmd, 2);
	udelay(2000);
	int ver0 = i2c_smbus_read_byte_data(client,48);
	int ver1 = i2c_smbus_read_byte_data(client,49);
	int ver2 = i2c_smbus_read_byte_data(client,50);
	int ver3 = i2c_smbus_read_byte_data(client,51);
	int ver4 = i2c_smbus_read_byte_data(client,52);
	printk("firmware version:%d.%d.%d.%d, subversion:%d\n",ver0, ver1, ver2, ver3, ver4);
	if (ver0 < 0 && ver1 < 0 && ver2 < 0 && ver3 < 0 && ver4 < 0)
		return -1;
	return 0;
}

static int __devinit tp050rcz_ts_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct tp050rcz_ts *data;
	struct input_dev *input_dev;
	int ret;
	data = kzalloc(sizeof(struct tp050rcz_ts), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_free_data;
	}

	data->client = client;
	data->input_dev = input_dev;
	tp050rcz_wake_up_device(client);
	ret = tp050rcz_7200_firmware_version(client);
#if 1
	if (ret < 0) {
		dev_err(&client->dev,
			"tp050rcz_ts: failed to read firmware version\n");
		ret = -EIO;
		goto err_free_dev;
	}
#endif
	input_dev->name = "tp050rcz_ts";
	input_dev->phys = "I2C",
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x0EEF;
	input_dev->id.product = 0x0020;
	input_dev->id.version = 0x0001;
	input_dev->dev.parent = &client->dev;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);
	__set_bit(ABS_PRESSURE, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_X, 0, MAX_X_VALUE, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_Y_VALUE, 0, 0);
#ifndef FORCE_SINGLE_POINTER_SUPPORT
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, MAX_X_VALUE, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, MAX_Y_VALUE, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 25, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0,
			     MAX_SUPPORT_POINTS, 0, 0);
#endif
	input_set_drvdata(input_dev, data);
	INIT_DELAYED_WORK(&data->work, tp050rcz_ts_handler);
	ret = request_threaded_irq(client->irq, NULL, tp050rcz_ts_interrupt,
				   IRQF_TRIGGER_LOW| IRQF_ONESHOT,
				   "tp050rcz_ts", data);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_dev;
	}

	ret = input_register_device(data->input_dev);
	if (ret < 0)
		goto err_free_irq;
	i2c_set_clientdata(client, data);
	return 0;

err_free_irq:
	free_irq(client->irq, data);
err_free_dev:
	input_free_device(input_dev);
err_free_data:
	kfree(data);

	return ret;
}

static __devexit int tp050rcz_ts_remove(struct i2c_client *client)
{
	struct tp050rcz_ts *data = i2c_get_clientdata(client);

	free_irq(client->irq, data);
	input_unregister_device(data->input_dev);
	input_free_device(data->input_dev);
	kfree(data);

	return 0;
}

static const struct i2c_device_id tp050rcz_ts_id[] = {
	{"tp050rcz_ts", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, tp050rcz_ts_id);

#ifdef CONFIG_PM_SLEEP
static int tp050rcz_ts_suspend(struct device *dev)
{
	int ret;
	u8 suspend_cmd[MAX_I2C_DATA_LEN] = {0x3, 0x6, 0xa, 0x3, 0x36,
					    0x3f, 0x2, 0, 0, 0};
	struct i2c_client *client = to_i2c_client(dev);
	ret = i2c_master_send(client, suspend_cmd,
			       MAX_I2C_DATA_LEN);
	return ret > 0 ? 0 : ret;
}

static int tp050rcz_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	return tp050rcz_wake_up_device(client);
}
#endif

static SIMPLE_DEV_PM_OPS(tp050rcz_ts_pm_ops, tp050rcz_ts_suspend, tp050rcz_ts_resume);
static struct i2c_driver tp050rcz_ts_driver = {
	.driver = {
		.name = "tp050rcz_ts",
#if 0
		.pm	= &tp050rcz_ts_pm_ops,
#endif
	},
	.id_table	= tp050rcz_ts_id,
	.probe		= tp050rcz_ts_probe,
	.remove		= __devexit_p(tp050rcz_ts_remove),
};

static int __init tp050rcz_ts_init(void)
{
	return i2c_add_driver(&tp050rcz_ts_driver);
}

static void __exit tp050rcz_ts_exit(void)
{
	i2c_del_driver(&tp050rcz_ts_driver);
}

module_init(tp050rcz_ts_init);
module_exit(tp050rcz_ts_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Touchscreen driver for EETI eGalax touch controller");
MODULE_LICENSE("GPL");
