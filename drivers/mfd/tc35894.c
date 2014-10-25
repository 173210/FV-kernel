/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License, version 2
 * Author: Hanumath Prasad <hanumath.prasad@stericsson.com> for ST-Ericsson
 * Author: Rabin Vincent <rabin.vincent@stericsson.com> for ST-Ericsson
 */


#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/gpio.h>
#include <linux/mfd/tc35894.h>

#define TC35894_CLKMODE_MODCTL_SLEEP            0x0
#define TC35894_CLKMODE_MODCTL_OPERATION        (1 << 0)
/**
 * tc35894_reg_read() - read a single TC35894 register
 * @tc35894:	Device to read from
 * @reg:	Register to read
 */
int tc35894_reg_read(struct tc35894 *tc35894, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(tc35894->i2c, reg);
	if (ret < 0)
		dev_err(tc35894->dev, "failed to read reg %#x: %d\n",
			reg, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(tc35894_reg_read);

/**
 * tc35894_reg_read() - write a single tc35894 register
 * @tc35894:	Device to write to
 * @reg:	Register to read
 * @data:	Value to write
 */
int tc35894_reg_write(struct tc35894 *tc35894, u8 reg, u8 data)
{
	int ret;

	ret = i2c_smbus_write_byte_data(tc35894->i2c, reg, data);
	if (ret < 0)
		dev_err(tc35894->dev, "failed to write reg %#x: %d\n",
			reg, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(tc35894_reg_write);

/**
 * tc35894_block_read() - read multiple TC35894 registers
 * @tc35894:	Device to read from
 * @reg:	First register
 * @length:	Number of registers
 * @values:	Buffer to write to
 */
int tc35894_block_read(struct tc35894 *tc35894, u8 reg, u8 length, u8 *values)
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(tc35894->i2c, reg, length, values);
	if (ret < 0)
		dev_err(tc35894->dev, "failed to read regs %#x: %d\n",
			reg, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(tc35894_block_read);

/**
 * tc35894_block_write() - write multiple TC35894 registers
 * @tc35894:	Device to write to
 * @reg:	First register
 * @length:	Number of registers
 * @values:	Values to write
 */
int tc35894_block_write(struct tc35894 *tc35894, u8 reg, u8 length,
			const u8 *values)
{
	int ret;

	ret = i2c_smbus_write_i2c_block_data(tc35894->i2c, reg, length,
					     values);
	if (ret < 0)
		dev_err(tc35894->dev, "failed to write regs %#x: %d\n",
			reg, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(tc35894_block_write);

/**
 * tc35894_set_bits() - set the value of a bitfield in a TC35894 register
 * @tc35894:	Device to write to
 * @reg:	Register to write
 * @mask:	Mask of bits to set
 * @values:	Value to set
 */
int tc35894_set_bits(struct tc35894 *tc35894, u8 reg, u8 mask, u8 val)
{
	int ret;

	mutex_lock(&tc35894->lock);

	ret = tc35894_reg_read(tc35894, reg);
	if (ret < 0)
		goto out;

	ret &= ~mask;
	ret |= val;

	ret = tc35894_reg_write(tc35894, reg, ret);

out:
	mutex_unlock(&tc35894->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(tc35894_set_bits);
/*
static struct resource gpio_resources[] = {
	{
		.start	= TC35894_INT_GPIIRQ,
		.end	= TC35894_INT_GPIIRQ,
		.flags	= IORESOURCE_IRQ,
	},
};
*/
static struct resource keypad_resources[] = {
        {
                .start  = TC35894_INT_KBDIRQ,
                .end    = TC35894_INT_KBDIRQ,
                .flags  = IORESOURCE_IRQ,
        },
};
/*
static struct mfd_cell tc35894_devs[] = {
	{
		.name		= "tc35894-gpio",
		.num_resources	= ARRAY_SIZE(gpio_resources),
		.resources	= &gpio_resources[0],
	},
};
*/
static struct mfd_cell tc35894_dev_keypad[] = {
        {
                .name           = "tc35894-keypad",
                .num_resources  = ARRAY_SIZE(keypad_resources),
                .resources      = &keypad_resources[0],
        },
};
static irqreturn_t tc35894_irq(int irq, void *data)
{
	struct tc35894 *tc35894 = data;
	int status;
//again:
//	printk("tc35894_irq no.1\n");
	status = tc35894_reg_read(tc35894, TC35894_IRQST);
	if (status <= 0){
		//printk("Interrupt status fail = 0x%x\n",status);
		return IRQ_NONE;
	}
	disable_irq_nosync(gpio_to_irq(MX53_TC35894_KEY_INT));

	 if(status) {
		//printk("Interrupt status = 0x%x\n",status);
		int bit = __ffs(status);
//		printk("print status = 0x%x and bit = 0x%x\n",status,bit);
		handle_nested_irq(tc35894->irq_base + bit);
		//status &= ~(1 << bit);
//		printk("tc35894_irq no.2\n");
	}

	/*
	 * A dummy read or write (to any register) appears to be necessary to
	 * have the last interrupt clear (for example, GPIO IC write) take
	 * effect.
	 */
/*
      	status = tc35894_reg_read(tc35894, TC35894_IRQST);
		if(status)
			printk("IRQ out an print status = 0x%x\n",status);

        if (status)
                goto again;
*/
	return IRQ_HANDLED;
}

static void tc35894_irq_dummy(unsigned int irq)
{
	/* No mask/unmask at this level */
}

static struct irq_chip tc35894_irq_chip = {
	.name	= "tc35894",
	.mask	= tc35894_irq_dummy,
	.unmask	= tc35894_irq_dummy,
};

static int tc35894_irq_init(struct tc35894 *tc35894)
{
	int base = tc35894->irq_base;
	int irq;
//	printk("tc35894_irq_base= %#x\n",base);
	for (irq = base; irq < base + TC35894_NR_INTERNAL_IRQS; irq++) {
		set_irq_chip_data(irq, tc35894);
		set_irq_chip_and_handler(irq, &tc35894_irq_chip,
					 handle_edge_irq);
		set_irq_nested_thread(irq, 1);
#ifdef CONFIG_ARM
		set_irq_flags(irq, IRQF_VALID);
#else
		set_irq_noprobe(irq);
#endif
	}

	return 0;
}

static void tc35894_irq_remove(struct tc35894 *tc35894)
{
	int base = tc35894->irq_base;
	int irq;

	for (irq = base; irq < base + TC35894_NR_INTERNAL_IRQS; irq++) {
#ifdef CONFIG_ARM
		set_irq_flags(irq, 0);
#endif
		set_irq_chip_and_handler(irq, NULL, NULL);
		set_irq_chip_data(irq, NULL);
	}
}

static int tc35894_chip_init(struct tc35894 *tc35894)
{
	int manf, ver, ret;
	manf = tc35894_reg_read(tc35894, TC35894_MANFCODE);
//	printk("tc35894 driver test manfcode = %#x\n",manf);
	if (manf < 0)
		return manf;

	ver = tc35894_reg_read(tc35894, TC35894_VERSION);
	if (ver < 0)
		return ver;

	if (manf != TC35894_MANFCODE_MAGIC) {
		dev_err(tc35894->dev, "unknown manufacturer: %#x\n", manf);
		return -EINVAL;
	}

	dev_info(tc35894->dev, "manufacturer: %#x, version: %#x\n", manf, ver);

	/* Put everything except the IRQ module into reset */
	ret = tc35894_reg_write(tc35894, TC35894_RSTCTRL,
				TC35894_RSTCTRL_TIMRST
				| TC35894_RSTCTRL_ROTRST
				| TC35894_RSTCTRL_KBDRST
				| TC35894_RSTCTRL_GPIRST);
	if (ret < 0)
		return ret;
	/* Clear the reset interrupt. */
	return tc35894_reg_write(tc35894, TC35894_RSTINTCLR, 0x1);
}
static int __devinit tc35894_device_init(struct tc35894 *tc35894)
{
        int ret = 0;
//        unsigned int blocks = tc35894->pdata->block;
/*
        if (blocks & TC35894_BLOCK_GPIO) {
                ret = mfd_add_devices(tc35894->dev, -1, tc35894_dev_gpio,
                                ARRAY_SIZE(tc35894_dev_gpio), NULL,
                                tc35894->irq_base);
                if (ret) {
                        dev_err(tc35894->dev, "failed to add gpio child\n");
                        return ret;
                }
                dev_info(tc35894->dev, "added gpio block\n");
        }
*/
//        if (blocks & TC35894_BLOCK_KEYPAD) {
                ret = mfd_add_devices(tc35894->dev, -1, tc35894_dev_keypad,
                                ARRAY_SIZE(tc35894_dev_keypad), NULL,
                                tc35894->irq_base);
                if (ret) {
                        dev_err(tc35894->dev, "failed to keypad child\n");
                        return ret;
                }
                dev_info(tc35894->dev, "added keypad block\n");
  //      }

        return ret;
}

static int __devinit tc35894_probe(struct i2c_client *i2c,
				   const struct i2c_device_id *id)
{
	struct tc35894_platform_data *pdata = i2c->dev.platform_data;
	struct tc35894 *tc35894;
	int ret;
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA
				     | I2C_FUNC_SMBUS_I2C_BLOCK)){
		return -EIO;
	}
	tc35894 = kzalloc(sizeof(struct tc35894), GFP_KERNEL);
	if (!tc35894)
	{
		return -ENOMEM;
	}
	mutex_init(&tc35894->lock);

	tc35894->dev = &i2c->dev;
	tc35894->i2c = i2c;
	tc35894->pdata = pdata;
	tc35894->irq_base = pdata->irq_base;
//	tc35894->num_gpio = id->driver_data;

	i2c_set_clientdata(i2c, tc35894);

	ret = tc35894_chip_init(tc35894);
	if (ret)
		goto out_free;

	ret = tc35894_irq_init(tc35894);
	if (ret)
		goto out_free;

	ret = request_threaded_irq(tc35894->i2c->irq, NULL, tc35894_irq,
				   IRQF_TRIGGER_LOW| IRQF_ONESHOT,
				   "tc35894", tc35894);
	if (ret) {
		dev_err(tc35894->dev, "failed to request IRQ: %d\n", ret);
		goto out_removeirq;
	}
        
	ret = tc35894_device_init(tc35894);
        if (ret) {
                dev_err(tc35894->dev, "failed to add child devices\n");
                goto out_freeirq;
        }
/*
	ret = mfd_add_devices(tc35894->dev, -1, tc35894_devs,
			      ARRAY_SIZE(tc35894_devs), NULL,
			      tc35894->irq_base);

	if (ret) {
		dev_err(tc35894->dev, "failed to add children\n");
		goto out_freeirq;
	}
*/
	return 0;

out_freeirq:
	free_irq(tc35894->i2c->irq, tc35894);
out_removeirq:
	tc35894_irq_remove(tc35894);
out_free:
	kfree(tc35894);
	return ret;
}

static int __devexit tc35894_remove(struct i2c_client *client)
{
	struct tc35894 *tc35894 = i2c_get_clientdata(client);

	mfd_remove_devices(tc35894->dev);

	free_irq(tc35894->i2c->irq, tc35894);
	tc35894_irq_remove(tc35894);

	kfree(tc35894);

	return 0;
}
static int tc35894_suspend(struct device *dev)
{
        struct tc35894 *tc35894 = dev_get_drvdata(dev);
        struct i2c_client *client = tc35894->i2c;
        int ret = 0;

	enable_irq_wake(tc35894->i2c->irq);
        /* put the system to sleep mode */
     //   if (!device_may_wakeup(&client->dev))
       //         ret = tc35894_reg_write(tc35894, TC35894_CLKMODE,
         //                       TC35894_CLKMODE_MODCTL_SLEEP);

        return ret;
}

static int tc35894_resume(struct device *dev)
{
        struct tc35894 *tc35894 = dev_get_drvdata(dev);
        struct i2c_client *client = tc35894->i2c;
        int ret = 0;

        /* enable the system into operation */
        if (!device_may_wakeup(&client->dev))
                ret = tc35894_reg_write(tc35894, TC35894_CLKMODE,
                                TC35894_CLKMODE_MODCTL_OPERATION);

        return ret;
}
static const SIMPLE_DEV_PM_OPS(tc35894_dev_pm_ops, tc35894_suspend,
                                                tc35894_resume);

static const struct i2c_device_id tc35894_id[] = {
	{ "tc35894", 24 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tc35894_id);

static struct i2c_driver tc35894_driver = {
	.driver.name	= "tc35894",
	.driver.owner	= THIS_MODULE,
	.probe		= tc35894_probe,
#ifdef CONFIG_PM
        .driver.pm      = &tc35894_dev_pm_ops,
#endif
	.remove		= __devexit_p(tc35894_remove),
	.id_table	= tc35894_id,
};

static int __init tc35894_init(void)
{
	return i2c_add_driver(&tc35894_driver);
}
subsys_initcall(tc35894_init);

static void __exit tc35894_exit(void)
{
	i2c_del_driver(&tc35894_driver);
}
module_exit(tc35894_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TC35894 MFD core driver");
MODULE_AUTHOR("Hanumath Prasad, Rabin Vincent");
