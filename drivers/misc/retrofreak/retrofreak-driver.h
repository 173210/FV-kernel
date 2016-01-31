#ifndef _RETROFREAK_DRIVER_H
#define _RETROFREAK_DRIVER_H

#include <linux/module.h>
#define KERNEL
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/earlysuspend.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/rfkill-rk.h>
#include <mach/iomux.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/interrupt.h>
#include <../drivers/spi/rk29_spim.h>

#define DRIVER_VER				0x0001
#define DEVICE_NAME				"retrofreak"
#define DEVICE_FILE				"retrofreak"
#define TAG						"RetroFreak: "

#define D_PRINTF(...)			printk(TAG __VA_ARGS__)
#define BAIL(errno)				{ rv = errno; goto end; }
#define LIMIT(in, max)			(((in) > (max)) ? (max) : (in))
#define GET_PROCESS_PID			(task_tgid_nr(current))
#define bswap_16(x) 			(((x) & 0x00ff) << 8 | ((x) & 0xff00) >> 8)

#define IOCTL_IDENT 'G'
#define IOCTL_DRIVER_VERSION 		_IOWR(IOCTL_IDENT, 13, int)
#define IOCTL_PCBA_VERSION 			_IOWR(IOCTL_IDENT, 14, int)

#define POWER_CTRL_ON			1
#define POWER_CTRL_OFF			0
#define POWER_CTRL_GPIO			303		// GPIO4_B7

#define HOST_DRV_GPIO			166		// GPIO0_A6
#define OTG_DRV_GPIO			165		// GPIO0_A5
#define OTG_DISABLE_GPIO		164		// GPIO0_A4

/*
 * PCB-A version detection
 *
 * Ver 0: B2=0,B3=0,B4=1 (0x04)
 * Ver 1: B2=1,B3=1,B4=0 (0x03)
 * Ver 2: B2=0,B3=1,B4=1 (0x06)
 */

#define PCBA_VER_0				(0x04)
#define PCBA_VER_1				(0x03)
#define PCBA_VER_2				(0x06)

#define PCBA_VER_GPIO0			(330)		// GPIO6_B2
#define PCBA_VER_GPIO1			(331)		// GPIO6_B3
#define PCBA_VER_GPIO2			(332)		// GPIO6_B4

struct rk30_gpio_bank {
	struct gpio_chip chip;
	unsigned short id;
	short irq;
	void __iomem *regbase;	/* Base of register bank */
	struct clk *clk;
	u32 suspend_wakeup;
	u32 saved_wakeup;
	spinlock_t lock;
};

struct tracked_mutex
{
	struct mutex *m;
	pid_t pid;
};

extern struct rk30_gpio_bank rk30_gpio_banks[6];
extern struct mutex init_lock;

void hwman_configure_pcba_hardware(void);

#endif // _RETROFREAK_DRIVER_H
