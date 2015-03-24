/*
 * linux/arch/mips/tx-boards/generic/7segled.c
 *
 * 7 Segment LED routines
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2005-2007
 * All Rights Reserved.
 *
 */
#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/sysdev.h>
#include <linux/slab.h>
#include <asm/tx-boards/generic.h>

/*
 * bit encoding:
 * bit7: dot
 * bit6: top bar
 * bit5: right upper
 * bit4: right lower
 * bit3: bottom bar
 * bit2: left lower
 * bit1: left upper
 * bit0: center bar
 */
static const unsigned char led_pattern_num[] = {	/* 0-9 */
	0x7e, 0x30, 0x6d, 0x79, 0x33,
	0x5b, 0x5f, 0x70, 0x7f, 0x7b,
};
static const unsigned char led_pattern_alpha[] = {
	0x77, 0x1f, 0x0d, 0x3d, 0x4f,	/* a-e */
	0x47, 0x5e, 0x17, 0x06, 0x3c,	/* f-j */
	0x2f, 0x0e, 0x76, 0x15, 0x1d,	/* k-o */
	0x67, 0x73, 0x05, 0x1b, 0x46,	/* p-t */
	0x3e, 0x1c, 0x3f, 0x37, 0x3b,	/* u-y */
	0x09,				/* z */
};
static const unsigned char led_pattern_misc[][2] = {
	{ ' ', 0x00 },
	{ '-', 0x01 },
	{ '.', 0x80 },
	{ '=', 0x48 },
	{ '_', 0x08 },
	{ '~', 0x40 },
};

static unsigned int tx_7segled_num;
static void (*tx_7segled_putc)(unsigned int pos, unsigned char val);

void __init txboard_7segled_init(unsigned int num, void (*putc)(unsigned int pos, unsigned char val))
{
	tx_7segled_num = num;
	tx_7segled_putc = putc;
}

void txboard_7segled_putc(unsigned int pos, char c)
{
	unsigned char val;
	if (pos >= tx_7segled_num)
		return;
	if (isdigit(c))
		val = led_pattern_num[c - '0'];
	else if (isalpha(c))
		val = led_pattern_alpha[tolower(c) - 'a'];
	else {
		int i;
		val = led_pattern_misc[0][1];	/* ' ' */
		for (i = 0; i < ARRAY_SIZE(led_pattern_misc); i++) {
			if (led_pattern_misc[i][0] == c) {
				val = led_pattern_misc[i][1];
				break;
			}
		}
	}
	tx_7segled_putc(pos, val);
}

static ssize_t ascii_store(struct sys_device *dev, const char *buf,
			   size_t size)
{
	unsigned int ch = dev->id;
	txboard_7segled_putc(ch, buf[0]);
	return size;
}

static ssize_t raw_store(struct sys_device *dev, const char *buf,
			 size_t size)
{
	unsigned int ch = dev->id;
	tx_7segled_putc(ch, buf[0]);
	return size;
}

static SYSDEV_ATTR(ascii, 0200, NULL, ascii_store);
static SYSDEV_ATTR(raw, 0200, NULL, raw_store);

static struct sysdev_class tx_7segled_sysdev_class = {
	set_kset_name("7segled"),
};

static int __init tx_7segled_init_sysfs(void)
{
	int error, i;
	if (!tx_7segled_num)
		return -ENODEV;
	error = sysdev_class_register(&tx_7segled_sysdev_class);
	if (error)
		return error;
	for (i = 0; i < tx_7segled_num; i++) {
		struct sys_device *dev;
		dev = kzalloc(sizeof(*dev), GFP_KERNEL);
		if (!dev) {
			error = -ENODEV;
			break;
		}
		dev->id = i;
		dev->cls = &tx_7segled_sysdev_class;
		error = sysdev_register(dev);
		if (!error) {
			sysdev_create_file(dev, &attr_ascii);
			sysdev_create_file(dev, &attr_raw);
		}
	}
	return error;
}

device_initcall(tx_7segled_init_sysfs);
