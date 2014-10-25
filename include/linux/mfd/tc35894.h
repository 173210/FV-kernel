/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License, version 2
 */

#ifndef __LINUX_MFD_TC35894_H
#define __LINUX_MFD_TC35894_H

#include <linux/device.h>


#define TC35894_RSTCTRL_IRQRST	(1 << 4)
#define TC35894_RSTCTRL_TIMRST	(1 << 3)
#define TC35894_RSTCTRL_ROTRST	(1 << 2)
#define TC35894_RSTCTRL_KBDRST	(1 << 1)
#define TC35894_RSTCTRL_GPIRST	(1 << 0)

/* Keyboard Configuration Registers */
#define TC35894_KBDSETTLE_REG   0x01
#define TC35894_KBDBOUNCE       0x02
#define TC35894_KBDSIZE         0x03
#define TC35894_KBCFG_LSB       0x04
#define TC35894_KBCFG_MSB       0x05
#define TC35894_KBDMIS		0x07
#define TC35894_KBDIC           0x08
#define TC35894_KBDMSK          0x09
#define TC35894_EVTCODE_FIFO    0x10
#define TC35894_KBDMFS          0x8F

#define TC35894_IRQST		0x91

#define TC35894_MANFCODE_MAGIC	0x03
#define TC35894_MANFCODE	0x80
#define TC35894_VERSION		0x81
#define TC35894_IOCFG		0xA7

#define TC35894_CLKMODE		0x88
#define TC35894_CLKCFG		0x89
#define TC35894_CLKEN		0x8A
#define TC35894_AUTOSLPENA	0x8B
#define TC35894_AUTOSLPTIMER	0x8C

#define TC35894_RSTCTRL		0x82
#define TC35894_EXTRSTN		0x83
#define TC35894_RSTINTCLR	0x84

/* Pull up/down configuration registers */
#define TC35894_IOCFG           0xA7
#define TC35894_IOPULLCFG0_LSB  0xAA
#define TC35894_IOPULLCFG0_MSB  0xAB
#define TC35894_IOPULLCFG1_LSB  0xAC
#define TC35894_IOPULLCFG1_MSB  0xAD
#define TC35894_IOPULLCFG2_LSB  0xAE
#define TC35894_IOPULLCFG2_MSB  0xAF
#define TC35894_GPIOIS0		0xC9
#define TC35894_GPIOIS1		0xCA
#define TC35894_GPIOIS2		0xCB
#define TC35894_GPIOIBE0	0xCC
#define TC35894_GPIOIBE1	0xCD
#define TC35894_GPIOIBE2	0xCE
#define TC35894_GPIOIEV0	0xCF
#define TC35894_GPIOIEV1	0xD0
#define TC35894_GPIOIEV2	0xD1
#define TC35894_GPIOIE0		0xD2
#define TC35894_GPIOIE1		0xD3
#define TC35894_GPIOIE2		0xD4
#define TC35894_GPIORIS0	0xD6
#define TC35894_GPIORIS1	0xD7
#define TC35894_GPIORIS2	0xD8
#define TC35894_GPIOMIS0	0xD9
#define TC35894_GPIOMIS1	0xDA
#define TC35894_GPIOMIS2	0xDB
#define TC35894_GPIOIC0		0xDC
#define TC35894_GPIOIC1		0xDD
#define TC35894_GPIOIC2		0xDE

#define TC35894_GPIODATA0	0xC0
#define TC35894_GPIOMASK0	0xc1
#define TC35894_GPIODATA1	0xC2
#define TC35894_GPIOMASK1	0xc3
#define TC35894_GPIODATA2	0xC4
#define TC35894_GPIOMASK2	0xC5

#define TC35894_GPIODIR0	0xC6
#define TC35894_GPIODIR1	0xC7
#define TC35894_GPIODIR2	0xC8

#define TC35894_GPIOSYNC0	0xE6
#define TC35894_GPIOSYNC1	0xE7
#define TC35894_GPIOSYNC2	0xE8

#define TC35894_GPIOWAKE0	0xE9
#define TC35894_GPIOWAKE1	0xEA
#define TC35894_GPIOWAKE2	0xEB
#define TC35894_DIRECT0		0xEC
#define TC35894_DIRECT1		0xED
#define TC35894_DIRECT2		0xEE
#define TC35894_DIRECT3		0xEF

#define TC35894_GPIOODM0	0xE0
#define TC35894_GPIOODE0	0xE1
#define TC35894_GPIOODM1	0xE2
#define TC35894_GPIOODE1	0xE3
#define TC35894_GPIOODM2	0xE4
#define TC35894_GPIOODE2	0xE5
#define TC35894_INT_GPIIRQ	0
#define TC35894_INT_TI0IRQ	1
#define TC35894_INT_TI1IRQ	2
#define TC35894_INT_TI2IRQ	3
#define TC35894_INT_ROTIRQ	5
#define TC35894_INT_KBDIRQ	6
#define TC35894_INT_PORIRQ	7

#define TC35894_NR_INTERNAL_IRQS	8
#define TC35894_INT_GPIO(x)	(TC35894_NR_INTERNAL_IRQS + (x))
#define MX53_TC35894_KEY_INT			(4*32 + 4)
enum tc35894_block {
        TC35894_BLOCK_GPIO        = 1 << 0,
        TC35894_BLOCK_KEYPAD      = 1 << 1,
};
/*
 * struct matrix_keymap_data {
 *         const uint32_t *keymap;
 *                 unsigned int    keymap_size;
 *                 };
 *                 */

struct matrix_keymap_data;
struct tc35894_keypad_platform_data {
        const struct matrix_keymap_data *keymap_data;
//	const struct matrix_keymap_data *keymap_data_num;
        u8                      krow;
        u8                      kcol;
        u8                      debounce_period;
        u8                      settle_time;
        unsigned long           irqtype;
        bool                    enable_wakeup;
        bool                    no_autorepeat;
};
struct tc35894_platform_data {
        unsigned int block;
        int irq_base;
       // struct tc35894_gpio_platform_data *gpio;
        const struct tc35894_keypad_platform_data *keypad;
};

struct tc35894 {
	struct mutex lock;
	struct device *dev;
	struct i2c_client *i2c;

	int irq_base;
	int num_gpio;
	struct tc35894_platform_data *pdata;
};

extern int tc35894_reg_write(struct tc35894 *tc35894, u8 reg, u8 data);
extern int tc35894_reg_read(struct tc35894 *tc35894, u8 reg);
extern int tc35894_block_read(struct tc35894 *tc35894, u8 reg, u8 length,
			      u8 *values);
extern int tc35894_block_write(struct tc35894 *tc35894, u8 reg, u8 length,
			       const u8 *values);
extern int tc35894_set_bits(struct tc35894 *tc35894, u8 reg, u8 mask, u8 val);
/*
 *  * Keypad related platform specific constants
 *   * These values may be modified for fine tuning
 *    */
#define TC_KPD_ROWS             0x8
#define TC_KPD_COLUMNS          0x8
#define TC_KPD_DEBOUNCE_PERIOD  0xA3
#define TC_KPD_SETTLE_TIME      0xA3
/**
 * struct tc35894_gpio_platform_data - TC35894 GPIO platform data
 * @gpio_base: first gpio number assigned to TC35894.  A maximum of
 *	       %TC35894_NR_GPIOS GPIOs will be allocated.
 */
struct tc35894_gpio_platform_data {
	int gpio_base;
};
#define TC35894_NR_GPIOS	24
#define TC35894_NR_IRQS		TC35894_INT_GPIO(TC35894_NR_GPIOS)

#endif
