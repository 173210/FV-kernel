/*
 * linux/include/asm-mips/tx-boards/generic.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2007
 * All Rights Reserved.
 *
 */
#ifndef __ASM_TX_BOARDS_GENERIC_H
#define __ASM_TX_BOARDS_GENERIC_H

#include <linux/init.h>
#include <linux/ioport.h>	// for struct resource
#include <linux/errno.h>

extern void (*txboard_arch_init)(void);
extern void (*txboard_device_init)(void);
struct sysdev_attribute;
int txboard_sysdev_create_file(struct sysdev_attribute *a);
struct sysdev_class_attribute;
int txboard_sysdev_class_create_file(struct sysdev_class_attribute *a);

struct uart_port;
#ifdef CONFIG_KGDB
extern int early_serial_kgdb_setup(struct uart_port *port) __init;
extern int early_serial_txx9_kgdb_setup(struct uart_port *port) __init;
#else
static inline int early_serial_kgdb_setup(struct uart_port *port) {return -ENODEV;}
static inline int early_serial_txx9_kgdb_setup(struct uart_port *port) {return -ENODEV;}
#endif
/* uart flag aliases */
#define UPF_TXX9_HAVE_CTS_LINE	UPF_BUGGY_UART
#define UPF_TXX9_USE_SCLK	UPF_MAGIC_MULTIPLIER

extern void tx4927_dump_irc_settings(void);
extern void tx4939_dump_irc_settings(void);

extern void txboard_dump_pci_config(void);
extern void tx4927_report_pcic_status(void);
extern void tx4927_dump_pcic_settings(void);

extern int txboard_ob_ether_disable;

extern int early_serial_txx9_setup(struct uart_port *port) __init;

extern int tx_ccfg_toeon;

extern struct resource tx_ce_res[];
#define TX_CE(n)	(tx_ce_res[(n)].start)

extern unsigned int txx9_pcode;
extern char txx9_pcode_str[8];
extern struct resource txx9_reg_res;
extern void txx9_reg_res_init(unsigned int pcode, unsigned long base, unsigned long size) __init;

extern unsigned int txx9_master_clock;
extern unsigned int txx9_cpu_clock;
extern unsigned int txx9_gbus_clock;
#define TXX9_GBUSCLK	txx9_gbus_clock
#define TXX9_IMCLK	(txx9_gbus_clock / 2)

extern int txboard_timer_irqno __initdata;

extern void tx4927_setup(void) __init;
extern void tx4927_time_init(unsigned int cpu_irq_base, unsigned int tmrnr) __init;
extern void tx4927_setup_pcierr_irq(void) __init;
extern void tx4927_setup_serial(unsigned int sclk, int irq_base, unsigned int cts_mask, unsigned int ch_mask) __init;
extern void tx4927_mtd_setup(int ch) __init;
extern void tx4938_init_ethaddr(int ch, const unsigned char *addr) __init;
extern void tx4938_init_ethaddr_by_seeprom(int busid, int chipid) __init;
extern void tx4938_setup_eth(void) __init;
struct spi_device;
extern void tx4938_spi_setup(int busid, int (*cs_func)(struct spi_device *spi, int on)) __init;
extern void tx4938_setup_ata(unsigned int irq, unsigned int gap, int tune) __init;
extern void tx4938_setup_ndfmc(unsigned int hold, unsigned int spw, int wp) __init;

extern void tx4939_setup(void) __init;
extern void tx4939_time_init(unsigned int cpu_irq_base, unsigned int tmrnr) __init;
extern void tx4939_setup_pcierr_irq(void) __init;
extern void tx4939_setup_serial(unsigned int sclk, int irq_base, unsigned int cts_mask, unsigned int ch_mask) __init;
extern void tx4939_setup_ata(void) __init;
extern void tx4939_init_ethaddr(int ch, const unsigned char *addr) __init;
extern void tx4939_setup_eth(void) __init;
#define tx4939_mtd_setup(ch) tx4927_mtd_setup(ch)
extern void tx4939_setup_ndfmc(unsigned int hold, unsigned int spw,
			       unsigned char wp_mask, unsigned char ch_mask,
			       unsigned char wide_mask) __init;

struct txx9_tmr_reg;
extern void txx9_time_init(struct txx9_tmr_reg *tmrptr) __init;

/* 7SEG LED */
extern void txboard_7segled_init(unsigned int num, void (*putc)(unsigned int pos, unsigned char val)) __init;
extern void txboard_7segled_putc(unsigned int pos, char c);

/* SPI */
extern int txx9_spi_init(int busid, unsigned long base, int irq,
			 int (*cs_func)(struct spi_device *spi, int on)) __init;

#ifdef CONFIG_SPI
extern int spi_eeprom_register(int busid, int chipid, int size) __init;
extern int spi_eeprom_read(int busid, int chipid,
			   int address, unsigned char *buf, int len);
#else
static inline int spi_eeprom_register(int busid, int chipid, int size) {
	return -ENODEV;
}
static inline int spi_eeprom_read(int busid, int chipid,
				  int address, unsigned char *buf, int len) {
	return -ENODEV;
}
#endif

/* helper for platform devices */
extern void register_txboard_led(unsigned long ioaddr, struct resource *parent,
				 unsigned int num, int lowactive) __init;
extern void register_rs5c348_info(u16 bus_num, u16 chip_select) __init;
extern void register_dallas_rtc_device(char *name, unsigned long base, int irq) __init;
extern void register_txboard_obne_device(unsigned long ioaddr, int irq) __init;

extern char *prom_getcmdline(void) __init;

#endif /* __ASM_TX_BOARDS_GENERIC_H */
