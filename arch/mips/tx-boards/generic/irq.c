/*
 * linux/arch/mips/tx-boards/generic/irq.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2007
 * All Rights Reserved.
 *
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/sysrq.h>
#include <linux/module.h>
#include <linux/notifier.h>

#include <asm/system.h>
#include <asm/mipsregs.h>
#include <asm/i8259.h>
#include <asm/io.h>
#include <asm/gdb-stub.h>
#include <asm/tx-boards/pmon.h>
#include <asm/tx-boards/irq.h>
#include <asm/tx-boards/generic.h>

ATOMIC_NOTIFIER_HEAD(spurious_irq_notifier_list);
EXPORT_SYMBOL(spurious_irq_notifier_list);

/* irq_setup() overrides this */
int (*txboard_irqdispatch)(void);

asmlinkage void plat_irq_dispatch(void)
{
	int irq = txboard_irqdispatch();
	if (likely(irq >= 0) ||
	    ((atomic_notifier_call_chain(&spurious_irq_notifier_list,
					 0, &irq) | NOTIFY_OK) &&
	     irq >= 0))
		do_IRQ(irq);
	else
		spurious_interrupt();
}

extern void txx9_rs_kgdb_hook(int, int);
extern void setupDebugInterrupt(void);

static int pmon_bp_enabled __initdata;
static int __init pmon_bp_setup(char *str)
{
	pmon_bp_enabled = 1;
	return 1;
}
__setup("pmon-bp", pmon_bp_setup);

#ifdef CONFIG_MAGIC_SYSRQ
#ifndef CONFIG_KGDB
static void sysrq_handle_pmon(int key, struct tty_struct *tty)
{
	pmon_breakpoint();
}

static struct sysrq_key_op sysrq_pmon_op =
{
	.handler =	sysrq_handle_pmon,
	.help_msg =	"X(pmon)",
	.action_msg =	"Entering pmon",
};
#endif /* !CONFIG_KGDB */
#endif /* CONFIG_MAGIC_SYSRQ */

void (*txboard_irq_setup)(void) __initdata;

void __init arch_init_irq(void)
{
#ifdef CONFIG_KGDB
	char *ctype;
	int line = 0;
	int baud = 9600;
	int use_gdb_on_txx9_serial = 0;
#endif

	clear_c0_status(ST0_IM|ST0_IE);	/* disable all interrupts */
	txboard_irq_setup();	/* board specific setup */

#ifdef CONFIG_KGDB
	if ((ctype = strstr(prom_getcmdline(), "kgdb=")) != NULL) {
		ctype += strlen("kgdb=");
		/*
		 * If we have both standard and txx9 serial,
		 * use ttyS0,1 for Standard-SIO, ttyTX0,1 for CPU-SIO.
		 */
		if (strncmp(ctype, "ttyS", 4) == 0) {
			ctype += 4;
#if defined(CONFIG_SERIAL_TXX9) && defined(CONFIG_SERIAL_TXX9_STDSERIAL)
			use_gdb_on_txx9_serial = 1;
#endif
		}
#if defined(CONFIG_SERIAL_TXX9) && !defined(CONFIG_SERIAL_TXX9_STDSERIAL)
		if (strncmp(ctype, "ttyTX", 5) == 0) {
			ctype += 5;
			use_gdb_on_txx9_serial = 1;
		}
#endif
	}
	if (use_gdb_on_txx9_serial) {
		line = *ctype - '0';
		ctype++;
		if (*ctype == ',') {
			ctype++;
			baud = simple_strtoul(ctype, NULL, 10);
		}

		printk("KGDB: Using serial line /dev/%s%d for "
		       "session (%dbps)\n",
#if defined(CONFIG_SERIAL_TXX9) && !defined(CONFIG_SERIAL_TXX9_STDSERIAL)
		       use_gdb_on_txx9_serial ? "ttyTX" : "ttyS",
#else
		       "ttyS",
#endif
		       line, baud);
		pmon_printf("Now you can start KGDB session (%dbps).\n", baud);
		/* flush FIFO :-) */
		pmon_printf("\r\r\r\r\r\r\r\r\r");
	}
#ifdef CONFIG_SERIAL_TXX9
	if (use_gdb_on_txx9_serial) {
		txx9_rs_kgdb_hook(line, baud);
	}
#endif
	if (use_gdb_on_txx9_serial) {
		setupDebugInterrupt();
	} else if (pmon_bp_enabled) {
		set_pmon_debug_traps();
	}
#else /* CONFIG_KGDB */
	if (pmon_bp_enabled) {
		set_pmon_debug_traps();
		register_sysrq_key('x', &sysrq_pmon_op);
	}
#endif /* CONFIG_KGDB */
}
