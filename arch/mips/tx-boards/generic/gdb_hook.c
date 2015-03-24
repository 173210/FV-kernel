/*
 * linux/arch/mips/tx-boards/generic/gdb_hook.c
 *
 * Based on arch/mips/mips-boards/generic/gdb_hook.c by Carsten Langgaard.
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
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <asm/errno.h>
#include <asm/gdb-stub.h>
#include <asm/signal.h>
#include <asm/io.h>
#include <asm/irq_regs.h>
#include <asm/tx-boards/generic.h>

static struct irqaction kgdb_irqaction;
static struct resource kgdb_resource;

static int (*generic_putDebugChar)(char);
static char (*generic_getDebugChar)(void);
static void (*generic_setupDebugInterrupt)(void) __initdata;

int putDebugChar(char c)
{
	return generic_putDebugChar(c);
}

char getDebugChar(void)
{
	return generic_getDebugChar();
}

void __init setupDebugInterrupt(void)
{
	if (generic_setupDebugInterrupt)
		generic_setupDebugInterrupt();
}

static void do_stop_command(void)
{
	struct pt_regs *regs = get_irq_regs();
	extern void breakpoint(void);
	if (!user_mode(regs))
		set_async_breakpoint(&regs->cp0_epc);
	else
		breakpoint();
}

static int kgdb_port = -1;

static struct uart_port rs_kgdb_port[2];
#define NR_PORTS	(sizeof(rs_kgdb_port)/sizeof(rs_kgdb_port[0]))

#if defined(CONFIG_SERIAL_TXX9)

#include <asm/tx-boards/txx9.h>

/* flag aliases */
#define UPF_TXX9_USE_SCLK	UPF_MAGIC_MULTIPLIER

static inline struct txx9_sio_reg *sio_reg(struct uart_port *port)
{
	return (struct txx9_sio_reg __force *)port->membase;
}

int txx9_rs_putDebugChar(char c)
{
	struct uart_port *port = &rs_kgdb_port[kgdb_port];
	if (kgdb_port < 0) { 	/* need to init device first */
		return 0;
	}

	while (!(sio_reg(port)->cisr & TXx9_SICISR_TXALS))
		;
	sio_reg(port)->tfifo = c;

	return 1;
}

char txx9_rs_getDebugChar(void)
{
	int dicr;
	char c;
	struct uart_port *port = &rs_kgdb_port[kgdb_port];
	if (kgdb_port < 0) { 	/* need to init device first */
		return 0;
	}
	/* diable RX int. */
	dicr = sio_reg(port)->dicr;
	sio_reg(port)->dicr = 0;

	/* read char */
	while (sio_reg(port)->disr & TXx9_SIDISR_UVALID)
		;
	c = sio_reg(port)->rfifo;

	/* clear RX int. status */
	sio_reg(port)->disr &= ~TXx9_SIDISR_RDIS;
	/* enable RX int. */
	sio_reg(port)->dicr = dicr;

	return c;
}

static irqreturn_t txx9_rs_interrupt_debug(int irq, void *dev_id)
{
	struct uart_port *port = &rs_kgdb_port[kgdb_port];
	int status;
	unsigned char ch;

	/* Look for kgdb 'stop' character, consult the gdb
	 * documentation for remote target debugging and
	 * arch/sparc/kernel/sparc-stub.c (or arch/mips/kernel/gdb-stub.c)
	 * to see how all this works.
	 */
	status = sio_reg(port)->disr;
	if (!(status & TXx9_SIDISR_UVALID)) {
		ch = sio_reg(port)->rfifo;
		if (ch =='\003')
			do_stop_command();
	}
	/* clear interrupt source... (required?) */
	while (sio_reg(port)->disr & (TXx9_SIDISR_RDIS|TXx9_SIDISR_TOUT)) {
		if (!(sio_reg(port)->disr & TXx9_SIDISR_UVALID)) {
			(void)sio_reg(port)->rfifo;
		}
		sio_reg(port)->disr &= ~(TXx9_SIDISR_RDIS|TXx9_SIDISR_TOUT);
	}
	return IRQ_HANDLED;
}

static void __init txx9_rs_setupDebugInterrupt(void)
{
	struct uart_port *port = &rs_kgdb_port[kgdb_port];

	/* Enable interrupts because we now want to receive the
	 * 'control-c' character from the client attached to us
	 * asynchronously.
	 */
	if (kgdb_port < 0)
		return;
	kgdb_irqaction.handler = txx9_rs_interrupt_debug;
	kgdb_irqaction.flags = IRQF_DISABLED;
	kgdb_irqaction.mask = CPU_MASK_NONE;
	kgdb_irqaction.name = "serial_txx9(debug)";
	if (setup_irq(port->irq, &kgdb_irqaction) == 0) {
		/* clear FIFO */
		sio_reg(port)->fcr |=
			TXx9_SIFCR_TFRST | TXx9_SIFCR_RFRST |
			TXx9_SIFCR_FRSTE;
		sio_reg(port)->fcr &=
			~(TXx9_SIFCR_TFRST | TXx9_SIFCR_RFRST |
			  TXx9_SIFCR_FRSTE);
		/* clear interrupts */
		sio_reg(port)->disr = 0;
		/* enable receiver data interrupt */
		sio_reg(port)->dicr |= TXx9_SIDICR_RIE;
		printk(KERN_INFO "ttyTX%d: enable interrupts for C-c.\n",
		       kgdb_port);
	}
}

void __init txx9_rs_kgdb_hook(int tty_no, int baud)
{
	struct uart_port *port = &rs_kgdb_port[tty_no];
	unsigned int quot;
	unsigned int tmout = 10000;

	if (tty_no < 0 || tty_no >= NR_PORTS)
		return;
	if (!port->membase || port->iotype != UPIO_MEM)
		return;
	kgdb_port = tty_no;

	/*
	 * Reset the UART.
	 */
	sio_reg(port)->fcr = TXx9_SIFCR_SWRST;
	/* TX4925 BUG WORKAROUND.  Accessing SIOC register
	 * immediately after soft reset causes bus error. */
	mmiowb();
	udelay(1);
	while ((sio_reg(port)->fcr & TXx9_SIFCR_SWRST) && --tmout)
		udelay(1);

	/*
	 * and set the speed of the serial port
	 */
	sio_reg(port)->lcr = TXx9_SILCR_UMODE_8BIT |
		TXx9_SILCR_USBL_1BIT |
		((port->flags & UPF_TXX9_USE_SCLK) ?
		 TXx9_SILCR_SCS_SCLK_BG : TXx9_SILCR_SCS_IMCLK_BG);
	quot = (port->uartclk / 16 + baud / 2) / baud;
	quot >>= 1;
	if (quot < 256)
		sio_reg(port)->bgr = quot | TXx9_SIBGR_BCLK_T0;
	else if (quot < (256 << 2))
		sio_reg(port)->bgr = (quot >> 2) | TXx9_SIBGR_BCLK_T2;
	else if (quot < (256 << 4))
		sio_reg(port)->bgr = (quot >> 4) | TXx9_SIBGR_BCLK_T4;
	else if (quot < (256 << 6))
		sio_reg(port)->bgr = (quot >> 6) | TXx9_SIBGR_BCLK_T6;
	else
		sio_reg(port)->bgr = 0xff | TXx9_SIBGR_BCLK_T6;

	/* no RTS/CTS control */
	sio_reg(port)->flcr = TXx9_SIFLCR_RTSTL_MAX /* 15 */;
	/* Enable RX/TX */
	sio_reg(port)->flcr &= ~(TXx9_SIFLCR_RSDE | TXx9_SIFLCR_TSDE);

	/* prevent initialization by driver */
	kgdb_resource.name = "serial_txx9(debug)";
	kgdb_resource.start = port->mapbase;
	kgdb_resource.end = port->mapbase + 36 - 1;
	kgdb_resource.flags = IORESOURCE_MEM | IORESOURCE_BUSY;
	insert_resource(&iomem_resource, &kgdb_resource);

	generic_putDebugChar = txx9_rs_putDebugChar;
	generic_getDebugChar = txx9_rs_getDebugChar;
	generic_setupDebugInterrupt = txx9_rs_setupDebugInterrupt;
}

int __init early_serial_txx9_kgdb_setup(struct uart_port *port)
{
	int i = port->line;
	if (i >= NR_PORTS)
		return(-ENOENT);
	rs_kgdb_port[i] = *port;
	return(0);
}

#endif
