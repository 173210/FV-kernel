/*
 * linux/arch/mips/tx-boards/tc90416/irq.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2008
 * All Rights Reserved.
 *
 */
#include <linux/init.h>
#include <linux/interrupt.h>

#include <asm/irq_cpu.h>
#include <asm/mipsregs.h>
#include <asm/tx-boards/tsb-generic.h>

#if TC90416_IRQ_END > NR_IRQS
#error TC90416_IRQ_END > NR_IRQS
#endif

static int tc90416_irc_irqdispatch(void)
{
	int irq;
	unsigned int csr;
	unsigned long cp0_cause = read_c0_status() & read_c0_cause();

	if (cp0_cause & CAUSEF_IP7)
		irq = TC90416_IRQ_LOCAL + 7;
	else if (cp0_cause & CAUSEF_IP6)
		irq = TC90416_IRQ_LOCAL + 6;
	else if (likely(!((csr = tc90416_ircptr->csr) & TXx9_IRCSR_IF))) {
		csr = (csr >> 24) & 0x3f;
		irq = TC90416_IRQ_IRC + csr;
	} else if (cp0_cause & CAUSEF_IP0)
		irq = TC90416_IRQ_LOCAL + 0;
	else if (cp0_cause & CAUSEF_IP1)
		irq = TC90416_IRQ_LOCAL + 1;
	else
		irq = -1;
	return irq;
}

int tc90416_irq_set_type_udly(unsigned int irq, unsigned int flow_type)
{
	unsigned long long idly;
	idly = (flow_type>>16) & 0xffff;
	if (idly)
		idly = ((txx9_gbus_clock / 1000000 * idly) << 4) | 1;

	switch (irq - TC90416_IRQ_IRC){
	case TC90416_IRQ2:
		tc90416_ircptr->dcntr[0] = idly;
		return 0;
	case TC90416_IRQ3:
		tc90416_ircptr->dcntr[1] = idly;
		return 0;
	case TC90417_IR_EMAC:
		tc90416_ircptr->dcntr[2] = idly;
		return 0;
	}
	return -EINVAL;
}

static void __init
tc90416_irq_init(int irq_base, int mips_irq_base)
{
	mips_cpu_irq_init(mips_irq_base);

	/* XXX only IRQ0-5 is supported */
	txx9_64_irq_init(irq_base, tc90416_ircptr,
			 TC90416_NUM_IR, TC90416_IRQ0, 6);

	set_irq_chained_handler(mips_irq_base + TC90416_IRC_INT,
				handle_simple_irq);
}

static irqreturn_t tc90416_gbuserr_interrupt(int irq, void * dev_id)
{
	printk("GBUS Err\n");
	return IRQ_HANDLED;
}

static struct irqaction tc90416_gbuserr_action = {
	tc90416_gbuserr_interrupt, IRQF_DISABLED, CPU_MASK_NONE, "GBUS Interrupt", NULL, NULL,
};


void __init tc90416_irq_setup(void)
{
	txboard_irqdispatch = tc90416_irc_irqdispatch;
	tc90416_irq_init(TC90416_IRQ_IRC, TC90416_IRQ_LOCAL);
	setup_irq(txx9_64_irq_to_irq(TC90416_IR_WTOUT), &tc90416_gbuserr_action);
}
