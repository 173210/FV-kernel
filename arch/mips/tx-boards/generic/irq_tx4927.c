/*
 * linux/arch/mips/tx-boards/generic/irq_tx4927.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2006
 * All Rights Reserved.
 *
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <asm/irq_cpu.h>
#include <asm/tx-boards/irq.h>
#include <asm/tx-boards/tx4927.h>

void __init
tx4927_irq_init(int irq_base, int mips_irq_base)
{
	int i;

	mips_cpu_irq_init(mips_irq_base);
	txx9_irq_init(irq_base, tx4927_ircptr,
		      TX4927_NUM_IR, TX4927_IR_INT(0), TX4927_NUM_IR_INT);
	/* raise priority for errors, timers, sio */
	txx9_irq_set_pri(TX4927_IR_ECCERR, 7);
	txx9_irq_set_pri(TX4927_IR_WTOERR, 7);
	txx9_irq_set_pri(TX4927_IR_PCIERR, 7);
	txx9_irq_set_pri(TX4927_IR_PCIPME, 7);
	for (i = 0; i < TX4927_NUM_IR_TMR; i++)
		txx9_irq_set_pri(TX4927_IR_TMR(i), 6);
	for (i = 0; i < TX4927_NUM_IR_SIO; i++)
		txx9_irq_set_pri(TX4927_IR_SIO(i), 5);
	set_irq_chained_handler(mips_irq_base + TX4927_IRC_INT,
				handle_simple_irq);
}
