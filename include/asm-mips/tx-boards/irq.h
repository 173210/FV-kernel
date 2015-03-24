/*
 * linux/include/asm-mips/tx-boards/irq.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2007
 * All Rights Reserved.
 *
 */
#ifndef __ASM_TX_BOARDS_IRQ_H
#define __ASM_TX_BOARDS_IRQ_H

#include <linux/types.h>              /* for off_t */

struct txx9_irc_reg;
extern void txx9_irq_init(int irq_base, struct txx9_irc_reg *ircptr,
			  int irq_num, int exint_start, int exint_num);
extern int txx9_irq_to_irq(int irq);
extern int txx9_irq_set_pri(int irc_irq, int new_pri);
extern int tx4939_irq_to_irq(int irq);
extern int tx4939_irq_set_pri(int irc_irq, int new_pri);
struct txx9_64_irc_reg;
extern void txx9_64_irq_init(int irq_base, struct txx9_64_irc_reg *ircptr,
			  int irq_num, int exint_start, int exint_num);
extern int txx9_64_irq_to_irq(int irq);
extern int txx9_64_irq_set_pri(int irc_irq, int new_pri);

extern void tx4927_irq_init(int irq_base, int mips_irq_base);
extern void tx4939_irq_init(int irq_base, int mips_irq_base);

extern int (*txboard_irqdispatch)(void);

/* Machine specific interrupt initialization  */
extern void (*txboard_irq_setup)(void);

/* 8 bit version of __ffs in linux/bitops.h */
/* find first bit set (returns 0..7) */
static inline unsigned int __ffs8(unsigned char word)
{
	int num = 0;

	if ((word & 0xf) == 0) {
		num += 4;
		word >>= 4;
	}
	if ((word & 0x3) == 0) {
		num += 2;
		word >>= 2;
	}
	if ((word & 0x1) == 0)
		num += 1;
	return num;
}

#define NR_ISA_IRQS 16

extern struct atomic_notifier_head spurious_irq_notifier_list;

#endif /* __ASM_TX_BOARDS_IRQ_H */
