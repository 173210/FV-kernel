/*
 * linux/include/asm-mips/tx-boards/tc90416-generic.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2008
 * All Rights Reserved.
 *
 */
#ifndef __ASM_TX_BOARDS_TC90416_GENERIC_H
#define __ASM_TX_BOARDS_TC90416_GENERIC_H

#include <asm/tx-boards/generic.h>
#include <asm/tx-boards/irq.h>
#include <asm/tx-boards/tc90416-regs.h>

/*
 * Clock Setting
 */
#define TC90416_GBUS_CLOCK	(txx9_master_clock / 6)
#define TC90416_CPU_CLOCK	(txx9_master_clock / 2)

/*
 * IRQ mappings
 */
#define TC90416_NR_IRQ_IOC	8	/* IOC */

#define TC90416_IRQ_LOCAL	NR_ISA_IRQS
#define TC90416_IRQ_IRC		(TC90416_IRQ_LOCAL + 8)
#define TC90416_IRQ_IOC		(TC90416_IRQ_IRC + TC90416_NUM_IR)
#define TC90416_IRQ_END		(TC90416_IRQ_IOC + TC90416_NR_IRQ_IOC)

/*
 *  For User Direct Driver
 */
#define UD_BASE_IRC  TC90416_IRQ_IRC
#define UD_MAXIRCNUM TC90416_NUM_IR
#define UD_PIO_BASE  TC90416_PIO_REG
#define UD_SSR       (tc90416_ircptr->ssr)
typedef unsigned long long UD_TYPE_T;

/*
 * Special memory barrier funcs
 */
#define tc90416_wmb()				\
	do {					\
		(void)*(volatile unsigned long *)TC90416_SXDRC_REG_MBUS; \
		__sync();			\
	} while (0)

#endif /* __ASM_TX_BOARDS_TC90416_GENERIC_H */
