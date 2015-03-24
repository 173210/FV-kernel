/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2006-2007
 * All Rights Reserved.
 * 
 */
#ifndef __ASM_MACH_TX_MEMORY_BARRIER_H
#define __ASM_MACH_TX_MEMORY_BARRIER_H

/* memory berrier funcs will depend on CPU types. */
/* So, we have to include the below file.         */
/* #include <asm/tx-boards/tsb-generic.h> */ /* this cause circular dependencies */
extern void __wmb(void);
extern void __rmb(void);
extern void __mb(void);
#define __iob()		fast_iob();

#endif /* __ASM_MACH_TX_MEMORY_BARRIER_H */
