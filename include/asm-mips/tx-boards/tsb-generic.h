/*
 * linux/include/asm-mips/tx-boards/tsb-generic.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2007
 * All Rights Reserved.
 *
 */
#ifndef __ASM_TX_BOARDS_TSB_GENERIC_H
#define __ASM_TX_BOARDS_TSB_GENERIC_H

#include <asm/bootinfo.h>

#include <asm/tx-boards/generic.h>

#ifdef CONFIG_TOSHIBA_TC90411
#include <asm/tx-boards/tc90411-generic.h>
#define IS_TC90411	(mips_machgroup == MACH_GROUP_TC90411)
#else
#define IS_TC90411	0
#endif

#if defined(CONFIG_TOSHIBA_OPBD4938) || defined(CONFIG_TOSHIBA_OPBD4939)
#ifdef CONFIG_TOSHIBA_OPBD4939
#include <asm/tx-boards/opbd4939.h>
#else
#include <asm/tx-boards/opbd4938.h>
#endif

#define IS_OPBD493X        ((mips_machgroup == MACH_GROUP_TX) && (mips_machtype == MACH_TX_OPBD493X))
#else
#define IS_OPBD493X        0
#endif

#ifdef CONFIG_TOSHIBA_TC90412
#include <asm/tx-boards/tc90412-generic.h>
#define IS_TC90412	(mips_machgroup == MACH_GROUP_TC90412)
#else
#define IS_TC90412	0
#endif

#ifdef CONFIG_TOSHIBA_TC90416
#include <asm/tx-boards/tc90416-generic.h>
#define IS_TC90416	(mips_machgroup == MACH_GROUP_TC90416)
#else
#define IS_TC90416	0
#endif

#ifdef CONFIG_TX_RBTX4939
#include <asm/tx-boards/rbtx4939.h>
#endif

#endif /* __ASM_TX_BOARDS_TSB_GENERIC_H */
