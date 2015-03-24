/*
 * linux/arch/mips/tx-boards/tc90411/prom.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2005-2006
 * All Rights Reserved.
 *
 */
#include <linux/init.h>
#include <linux/string.h>
#include <asm/bootinfo.h>
#include <asm/tx-boards/tsb-generic.h>

int __init tc90416_prom_init(int argc, char **argv, char **envp)
{
	if (argc == -1) {
		return 0;
	}

#ifdef CONFIG_NATSEMI
	strcat(arcs_cmdline, " natsemi.rx_copybreak=1519");
#endif

#if defined(CONFIG_TXSSETH)
	switch (mips_machtype) {
	case MACH_TC90417_CELLTV1:
		switch (((unsigned int)tc90416_ccfgptr->ccfg >> 8) & 0x3) {
		case 0x0:
		case 0x1:
			strcat(arcs_cmdline, " txss-eth.speed=1000");
			strcat(arcs_cmdline, " txss-eth.phyaddr=8");
			strcat(arcs_cmdline, " txss-eth.phymask=0xff");
			break;
		default:
			break;
		}
		break;
	case MACH_TC90417_STBF1:
		strcat(arcs_cmdline, " txss-eth.speed=1000");
		strcat(arcs_cmdline, " txss-eth.phyaddr=8");
		strcat(arcs_cmdline, " txss-eth.phymask=0xff");
		break;
	case MACH_TC90417_STBR1:
	case MACH_TC90417_STBF2:
	case MACH_TC90417_STBR2:
	case MACH_TC90417_JPTVZF1:
	case MACH_TC90417_JPTVZR1:
		strcat(arcs_cmdline, " txss-eth.speed=100");
		strcat(arcs_cmdline, " txss-eth.phyaddr=8");
		strcat(arcs_cmdline, " txss-eth.phymask=0xff");
		break;
	}
#endif

	if (boot_mem_map.nr_map == 0) {
		/* default setting: 128MB */
		add_memory_region(0, 0x08000000, BOOT_MEM_RAM);
	}
	return 0;
}
