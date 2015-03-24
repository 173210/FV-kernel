/*
 * linux/arch/mips/tx-boards/generic/dump_tx4927.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2006
 * All Rights Reserved.
 *
 */
#include <linux/kernel.h>
#include <linux/pci.h>
#include <asm/tx-boards/generic.h>
#include <asm/tx-boards/pci.h>
#include <asm/tx-boards/tx4927.h>
#include <asm/tx-boards/tx4938.h>
#include <asm/tx-boards/tx4939.h>

#ifdef CONFIG_PCI
static void tx4927_dump_pcic_settings1(struct tx4927_pcic_reg *pcicptr)
{
	int i;
	volatile __u32 *preg = (volatile __u32 *)pcicptr;

	printk("tx4927 pcic (0x%p) settings:", pcicptr);
	for (i = 0; i < sizeof(struct tx4927_pcic_reg); i += 4) {
		if (i % 32 == 0)
			printk("\n%04x:", i);
		/* skip registers with side-effects */
		if (i == offsetof(struct tx4927_pcic_reg, g2pintack)
		    || i == offsetof(struct tx4927_pcic_reg, g2pspc)
		    || i == offsetof(struct tx4927_pcic_reg, g2pcfgadrs)
		    || i == offsetof(struct tx4927_pcic_reg, g2pcfgdata)
			) {
			printk(" XXXXXXXX");
			preg++;
			continue;
		}
		printk(" %08x", *preg++);
	}
	printk("\n");
}

extern struct pci_controller *hose_head;

void tx4927_dump_pcic_settings(void)
{
	struct pci_controller *hose;
	struct tx4927_pcic_reg *pcicptr;
	for (hose = hose_head; hose; hose = hose->next) {
		if ((pcicptr = get_tx4927_pcicptr(hose)) != NULL)
			tx4927_dump_pcic_settings1(pcicptr);
	}
}
#endif /* CONFIG_PCI */

#if 0	/* for debugging */
void tx4927_dump_irc_settings(void)
{
	int i;

	printk("tx%x irc settings:\n", txx9_pcode);
	printk("cer:\t%08x\n", tx4927_ircptr->cer);
	for (i = 0; i < 2; i++)
		printk("cr%d:\t%08x\n", i, tx4927_ircptr->cr[i]);
	for (i = 0; i < 8; i++)
		printk("ilr%d:\t%08x\n", i, tx4927_ircptr->ilr[i]);
	printk("imr:\t%08x\n", tx4927_ircptr->imr);
	printk("scr:\t%08x\n", tx4927_ircptr->scr);
	printk("ssr:\t%08x\n", tx4927_ircptr->ssr);
	printk("csr:\t%08x\n", tx4927_ircptr->csr);
}

void tx4939_dump_irc_settings(void)
{
	int i;

	printk("tx%x irc settings:\n", txx9_pcode);
	printk("den:\t%08x\n", tx4939_ircptr->den.r);
	printk("scipb:\t%08x\n", tx4939_ircptr->scipb.r);
	for (i = 0; i < 2; i++)
		printk("dm%d:\t%08x\n", i, tx4939_ircptr->dm[i].r);
	for (i = 0; i < 2; i++)
		printk("dm%d:\t%08x\n", 2 + i, tx4939_ircptr->dm2[i].r);
	for (i = 0; i < 16; i++)
		printk("lvl%d:\t%08x\n", i, tx4939_ircptr->lvl[i].r);
	printk("msk:\t%08x\n", tx4939_ircptr->msk.r);
	printk("edc:\t%08x\n", tx4939_ircptr->edc.r);
	printk("cs:\t%08x\n", tx4939_ircptr->cs.r);
	printk("pnd0:\t%08x\n", tx4939_ircptr->pnd0.r);
	printk("pnd1:\t%08x\n", tx4939_ircptr->pnd1.r);
}

void tx4927_dump_dmac_settings(int dmach)
{
	struct tx4927_dma_reg *dmaptr;
	if ((txx9_pcode & 0xfff0) == 0x4930)
		dmaptr = tx4938_dmaptr(dmach < 4 ? 0 : 1);
	else
		dmaptr = tx4927_dmaptr;
	printk("tx%x dmac[%d] settings:\n", txx9_pcode, dmach);
	printk("cha:\t%016llx\n", (unsigned long long)dmaptr->ch[dmach].cha);
	printk("sar:\t%016llx\n", (unsigned long long)dmaptr->ch[dmach].sar);
	printk("dar:\t%016llx\n", (unsigned long long)dmaptr->ch[dmach].dar);
	printk("cntr:\t%08x\tsair:\t%08x\tdair:\t%08x\n",
	       dmaptr->ch[dmach].cntr,
	       dmaptr->ch[dmach].sair,
	       dmaptr->ch[dmach].dair);
	printk("ccr:\t%08x\tcsr:\t%08x\n",
	       dmaptr->ch[dmach].ccr,
	       dmaptr->ch[dmach].csr);
}

void tx4927_dump_dmac_settings_all(void)
{
	int i;

	printk("tx%x dmac settings:\n", txx9_pcode);
	printk("mcr:\t%08x\n", tx4927_dmaptr->mcr);
	for (i = 0; i < 4; i++)
		tx4927_dump_dmac_settings(i);
	if ((txx9_pcode & 0xfff0) == 0x4930) {
		printk("mcr:\t%08x\n", tx4927_dmaptr->mcr);
		for (; i < 8; i++)
			tx4927_dump_dmac_settings(i);
	}
}
#endif

#ifdef CONFIG_PCI
static void tx4927_report_pcic_status1(struct tx4927_pcic_reg *pcicptr)
{
	__u16 pcistatus = (__u16)(pcicptr->pcistatus >> 16);
	__u32 g2pstatus = pcicptr->g2pstatus;
	__u32 pcicstatus = pcicptr->pcicstatus;
	static struct {
		__u32 flag;
		const char *str;
	} pcistat_tbl[] = {
		{ PCI_STATUS_DETECTED_PARITY,	"DetectedParityError" },
		{ PCI_STATUS_SIG_SYSTEM_ERROR,	"SignaledSystemError" },
		{ PCI_STATUS_REC_MASTER_ABORT,	"ReceivedMasterAbort" },
		{ PCI_STATUS_REC_TARGET_ABORT,	"ReceivedTargetAbort" },
		{ PCI_STATUS_SIG_TARGET_ABORT,	"SignaledTargetAbort" },
		{ PCI_STATUS_PARITY,	"MasterParityError" },
	}, g2pstat_tbl[] = {
		{ TX4927_PCIC_G2PSTATUS_TTOE,	"TIOE" },
		{ TX4927_PCIC_G2PSTATUS_RTOE,	"RTOE" },
	}, pcicstat_tbl[] = {
		{ TX4927_PCIC_PCICSTATUS_PME,	"PME" },
		{ TX4927_PCIC_PCICSTATUS_TLB,	"TLB" },
		{ TX4927_PCIC_PCICSTATUS_NIB,	"NIB" },
		{ TX4927_PCIC_PCICSTATUS_ZIB,	"ZIB" },
		{ TX4927_PCIC_PCICSTATUS_PERR,	"PERR" },
		{ TX4927_PCIC_PCICSTATUS_SERR,	"SERR" },
		{ TX4927_PCIC_PCICSTATUS_GBE,	"GBE" },
		{ TX4927_PCIC_PCICSTATUS_IWB,	"IWB" },
	};
	int i, cont;

	if (pcistatus & TX4927_PCIC_PCISTATUS_ALL) {
		printk("pcistat:%04x(", pcistatus);
		for (i = 0, cont = 0; i < ARRAY_SIZE(pcistat_tbl); i++)
			if (pcistatus & pcistat_tbl[i].flag)
				printk("%s%s",
				       cont++ ? " " : "", pcistat_tbl[i].str);
		printk(") ");
	}
	if (g2pstatus & TX4927_PCIC_G2PSTATUS_ALL) {
		printk("g2pstatus:%08x(", g2pstatus);
		for (i = 0, cont = 0; i < ARRAY_SIZE(g2pstat_tbl); i++)
			if (g2pstatus & g2pstat_tbl[i].flag)
				printk("%s%s",
				       cont++ ? " " : "", g2pstat_tbl[i].str);
		printk(") ");
	}
	if (pcicstatus & TX4927_PCIC_PCICSTATUS_ALL) {
		printk("pcicstatus:%08x(", pcicstatus);
		for (i = 0, cont = 0; i < ARRAY_SIZE(pcicstat_tbl); i++)
			if (pcicstatus & pcicstat_tbl[i].flag)
				printk("%s%s",
				       cont++ ? " " : "", pcicstat_tbl[i].str);
		printk(")");
	}
	printk("\n");
}
void tx4927_report_pcic_status(void)
{
	struct pci_controller *hose;
	struct tx4927_pcic_reg *pcicptr;
	for (hose = hose_head; hose; hose = hose->next) {
		if ((pcicptr = get_tx4927_pcicptr(hose)) != NULL)
			tx4927_report_pcic_status1(pcicptr);
	}
}
#else
void tx4927_report_pcic_status(void) {}
#endif /* CONFIG_PCI */
