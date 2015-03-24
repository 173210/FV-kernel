/*
 * linux/arch/mips/tx-boards/generic/pci_tx4927.c
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
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <asm/byteorder.h>
#include <asm/tx-boards/generic.h>
#include <asm/tx-boards/pci.h>
#include <asm/tx-boards/irq.h>
#include <asm/tx-boards/tx4927.h>
#include <asm/tx-boards/tx4938.h>
#include <asm/tx-boards/tx4939.h>

static struct {
	struct pci_controller *channel;
	struct tx4927_pcic_reg *pcicptr;
} pcicptrs[2];	/* TX4938 has 2 pcic */

#if !defined(CONFIG_TOSHIBA_TC90411) && !defined(CONFIG_TOSHIBA_TC90412)
static
#endif
void __init set_tx4927_pcicptr(struct pci_controller *channel,
				      struct tx4927_pcic_reg *pcicptr)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(pcicptrs); i++) {
		if (pcicptrs[i].channel == channel) {
			pcicptrs[i].pcicptr = pcicptr;
			return;
		}
	}
	for (i = 0; i < ARRAY_SIZE(pcicptrs); i++) {
		if (!pcicptrs[i].channel) {
			pcicptrs[i].channel = channel;
			pcicptrs[i].pcicptr = pcicptr;
			return;
		}
	}
	BUG();
}

struct tx4927_pcic_reg *get_tx4927_pcicptr(struct pci_controller *channel)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(pcicptrs); i++) {
		if (pcicptrs[i].channel == channel) {
			return pcicptrs[i].pcicptr;
		}
	}
	return NULL;
}

#ifdef CONFIG_TOSHIBA_TC90411_USBHC_WORKAROUND
static void
set_g2ptocnt(struct tx4927_pcic_reg *pcicptr, u8 trdyto, u8 retryto)
{
	pcicptr->g2ptocnt &= ~0xffff;
	pcicptr->g2ptocnt |= trdyto;
	pcicptr->g2ptocnt |= retryto << 8;
}
#endif

static int
mkaddr(struct pci_bus *bus, unsigned int devfn, int where, struct tx4927_pcic_reg *pcicptr)
{
	if (bus->parent == NULL &&
	    devfn >= PCI_DEVFN(TX4927_PCIC_MAX_DEVNU, 0))
		return -1;
	pcicptr->g2pcfgadrs = ((bus->number & 0xff) << 0x10) |
		((devfn & 0xff) << 0x08) |
		(where & 0xfc) |
		(bus->parent ? 1 : 0);
	/* clear M_ABORT and Disable M_ABORT Int. */
	pcicptr->pcistatus =
		(pcicptr->pcistatus & 0x0000ffff) |
		(PCI_STATUS_REC_MASTER_ABORT << 16);

#ifdef CONFIG_TOSHIBA_TC90411_USBHC_WORKAROUND
	/* Enable Retry & TRDY timeout. */
	set_g2ptocnt(pcicptr, 0xff, 0xff);
#endif

	return 0;
}

static int
check_abort(struct tx4927_pcic_reg *pcicptr)
{
	int code = PCIBIOS_SUCCESSFUL;
	/* wait write cycle completion before checking error status */
	while (pcicptr->pcicstatus & TX4927_PCIC_PCICSTATUS_IWB)
		;
	if (pcicptr->pcistatus & (PCI_STATUS_REC_MASTER_ABORT << 16)) {
		pcicptr->pcistatus =
			(pcicptr->pcistatus & 0x0000ffff) |
			(PCI_STATUS_REC_MASTER_ABORT << 16);
		code = PCIBIOS_DEVICE_NOT_FOUND;
	}
#if 1 /* TX4927 PCIC BUG WORKAROND */
	/* if G2PCFGADRS[2] is 0, G2PINTACK may cause problems. */
	pcicptr->g2pcfgadrs |= 4;
#endif

#ifdef CONFIG_TOSHIBA_TC90411_USBHC_WORKAROUND
	/* Disable Retry & TRDY timeout. */
	set_g2ptocnt(pcicptr, 0x00, 0x00);
#endif

	return code;
}

static inline u8 icd_readb(int offset, struct tx4927_pcic_reg *pcicptr)
{
#ifdef __BIG_ENDIAN
	offset ^= 3;
#endif
	return *(volatile u8 *)((unsigned long)&pcicptr->g2pcfgdata | offset);
}
static inline u16 icd_readw(int offset, struct tx4927_pcic_reg *pcicptr)
{
#ifdef __BIG_ENDIAN
	offset ^= 2;
#endif
	return *(volatile u16 *)((unsigned long)&pcicptr->g2pcfgdata | offset);
}
static inline u32 icd_readl(struct tx4927_pcic_reg *pcicptr)
{
	return pcicptr->g2pcfgdata;
}
static inline void icd_writeb(u8 val, int offset, struct tx4927_pcic_reg *pcicptr)
{
#ifdef __BIG_ENDIAN
	offset ^= 3;
#endif
	*(volatile u8 *)((unsigned long)&pcicptr->g2pcfgdata | offset) = val;
}
static inline void icd_writew(u16 val, int offset, struct tx4927_pcic_reg *pcicptr)
{
#ifdef __BIG_ENDIAN
	offset ^= 2;
#endif
	*(volatile u16 *)((unsigned long)&pcicptr->g2pcfgdata | offset) = val;
}
static inline void icd_writel(u32 val, struct tx4927_pcic_reg *pcicptr)
{
	pcicptr->g2pcfgdata = val;
}

static inline struct tx4927_pcic_reg *pci_bus_to_pcicptr(struct pci_bus *bus)
{
	struct pci_controller *channel = bus->sysdata;
	return get_tx4927_pcicptr(channel);
}

static int tx4927_pci_config_read(struct pci_bus *bus, unsigned int devfn,
				  int where, int size, u32 *val)
{
	struct tx4927_pcic_reg *pcicptr = pci_bus_to_pcicptr(bus);
	if ((txboard_pci_option & TXBOARD_PCI_OPT_NOOFFBOARD) &&
	    (!txboard_pci_onboard || !txboard_pci_onboard(bus, devfn))) {
		*val = 0xffffffff;
		return -1;
	}
	if (mkaddr(bus, devfn, where, pcicptr)) {
		*val = 0xffffffff;
		return -1;
	}
	switch (size) {
	case 1:
		*val = icd_readb(where & 3, pcicptr);
		break;
	case 2:
		*val = icd_readw(where & 3, pcicptr);
		break;
	default:
		*val = icd_readl(pcicptr);
	}
	return check_abort(pcicptr);
}

static int tx4927_pci_config_write(struct pci_bus *bus, unsigned int devfn,
				   int where, int size, u32 val)
{
	struct tx4927_pcic_reg *pcicptr = pci_bus_to_pcicptr(bus);
	if (mkaddr(bus, devfn, where, pcicptr))
		return -1;
	switch (size) {
	case 1:
		icd_writeb(val, where & 3, pcicptr);
		break;
	case 2:
		icd_writew(val, where & 3, pcicptr);
		break;
	default:
		icd_writel(val, pcicptr);
	}
	return check_abort(pcicptr);
}

#if !defined(CONFIG_TOSHIBA_TC90411) && !defined(CONFIG_TOSHIBA_TC90412)
static
#endif
struct pci_ops tx4927_pci_ops = {
	.read      = tx4927_pci_config_read,
	.write     = tx4927_pci_config_write,
};

#if !defined(CONFIG_TOSHIBA_TC90411) && !defined(CONFIG_TOSHIBA_TC90412)
static
#endif
struct {
	u8 trdyto;
	u8 retryto;
	u16 gbwc;
} tx4927_pci_opts __initdata = { 0, 0, 0xfff };
char * __init tx4927_pcibios_setup(char *str)
{
	if (!strncmp(str, "trdyto=", 7)) {
		tx4927_pci_opts.trdyto = simple_strtoul(str + 7, NULL, 0);
		return NULL;
	} else if (!strncmp(str, "retryto=", 8)) {
		tx4927_pci_opts.retryto = simple_strtoul(str + 8, NULL, 0);
		return NULL;
	} else if (!strncmp(str, "gbwc=", 5)) {
		tx4927_pci_opts.gbwc = simple_strtoul(str + 5, NULL, 0);
		return NULL;
	} else if (!strcmp(str, "off")) {
		if ((txx9_pcode & 0xfff0) == 0x4920) {
			/* stop PCIC, PCIC1 */
			txx9_set64(&tx4927_ccfgptr->clkctr,
				   TX4927_CLKCTR_PCIRST);
			txx9_set64(&tx4927_ccfgptr->clkctr,
				   TX4927_CLKCTR_PCICKD);
			txx9_clear64(&tx4927_ccfgptr->pcfg,
				     TX4927_PCFG_PCICLKEN_ALL);
		} else if (txx9_pcode == 0x4939) {
			/* stop PCIC, PCIC1, ETH0, ETH1 */
			txx9_set64(&tx4939_ccfgptr->clkctr,
				   TX4939_CLKCTR_PCICRST |
				   TX4939_CLKCTR_PCI1RST |
				   TX4939_CLKCTR_ETH0RST |
				   TX4939_CLKCTR_ETH1RST);
			txx9_set64(&tx4939_ccfgptr->clkctr,
				   TX4939_CLKCTR_PCICCKD |
				   TX4939_CLKCTR_PCI1CKD |
				   TX4939_CLKCTR_ETH0CKD |
				   TX4939_CLKCTR_ETH1CKD);
			txx9_clear64(&tx4939_ccfgptr->pcfg,
				     TX4939_PCFG_PCICLKEN_ALL);
		} else {
			/* stop PCIC, PCIC1, ETH0, ETH1 */
			txx9_set64(&tx4938_ccfgptr->clkctr,
				   TX4938_CLKCTR_PCIRST |
				   TX4938_CLKCTR_PCIC1RST |
				   TX4938_CLKCTR_ETH0RST |
				   TX4938_CLKCTR_ETH1RST);
			txx9_set64(&tx4938_ccfgptr->clkctr,
				   TX4938_CLKCTR_PCICKD |
				   TX4938_CLKCTR_PCIC1CKD |
				   TX4938_CLKCTR_ETH0CKD |
				   TX4938_CLKCTR_ETH1CKD);
			txx9_clear64(&tx4938_ccfgptr->pcfg,
				     TX4938_PCFG_PCICLKEN_ALL);
		}
	}
	return str;
}

void tx4927_pci_fixup(struct pci_dev *dev)
{
}

void __init
tx4927_pcic_setup(struct tx4927_pcic_reg *pcicptr,
		  struct pci_controller *channel,
		  int extarb)
{
	int i;
	unsigned long flags;

	set_tx4927_pcicptr(channel, pcicptr);

	channel->pci_ops = &tx4927_pci_ops;

	local_irq_save(flags);

	/* Disable All Initiator Space */
	pcicptr->pciccfg &= ~(TX4927_PCIC_PCICCFG_G2PMEN(0)|
			      TX4927_PCIC_PCICCFG_G2PMEN(1)|
			      TX4927_PCIC_PCICCFG_G2PMEN(2)|
			      TX4927_PCIC_PCICCFG_G2PIOEN);

	/* GB->PCI mappings */
	pcicptr->g2piomask = (channel->io_resource->end - channel->io_resource->start) >> 4;
	__txx9_write64((channel->io_resource->start + channel->io_map_base - IO_BASE) |
#ifdef __BIG_ENDIAN
		       TX4927_PCIC_G2PIOGBASE_ECHG
#else
		       TX4927_PCIC_G2PIOGBASE_BSDIS
#endif
		       , &pcicptr->g2piogbase);
	__txx9_write64(channel->io_resource->start - channel->io_offset,
		       &pcicptr->g2piopbase);
	for (i = 0; i < 3; i++) {
		pcicptr->g2pmmask[i] = 0;
		__txx9_write64(0, &pcicptr->g2pmgbase[i]);
		__txx9_write64(0, &pcicptr->g2pmpbase[i]);
	}
	if (channel->mem_resource->end) {
		pcicptr->g2pmmask[0] = (channel->mem_resource->end - channel->mem_resource->start) >> 4;
		__txx9_write64(channel->mem_resource->start |
#ifdef __BIG_ENDIAN
			       TX4927_PCIC_G2PMnGBASE_ECHG
#else
			       TX4927_PCIC_G2PMnGBASE_BSDIS
#endif
			       , &pcicptr->g2pmgbase[0]);
			;
		__txx9_write64(channel->mem_resource->start - channel->mem_offset,
			       &pcicptr->g2pmpbase[0]);
	}
	/* PCI->GB mappings (I/O 256B) */
	pcicptr->p2giopbase = 0; /* 256B */
	__txx9_write64(0, &pcicptr->p2giogbase);
	/* PCI->GB mappings (MEM 512MB (64MB on R1.x)) */
	pcicptr->p2gm0plbase = 0;
	pcicptr->p2gm0pubase = 0;
	__txx9_write64(TX4927_PCIC_P2GMnGBASE_TMEMEN |
#ifdef __BIG_ENDIAN
		       TX4927_PCIC_P2GMnGBASE_TECHG
#else
		       TX4927_PCIC_P2GMnGBASE_TBSDIS
#endif
		       , &pcicptr->p2gmgbase[0]);
	/* PCI->GB mappings (MEM 16MB) */
	pcicptr->p2gm1plbase = 0xffffffff;
	pcicptr->p2gm1pubase = 0xffffffff;
	__txx9_write64(0, &pcicptr->p2gmgbase[1]);
	/* PCI->GB mappings (MEM 1MB) */
	pcicptr->p2gm2pbase = 0xffffffff; /* 1MB */
	__txx9_write64(0, &pcicptr->p2gmgbase[2]);

	/* Clear all (including IRBER) except for GBWC */
	pcicptr->pciccfg =
		(tx4927_pci_opts.gbwc << 16) & TX4927_PCIC_PCICCFG_GBWC_MASK;
	/* Enable Initiator Memory Space */
	if (channel->mem_resource->end)
		pcicptr->pciccfg |= TX4927_PCIC_PCICCFG_G2PMEN(0);
	/* Enable Initiator I/O Space */
	if (channel->io_resource->end)
		pcicptr->pciccfg |= TX4927_PCIC_PCICCFG_G2PIOEN;
	/* Enable Initiator Config */
	pcicptr->pciccfg |=
		TX4927_PCIC_PCICCFG_ICAEN |
		TX4927_PCIC_PCICCFG_TCAR;

	/* Do not use MEMMUL, MEMINF: YMFPCI card causes M_ABORT. */
	pcicptr->pcicfg1 = 0;

	pcicptr->g2ptocnt &= ~0xffff;
	pcicptr->g2ptocnt |= tx4927_pci_opts.trdyto & 0xff;
	pcicptr->g2ptocnt |= (tx4927_pci_opts.retryto & 0xff) << 8;

	/* Clear All Local Bus Status */
	pcicptr->pcicstatus = TX4927_PCIC_PCICSTATUS_ALL;
	/* Enable All Local Bus Interrupts */
	pcicptr->pcicmask = TX4927_PCIC_PCICSTATUS_ALL;
	/* Clear All Initiator Status */
	pcicptr->g2pstatus = TX4927_PCIC_G2PSTATUS_ALL;
	/* Enable All Initiator Interrupts */
	pcicptr->g2pmask = TX4927_PCIC_G2PSTATUS_ALL;
	/* Clear All PCI Status Error */
	pcicptr->pcistatus =
		(pcicptr->pcistatus & 0x0000ffff) |
		(TX4927_PCIC_PCISTATUS_ALL << 16);
	/* Enable All PCI Status Error Interrupts */
	pcicptr->pcimask = TX4927_PCIC_PCISTATUS_ALL;

	if (!extarb) {
		/* Reset Bus Arbiter */
		pcicptr->pbacfg = TX4927_PCIC_PBACFG_RPBA;
		pcicptr->pbabm = 0;
		/* Enable Bus Arbiter */
		pcicptr->pbacfg = TX4927_PCIC_PBACFG_PBAEN;
	}

	pcicptr->pcistatus = PCI_COMMAND_MASTER |
		PCI_COMMAND_MEMORY |
		PCI_COMMAND_SERR;
	if (txboard_pci_option & TXBOARD_PCI_OPT_NOPARITY) {
		pcicptr->pcimask &=
			~(PCI_STATUS_DETECTED_PARITY|PCI_STATUS_PARITY);
	} else {
		pcicptr->pcistatus |= PCI_COMMAND_PARITY;
	}
	local_irq_restore(flags);

	printk(KERN_DEBUG "PCI: COMMAND=%04x,PCIMASK=%04x,TRDYTO=%02x,RETRYTO=%02x,GBWC=%03x\n",
	       pcicptr->pcistatus & 0xffff,
	       pcicptr->pcimask & 0xffff,
	       pcicptr->g2ptocnt & 0xff,
	       (pcicptr->g2ptocnt & 0xff00) >> 8,
	       (pcicptr->pciccfg >> 16) & 0xfff);
}

int __init
tx4927_report_pciclk(void)
{
	int pciclk = 0;
	printk("%s PCIC --%s PCICLK:",
	       txx9_pcode_str,
	       (txx9_read64(&tx4927_ccfgptr->ccfg) & TX4927_CCFG_PCI66) ? " PCI66" : "");
	if (txx9_read64(&tx4927_ccfgptr->pcfg) & TX4927_PCFG_PCICLKEN_ALL) {
		if ((txx9_pcode & 0xfff0) == 0x4920) {
			switch ((unsigned long)txx9_read64(&tx4927_ccfgptr->ccfg) & TX4927_CCFG_PCIDIVMODE_MASK) {
			case TX4927_CCFG_PCIDIVMODE_2_5:
				pciclk = txx9_cpu_clock * 2 / 5; break;
			case TX4927_CCFG_PCIDIVMODE_3:
				pciclk = txx9_cpu_clock / 3; break;
			case TX4927_CCFG_PCIDIVMODE_5:
				pciclk = txx9_cpu_clock / 5; break;
			case TX4927_CCFG_PCIDIVMODE_6:
				pciclk = txx9_cpu_clock / 6; break;
			}
		} else if (txx9_pcode == 0x4939) {
			pciclk = txx9_master_clock * 20 / 6;
			if (!(txx9_read64(&tx4939_ccfgptr->ccfg) & TX4939_CCFG_PCI66))
				pciclk /= 2;
		} else {
			switch ((unsigned long)txx9_read64(&tx4938_ccfgptr->ccfg) & TX4938_CCFG_PCIDIVMODE_MASK) {
			case TX4938_CCFG_PCIDIVMODE_4:
				pciclk = txx9_cpu_clock / 4; break;
			case TX4938_CCFG_PCIDIVMODE_4_5:
				pciclk = txx9_cpu_clock * 2 / 9; break;
			case TX4938_CCFG_PCIDIVMODE_5:
				pciclk = txx9_cpu_clock / 5; break;
			case TX4938_CCFG_PCIDIVMODE_5_5:
				pciclk = txx9_cpu_clock * 2 / 11; break;
			case TX4938_CCFG_PCIDIVMODE_8:
				pciclk = txx9_cpu_clock / 8; break;
			case TX4938_CCFG_PCIDIVMODE_9:
				pciclk = txx9_cpu_clock / 9; break;
			case TX4938_CCFG_PCIDIVMODE_10:
				pciclk = txx9_cpu_clock / 10; break;
			case TX4938_CCFG_PCIDIVMODE_11:
				pciclk = txx9_cpu_clock / 11; break;
			}
		}
		printk("Internal(%u.%uMHz)",
		       (pciclk + 50000) / 1000000,
		       ((pciclk + 50000) / 100000) % 10);
	} else {
		printk("External");
		pciclk = -1;
	}
	printk("\n");
	return pciclk;
}

void __init
tx4938_report_pci1clk(void)
{
	__u64 ccfg = txx9_read64(&tx4938_ccfgptr->ccfg);
	unsigned int pciclk =
		txx9_gbus_clock / ((ccfg & TX4938_CCFG_PCI1DMD) ? 4 : 2);
	printk("%s PCIC1 -- ", txx9_pcode_str);
	if (ccfg & TX4938_CCFG_PCI1_66)
		printk("PCI66 ");
	printk("PCICLK:%u.%uMHz\n",
	       (pciclk + 50000) / 1000000,
	       ((pciclk + 50000) / 100000) % 10);
}

void __init
tx4939_report_pci1clk(void)
{
	unsigned int pciclk = txx9_master_clock * 20 / 6;
	printk("%s PCIC1 -- ", txx9_pcode_str);
	printk("PCICLK:%u.%uMHz\n",
	       (pciclk + 50000) / 1000000,
	       ((pciclk + 50000) / 100000) % 10);
}

int __init
tx4927_pciclk66_setup(void)
{
	int pciclk;

	/* Assert M66EN */
	tx4927_ccfg_set(TX4927_CCFG_PCI66);
	/* Double PCICLK (if possible) */
	if (txx9_read64(&tx4927_ccfgptr->pcfg) & TX4927_PCFG_PCICLKEN_ALL) {
		unsigned int pcidivmode = 0;
		if ((txx9_pcode & 0xfff0) == 0x4920) {
			pcidivmode = (unsigned long)txx9_read64(&tx4927_ccfgptr->ccfg) & TX4927_CCFG_PCIDIVMODE_MASK;
			switch (pcidivmode) {
			case TX4927_CCFG_PCIDIVMODE_5:
			case TX4927_CCFG_PCIDIVMODE_2_5:
				pcidivmode = TX4927_CCFG_PCIDIVMODE_2_5;
				pciclk = txx9_cpu_clock * 2 / 5;
				break;
			case TX4927_CCFG_PCIDIVMODE_6:
			case TX4927_CCFG_PCIDIVMODE_3:
			default:
				pcidivmode = TX4927_CCFG_PCIDIVMODE_3;
				pciclk = txx9_cpu_clock / 3;
			}
			tx4927_ccfg_change(TX4927_CCFG_PCIDIVMODE_MASK,
					   pcidivmode);
		} else {
			pcidivmode = (unsigned long)txx9_read64(&tx4938_ccfgptr->ccfg) & TX4938_CCFG_PCIDIVMODE_MASK;
			switch (pcidivmode) {
			case TX4938_CCFG_PCIDIVMODE_8:
			case TX4938_CCFG_PCIDIVMODE_4:
				pcidivmode = TX4938_CCFG_PCIDIVMODE_4;
				pciclk = txx9_cpu_clock / 4;
				break;
			case TX4938_CCFG_PCIDIVMODE_9:
			case TX4938_CCFG_PCIDIVMODE_4_5:
				pcidivmode = TX4938_CCFG_PCIDIVMODE_4_5;
				pciclk = txx9_cpu_clock * 2 / 9;
				break;
			case TX4938_CCFG_PCIDIVMODE_10:
			case TX4938_CCFG_PCIDIVMODE_5:
				pcidivmode = TX4938_CCFG_PCIDIVMODE_5;
				pciclk = txx9_cpu_clock / 5;
				break;
			case TX4938_CCFG_PCIDIVMODE_11:
			case TX4938_CCFG_PCIDIVMODE_5_5:
			default:
				pcidivmode = TX4938_CCFG_PCIDIVMODE_5_5;
				pciclk = txx9_cpu_clock * 2 / 11;
				break;
			}
			tx4927_ccfg_change(TX4938_CCFG_PCIDIVMODE_MASK,
					   pcidivmode);
		}
		printk(KERN_DEBUG "PCICLK: ccfg:%08lx\n",
		       (unsigned long)txx9_read64(&tx4927_ccfgptr->ccfg));
	} else {
		pciclk = -1;
	}
	return pciclk;
}

int tx4938_pcic1_map_irq(struct pci_dev *dev, u8 slot)
{
	if (pci_bus_to_pcicptr(dev->bus) == tx4938_pcic1ptr) {
		switch (slot) {
		case TX4927_PCIC_IDSEL_AD_TO_SLOT(31):
			if (txx9_read64(&tx4938_ccfgptr->pcfg) & TX4938_PCFG_ETH0_SEL)
				return txx9_irq_to_irq(TX4938_IR_ETH0);
			break;
		case TX4927_PCIC_IDSEL_AD_TO_SLOT(30):
			if (txx9_read64(&tx4938_ccfgptr->pcfg) & TX4938_PCFG_ETH1_SEL)
				return txx9_irq_to_irq(TX4938_IR_ETH1);
			break;
		}
		return 0;
	}
	return -1;
}

int tx4939_pcic1_map_irq(struct pci_dev *dev, u8 slot)
{
	if (pci_bus_to_pcicptr(dev->bus) == tx4939_pcic1ptr) {
		switch (slot) {
		case TX4927_PCIC_IDSEL_AD_TO_SLOT(31):
			if (txx9_read64(&tx4939_ccfgptr->pcfg) & TX4939_PCFG_ET0MODE)
				return tx4939_irq_to_irq(TX4939_IR_ETH(0));
			break;
		case TX4927_PCIC_IDSEL_AD_TO_SLOT(30):
			if (txx9_read64(&tx4939_ccfgptr->pcfg) & TX4939_PCFG_ET1MODE)
				return tx4939_irq_to_irq(TX4939_IR_ETH(1));
			break;
		}
		return 0;
	}
	return -1;
}

static ssize_t
show_regdump(struct sys_device *dev, char *buf)
{
	int i;
	char *p = buf;
	volatile __u32 *preg = (volatile __u32 *)pcicptrs[dev->id].pcicptr;

	for (i = 0; i < sizeof(struct tx4927_pcic_reg); i += 4) {
		if (i % 32 == 0) {
			if (i != 0)
				p += sprintf(p, "\n");
			p += sprintf(p, "%04x:", i);
		}
		/* skip registers with side-effects */
		if (i == offsetof(struct tx4927_pcic_reg, g2pintack)
		    || i == offsetof(struct tx4927_pcic_reg, g2pspc)
		    || i == offsetof(struct tx4927_pcic_reg, g2pcfgadrs)
		    || i == offsetof(struct tx4927_pcic_reg, g2pcfgdata)
			) {
			p += sprintf(p, " XXXXXXXX");
			preg++;
			continue;
		}
		p += sprintf(p, " %08x", *preg++);
	}
	p += sprintf(p, "\n");
	return p - buf;
}

static SYSDEV_ATTR(regdump, 0400, show_regdump, NULL);

#ifdef CONFIG_PM
struct tx4927_pcic_state {
	u32 pcistatus;
	u32 pcicfg1;
	u32 p2gm0plbase;
	u32 p2gm0pubase;
	u32 p2gm1plbase;
	u32 p2gm1pubase;
	u32 p2gm2pbase;
	u32 p2giopbase;
	u32 g2ptocnt;
	u32 g2pmask;
	u32 pcimask;
	u32 pbacfg;
	u64 g2pmgbase[3];
	u64 g2piogbase;
	u32 g2pmmask[3];
	u32 g2piomask;
	u64 g2pmpbase[3];
	u64 g2piopbase;
	u32 pciccfg;
	u32 pcicmask;
	u64 p2gmgbase[3];
	u64 p2giogbase;
};
#endif

struct tx4927pcic_sysdev {
	struct sys_device dev;
#ifdef CONFIG_PM
	struct tx4927_pcic_state state;
#endif
};

#ifdef CONFIG_PM
static void tx4927_pcic_save_state(struct tx4927_pcic_state *state,
				   struct tx4927_pcic_reg *pcicptr)
{
	int i;

	state->p2gm0plbase = pcicptr->p2gm0plbase;
	state->p2gm0pubase = pcicptr->p2gm0pubase;
	state->p2gm1plbase = pcicptr->p2gm1plbase;
	state->p2gm1pubase = pcicptr->p2gm1pubase;
	state->p2gm2pbase = pcicptr->p2gm2pbase;
	state->p2giopbase = pcicptr->p2giopbase;
	state->pbacfg = pcicptr->pbacfg;

	state->pcicfg1 = pcicptr->pcicfg1;
	state->g2ptocnt = pcicptr->g2ptocnt;
	state->pcicmask = pcicptr->pcicmask;
	state->g2pmask = pcicptr->g2pmask;
	state->pcistatus = pcicptr->pcistatus;
	state->pcimask = pcicptr->pcimask;

	state->g2piomask = pcicptr->g2piomask;
	state->pciccfg = pcicptr->pciccfg;

	state->g2piogbase = pcicptr->g2piogbase;
	state->g2piopbase = pcicptr->g2piopbase;
	state->p2giogbase = pcicptr->p2giogbase;

	for (i = 0; i < 3; i++) {
		state->g2pmmask[i] = pcicptr->g2pmmask[i];
		state->g2pmgbase[i] = pcicptr->g2pmgbase[i];
		state->g2pmpbase[i] = pcicptr->g2pmpbase[i];
		state->p2gmgbase[i] = pcicptr->p2gmgbase[i];
	}
}

static void tx4927_pcic_restore_state(struct tx4927_pcic_state *state,
				      struct tx4927_pcic_reg *pcicptr)
{
	int i;

	pcicptr->p2gm0plbase = state->p2gm0plbase;
	pcicptr->p2gm0pubase = state->p2gm0pubase;
	pcicptr->p2gm1plbase = state->p2gm1plbase;
	pcicptr->p2gm1pubase = state->p2gm1pubase;
	pcicptr->p2gm2pbase = state->p2gm2pbase;
	pcicptr->p2giopbase = state->p2giopbase;
	pcicptr->g2piomask = state->g2piomask;
	pcicptr->g2piogbase = state->g2piogbase;
	pcicptr->g2piopbase = state->g2piopbase;
	pcicptr->p2giogbase = state->p2giogbase;

	for (i = 0; i < 3; i++) {
		pcicptr->g2pmmask[i] = state->g2pmmask[i];
		pcicptr->g2pmgbase[i] = state->g2pmgbase[i];
		pcicptr->g2pmpbase[i] = state->g2pmpbase[i];
		pcicptr->p2gmgbase[i] = state->p2gmgbase[i];
	}

	pcicptr->pciccfg = state->pciccfg;
	pcicptr->pbacfg = state->pbacfg;

	pcicptr->pcicfg1 = state->pcicfg1;
	pcicptr->g2ptocnt = state->g2ptocnt;
	pcicptr->pcicmask = state->pcicmask;
	pcicptr->g2pmask = state->g2pmask;
	pcicptr->pcistatus = state->pcistatus;
	pcicptr->pcimask = state->pcimask;

	pcicptr->pcicstatus = TX4927_PCIC_PCICSTATUS_ALL;
	pcicptr->g2pstatus = TX4927_PCIC_G2PSTATUS_ALL;
	pcicptr->pcistatus =
		(pcicptr->pcistatus & 0x0000ffff) |
		(TX4927_PCIC_PCISTATUS_ALL << 16);
}

static int tx4927pcic_suspend(struct sys_device *dev, pm_message_t state)
{
	struct tx4927pcic_sysdev *pcic_dev =
		container_of(dev, struct tx4927pcic_sysdev, dev);
	tx4927_pcic_save_state(&pcic_dev->state, pcicptrs[dev->id].pcicptr);
	return 0;
}
static int tx4927pcic_resume(struct sys_device *dev)
{
	struct tx4927pcic_sysdev *pcic_dev =
		container_of(dev, struct tx4927pcic_sysdev, dev);
	tx4927_pcic_restore_state(&pcic_dev->state, pcicptrs[dev->id].pcicptr);
	return 0;
}
#else
#define tx4927pcic_suspend NULL
#define tx4927pcic_resume NULL
#endif

static struct sysdev_class tx4927pcic_sysdev_class = {
	.suspend = tx4927pcic_suspend,
	.resume = tx4927pcic_resume,
};

static int __init tx4927pcic_sysdev_register(int id)
{
	int error;
	struct tx4927pcic_sysdev *pcic_dev;

	pcic_dev = kzalloc(sizeof(*pcic_dev), GFP_KERNEL);
	if (!pcic_dev)
		return -ENOMEM;
	pcic_dev->dev.id = id;
	pcic_dev->dev.cls = &tx4927pcic_sysdev_class;
	error = sysdev_register(&pcic_dev->dev);
	if (error) {
		kfree(pcic_dev);
		return error;
	}
	error = sysdev_create_file(&pcic_dev->dev, &attr_regdump);
	if (error) {
		sysdev_unregister(&pcic_dev->dev);
		kfree(pcic_dev);
		return error;
	}
	return error;
}

static int __init tx4927pcic_init_sysfs(void)
{
	int error, i;

	if (!pcicptrs[0].pcicptr)
		return -ENODEV;
	sprintf(tx4927pcic_sysdev_class.kset.kobj.name,
		"%spcic", txx9_pcode_str);
	error = sysdev_class_register(&tx4927pcic_sysdev_class);
	if (error)
		return error;

	for (i = 0; i < ARRAY_SIZE(pcicptrs); i++) {
		if (pcicptrs[i].pcicptr) {
			error = tx4927pcic_sysdev_register(i);
			if (error)
				break;
		}
	}
	return error;
}
device_initcall(tx4927pcic_init_sysfs);
