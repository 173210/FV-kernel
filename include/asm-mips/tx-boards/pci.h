/*
 * linux/include/asm-mips/tx-boards/pci.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2006
 * All Rights Reserved.
 *
 */
#ifndef __ASM_TX_BOARDS_PCI_H
#define __ASM_TX_BOARDS_PCI_H

#include <linux/init.h>
#include <linux/pci.h>

extern struct pci_controller txboard_primary_pcic;
extern struct pci_controller *
txboard_alloc_pci_controller(struct pci_controller *pcic,
			     unsigned long mem_base, unsigned long mem_size,
			     unsigned long io_base, unsigned long io_size);

extern int (*txboard_pci_map_irq)(struct pci_dev *dev, u8 slot, u8 pin);
extern void (*txboard_pci_fixup)(struct pci_dev *dev);
extern char * (*txboard_pcibios_setup)(char *str) __initdata;
extern int (*txboard_pci_onboard)(struct pci_bus *bus, unsigned int devfn);
extern int (*txboard_pci_scan_hook)(struct pci_dev *dev);
extern int (*txboard_pci_plat_dev_init)(struct pci_dev *dev);
extern int txboard_pci66_check(struct pci_controller *hose, int top_bus, int current_bus);
extern int txboard_pci_mem_high __initdata;

extern int txboard_pci_option;
#define TXBOARD_PCI_OPT_NOPARITY	0x0001
#define TXBOARD_PCI_OPT_PICMG	0x0002
#define TXBOARD_PCI_OPT_OFF	0x0004
#define TXBOARD_PCI_OPT_CLK_33	0x0008
#define TXBOARD_PCI_OPT_CLK_66	0x0010
#define TXBOARD_PCI_OPT_CLK_MASK	(TXBOARD_PCI_OPT_CLK_33 | TXBOARD_PCI_OPT_CLK_66)
#define TXBOARD_PCI_OPT_CLK_AUTO	TXBOARD_PCI_OPT_CLK_MASK
#define TXBOARD_PCI_OPT_CLK_OFF	0
#define TXBOARD_PCI_OPT_NOOFFBOARD	0x0020
#define TXBOARD_PCI_OPT_NOREMOTE	0x0040

enum txboard_pci_err_action {
	TXBOARD_PCI_ERR_REPORT,
	TXBOARD_PCI_ERR_IGNORE,
	TXBOARD_PCI_ERR_PANIC,
};
extern enum txboard_pci_err_action txboard_pci_err_action;

struct tx4927_pcic_reg;
extern void
tx4927_pcic_setup(struct tx4927_pcic_reg *pcicptr,
		  struct pci_controller *channel, int extarb) __init;
extern struct tx4927_pcic_reg *
get_tx4927_pcicptr(struct pci_controller *channel);
extern int
tx4927_report_pciclk(void) __init;
extern void
tx4938_report_pci1clk(void) __init;
extern void
tx4939_report_pci1clk(void) __init;
extern int
tx4927_pciclk66_setup(void) __init;
extern char *tx4927_pcibios_setup(char *str) __init;
extern void tx4927_pci_fixup(struct pci_dev *dev);
extern int tx4938_pcic1_map_irq(struct pci_dev *dev, u8 slot);
extern int tx4939_pcic1_map_irq(struct pci_dev *dev, u8 slot);

#endif /* __ASM_TX_BOARDS_PCI_H */
