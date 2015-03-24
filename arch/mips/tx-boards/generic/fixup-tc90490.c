/*
 * linux/arch/mips/tx-boards/generic/fixup-tc90490.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2008
 * All Rights Reserved.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/pci.h>

static void __devinit tc90490_fixup(struct pci_dev *dev)
{
	dev->class &= 0xff;
	dev->class |= PCI_CLASS_BRIDGE_OTHER << 8;
}

DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_TOSHIBA_2, PCI_DEVICE_ID_TOSHIBA_TC90490, tc90490_fixup);

