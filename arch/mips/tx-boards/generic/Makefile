#
# Makefile for the TX-RISC board generic parts of the kernel
#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#
# (C) Copyright TOSHIBA CORPORATION 2004-2007
# All Rights Reserved.
#

obj-$(CONFIG_SPI)	+= spi_txx9.o
lib-$(CONFIG_SPI)	+= spi_eeprom.o
obj-y	+= setup.o pmon.o pmon-low.o prom.o irq.o
lib-y	+= dma.o 7segled.o
lib-$(CONFIG_KGDB)	+= gdb_hook.o
ifdef CONFIG_PCI
obj-y	+= pci.o
lib-$(CONFIG_CPU_TX49XX)	+= pci_tx4927.o
endif
obj-$(CONFIG_PM)	+= pm.o
lib-y	+= setup_tx4927.o \
	setup_tx4939.o \
	irq_txx9.o irq_txx9_64.o time_txx9.o \
	irq_tx4927.o irq_tx4939.o \
	dma_tx4927.o \
	dump_tx4927.o
obj-$(CONFIG_TOSHIBA_TC90490)   += fixup-tc90490.o
