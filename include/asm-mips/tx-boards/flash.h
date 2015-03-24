/*
 *  linux/include/asm-mips/tx-boards/flash.h
 *  Based on include/asm-arm/mach/flash.h by Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * (C) Copyright TOSHIBA CORPORATION 2005-2006
 * All Rights Reserved.
 */
#ifndef __ASM_TX_BOARDS_FLASH_H
#define __ASM_TX_BOARDS_FLASH_H

/*
 * map_name:	the map probe function name
 * width:	width of mapped device
 * init:	method called at driver/device initialisation
 * exit:	method called at driver/device removal
 * set_vpp:	method called to enable or disable VPP
 * fixup_ofs:	method called to fixup offset
 * copy_from:	method called to copy from the device
 * parts:	optional array of mtd_partitions for static partitioning
 * nr_parts:	number of mtd_partitions for static partitoning
 */
struct flash_platform_data {
	const char	*map_name;
	unsigned int	width;
	int		(*init)(void);
	void		(*exit)(void);
	void		(*set_vpp)(int on);
	unsigned long	(*fixup_ofs)(unsigned long ofs);
	void		(*copy_from)(void *to, void *from, ssize_t len);
	struct mtd_partition *parts;
	unsigned int	nr_parts;
};

extern struct platform_device *
register_txboard_flash(int no, unsigned long addr, unsigned long size,
		       int bankwidth, struct resource *parent);

#endif /* __ASM_TX_BOARDS_FLASH_H */
