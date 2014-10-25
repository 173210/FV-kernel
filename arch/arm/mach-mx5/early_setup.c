/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/init.h>
#include <linux/string.h>
#include <linux/kernel.h>


int pcb_version = -1;
EXPORT_SYMBOL_GPL(pcb_version);
static int __init pcb_setup(char *options)
{
	/* unkown pcb version, we think it's the newest */
	if(0 == strcmp(options, "QA0"))
		pcb_version = 0;
	else if(0 == strcmp(options, "QA1"))
		pcb_version = 1;
	else if(0 == strcmp(options, "QA2"))
		pcb_version = 2;
	else if(0 == strcmp(options, "QA21"))
		pcb_version = 3;
	else if(0 == strcmp(options, "QA25"))
		pcb_version = 4;
	else if(0 == strcmp(options, "PP"))
		pcb_version = 5;
	else 
		pcb_version = 5;
}
__setup("hw=", pcb_setup);
