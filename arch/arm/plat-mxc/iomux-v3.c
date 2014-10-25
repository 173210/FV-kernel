/*
 * Copyright 2004-2011 Freescale Semiconductor, Inc.
 * Copyright (C) 2008 by Sascha Hauer <kernel@pengutronix.de>
 * Copyright (C) 2009 by Jan Weitzel Phytec Messtechnik GmbH,
 *                       <armlinux@phytec.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <asm/mach/map.h>
#include <mach/iomux-mx53.h>
static void __iomem *base;

/*
 * Read a single pad in the iomuxer
 */
int mxc_iomux_v3_get_pad(iomux_v3_cfg_t *pad)
{
	u32 mux_ctrl_ofs = (*pad & MUX_CTRL_OFS_MASK) >> MUX_CTRL_OFS_SHIFT;
	u32 pad_ctrl_ofs = (*pad & MUX_PAD_CTRL_OFS_MASK) >> MUX_PAD_CTRL_OFS_SHIFT;
	u32 sel_input_ofs = (*pad & MUX_SEL_INPUT_OFS_MASK) >> MUX_SEL_INPUT_OFS_SHIFT;
	u32 mux_mode = 0;
	u32 sel_input = 0;
	u32 pad_ctrl = 0;
	iomux_v3_cfg_t pad_info = 0;

	mux_mode = __raw_readl(base + mux_ctrl_ofs) & 0xFF;
	pad_ctrl = __raw_readl(base + pad_ctrl_ofs) & 0x3FFFF;
	sel_input = __raw_readl(base + sel_input_ofs) & 0x7;
	
	pad_info = (((iomux_v3_cfg_t)mux_mode << MUX_MODE_SHIFT) | \
		((iomux_v3_cfg_t)pad_ctrl << MUX_PAD_CTRL_SHIFT) | \
		((iomux_v3_cfg_t)sel_input << MUX_SEL_INPUT_SHIFT));
	
	*pad &= ~(MUX_MODE_MASK | MUX_PAD_CTRL_MASK | MUX_SEL_INPUT_MASK);
	*pad |= pad_info;

	return 0;
}
EXPORT_SYMBOL(mxc_iomux_v3_get_pad);
/*
 * Read multiple pads in the iomuxer
 */
int mxc_iomux_v3_scan_pads_confs(iomux_conf *pad_list, unsigned count)
{
	iomux_v3_cfg_t temp;
	iomux_conf *p = pad_list;
	int i;
	printk("==================MXC IOMUX Runtime Configuration ================\n");
        for (i = 0; i < count; i++) {
		temp = p->pad;
                mxc_iomux_v3_get_pad(&temp);
		if(temp == p->pad)
			printk("%s \n",p->pad_name);	
		else if((temp&(~MUX_PAD_CTRL_MASK)) == (p->pad & (~MUX_PAD_CTRL_MASK))){
			if((((iomux_v3_cfg_t)0x1ff<<MUX_PAD_CTRL_SHIFT)&p->pad) ==(temp&((iomux_v3_cfg_t)0x1ff<<MUX_PAD_CTRL_SHIFT)))
				printk("%s \n",p->pad_name);	
			else
				printk("%s  WARNING: MUX_PAD_CTRL incorrect: Set value = 0x%llx; Read value = 0x%llx\n",p->pad_name,
				(p->pad &MUX_PAD_CTRL_MASK)>>MUX_PAD_CTRL_SHIFT,(temp&MUX_PAD_CTRL_MASK)>>MUX_PAD_CTRL_SHIFT);
		}
		else if((temp&(~MUX_SEL_INPUT_MASK)) == (p->pad &(~MUX_SEL_INPUT_MASK))){
				printk("%s  WARNING: MUX_SEL_INPUT incorrect: origial = 0x%llx; reading = 0x%llx\n",p->pad_name,
				(p->pad &MUX_SEL_INPUT_MASK)>>MUX_SEL_INPUT_SHIFT,(temp&MUX_SEL_INPUT_MASK)>>MUX_SEL_INPUT_SHIFT);

		}
		if(p->pad ==MX53_PAD_CSI0_DAT10__UART1_TXD_MUX)
			printk("MX53_PAD_CSI0_DAT10__UART1_TXD_MUX original = 0x%llx and reading = 0x%llx\n",p->pad,temp);
                if(p->pad ==MX53_PAD_EIM_D24__UART3_TXD_MUX )
                        printk("MX53_PAD_EIM_D24__UART3_TXD_MUX original = 0x%llx and reading = 0x%llx\n",p->pad,temp);
/*
		if(p->pad ==MX53_PAD_CSI0_DAT5__GPIO5_23)
			printk("MX53_PAD_CSI0_DAT5__GPIO5_23 original = 0x%llx and reading = 0x%llx\n",p->pad,temp);
                if(p->pad ==MX53_PAD_CSI0_DAT6__GPIO5_24)
                        printk("MX53_PAD_CSI0_DAT6__GPIO5_24 original = 0x%llx and reading = 0x%llx\n",p->pad,temp);
		if(p->pad ==MX53_PAD_CSI0_DAT7__GPIO5_25)
			printk("MX53_PAD_CSI0_DAT7__GPIO5_25 original = 0x%llx and reading = 0x%llx\n",p->pad,temp);
                if(p->pad ==MX53_PAD_KEY_ROW1__GPIO4_9)
                        printk("MX53_PAD_KEY_ROW1__GPIO4_9 original = 0x%llx and reading = 0x%llx\n",p->pad,temp);
*/
		p++;
        }
	printk("==================MXC IOMUX scanning end================\n");
        return 0;
}
EXPORT_SYMBOL(mxc_iomux_v3_scan_pads_confs);

			

/*
 * configures a single pad in the iomuxer
 */
int mxc_iomux_v3_setup_pad(iomux_v3_cfg_t pad)
{
	u32 mux_ctrl_ofs = (pad & MUX_CTRL_OFS_MASK) >> MUX_CTRL_OFS_SHIFT;
	u32 mux_mode = (pad & MUX_MODE_MASK) >> MUX_MODE_SHIFT;
	u32 sel_input_ofs = (pad & MUX_SEL_INPUT_OFS_MASK) >> MUX_SEL_INPUT_OFS_SHIFT;
	u32 sel_input = (pad & MUX_SEL_INPUT_MASK) >> MUX_SEL_INPUT_SHIFT;
	u32 pad_ctrl_ofs = (pad & MUX_PAD_CTRL_OFS_MASK) >> MUX_PAD_CTRL_OFS_SHIFT;
	u32 pad_ctrl = (pad & MUX_PAD_CTRL_MASK) >> MUX_PAD_CTRL_SHIFT;

	if (mux_ctrl_ofs)
		__raw_writel(mux_mode, base + mux_ctrl_ofs);

	if (sel_input_ofs)
		__raw_writel(sel_input, base + sel_input_ofs);

	if (!(pad_ctrl & NO_PAD_CTRL) && pad_ctrl_ofs)
		__raw_writel(pad_ctrl, base + pad_ctrl_ofs);

	return 0;
}
EXPORT_SYMBOL(mxc_iomux_v3_setup_pad);

int mxc_iomux_v3_setup_multiple_pads(iomux_v3_cfg_t *pad_list, unsigned count)
{
	iomux_v3_cfg_t *p = pad_list;
	int i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = mxc_iomux_v3_setup_pad(*p);
		if (ret)
			return ret;
		p++;
	}
	return 0;
}
EXPORT_SYMBOL(mxc_iomux_v3_setup_multiple_pads);

void mxc_iomux_set_gpr_register(int group, int start_bit, int num_bits, int value)
{
	int i = 0;
	u32 reg;
	reg = __raw_readl(base + group * 4);
	while (num_bits) {
		reg &= ~(1<<(start_bit + i));
		i++;
		num_bits--;
	}
	reg |= (value << start_bit);
	__raw_writel(reg, base + group * 4);
}
EXPORT_SYMBOL(mxc_iomux_set_gpr_register);

void mxc_iomux_v3_init(void __iomem *iomux_v3_base)
{
	base = iomux_v3_base;
}
