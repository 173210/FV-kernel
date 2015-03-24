/*
 * linux/arch/mips/tx-boards/generic/irq_tx4939.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2005-2007
 * All Rights Reserved.
 *
 */
/*
 * TX4939 defines 64 IRQs.
 * Similer to irq_txx9.c but different register layouts.
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/module.h>
#include <asm/irq_cpu.h>
#include <asm/tx-boards/generic.h>
#include <asm/tx-boards/irq.h>
#include <asm/tx-boards/tx4939.h>

#define irc_dlevel	0
#define irc_elevel	1
/* initially, all source have middle priprity (4) */
static unsigned char irc_level[TX4939_NUM_IR] = {[ 0 ... TX4939_NUM_IR - 1] = 4};

static unsigned int tx4939_irq_base;
static const unsigned int tx4939_irq_num = TX4939_NUM_IR;

static void tx4939_irq_unmask(unsigned int irq)
{
	unsigned int irq_nr = irq - tx4939_irq_base;
	volatile __u32 *lvlp;
	int ofs;
	if (irq_nr < 32) {
		irq_nr--;
		lvlp = &tx4939_ircptr->lvl[(irq_nr % 16 ) / 2].r;
	} else {
		irq_nr -= 32;
		lvlp = &tx4939_ircptr->lvl[8 + (irq_nr % 16 ) / 2].r;
	}
	ofs = (irq_nr & 16) + (irq_nr & 1) * 8;
	*lvlp = (*lvlp & ~(0xff << ofs)) | (irc_level[irq_nr] << ofs);
}

static inline void tx4939_irq_mask(unsigned int irq)
{
	unsigned int irq_nr = irq - tx4939_irq_base;
	volatile __u32 *lvlp;
	int ofs;
	if (irq_nr < 32) {
		irq_nr--;
		lvlp = &tx4939_ircptr->lvl[(irq_nr % 16 ) / 2].r;
	} else {
		irq_nr -= 32;
		lvlp = &tx4939_ircptr->lvl[8 + (irq_nr % 16 ) / 2].r;
	}
	ofs = (irq_nr & 16) + (irq_nr & 1) * 8;
	*lvlp = (*lvlp & ~(0xff << ofs)) | (irc_dlevel << ofs);
	iob();
}

static void tx4939_irq_mask_ack(unsigned int irq)
{
	unsigned int irq_nr = irq - tx4939_irq_base;
	tx4939_irq_mask(irq);
	if ((irq_nr >= TX4939_IR_INT(0) &&
	     irq_nr < TX4939_IR_INT(0) + TX4939_NUM_IR_INT) ||
	    (irq_nr >= TX4939_IR_INTA &&
	     irq_nr < TX4939_IR_INTA + 4)) {
		__u32 cr;
		irq_nr--;
		/* clear edge detection */
		cr = tx4939_ircptr->dm[(irq_nr & 8) >> 3].r;
		cr = (cr >> ((((irq_nr & 16) >> 1) | (irq_nr & (8 - 1))) * 2)) & 3;
		if (TXx9_IRCR_EDGE(cr))
			tx4939_ircptr->edc.r =
				(TXx9_IRSCR_EIClrE | (irq_nr & 0xf)) << (irq_nr & 0x10);
	}
}

static int tx4939_irq_set_type(unsigned int irq, unsigned int flow_type)
{
	unsigned int irq_nr = irq - tx4939_irq_base;
	__u32 cr;
	volatile __u32 *crp;
	int ofs;
	int mode;

	if (flow_type & IRQF_TRIGGER_PROBE)
		return 0;
	switch (flow_type & IRQF_TRIGGER_MASK) {
	case IRQF_TRIGGER_RISING:	mode = TXx9_IRCR_UP;	break;
	case IRQF_TRIGGER_FALLING:	mode = TXx9_IRCR_DOWN;	break;
	case IRQF_TRIGGER_HIGH:	mode = TXx9_IRCR_HIGH;	break;
	case IRQF_TRIGGER_LOW:	mode = TXx9_IRCR_LOW;	break;
	default:
		return -EINVAL;
	}
	if (irq_nr < 32) {
		irq_nr--;
		crp = &tx4939_ircptr->dm[(irq_nr & 8) >> 3].r;
	} else {
		irq_nr -= 32;
		crp = &tx4939_ircptr->dm2[((irq_nr & 8) >> 3)].r;
	}
	ofs = (((irq_nr & 16) >> 1) | (irq_nr & (8 - 1))) * 2;
	cr = *crp;
	cr &= ~(0x3 << ofs);
	cr |= (mode & 0x3) << ofs;
	*crp = cr;
	return 0;
}

static struct irq_chip tx4939_irq_chip = {
	.name		= "TX4939",
	.ack		= tx4939_irq_mask_ack,
	.mask		= tx4939_irq_mask,
	.mask_ack	= tx4939_irq_mask_ack,
	.unmask		= tx4939_irq_unmask,
	.set_type	= tx4939_irq_set_type,
};

/* convert txx9 irc number to global irq number */
int tx4939_irq_to_irq(int irc_irq)
{
	if ((unsigned int)irc_irq >= tx4939_irq_num)
		return -1;

	return irc_irq + tx4939_irq_base;
}

EXPORT_SYMBOL(tx4939_irq_to_irq);

int tx4939_irq_set_pri(int irc_irq, int new_pri)
{
	int old_pri;
	if ((unsigned int)irc_irq >= tx4939_irq_num)
		return 0;
	old_pri = irc_level[irc_irq];
	irc_level[irc_irq] = new_pri;
	return old_pri;
}

void __init
tx4939_irq_init(int irq_base, int mips_irq_base)
{
	int i;

	mips_cpu_irq_init(mips_irq_base);

	tx4939_irq_base = irq_base;

	/* irq_base + 0 is not used */
	for (i = irq_base + 1; i < irq_base + tx4939_irq_num; i++)
		set_irq_chip_and_handler(i, &tx4939_irq_chip,
					 handle_level_irq);

	/* mask all IRC interrupts */
	tx4939_ircptr->msk.r = 0;
	for (i = 0; i < 16; i++)
		tx4939_ircptr->lvl[i].r = 0;
	/* setup IRC interrupt mode (Low Active) */
	for (i = 0; i < 2; i++)
		tx4939_ircptr->dm[i].r = 0;
	for (i = 0; i < 2; i++)
		tx4939_ircptr->dm2[i].r = 0;
	/* enable interrupt control */
	tx4939_ircptr->den.r = TXx9_IRCER_ICE;
	tx4939_ircptr->msk.r = irc_elevel;

	/* raise priority for errors, timers, sio */
	tx4939_irq_set_pri(TX4939_IR_WTOERR, 7);
	tx4939_irq_set_pri(TX4939_IR_PCIERR, 7);
	tx4939_irq_set_pri(TX4939_IR_PCIPME, 7);
	for (i = 0; i < TX4939_NUM_IR_TMR; i++)
		tx4939_irq_set_pri(TX4939_IR_TMR(i), 6);
	for (i = 0; i < TX4939_NUM_IR_SIO; i++)
		tx4939_irq_set_pri(TX4939_IR_SIO(i), 5);
	set_irq_chained_handler(mips_irq_base + TX4939_IRC_INT,
				handle_simple_irq);
}

static ssize_t
show_regdump(struct sys_device *dev, char *buf)
{
	int i;
	char *p = buf;
	struct tx4939_irc_reg *ircptr = tx4939_ircptr;

	p += sprintf(p, "den:\t%08x\n", ircptr->den.r);
	p += sprintf(p, "scipb:\t%08x\n", ircptr->scipb.r);
	for (i = 0; i < 2; i++)
		p += sprintf(p, "dm%d:\t%08x\n", i, ircptr->dm[i].r);
	for (i = 0; i < 2; i++)
		p += sprintf(p, "dm%d:\t%08x\n", 2 + i, ircptr->dm2[i].r);
	for (i = 0; i < 16; i++)
		p += sprintf(p, "lvl%d:\t%08x\n", i, ircptr->lvl[i].r);
	p += sprintf(p, "msk:\t%08x\n", ircptr->msk.r);
	p += sprintf(p, "edc:\t%08x\n", ircptr->edc.r);
	p += sprintf(p, "cs:\t%08x\n", ircptr->cs.r);
	p += sprintf(p, "pnd0:\t%08x\n", ircptr->pnd0.r);
	p += sprintf(p, "pnd1:\t%08x\n", ircptr->pnd1.r);
	return p - buf;
}

static SYSDEV_ATTR(regdump, 0400, show_regdump, NULL);

#ifdef CONFIG_PM
struct tx4939_irc_state {
	u32 den;
	u32 dm[2];
	u32 lvl[16];
	u32 msk;
	u32 edc;
	u32 dm2[2];
};

static void tx4939_irc_save_state(struct tx4939_irc_state *state,
				  struct tx4939_irc_reg *ircptr)
{
	int i;

	state->den = ircptr->den.r;
	state->msk = ircptr->msk.r;
	state->edc = ircptr->edc.r;
	for (i = 0; i < ARRAY_SIZE(state->dm); i++)
		state->dm[i] = ircptr->dm[i].r;
	for (i = 0; i < ARRAY_SIZE(state->dm2); i++)
		state->dm2[i] = ircptr->dm2[i].r;
	for (i = 0; i < ARRAY_SIZE(state->lvl); i++)
		state->lvl[i] = ircptr->lvl[i].r;
}

static void tx4939_irc_restore_state(struct tx4939_irc_state *state,
				     struct tx4939_irc_reg *ircptr)
{
	int i;

	ircptr->den.r = state->den;
	ircptr->edc.r = state->edc;
	for (i = 0; i < ARRAY_SIZE(state->lvl); i++)
		ircptr->lvl[i].r = state->lvl[i];
	for (i = 0; i < ARRAY_SIZE(state->dm); i++)
		ircptr->dm[i].r = state->dm[i];
	for (i = 0; i < ARRAY_SIZE(state->dm2); i++)
		ircptr->dm2[i].r = state->dm2[i];
	ircptr->msk.r = state->msk;
}

static struct tx4939_irc_state tx4939irc_state;
static int tx4939irc_suspend(struct sys_device *dev, pm_message_t state)
{
	tx4939_irc_save_state(&tx4939irc_state, tx4939_ircptr);
	return 0;
}
static int tx4939irc_resume(struct sys_device *dev)
{
	tx4939_irc_restore_state(&tx4939irc_state, tx4939_ircptr);
	return 0;
}
#else
#define tx4939irc_suspend NULL
#define tx4939irc_resume NULL
#endif

static struct sysdev_class tx4939irc_sysdev_class = {
	.suspend = tx4939irc_suspend,
	.resume = tx4939irc_resume,
};

static struct sys_device device_tx4939irc = {
	.cls	= &tx4939irc_sysdev_class,
};

static int __init tx4939irc_init_sysfs(void)
{
	int error;

	if (!tx4939_irq_base)
		return -ENODEV;
	sprintf(tx4939irc_sysdev_class.kset.kobj.name,
		"%sirc", txx9_pcode_str);
	error = sysdev_class_register(&tx4939irc_sysdev_class);
	if (!error) {
		error = sysdev_register(&device_tx4939irc);
		if (!error)
			error = sysdev_create_file(&device_tx4939irc,
						   &attr_regdump);
	}
	return error;
}
device_initcall(tx4939irc_init_sysfs);
