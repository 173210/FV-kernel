/*
 * linux/arch/mips/tx-boards/generic/irq_txx9.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2007
 * All Rights Reserved.
 *
 */
/*
 * TX4925,TX4927,TX4938,etc. defines 32 IRQs.
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/module.h>
#include <asm/tx-boards/generic.h>
#include <asm/tx-boards/irq.h>
#include <asm/tx-boards/txx9.h>

#define TXx9_MAX_IR 32
#define irc_dlevel	0
#define irc_elevel	1
/* initially, all source have middle priprity (4) */
static unsigned char irc_level[TXx9_MAX_IR] = {[ 0 ... TXx9_MAX_IR - 1] = 4};

static unsigned int txx9_irq_base;
static struct txx9_irc_reg *txx9_ircptr;
static unsigned char txx9_exint_start, txx9_exint_num;
static unsigned int txx9_irq_num;

static void txx9_irq_unmask(unsigned int irq)
{
	unsigned int irq_nr = irq - txx9_irq_base;
	volatile __u32 *ilrp = &txx9_ircptr->ilr[(irq_nr % 16 ) / 2];
	int ofs = irq_nr / 16 * 16 + (irq_nr & 1) * 8;
	*ilrp = (*ilrp & ~(0xff << ofs)) | (irc_level[irq_nr] << ofs);
}

static inline void txx9_irq_mask(unsigned int irq)
{
	unsigned int irq_nr = irq - txx9_irq_base;
	volatile __u32 *ilrp = &txx9_ircptr->ilr[(irq_nr % 16) / 2];
	int ofs = irq_nr / 16 * 16 + (irq_nr & 1) * 8;
	*ilrp = (*ilrp & ~(0xff << ofs)) | (irc_dlevel << ofs);
	iob();
}

static void txx9_irq_mask_ack(unsigned int irq)
{
	unsigned int irq_nr = irq - txx9_irq_base;
	txx9_irq_mask(irq);
	if (irq_nr - txx9_exint_start < txx9_exint_num) {
		/* clear edge detection */
		__u32 cr = txx9_ircptr->cr[irq_nr / 8];
		cr = (cr >> ((irq_nr & (8 - 1)) * 2)) & 3;
		if (TXx9_IRCR_EDGE(cr))
			txx9_ircptr->scr = TXx9_IRSCR_EIClrE | irq_nr;
	}
}

static int txx9_irq_set_type(unsigned int irq, unsigned int flow_type)
{
	unsigned int irq_nr = irq - txx9_irq_base;
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
	crp = &txx9_ircptr->cr[(unsigned int)irq_nr / 8];
	cr = *crp;
	ofs = (irq_nr & (8 - 1)) * 2;
	cr &= ~(0x3 << ofs);
	cr |= (mode & 0x3) << ofs;
	*crp = cr;
	return 0;
}

static struct irq_chip txx9_irq_chip = {
	.name		= txx9_pcode_str,
	.ack		= txx9_irq_mask_ack,
	.mask		= txx9_irq_mask,
	.mask_ack	= txx9_irq_mask_ack,
	.unmask		= txx9_irq_unmask,
	.set_type	= txx9_irq_set_type,
};

void __init
txx9_irq_init(int irq_base, struct txx9_irc_reg *ircptr, int irq_num, int exint_start, int exint_num)
{
	int i;

	txx9_irq_base = irq_base;
	txx9_irq_num = irq_num;
	txx9_ircptr = ircptr;
	txx9_exint_start = exint_start;
	txx9_exint_num = exint_num;

	for (i = irq_base; i < irq_base + txx9_irq_num; i++)
		set_irq_chip_and_handler(i, &txx9_irq_chip, handle_level_irq);

	/* mask all IRC interrupts */
	txx9_ircptr->imr = 0;
	for (i = 0; i < 8; i++)
		txx9_ircptr->ilr[i] = 0;
	/* setup IRC interrupt mode (Low Active) */
	for (i = 0; i < 2; i++)
		txx9_ircptr->cr[i] = 0;
	/* enable interrupt control */
	txx9_ircptr->cer = TXx9_IRCER_ICE;
	txx9_ircptr->imr = irc_elevel;
}

/* convert txx9 irc number to global irq number */
int txx9_irq_to_irq(int irc_irq)
{
	if ((unsigned int)irc_irq >= txx9_irq_num)
		return -1;

	return irc_irq + txx9_irq_base;
}

EXPORT_SYMBOL(txx9_irq_to_irq);

int txx9_irq_set_pri(int irc_irq, int new_pri)
{
	int old_pri;
	if ((unsigned int)irc_irq >= txx9_irq_num)
		return 0;
	old_pri = irc_level[irc_irq];
	irc_level[irc_irq] = new_pri;
	return old_pri;
}

static ssize_t
show_regdump(struct sys_device *dev, char *buf)
{
	int i;
	char *p = buf;
	struct txx9_irc_reg *ircptr = txx9_ircptr;

	p += sprintf(p, "cer:\t%08x\n", ircptr->cer);
	for (i = 0; i < 2; i++)
		p += sprintf(p, "cr%d:\t%08x\n", i, ircptr->cr[i]);
	for (i = 0; i < 8; i++)
		p += sprintf(p, "ilr%d:\t%08x\n", i, ircptr->ilr[i]);
	p += sprintf(p, "imr:\t%08x\n", ircptr->imr);
	p += sprintf(p, "scr:\t%08x\n", ircptr->scr);
	p += sprintf(p, "ssr:\t%08x\n", ircptr->ssr);
	p += sprintf(p, "csr:\t%08x\n", ircptr->csr);
	return p - buf;
}

static SYSDEV_ATTR(regdump, 0400, show_regdump, NULL);

#ifdef CONFIG_PM
struct txx9_irc_state {
	u32 cer;
	u32 cr[2];
	u32 ilr[8];
	u32 imr;
};

static void txx9_irc_save_state(struct txx9_irc_state *state,
				struct txx9_irc_reg *ircptr)
{
	int i;

	state->imr = ircptr->imr;
	state->cer = ircptr->cer;
	for (i = 0; i < ARRAY_SIZE(state->ilr); i++)
		state->ilr[i] = ircptr->ilr[i];
	for (i = 0; i < ARRAY_SIZE(state->cr); i++)
		state->cr[i] = ircptr->cr[i];
}

static void txx9_irc_restore_state(struct txx9_irc_state *state,
				   struct txx9_irc_reg *ircptr)
{
	int i;

	ircptr->cer = state->cer;
	for (i = 0; i < ARRAY_SIZE(state->ilr); i++)
		ircptr->ilr[i] = state->ilr[i];
	for (i = 0; i < ARRAY_SIZE(state->cr); i++)
		ircptr->cr[i] = state->cr[i];
	ircptr->imr = state->imr;
}

static struct txx9_irc_state txx9irc_state;
static int txx9irc_suspend(struct sys_device *dev, pm_message_t state)
{
	txx9_irc_save_state(&txx9irc_state, txx9_ircptr);
	return 0;
}
static int txx9irc_resume(struct sys_device *dev)
{
	txx9_irc_restore_state(&txx9irc_state, txx9_ircptr);
	return 0;
}
#else
#define txx9irc_suspend NULL
#define txx9irc_resume NULL
#endif

static struct sysdev_class txx9irc_sysdev_class = {
	.suspend = txx9irc_suspend,
	.resume = txx9irc_resume,
};
static struct sys_device device_txx9irc = {
	.cls	= &txx9irc_sysdev_class,
};

static int __init txx9irc_init_sysfs(void)
{
	int error;

	if (!txx9_ircptr)
		return -ENODEV;
	sprintf(txx9irc_sysdev_class.kset.kobj.name,
		"%sirc", txx9_pcode_str);
	error = sysdev_class_register(&txx9irc_sysdev_class);
	if (!error) {
		error = sysdev_register(&device_txx9irc);
		if (!error)
			error = sysdev_create_file(&device_txx9irc,
						   &attr_regdump);
	}
	return error;
}
device_initcall(txx9irc_init_sysfs);
