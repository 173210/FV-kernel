/*
 * linux/arch/mips/tx-boards/generic/irq_txx9_64.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2005-2007
 * All Rights Reserved.
 *
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/module.h>
#include <asm/tx-boards/generic.h>
#include <asm/tx-boards/irq.h>
#include <asm/tx-boards/txx9.h>

#define TXx9_MAX_IR 64
#define irc_dlevel	0
#define irc_elevel	1
/* initially, all source have middle priprity (4) */
static unsigned char irc_level[TXx9_MAX_IR] = {[ 0 ... TXx9_MAX_IR - 1] = 4};

static unsigned int txx9_64_irq_base = -1;
static struct txx9_64_irc_reg *txx9_64_ircptr;
static unsigned char txx9_64_exint_start, txx9_64_exint_num;
static unsigned int txx9_64_irq_num;

static void txx9_64_irq_unmask(unsigned int irq)
{
	unsigned int irq_nr = irq - txx9_64_irq_base;
	volatile __u64 *ilrp = &txx9_64_ircptr->ilr[(irq_nr % 16 ) / 2];
	int ofs = irq_nr / 16 * 16 + (irq_nr & 1) * 8;
	*ilrp = (*ilrp & ~((__u64)0xff << ofs)) | ((__u64)irc_level[irq_nr] << ofs);
	iob();
}

static inline void txx9_64_irq_mask(unsigned int irq)
{
	unsigned int irq_nr = irq - txx9_64_irq_base;
	volatile __u64 *ilrp = &txx9_64_ircptr->ilr[(irq_nr % 16) / 2];
	int ofs = irq_nr / 16 * 16 + (irq_nr & 1) * 8;
	*ilrp = (*ilrp & ~((__u64)0xff << ofs)) | ((__u64)irc_dlevel << ofs);
	/* update IRCSR */
	txx9_64_ircptr->imr = 0;
	txx9_64_ircptr->imr = irc_elevel;
}

static void txx9_64_irq_mask_ack(unsigned int irq)
{
	unsigned int irq_nr = irq - txx9_64_irq_base;
	txx9_64_irq_mask(irq);
	if (irq_nr - txx9_64_exint_start < txx9_64_exint_num) {
		/* clear edge detection */
		__u64 cr = txx9_64_ircptr->cr[irq_nr / 8];
		cr = (cr >> ((irq_nr & (8 - 1)) * 2)) & 3;
		if (TXx9_IRCR_EDGE(cr))
			txx9_64_ircptr->scr = TXx9_IRSCR_EIClrE | irq_nr;
	}
}
static void txx9_64_irq_end(unsigned int irq)
{
	if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS)))
		txx9_64_irq_unmask(irq);
}

static int txx9_64_irq_set_type(unsigned int irq, unsigned int flow_type)
{
	unsigned int irq_nr = irq - txx9_64_irq_base;
	__u64 cr;
	volatile __u64 *crp;
	int ofs;
	int mode;

#ifdef CONFIG_TOSHIBA_TC90416
	extern int tc90416_irq_set_type_udly(unsigned int irq, unsigned int flow_type);
	if (flow_type & IRQ_TYPE_UDLY_FLAG)
		return tc90416_irq_set_type_udly(irq, flow_type);
#endif
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
	crp = &txx9_64_ircptr->cr[((unsigned int)irq_nr % 16) / 8];
	cr = *crp;
	ofs = (irq_nr & 0x30) | ((irq_nr & 0x7) << 1);
	cr &= ~((__u64)0x3 << ofs);
	cr |= ((__u64)(mode & 0x3)) << ofs;
	*crp = cr;
	return 0;
}

static struct irq_chip txx9_64_irq_chip = {
	.typename	= "TXx9_64",
	.ack		= txx9_64_irq_mask_ack,
	.mask		= txx9_64_irq_mask,
	.mask_ack	= txx9_64_irq_mask_ack,
	.unmask		= txx9_64_irq_unmask,
	.end		= txx9_64_irq_end,
	.set_type	= txx9_64_irq_set_type,
};

void __init
txx9_64_irq_init(int irq_base, struct txx9_64_irc_reg *ircptr, int irq_num, int exint_start, int exint_num)
{
	int i;

	txx9_64_irq_base = irq_base;
	txx9_64_irq_num = irq_num;
	txx9_64_ircptr = ircptr;
	txx9_64_exint_start = exint_start;
	txx9_64_exint_num = exint_num;

	for (i = irq_base; i < irq_base + txx9_64_irq_num; i++)
		set_irq_chip_and_handler(i, &txx9_64_irq_chip,
					 handle_level_irq);

	/* mask all IRC interrupts */
	txx9_64_ircptr->imr = 0;
	for (i = 0; i < 8; i++)
		txx9_64_ircptr->ilr[i] = 0;
	/* setup IRC interrupt mode (Low Active) */
	for (i = 0; i < 2; i++)
		txx9_64_ircptr->cr[i] = 0;
	/* enable interrupt control */
	txx9_64_ircptr->cer = TXx9_IRCER_ICE;
	txx9_64_ircptr->imr = irc_elevel;
}

/* convert txx9 irc number to global irq number */
int txx9_64_irq_to_irq(int irc_irq)
{
	if ((unsigned int)irc_irq >= txx9_64_irq_num)
		return -1;

	return irc_irq + txx9_64_irq_base;
}

EXPORT_SYMBOL(txx9_64_irq_to_irq);

int txx9_64_irq_set_pri(int irc_irq, int new_pri)
{
	int old_pri;
	if ((unsigned int)irc_irq >= txx9_64_irq_num)
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
	struct txx9_64_irc_reg *ircptr = txx9_64_ircptr;

	p += sprintf(p, "cer:\t%016llx\n", ircptr->cer);
	for (i = 0; i < 2; i++)
		p += sprintf(p, "cr%d:\t%016llx\n", i, ircptr->cr[i]);
	for (i = 0; i < 8; i++)
		p += sprintf(p, "ilr%d:\t%016llx\n", i, ircptr->ilr[i]);
	p += sprintf(p, "imr:\t%016llx\n", ircptr->imr);
	p += sprintf(p, "scr:\t%016llx\n", ircptr->scr);
	p += sprintf(p, "ssr:\t%016llx\n", ircptr->ssr);
	p += sprintf(p, "csr:\t%016llx\n", ircptr->csr);
	return p - buf;
}

static SYSDEV_ATTR(regdump, 0400, show_regdump, NULL);

#ifdef CONFIG_SOFTWARE_SUSPEND
/*
 * tc904xx have its own method for suspend-to-mem.  Use this only if
 * suspend-to-disk enabled.  (i.e. use CONFIG_SOFTWARE_SUSPEND instead
 * of CONFIG_PM)
 */
struct txx9_64_irc_state {
	u64 cer;
	u64 cr[2];
	u64 ilr[8];
	u64 imr;
};

static void txx9_64_irc_save_state(struct txx9_64_irc_state *state,
				struct txx9_64_irc_reg *ircptr)
{
	int i;

	state->imr = ircptr->imr;
	state->cer = ircptr->cer;
	for (i = 0; i < ARRAY_SIZE(state->ilr); i++)
		state->ilr[i] = ircptr->ilr[i];
	for (i = 0; i < ARRAY_SIZE(state->cr); i++)
		state->cr[i] = ircptr->cr[i];
}

static void txx9_64_irc_restore_state(struct txx9_64_irc_state *state,
				   struct txx9_64_irc_reg *ircptr)
{
	int i;

	ircptr->cer = state->cer;
	for (i = 0; i < ARRAY_SIZE(state->ilr); i++)
		ircptr->ilr[i] = state->ilr[i];
	for (i = 0; i < ARRAY_SIZE(state->cr); i++)
		ircptr->cr[i] = state->cr[i];
	ircptr->imr = state->imr;
}

static struct txx9_64_irc_state txx9_64irc_state;
static int txx9_64irc_suspend(struct sys_device *dev, pm_message_t state)
{
	txx9_64_irc_save_state(&txx9_64irc_state, txx9_64_ircptr);
	return 0;
}
static int txx9_64irc_resume(struct sys_device *dev)
{
	txx9_64_irc_restore_state(&txx9_64irc_state, txx9_64_ircptr);
	return 0;
}
#else
#define txx9_64irc_suspend NULL
#define txx9_64irc_resume NULL
#endif

static struct sysdev_class txx9_64irc_sysdev_class = {
	.suspend = txx9_64irc_suspend,
	.resume = txx9_64irc_resume,
};
static struct sys_device device_txx9_64irc = {
	.cls	= &txx9_64irc_sysdev_class,
};

static int __init txx9_64irc_init_sysfs(void)
{
	int error;

	if (!txx9_64_ircptr)
		return -ENODEV;
	sprintf(txx9_64irc_sysdev_class.kset.kobj.name,
		"%sirc", txx9_pcode_str);
	error = sysdev_class_register(&txx9_64irc_sysdev_class);
	if (!error) {
		error = sysdev_register(&device_txx9_64irc);
		if (!error)
			error = sysdev_create_file(&device_txx9_64irc,
						   &attr_regdump);
	}
	return error;
}
device_initcall(txx9_64irc_init_sysfs);
