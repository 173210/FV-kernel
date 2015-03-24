/*
 * linux/arch/mips/tx-boards/generic/time_txx9.c
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2006
 * All Rights Reserved.
 *
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <asm/param.h>
#include <asm/time.h>
#include <asm/tx-boards/generic.h>
#include <asm/tx-boards/irq.h>
#include <asm/tx-boards/txx9.h>
#include <asm/tx-boards/tsb-generic.h>

/* Tick Timer divider */
#define TIMER_CCD	0	/* 1/2 */
#define TIMER_CLK	(TXX9_IMCLK / (2 << TIMER_CCD))

static struct txx9_tmr_reg *txx9_tmrptr;

static int txx9_timer_state(void)
{
	/* never called since we must know mips_hpt_frequency */
	return 0;
}

static u32 txx9_cpra;
static u32 txx9_delta_per_tick;
static u32 txx9_delta;
static u32 txx9_adjusted;

static void txx9_timer_ack(void)
{
	txx9_tmrptr->tisr = 0;	/* ack interrupt */
	if (txx9_delta_per_tick) {
		txx9_delta += txx9_delta_per_tick;
		if (txx9_delta >= HZ) {
			if (likely(txx9_tmrptr->trr < txx9_cpra / 2)) {
				txx9_delta -= HZ;
				txx9_tmrptr->cpra = txx9_cpra + 1;
				txx9_adjusted = 1;
			}
		} else if (txx9_adjusted) {
			if (likely(txx9_tmrptr->trr < txx9_cpra / 2)) {
				txx9_tmrptr->cpra = txx9_cpra;
				txx9_adjusted = 0;
			}
		}
	}
}

void __init txx9_time_init(struct txx9_tmr_reg *tmrptr)
{
	txx9_tmrptr = tmrptr;
	mips_timer_state = txx9_timer_state;
	mips_timer_ack = txx9_timer_ack;
	/* use R4k timer as HPT and TXX9 timer as timer IRQ */
	mips_hpt_frequency = txx9_cpu_clock / 2;

	txx9_cpra = TIMER_CLK / HZ;
	txx9_delta_per_tick = TIMER_CLK - txx9_cpra * HZ;
	/* Timer require 2IMCLK (i.e. one count on CCD=0) to wrap to zero */
	if ((IS_TC90411 || IS_TC90412 || IS_TC90416) && TIMER_CCD == 0)
		txx9_cpra--;
	/* reset the counter */
	txx9_tmrptr->tcr = TXx9_TMTCR_TCE;
	txx9_tmrptr->tcr = TXx9_TMTCR_CRE;

	txx9_tmrptr->cpra = txx9_cpra;
	txx9_tmrptr->itmr = TXx9_TMITMR_TIIE | TXx9_TMITMR_TZCE;
	txx9_tmrptr->ccdr = TIMER_CCD;
	txx9_tmrptr->tcr = TXx9_TMTCR_TCE | TXx9_TMTCR_CCDE | TXx9_TMTCR_TMODE_ITVL;
	txx9_tmrptr->tisr = 0;	/* clear pending interrupt */
}
