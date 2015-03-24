/*
 * TC90417 Power Management Core Routines
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2008
 * All Rights Reserved.
 */

#include <linux/init.h>
#include <asm/asm.h>
#include <asm/mipsregs.h>
#include <asm/regdef.h>
#include <asm/stackframe.h>
#include <asm/addrspace.h>
#include <asm/tx-boards/tc90416-regs.h>
#include <asm/tx-boards/tc904xx-pm.h>

//#define EMULATE_SUSPEND /* for test */
//#define SEND_TO_TVMICOM_AFTER_SEFLREFRESH /* depend on TV micom */
//#define PMTEST_BROKEN_KERNEL
//#define PMTEST_BROKEN_STACK
//#define PMTEST_BROKEN_PMWORK

/* PIO registers */
#define XPIODO0SET	0x0080
#define XPIODO0RST	0x0084
#define XPIODO1SET	0x0088
#define XPIODO1RST	0x008c
#define XPIODO2SET	0x0090
#define XPIODO2RST	0x0094

/* UART registers */
#define UALCR		0x00
#define UADICR		0x04
#define UADISR		0x08
#define UASCISR		0x0c
#define UAFCR		0x10
#define UAFLCR		0x14
#define UABGR		0x18
#define UATFIFO		0x1c
#define UARFIFO		0x20

/* UART base address */
#define TVMICOM_UART_BASE TC90416_SIO_REG(CONFIG_UAT_UCOM_CH)

/* DDR phy enable pin */
#define DDRPHY_PIN	CONFIG_PM_DDRPHY_PIN

/* Select PIO register */
#if DDRPHY_PIN >= 64
#define DDRPHY_PIOSET XPIODO2SET
#define DDRPHY_PIORST XPIODO2RST
#define DDRPHY_BIT (1<<(DDRPHY_PIN-64))
#elif DDRPHY_PIN >= 32
#define DDRPHY_PIOSET XPIODO1SET
#define DDRPHY_PIORST XPIODO1RST
#define DDRPHY_BIT (1<<(DDRPHY_PIN-32))
#else
#define DDRPHY_PIOSET XPIODO0SET
#define DDRPHY_PIORST XPIODO0RST
#define DDRPHY_BIT (1<<DDRPHY_PIN)
#endif

/* checksum area */
#define CKSUM_AREA_START	_text
#define CKSUM_AREA_END		_etext

	.data
pwrdwn_cmd:	/* TV micom power down command */
	.byte	0x3a,0x03,0x2a,0x64
pwrdwn_cmd_size:
	.byte	(pwrdwn_cmd_size-pwrdwn_cmd)

	.text
	.set	noreorder
	.set	at
/*
 * Enter the system to suspend state.
 */
/*
 * void tc90416_pm_suspend(unsigned long pm_work)
 */
LEAF(tc90416_pm_suspend)
	/* Save TX core registers.
	 *
	 *  We have to save the below registers.
	 *    cp0 registers     : pagemask, status, config
	 *    general registers : sp, gp, fp, ra
	 */
	addiu	sp, sp, -SAVED_AREA_SIZE
	LONG_S	fp, SAVED_COREREGS_FP_OFF(sp)
	LONG_S	gp, SAVED_COREREGS_GP_OFF(sp)
	LONG_S	ra, SAVED_COREREGS_RA_OFF(sp)

	LONG_S	s0, SAVED_COREREGS_S0_OFF(sp)
	LONG_S	s1, SAVED_COREREGS_S1_OFF(sp)
	LONG_S	s2, SAVED_COREREGS_S2_OFF(sp)
	LONG_S	s3, SAVED_COREREGS_S3_OFF(sp)
	LONG_S	s4, SAVED_COREREGS_S4_OFF(sp)
	LONG_S	s5, SAVED_COREREGS_S5_OFF(sp)
	LONG_S	s6, SAVED_COREREGS_S6_OFF(sp)
	LONG_S	s7, SAVED_COREREGS_S7_OFF(sp)

	MFC0	t2, CP0_PAGEMASK
	MFC0	t3, CP0_STATUS
	MFC0	t4, CP0_CONFIG
	LONG_S	t2, SAVED_COREREGS_PAGEMASK_OFF(sp)
	LONG_S	t3, SAVED_COREREGS_STATUS_OFF(sp)
	LONG_S	t4, SAVED_COREREGS_CONFIG_OFF(sp)
	MFC0	t2, CP0_ENTRYHI
	MFC0	t3, CP0_CONTEXT
	MFC0	t4, CP0_XCONTEXT
	LONG_S	t2, SAVED_COREREGS_ENTRYHI_OFF(sp)
	LONG_S	t3, SAVED_COREREGS_CONTEXT_OFF(sp)
	LONG_S	t4, SAVED_COREREGS_XCONTEXT_OFF(sp)

	/* calculate checksum */
	PTR_LA	t0, CKSUM_AREA_START
	PTR_LA	t1, CKSUM_AREA_END
	LONG_S	t0, SAVED_CKSUM_START(sp)
	LONG_S	t1, SAVED_CKSUM_END(sp)
	sltu	v0, t0, t1
	beqz	v0, 2f
	move	t2, zero
1:	lw	v1,0(t0)
 	addiu	t0,t0,4
 	sltu	v0,t0,t1
 	bnez	v0,1b
 	addu	t2,t2,v1
2:	sw	t2, SAVED_CKSUM_VALUE(sp)

	/* calculate entry checksum */
	PTR_LA	t0, tc90416_pm_resume		/* start */
	LONG_S	t0, PMWORK_ENTRY_ADDR_OFF(a0)
	PTR_LA	t2, tc90416_pm_resume_end	/* end */
	LONG_S	t2, PMWORK_ENTRY_END_OFF(a0)
	j	2f
	move	t1,zero
1:	addiu	t0, t0, 4
	addu	t1, t1, v1
2:	sltu	v0, t0, t2
	bnezl	v0, 1b
	LONG_L	v1, 0(t0)
	LONG_S	t1, PMWORK_ENTRY_CKSUM_OFF(a0)

	/* calculate stack checksum */
	move	t0, sp		/* start */
	LONG_S	t0, PMWORK_STACK_ADDR_OFF(a0)
	addi	t2, sp, 0xfff
	li	t1, 0xfffff000
	and	t2, t2, t1	/* end */
	LONG_S	t2, PMWORK_STACK_END_OFF(a0)
	j	2f
	move	t1,zero
1:	addiu	t0, t0, 4
	addu	t1, t1, v1
2:	sltu	v0, t0, t2
	bnezl	v0, 1b
	LONG_L	v1, 0(t0)
	LONG_S	t1, PMWORK_STACK_CKSUM_OFF(a0)

	/* calculate checksum */
	move	t0, a0
	addi	t2, t0, 24   /* 6 word */
	j	2f
	move	t1, zero
1:	addiu	t0, t0, 4
	addu	t1, t1, v1
2:	sltu	v0, t0, t2
	bnezl	v0, 1b
	LONG_L	v1, 0(t0)
	LONG_S	t1, PMWORK_CKSUM_OFF(a0)

#if defined(PMTEST_BROKEN_KERNEL)
	PTR_LA	t2, CKSUM_AREA_START
	LONG_L	t1, 0(t2)
	addi	t1, t1, 1
	LONG_S	t1, 0(t2)
#elif defined(PMTEST_BROKEN_STACK)
	li	t1, 0x0
	LONG_S	t1, 0(sp)
#elif defined(PMTEST_BROKEN_PMWORK)
	li	t1, 0x0
	LONG_S	t1, PMWORK_ENTRY_ADDR_OFF(a0)
#endif

	/* flush cache with TXMBR flush */
	PTR_LA	t2, local_flush_cache_all
	LONG_L	t2, 0(t2)
	jalr	t2
	nop

#ifndef EMULATE_SUSPEND
	/* send power down command to TV micom */
	li	t3, TC90416_PIO_REG
	PTR_LA	a2, pwrdwn_cmd_size
	PTR_LA	a1, pwrdwn_cmd
	lb	a2, 0(a2)
	PTR_LA	t5, uat_write_cmd
#ifdef SEND_TO_TVMICOM_AFTER_SEFLREFRESH
	jalr	t5
	li	a0, 0  /* delay slot */
	li	t4, TVMICOM_UART_BASE
#else
	jalr	t5
	li	a0, 1  /* delay slot */
#endif /* SEND_TO_TVMICOM_AFTER_SEFLREFRESH */
	/* TV Micom power down CPU in 100ms */

	/* Enter self-refresh mode then set DDRPHY low. */
	li	t7, DDRPHY_BIT
	li	t1, TC90417_DDRC_TOP_REG
	li	t2, TC90417_DDRC_2ND_REG
	li	t3, TC90417_DDRC_3RD_REG
	li	t4, TC90416_PIO_REG

	/* fill DO_SELFREFRESH routine into i-cache */
	PTR_LA	a0, DO_SELFREFRESH
	PTR_LA	a1, DONE_SELFREFRESH
1:	cache	0x14, 0(a0)
	addiu	a0, a0, 32
	bltu	a0, a1, 1b
	nop

	.align	5		/* Run on cache (only 8 instructions) */
DO_SELFREFRESH:
	/* write 0xb8488000 = 0 */
	LONG_S	zero, 0(t3)
	/* read  0xb8488000 */
	LONG_L	t0, 0(t3)
	/* write 0xb8474000 = 0 */
	LONG_S	zero, 0(t1)
	/* write 0xb8474004 = 4 */
	li	a0, 4
	LONG_S	a0, 4(t1)
	/* wait  0xb8474004 == 0 */
1:	LONG_L	a1, 4(t1)
	bne	a1,zero,1b
	nop
	/* DDRPHY=0 */
	LONG_S	t7, DDRPHY_PIORST(t4)	/* PowerDown DDRPHY */
	/* write 0xb8484000 = 0 */
	LONG_S	zero, 0(t2)
	/* 22.5ns */
	mtc0	zero, CP0_COUNT
	sync
2:	mfc0	a1, CP0_COUNT
	bltu	a1, 6, 2b		/* wait 6cycle */
	nop
	/* write 0xb8484004 = 1 */
	li	a0, 1
	LONG_S	a0, 4(t2)
	/* wait  0xb8484008 == 1 */
3:	LONG_L	a1, 8(t2)
	bne	a1,a0,3b
	nop
	/* write 0xb8484004 = 0 */
	LONG_S	zero, 4(t2)
	/* wait  0xb8484008 == 0 */
4:	LONG_L	a1, 8(t2)
	bne	a1,zero,4b
	nop
	/* power off */
#ifdef SEND_TO_TVMICOM_AFTER_SEFLREFRESH
	LONG_S	v0, UATFIFO(t4)		/* Send checksum to TV Micom */
#endif
1:	b	1b
	nop
#endif /* EMULATE_SUSPEND */
DONE_SELFREFRESH:
	END(tc90416_pm_suspend)

/*
 * Wake up the system from suspend state.
 */
	.align	5
/*
 * void tc90416_pm_resume(unsigned long sp)
 */
LEAF(tc90416_pm_resume)
#ifdef EMULATE_SUSPEND
	move	a0, sp
#endif
	/* calculate checksum */
	/* t0 -> t1  tmp=v1,v0, sum=t2 */
	LONG_L	t0, SAVED_CKSUM_START(a0)
	LONG_L	t1, SAVED_CKSUM_END(a0)
	sltu	v0, t0, t1
	beqz	v0, 2f
	move	t2, zero
1:	lw	v1,0(t0)
 	addiu	t0,t0,4
 	sltu	v0,t0,t1
 	bnez	v0,1b
 	addu	t2,t2,v1
2:	
	/* compare checksum */
	lw	t3, SAVED_CKSUM_VALUE(a0)
	beq	t2, t3, 3f
	nop
	la	t0, 0xa7000000
	LONG_S	t2, 0(t0)
	LONG_S	t3, 4(t0)
	jr	ra
	nop
3:
	LONG_L	t4, SAVED_COREREGS_PAGEMASK_OFF(sp)
	LONG_L	t5, SAVED_COREREGS_STATUS_OFF(sp)
	LONG_L	t6, SAVED_COREREGS_CONFIG_OFF(sp)
	MTC0	t4, CP0_PAGEMASK
	MTC0	t5, CP0_STATUS
	MTC0	t6, CP0_CONFIG
	LONG_L	t4, SAVED_COREREGS_ENTRYHI_OFF(sp)
	LONG_L	t5, SAVED_COREREGS_CONTEXT_OFF(sp)
	LONG_L	t6, SAVED_COREREGS_XCONTEXT_OFF(sp)
	MTC0	t4, CP0_ENTRYHI
	MTC0	t5, CP0_CONTEXT
	MTC0	t6, CP0_XCONTEXT

	LONG_L	s0, SAVED_COREREGS_S0_OFF(sp)
	LONG_L	s1, SAVED_COREREGS_S1_OFF(sp)
	LONG_L	s2, SAVED_COREREGS_S2_OFF(sp)
	LONG_L	s3, SAVED_COREREGS_S3_OFF(sp)
	LONG_L	s4, SAVED_COREREGS_S4_OFF(sp)
	LONG_L	s5, SAVED_COREREGS_S5_OFF(sp)
	LONG_L	s6, SAVED_COREREGS_S6_OFF(sp)
	LONG_L	s7, SAVED_COREREGS_S7_OFF(sp)

	LONG_L	fp, SAVED_COREREGS_FP_OFF(sp)
	LONG_L	gp, SAVED_COREREGS_GP_OFF(sp)
	LONG_L	ra, SAVED_COREREGS_RA_OFF(sp)

	j	ra
	addiu	sp, sp, SAVED_AREA_SIZE

tc90416_pm_resume_end:
	END(tc90416_pm_resume)
