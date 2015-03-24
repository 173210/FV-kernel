/*
 * TC90416 Power Management Routines
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Original code for the PXA250 by Nicolas Pitre:
 * Copyright (c) 2001 Cliff Brake <cbrake@accelent.com>
 * Copyright (c) 2002 Monta Vista Software, Inc.
 *
 * (C) Copyright TOSHIBA CORPORATION 2006-2008
 * All Rights Reserved.
 */
#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>

#include <asm/tlbflush.h>
#include <asm/fpu.h>
#include <asm/cacheflush.h>

#include <asm/tx-boards/tsb-generic.h>
#include <asm/time.h>
#include <asm/tx-boards/tc904xx-pm.h>
#include <asm/tx-boards/pmon.h>

#include <linux/module.h>

#define DEFAULT_PM_WORK		0xa0000400
#define DEFAULT_PM_MEM_WORK	0xa7000000

static __u32 *tc90416_pm_work;
static __u32 *tc90416_pm_mem_work;

extern asmlinkage void tc90416_pm_suspend(void *,void*);
extern asmlinkage void tc90416_pm_resume(void);
extern void time_resume_init(void);

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int tc90416_pm_prepare(suspend_state_t state)
{
	return 0;
}

/*
 * Actually, shut-down / resume the system.
 */
static int tc90416_pm_enter(suspend_state_t state)
{
#ifdef CONFIG_SOFTWARE_SUSPEND
	unsigned long saved_compare;
#else
	unsigned long saved_registers32[SAVED32_REGS_SIZE];
	unsigned long long saved_registers64[SAVED64_REGS_SIZE];
	int i;
#endif
	struct timespec tv;
	int count,sec,nsec;

	if (state != PM_SUSPEND_MEM)
		return -EINVAL;

	/* Save the DSP context */
	if (cpu_has_dsp)
		__save_dsp(current);
	
#ifdef CONFIG_SOFTWARE_SUSPEND
	/*
	 * If SOFTWARE_SUSPEND enabled, states of DMAC, IRC, PCIC are
	 * saved/restored by corresponding sys_device.
	 */
	/* Save cp0 compare registers to context */
	saved_compare = read_c0_compare();
#else /* CONFIG_SOFTWARE_SUSPEND */
	/* Save the registers of DMAC   */
	saved_registers32[SAVED32_DMAC_MCR_OFF] = tc90416_gdmaptr->mcr;
		
	/* Save the registers of IRC   */
	saved_registers64[SAVED64_IRC_IMR_OFF] = tc90416_ircptr->imr;
	saved_registers64[SAVED64_IRC_CER_OFF] = tc90416_ircptr->cer;
	for (i = 0; i < 8; i++) saved_registers64[SAVED64_IRC_ILR_OFF + i] = tc90416_ircptr->ilr[i];
	for (i = 0; i < 2; i++) saved_registers64[SAVED64_IRC_CR_OFF + i]  = tc90416_ircptr->cr[i];
	tc90416_ircptr->imr = 0;

#ifdef CONFIG_PCI
	/* Save the registers of PCIC  */
	saved_registers32[SAVED32_PCIC_P2GM0PLBASE_OFF] = tc90416_pcicptr->p2gm0plbase;
	saved_registers32[SAVED32_PCIC_P2GM0PUBASE_OFF] = tc90416_pcicptr->p2gm0pubase;
	saved_registers32[SAVED32_PCIC_P2GM1PLBASE_OFF] = tc90416_pcicptr->p2gm1plbase;
	saved_registers32[SAVED32_PCIC_P2GM1PUBASE_OFF] = tc90416_pcicptr->p2gm1pubase;
	saved_registers32[SAVED32_PCIC_P2GM2PLBASE_OFF] = tc90416_pcicptr->p2gm2pbase;
	saved_registers32[SAVED32_PCIC_P2GIOPBASE_OFF]  = tc90416_pcicptr->p2giopbase;
	saved_registers32[SAVED32_PCIC_PBACFG_OFF]      = tc90416_pcicptr->pbacfg;

	saved_registers32[SAVED32_PCIC_CFG_OFF]      = tc90416_pcicptr->pcicfg1;
	saved_registers32[SAVED32_PCIC_G2PTOCNT_OFF] = tc90416_pcicptr->g2ptocnt;
	saved_registers32[SAVED32_PCIC_PCICMSK_OFF]  = tc90416_pcicptr->pcicmask;
	saved_registers32[SAVED32_PCIC_G2PMSK_OFF]   = tc90416_pcicptr->g2pmask;
	saved_registers32[SAVED32_PCIC_PCISTS_OFF]   = tc90416_pcicptr->pcistatus;
	saved_registers32[SAVED32_PCIC_PCIMSK_OFF]   = tc90416_pcicptr->pcimask;

	saved_registers32[SAVED32_PCIC_G2PIOMSK_OFF] = tc90416_pcicptr->g2piomask;
	saved_registers32[SAVED32_PCIC_PCICFG_OFF]   = tc90416_pcicptr->pciccfg;

	saved_registers64[SAVED64_PCIC_G2PIOGBASE_OFF]   = tc90416_pcicptr->g2piogbase;
	saved_registers64[SAVED64_PCIC_G2PIOPBASE_OFF]   = tc90416_pcicptr->g2piopbase;
	saved_registers64[SAVED64_PCIC_P2GIOGBASE_OFF]   = tc90416_pcicptr->p2giogbase;

	for (i = 0; i < 3; i++) {
		saved_registers32[SAVED32_PCIC_G2PMMASK_OFF + i]  = tc90416_pcicptr->g2pmmask[i];
		saved_registers64[SAVED64_PCIC_G2PMBASE_OFF + i]  = tc90416_pcicptr->g2pmgbase[i];
		saved_registers64[SAVED64_PCIC_G2PMPBASE_OFF + i] = tc90416_pcicptr->g2pmpbase[i];
		saved_registers64[SAVED64_PCIC_P2GMGBASE_OFF + i] = tc90416_pcicptr->p2gmgbase[i];
	}
#endif /* CONFIG_PCI */

	/* Save the registers of TMR   */
	saved_registers32[SAVED32_TMR_CPRA_OFF] = tc90416_tmrptr(0)->cpra;
	saved_registers32[SAVED32_TMR_ITMR_OFF] = tc90416_tmrptr(0)->itmr;
	saved_registers32[SAVED32_TMR_CCDR_OFF] = tc90416_tmrptr(0)->ccdr;
	saved_registers32[SAVED32_TMR_TCR_OFF]  = tc90416_tmrptr(0)->tcr;
	/* Save cp0 count/compare registers to context */
	saved_registers32[SAVED32_C0_COUNT_OFF] = read_c0_count();
	saved_registers32[SAVED32_C0_COMPARE_OFF] = read_c0_compare();
#endif /* CONFIG_SOFTWARE_SUSPEND */


	/* Save FPU registers to context */
	if (is_fpu_owner()) {
		save_fp(current);
	}

	/* Enter shut-down */
	tc90416_pm_suspend(tc90416_pm_work,tc90416_pm_mem_work);

	/* Now, wakes up the system. So, we have to restore the registers */

	/* Restore FPU registers from context */
	if (is_fpu_owner()) {
		restore_fp(current);
	}

	/* TMR */
	count = clocksource_mips.read();
	sec = count/mips_hpt_frequency;
	nsec = (count-sec*mips_hpt_frequency)*(NSEC_PER_SEC/mips_hpt_frequency);
#ifdef CONFIG_SOFTWARE_SUSPEND
	/* Restore cp0 count/compare registers from context */
	write_c0_compare(saved_compare);
	write_c0_count(saved_compare-1);
#else /* CONFIG_SOFTWARE_SUSPEND */
	tc90416_tmrptr(0)->cpra = saved_registers32[SAVED32_TMR_CPRA_OFF];
	tc90416_tmrptr(0)->itmr = saved_registers32[SAVED32_TMR_ITMR_OFF];
	tc90416_tmrptr(0)->ccdr = saved_registers32[SAVED32_TMR_CCDR_OFF];
	tc90416_tmrptr(0)->tcr  = saved_registers32[SAVED32_TMR_TCR_OFF];
	tc90416_tmrptr(0)->tisr = 0;

	/* Restore cp0 count/compare registers to context */
	write_c0_compare(saved_registers32[SAVED32_C0_COMPARE_OFF]);
	write_c0_count(saved_registers32[SAVED32_C0_COMPARE_OFF]-1);
#endif /* CONFIG_SOFTWARE_SUSPEND */

	do_posix_clock_monotonic_gettime(&tv);
	printk("Kernel resume: %d.%09d sec @ monotonic %ld.%09ld sec\n",sec,nsec,tv.tv_sec,tv.tv_nsec);

#ifndef CONFIG_SOFTWARE_SUSPEND
#ifdef CONFIG_PCI
	/* PCIC */
	tc90416_pcicptr->p2gm0plbase = saved_registers32[SAVED32_PCIC_P2GM0PLBASE_OFF];
	tc90416_pcicptr->p2gm0pubase = saved_registers32[SAVED32_PCIC_P2GM0PUBASE_OFF];
	tc90416_pcicptr->p2gm1plbase = saved_registers32[SAVED32_PCIC_P2GM1PLBASE_OFF];
	tc90416_pcicptr->p2gm1pubase = saved_registers32[SAVED32_PCIC_P2GM1PUBASE_OFF];
	tc90416_pcicptr->p2gm2pbase  = saved_registers32[SAVED32_PCIC_P2GM2PLBASE_OFF];
	tc90416_pcicptr->p2giopbase  = saved_registers32[SAVED32_PCIC_P2GIOPBASE_OFF];
	tc90416_pcicptr->g2piomask   = saved_registers32[SAVED32_PCIC_G2PIOMSK_OFF];
	tc90416_pcicptr->g2piogbase  = saved_registers64[SAVED64_PCIC_G2PIOGBASE_OFF];
	tc90416_pcicptr->g2piopbase  = saved_registers64[SAVED64_PCIC_G2PIOPBASE_OFF];
	tc90416_pcicptr->p2giogbase  = saved_registers64[SAVED64_PCIC_P2GIOGBASE_OFF];
	for (i = 0; i < 3; i++) {
		tc90416_pcicptr->g2pmmask[i]  = saved_registers32[SAVED32_PCIC_G2PMMASK_OFF + i];
		tc90416_pcicptr->g2pmgbase[i] = saved_registers64[SAVED64_PCIC_G2PMBASE_OFF + i];
		tc90416_pcicptr->g2pmpbase[i] = saved_registers64[SAVED64_PCIC_G2PMPBASE_OFF + i];
		tc90416_pcicptr->p2gmgbase[i] = saved_registers64[SAVED64_PCIC_P2GMGBASE_OFF + i];
	}
	tc90416_pcicptr->pciccfg    = saved_registers32[SAVED32_PCIC_PCICFG_OFF];
	tc90416_pcicptr->pbacfg      = saved_registers32[SAVED32_PCIC_PBACFG_OFF];
	tc90416_pcicptr->pcicfg1    = saved_registers32[SAVED32_PCIC_CFG_OFF];
	tc90416_pcicptr->g2ptocnt   = saved_registers32[SAVED32_PCIC_G2PTOCNT_OFF];
	tc90416_pcicptr->pcicmask   = saved_registers32[SAVED32_PCIC_PCICMSK_OFF];
	tc90416_pcicptr->g2pmask    = saved_registers32[SAVED32_PCIC_G2PMSK_OFF];
	tc90416_pcicptr->pcistatus  = saved_registers32[SAVED32_PCIC_PCISTS_OFF];
	tc90416_pcicptr->pcimask    = saved_registers32[SAVED32_PCIC_PCIMSK_OFF];
	tc90416_pcicptr->pcicstatus = TX4927_PCIC_PCICSTATUS_ALL;
	tc90416_pcicptr->g2pstatus  = TX4927_PCIC_G2PSTATUS_ALL;
	tc90416_pcicptr->pcistatus  = (tc90416_pcicptr->pcistatus & 0x0000ffff) | (TX4927_PCIC_PCISTATUS_ALL << 16);
#endif  /* CONFIG_PCI */

	/* IRC */
	tc90416_ircptr->cer = saved_registers64[SAVED64_IRC_CER_OFF];
	for (i = 0; i < 8; i++) tc90416_ircptr->ilr[i] = saved_registers64[SAVED64_IRC_ILR_OFF + i];
	for (i = 0; i < 2; i++) tc90416_ircptr->cr[i]  = saved_registers64[SAVED64_IRC_CR_OFF + i];
	tc90416_ircptr->imr = saved_registers64[SAVED64_IRC_IMR_OFF];

	/* Save the registers of DMAC   */
	tc90416_gdmaptr->mcr = saved_registers32[SAVED32_DMAC_MCR_OFF];
#endif /* CONFIG_SOFTWARE_SUSPEND */

	/* Initialize TLB */
	write_c0_wired(0);
	local_flush_tlb_all();

	/* Restore DSP context */
	if (cpu_has_dsp){
		set_c0_status(ST0_MX);
		__restore_dsp(current);
	}

	/* Restore TLS context */
        if (cpu_has_mips_r2) {
                unsigned int enable = 0x0000000f;
 
                if (cpu_has_userlocal)
                        enable |= (1 << 29);
 
                write_c0_hwrena(enable);
		iob();
		
		if (cpu_has_userlocal){
			write_c0_userlocal(current_thread_info()->tp_value);
			iob();
		}
        }

	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static int tc90416_pm_finish(suspend_state_t state)
{
	return 0;
}


static struct pm_ops tc90416_pm_ops = {
	.prepare	= tc90416_pm_prepare,
	.enter		= tc90416_pm_enter,
	.finish		= tc90416_pm_finish,
};

static ssize_t pmon_memtest_show(struct sysdev_class *class, char *buf)
{
	return sprintf(buf, "0x%08x\n", (__u32)tc90416_pm_mem_work);
}
static ssize_t pmon_memtest_store(struct sysdev_class *class, const char *buf,
				  size_t size)
{
	tc90416_pm_mem_work = (__u32 *)simple_strtoul(buf, NULL, 0);
	return size;
}
static SYSDEV_CLASS_ATTR(pmon_memtest, 0600, pmon_memtest_show, pmon_memtest_store);

static int __init tc90416_pm_init(void)
{
	pm_set_ops(&tc90416_pm_ops);
	if (tc90416_pm_work == 0){
		char *str = (char*)pmon_getenv("pm_work");
		if (str)
			tc90416_pm_work = (__u32 *)simple_strtoul(str, NULL, 0);
		if (tc90416_pm_work == 0)
			tc90416_pm_work = (__u32 *)DEFAULT_PM_WORK;
	}
	tc90416_pm_work = (__u32 *)KSEG1ADDR(tc90416_pm_work);
	if (tc90416_pm_mem_work == 0){
		tc90416_pm_mem_work = (__u32 *)tc90416_pm_work[PMWORK_MEMTEST_ADDR_OFF/4];
		if (((__u32)tc90416_pm_mem_work < KSEG0) || ((__u32)tc90416_pm_mem_work >= KSEG2))
			tc90416_pm_mem_work = (__u32 *)DEFAULT_PM_MEM_WORK;
	}
	tc90416_pm_mem_work = (__u32 *)KSEG1ADDR(tc90416_pm_mem_work);
	printk("PM: pm_work=0x%8p,pm_mem_work=0x%8p\n",tc90416_pm_work,tc90416_pm_mem_work);
	if (txboard_sysdev_class_create_file(&attr_pmon_memtest))
		printk(KERN_ERR "PM: unable to register /sys/devices/system/tx-board/pmon_memtest\n");
	return 0;
}

late_initcall(tc90416_pm_init);


static int __init tc90416_work_area_setup(char *str)
{
	tc90416_pm_work = (__u32 *)simple_strtoul(str, NULL, 0);

	return 1;
}
__setup("pm_work=", tc90416_work_area_setup);
