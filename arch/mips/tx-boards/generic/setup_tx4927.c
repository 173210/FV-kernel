/*
 * linux/arch/mips/tx-boards/generic/setup_tx4927.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2007
 * All Rights Reserved.
 *
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/netdevice.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/spi/spi.h>
#include <asm/param.h>
#include <asm/ptrace.h>
#include <asm/reboot.h>
#include <asm/signal.h>
#include <asm/system.h>
#include <asm/time.h>
#include <asm/traps.h>
#include <asm/irq_regs.h>
#include <asm/tx-boards/generic.h>
#include <asm/tx-boards/irq.h>
#include <asm/tx-boards/dma.h>
#include <asm/tx-boards/pci.h>
#include <asm/tx-boards/pmon.h>
#include <asm/tx-boards/tx4927.h>
#include <asm/tx-boards/tx4938.h>
#include <asm/tx-boards/flash.h>
#include <asm/tx-boards/ndfmc.h>

extern void show_registers(struct pt_regs *regs);
static int tx4927_be_handler(struct pt_regs *regs, int is_fixup)
{
	extern void breakpoint(void);
	int data = regs->cp0_cause & 4;
	console_verbose();
	printk("%cBE exception at 0x%08lx\n",
	       data ? 'D' : 'I', regs->cp0_epc);
	printk("ccfg:%llx, toea:%llx\n",
	       (unsigned long long)__txx9_read64(&tx4927_ccfgptr->ccfg),
	       (unsigned long long)__txx9_read64(&tx4927_ccfgptr->toea));
	tx4927_report_pcic_status();
	show_registers(regs);
	printk("BusError!\n");
#ifdef CONFIG_KGDB
	breakpoint();
#endif
	pmon_halt();	/* never return */
}
void __init tx4927_be_init(void) /* export for setup_tx4939.c */
{
	board_be_handler = tx4927_be_handler;
}

/* default restart routine */
void tx4927_machine_restart(char *command) /* export for setup_tx4939.c */
{
	local_irq_disable();
	printk("Rebooting (with %s watchdog reset)...",
	       (__txx9_read64(&tx4927_ccfgptr->ccfg) & TX4927_CCFG_WDREXEN) ? "external" : "internal");
	/* clear watchdog status */
	tx4927_ccfg_set(TX4927_CCFG_WDRST);	/* W1C */
	/* disable watch dog timer */
	tx4927_tmrptr(2)->wtmr = TXx9_TMWTMR_WDIS | TXx9_TMWTMR_TWC;
	tx4927_tmrptr(2)->tcr = 0;
	/* kick watchdog */
	tx4927_tmrptr(2)->wtmr = TXx9_TMWTMR_TWIE;
	tx4927_tmrptr(2)->cpra = 1; /* immediate */
	tx4927_tmrptr(2)->tcr =
		TXx9_TMTCR_TCE | TXx9_TMTCR_CCDE | TXx9_TMTCR_TMODE_WDOG;
	while (!(__txx9_read64(&tx4927_ccfgptr->ccfg) & TX4927_CCFG_WDRST))
		;
	mdelay(10);
	if (__txx9_read64(&tx4927_ccfgptr->ccfg) & TX4927_CCFG_WDREXEN) {
		printk("failed.\nRebooting (with internal watchdog reset)...");
		/* External WDRST failed.  Do internal watchdog reset */
		tx4927_ccfg_clear(TX4927_CCFG_WDREXEN);
	}
	while(1);
}

void __init tx4927_wdr_init(void)
{
	/* report watchdog reset status */
	if (__txx9_read64(&tx4927_ccfgptr->ccfg) & TX4927_CCFG_WDRST) {
		unsigned long errepc = read_c0_errorepc();
		/* 0x200 will be enough... */
		if (errepc < (unsigned long)tx4927_machine_restart ||
		    errepc >= (unsigned long)tx4927_machine_restart + 0x200)
			printk(KERN_WARNING
			       "Watchdog reset detected at 0x%08lx\n",
			       errepc);
	}
	/* clear WatchDogReset (W1C) */
	tx4927_ccfg_set(TX4927_CCFG_WDRST);
	/* do reset on watchdog */
	tx4927_ccfg_set(TX4927_CCFG_WR);
}

static int __init tx4927_wdrex_setup(char *str)
{
	printk(KERN_DEBUG "%s: external WDRST: ", txx9_pcode_str);
	if (!strcmp(str, "1") || !strcmp(str, "on")) {
		tx4927_ccfg_set(TX4927_CCFG_WDREXEN);
		printk("on\n");
	} else {
		tx4927_ccfg_clear(TX4927_CCFG_WDREXEN);
		printk("off\n");
	}

	return 1;
}
__setup("wdrex=", tx4927_wdrex_setup);

static struct resource tx4927_sdram_resource[4];
struct resource tx4927_sram_resource;	/* export for setup_tx4939.c */

#define TX4938_SRAM_SIZE 0x800

static struct sysdev_class tx4927_sysdev_class;

void __init tx4927_setup(void)
{
	int i;
	__u32 divmode;
	int cpuclk = 0;
	extern int mips_config_cwfon;

	txx9_reg_res_init(TX4927_REV_PCODE(), TX4927_REG_BASE, TX4927_REG_SIZE);
	strcpy(tx4927_sysdev_class.kset.kobj.name, txx9_pcode_str);

	if (mips_config_cwfon)
		set_c0_config(TX49_CONF_CWFON);
	else
		clear_c0_config(TX49_CONF_CWFON);

	/* SDRAMC,EBUSC are configured by PROM */
	for (i = 0; i < 8; i++) {
		if (!(tx4927_ebuscptr->cr[i] & 0x8))
			continue;	/* disabled */
		tx_ce_res[i].start = (unsigned long)TX4927_EBUSC_BA(i);
		tx_ce_res[i].end = tx_ce_res[i].start + TX4927_EBUSC_SIZE(i) - 1;
		request_resource(&iomem_resource, &tx_ce_res[i]);
	}

	/* clocks */
	if (txx9_master_clock) {
		/* calculate gbus_clock and cpu_clock from master_clock */
		if ((txx9_pcode & 0xfff0) == 0x4920) {
			divmode = (__u32)__txx9_read64(&tx4927_ccfgptr->ccfg) & TX4927_CCFG_DIVMODE_MASK;
			switch (divmode) {
			case TX4927_CCFG_DIVMODE_8:
			case TX4927_CCFG_DIVMODE_10:
			case TX4927_CCFG_DIVMODE_12:
			case TX4927_CCFG_DIVMODE_16:
				txx9_gbus_clock = txx9_master_clock * 4; break;
			default:
				txx9_gbus_clock = txx9_master_clock;
			}
			switch (divmode) {
			case TX4927_CCFG_DIVMODE_2:
			case TX4927_CCFG_DIVMODE_8:
				cpuclk = txx9_gbus_clock * 2; break;
			case TX4927_CCFG_DIVMODE_2_5:
			case TX4927_CCFG_DIVMODE_10:
				cpuclk = txx9_gbus_clock * 5 / 2; break;
			case TX4927_CCFG_DIVMODE_3:
			case TX4927_CCFG_DIVMODE_12:
				cpuclk = txx9_gbus_clock * 3; break;
			case TX4927_CCFG_DIVMODE_4:
			case TX4927_CCFG_DIVMODE_16:
				cpuclk = txx9_gbus_clock * 4; break;
			}
		} else {
			divmode = (__u32)__txx9_read64(&tx4938_ccfgptr->ccfg) & TX4938_CCFG_DIVMODE_MASK;
			switch (divmode) {
			case TX4938_CCFG_DIVMODE_8:
			case TX4938_CCFG_DIVMODE_10:
			case TX4938_CCFG_DIVMODE_12:
			case TX4938_CCFG_DIVMODE_16:
			case TX4938_CCFG_DIVMODE_18:
				txx9_gbus_clock = txx9_master_clock * 4; break;
			default:
				txx9_gbus_clock = txx9_master_clock;
			}
			switch (divmode) {
			case TX4938_CCFG_DIVMODE_2:
			case TX4938_CCFG_DIVMODE_8:
				cpuclk = txx9_gbus_clock * 2; break;
			case TX4938_CCFG_DIVMODE_2_5:
			case TX4938_CCFG_DIVMODE_10:
				cpuclk = txx9_gbus_clock * 5 / 2; break;
			case TX4938_CCFG_DIVMODE_3:
			case TX4938_CCFG_DIVMODE_12:
				cpuclk = txx9_gbus_clock * 3; break;
			case TX4938_CCFG_DIVMODE_4:
			case TX4938_CCFG_DIVMODE_16:
				cpuclk = txx9_gbus_clock * 4; break;
			case TX4938_CCFG_DIVMODE_4_5:
			case TX4938_CCFG_DIVMODE_18:
				cpuclk = txx9_gbus_clock * 9 / 2; break;
			}
		}
		txx9_cpu_clock = cpuclk;
	} else {
		if (txx9_cpu_clock == 0) {
			/* use PMON's clkfreq variable */
			txx9_cpu_clock = get_pmon_clkfreq();
		}
		if (txx9_cpu_clock == 0) {
			if ((txx9_pcode & 0xfff0) == 0x4920)
				txx9_cpu_clock = 200000000;	/* 200MHz */
			else
				txx9_cpu_clock = 300000000;	/* 300MHz */
		}
		/* calculate gbus_clock and master_clock from cpu_clock */
		cpuclk = txx9_cpu_clock;
		if ((txx9_pcode & 0xfff0) == 0x4920) {
			divmode = (__u32)__txx9_read64(&tx4927_ccfgptr->ccfg) & TX4927_CCFG_DIVMODE_MASK;
			switch (divmode) {
			case TX4927_CCFG_DIVMODE_2:
			case TX4927_CCFG_DIVMODE_8:
				txx9_gbus_clock = cpuclk / 2; break;
			case TX4927_CCFG_DIVMODE_2_5:
			case TX4927_CCFG_DIVMODE_10:
				txx9_gbus_clock = cpuclk * 2 / 5; break;
			case TX4927_CCFG_DIVMODE_3:
			case TX4927_CCFG_DIVMODE_12:
				txx9_gbus_clock = cpuclk / 3; break;
			case TX4927_CCFG_DIVMODE_4:
			case TX4927_CCFG_DIVMODE_16:
				txx9_gbus_clock = cpuclk / 4; break;
			}
			switch (divmode) {
			case TX4927_CCFG_DIVMODE_8:
			case TX4927_CCFG_DIVMODE_10:
			case TX4927_CCFG_DIVMODE_12:
			case TX4927_CCFG_DIVMODE_16:
				txx9_master_clock = txx9_gbus_clock / 4; break;
			default:
				txx9_master_clock = txx9_gbus_clock;
			}
		} else {
			divmode = (__u32)__txx9_read64(&tx4938_ccfgptr->ccfg) & TX4938_CCFG_DIVMODE_MASK;
			switch (divmode) {
			case TX4938_CCFG_DIVMODE_2:
			case TX4938_CCFG_DIVMODE_8:
				txx9_gbus_clock = cpuclk / 2; break;
			case TX4938_CCFG_DIVMODE_2_5:
			case TX4938_CCFG_DIVMODE_10:
				txx9_gbus_clock = cpuclk * 2 / 5; break;
			case TX4938_CCFG_DIVMODE_3:
			case TX4938_CCFG_DIVMODE_12:
				txx9_gbus_clock = cpuclk / 3; break;
			case TX4938_CCFG_DIVMODE_4:
			case TX4938_CCFG_DIVMODE_16:
				txx9_gbus_clock = cpuclk / 4; break;
			case TX4938_CCFG_DIVMODE_4_5:
			case TX4938_CCFG_DIVMODE_18:
				txx9_gbus_clock = cpuclk * 2 / 9; break;
			}
			switch (divmode) {
			case TX4938_CCFG_DIVMODE_8:
			case TX4938_CCFG_DIVMODE_10:
			case TX4938_CCFG_DIVMODE_12:
			case TX4938_CCFG_DIVMODE_16:
			case TX4938_CCFG_DIVMODE_18:
				txx9_master_clock = txx9_gbus_clock / 4; break;
			default:
				txx9_master_clock = txx9_gbus_clock;
			}
		}
	}
	/* change default value to udelay/mdelay take reasonable time */
	loops_per_jiffy = txx9_cpu_clock / HZ / 2;

	/* CCFG */
	tx4927_wdr_init();
	/* clear BusErrorOnWrite flag (W1C) */
	tx4927_ccfg_set(TX4927_CCFG_BEOW);
	/* enable Timeout BusError */
	if (tx_ccfg_toeon)
		tx4927_ccfg_set(TX4927_CCFG_TOE);

	/* DMA selection */
	if ((txx9_pcode & 0xfff0) == 0x4920)
		txx9_clear64(&tx4927_ccfgptr->pcfg, TX4927_PCFG_DMASEL_ALL);
	else
		txx9_clear64(&tx4938_ccfgptr->pcfg, TX4938_PCFG_DMASEL_ALL);

	/* Use external clock for external arbiter */
	if (!(__txx9_read64(&tx4927_ccfgptr->ccfg) & TX4927_CCFG_PCIARB))
		txx9_clear64(&tx4927_ccfgptr->pcfg, TX4927_PCFG_PCICLKEN_ALL);

	printk("%s -- %dMHz(M%dMHz) CRIR:%08x CCFG:%llx PCFG:%llx\n",
	       txx9_pcode_str,
	       (cpuclk + 500000) / 1000000,
	       (txx9_master_clock + 500000) / 1000000,
	       (__u32)__txx9_read64(&tx4927_ccfgptr->crir),
	       (unsigned long long)__txx9_read64(&tx4927_ccfgptr->ccfg),
	       (unsigned long long)__txx9_read64(&tx4927_ccfgptr->pcfg));

	printk("%s SDRAMC --", txx9_pcode_str);
	for (i = 0; i < 4; i++) {
		__u64 cr = tx4927_sdramcptr->cr[i];
		unsigned long base, size;
		if (!((__u32)cr & 0x00000400))
			continue;	/* disabled */
		base = (unsigned long)(cr >> 49) << 21;
		size = (((unsigned long)(cr >> 33) & 0x7fff) + 1) << 21;
		printk(" CR%d:%016llx", i, (unsigned long long)cr);
		tx4927_sdram_resource[i].name = "SDRAM";
		tx4927_sdram_resource[i].start = base;
		tx4927_sdram_resource[i].end = base + size - 1;
		tx4927_sdram_resource[i].flags = IORESOURCE_MEM;
		request_resource(&iomem_resource, &tx4927_sdram_resource[i]);
	}
	printk(" TR:%09llx\n", (unsigned long long)tx4927_sdramcptr->tr);

	/* SRAM */
	if (txx9_pcode == 0x4938 && tx4938_sramcptr->cr & 1) {
		unsigned int size = TX4938_SRAM_SIZE;
		tx4927_sram_resource.name = "SRAM";
		tx4927_sram_resource.start =
			(tx4938_sramcptr->cr >> (39-11)) & ~(size - 1);
		tx4927_sram_resource.end =
			tx4927_sram_resource.start + TX4938_SRAM_SIZE - 1;
		tx4927_sram_resource.flags = IORESOURCE_MEM;
		request_resource(&iomem_resource, &tx4927_sram_resource);
	}

	/* IRC */
	/* disable interrupt control */
	tx4927_ircptr->cer = 0;

	/* TMR */
	/* disable all timers */
	for (i = 0; i < TX4927_NR_TMR; i++) {
		tx4927_tmrptr(i)->tcr = TXx9_TMTCR_CRE;
		tx4927_tmrptr(i)->tisr = 0;
		tx4927_tmrptr(i)->cpra = 0xffffffff;
		tx4927_tmrptr(i)->itmr = 0;
		tx4927_tmrptr(i)->ccdr = 0;
		tx4927_tmrptr(i)->pgmr = 0;
	}

	/* DMA */
	if ((txx9_pcode & 0xfff0) == 0x4920)
		tx4927_dma_init();
	else
		tx4938_dma_init();

	/* PIO */
	tx4927_pioptr->maskcpu = 0;
	tx4927_pioptr->maskext = 0;

	if (txx9_pcode == 0x4938) {
		__u64 pcfg = __txx9_read64(&tx4938_ccfgptr->pcfg);
		/* set PCIC1 reset */
		txx9_set64(&tx4938_ccfgptr->clkctr, TX4938_CLKCTR_PCIC1RST);
		if (pcfg & (TX4938_PCFG_ETH0_SEL | TX4938_PCFG_ETH1_SEL)) {
			mdelay(1);	/* at least 128 cpu clock */
			/* clear PCIC1 reset */
			txx9_clear64(&tx4938_ccfgptr->clkctr, TX4938_CLKCTR_PCIC1RST);
		} else {
			printk("%s: stop PCIC1\n", txx9_pcode_str);
			/* stop PCIC1 */
			txx9_set64(&tx4938_ccfgptr->clkctr,
				   TX4938_CLKCTR_PCIC1CKD);
		}
		if (!(pcfg & TX4938_PCFG_ETH0_SEL)) {
			printk("%s: stop ETH0\n", txx9_pcode_str);
			txx9_set64(&tx4938_ccfgptr->clkctr,
				   TX4938_CLKCTR_ETH0RST);
			txx9_set64(&tx4938_ccfgptr->clkctr,
				   TX4938_CLKCTR_ETH0CKD);
		}
		if (!(pcfg & TX4938_PCFG_ETH1_SEL)) {
			printk("%s: stop ETH1\n", txx9_pcode_str);
			txx9_set64(&tx4938_ccfgptr->clkctr,
				   TX4938_CLKCTR_ETH1RST);
			txx9_set64(&tx4938_ccfgptr->clkctr,
				   TX4938_CLKCTR_ETH1CKD);
		}
	}

	board_be_init = tx4927_be_init;
	_machine_restart = tx4927_machine_restart;
}

void __init tx4927_time_init(unsigned int cpu_irq_base, unsigned int tmrnr)
{
	/* We use onchip r4k counter or TMR timer as our system wide timer
	 * interrupt running at HZ. */
	if (tx4927_ccfgptr->ccfg & TX4927_CCFG_TINTDIS) {
		printk("Tick timer: use %s timer%d interrupt.\n",
		       txx9_pcode_str, tmrnr);
		/* use R4k timer as HPT and TXX9 timer as timer IRQ source */
		txx9_time_init(tx4927_tmrptr(tmrnr));
		txboard_timer_irqno = txx9_irq_to_irq(TX4927_IR_TMR(tmrnr));
	} else {
		printk("Tick timer: use r4k counter interrupt.\n");
		/* we are using the cpu counter for timer interrupts */
		mips_hpt_frequency = txx9_cpu_clock / 2;
		txboard_timer_irqno = cpu_irq_base + 7;
	}
}

static void __init tx4927_stop_unused_modules(void)
{
	__u64 pcfg, rst = 0, ckd = 0;
	unsigned long flags;
	char buf[128];
	buf[0] = '\0';
	local_irq_save(flags);
	pcfg = __txx9_read64(&tx4927_ccfgptr->pcfg);
	switch (txx9_pcode) {
	case 0x4927:
	case 0x4937:
		if (!(pcfg & TX4927_PCFG_SEL2)) {
			rst |= TX4927_CLKCTR_ACLRST;
			ckd |= TX4927_CLKCTR_ACLCKD;
			strcat(buf, " ACLC");
		}
		break;
	case 0x4938:
		if (!(pcfg & TX4938_PCFG_SEL2) ||
		    (pcfg & TX4938_PCFG_ETH0_SEL)) {
			rst |= TX4938_CLKCTR_ACLRST;
			ckd |= TX4938_CLKCTR_ACLCKD;
			strcat(buf, " ACLC");
		}
		if ((pcfg &
		     (TX4938_PCFG_ATA_SEL|TX4938_PCFG_ISA_SEL|TX4938_PCFG_NDF_SEL))
		    != TX4938_PCFG_NDF_SEL) {
			rst |= TX4938_CLKCTR_NDFRST;
			ckd |= TX4938_CLKCTR_NDFCKD;
			strcat(buf, " NDFMC");
		}
		if (!(pcfg & TX4938_PCFG_SPI_SEL)) {
			rst |= TX4938_CLKCTR_SPIRST;
			ckd |= TX4938_CLKCTR_SPICKD;
			strcat(buf, " SPI");
		}
		break;
	}
	if (rst | ckd) {
		txx9_set64(&tx4927_ccfgptr->clkctr, rst);
		txx9_set64(&tx4927_ccfgptr->clkctr, ckd);
	}
	local_irq_restore(flags);
	if (buf[0])
		printk(KERN_INFO "%s: stop%s\n", txx9_pcode_str, buf);
}

#ifdef CONFIG_PCI
 /* export for setup_tx4939.c */
irqreturn_t tx4927_pcierr_interrupt(int irq, void *dev_id)
{
	struct pt_regs *regs = get_irq_regs();
	if (txboard_pci_err_action != TXBOARD_PCI_ERR_IGNORE) {
		printk(KERN_WARNING "PCIERR interrupt at 0x%08lx.\n", regs->cp0_epc);
		tx4927_report_pcic_status();
	}
	if (txboard_pci_err_action != TXBOARD_PCI_ERR_PANIC) {
		struct tx4927_pcic_reg *pcicptr = tx4927_pcicptr;
		/* clear all pci errors */
		pcicptr->pcistatus =
			(pcicptr->pcistatus & 0x0000ffff) |
			(TX4927_PCIC_PCISTATUS_ALL << 16);
		pcicptr->g2pstatus = TX4927_PCIC_G2PSTATUS_ALL;
		pcicptr->pbastatus = TX4927_PCIC_PBASTATUS_ALL;
		pcicptr->pcicstatus = TX4927_PCIC_PCICSTATUS_ALL;
		return IRQ_HANDLED;
	}
	console_verbose();
	printk("ccfg:%llx, toea:%llx\n",
	       (unsigned long long)__txx9_read64(&tx4927_ccfgptr->ccfg),
	       (unsigned long long)__txx9_read64(&tx4927_ccfgptr->toea));
	show_registers(regs);
	tx4927_dump_pcic_settings();
	txboard_dump_pci_config();
	panic("PCI error.");
}

void __init tx4927_setup_pcierr_irq(void)
{
	request_irq(txx9_irq_to_irq(TX4927_IR_PCIERR), tx4927_pcierr_interrupt,
		    IRQF_DISABLED, "PCI error", NULL);
}
#endif /* CONFIG_PCI */

void __init tx4927_setup_serial(unsigned int sclk, int irq_base, unsigned int cts_mask, unsigned int ch_mask)
{
#ifdef CONFIG_SERIAL_TXX9
	int i;
	struct uart_port req;
	if ((txx9_pcode & 0xfff0) == 0x4930 &&
	    (__txx9_read64(&tx4938_ccfgptr->pcfg) & TX4938_PCFG_ETH0_SEL))
		ch_mask |= 1 << 1; /* disable SIO1 by PCFG setting */
	for(i = 0; i < 2; i++) {
		if ((1 << i) & ch_mask)
			continue;
		memset(&req, 0, sizeof(req));
		req.line = i;
		req.iotype = UPIO_MEM;
		req.membase = (__force unsigned char __iomem *)TX4927_SIO_REG(i);
		req.mapbase = TX4927_SIO_REG(i) & 0xfffffffffULL;
		req.irq = irq_base + TX4927_IR_SIO(i);
		if (!((1 << i) & cts_mask))
			req.flags |= UPF_TXX9_HAVE_CTS_LINE;
		if (sclk) {
			req.flags |= UPF_TXX9_USE_SCLK;
			req.uartclk = sclk;
		} else {
			req.uartclk = TXX9_IMCLK;
		}
		early_serial_txx9_setup(&req);
		early_serial_txx9_kgdb_setup(&req);
	}
#endif /* CONFIG_SERIAL_TXX9 */
}

void __init tx4927_mtd_setup(int ch)
{
	if (!(tx4927_ebuscptr->cr[ch] & 0x8))
		return;	/* disabled */
	register_txboard_flash(ch, tx_ce_res[ch].start,
			       tx_ce_res[ch].end - tx_ce_res[ch].start + 1,
			       TX4927_EBUSC_WIDTH(ch) / 8, &tx_ce_res[ch]);
}

#if defined(CONFIG_TC35815_1) || defined(CONFIG_TC35815_1_MODULE)
static unsigned char tx4938_ethaddr[2][6];
void __init tx4938_init_ethaddr(int ch, const unsigned char *addr)
{
	memcpy(tx4938_ethaddr[ch], addr, 6);
}
/* Get ethaddr from GEMINI-style SEEPROM */
void __init tx4938_init_ethaddr_by_seeprom(int busid, int chipid)
{
	unsigned char dat[17];
	int i;
	unsigned char sum;
	/* 0-3: "MAC\0", 4-9:eth0, 10-15:eth1, 16:sum */
	if (spi_eeprom_read(busid, chipid, 0, dat, sizeof(dat))) {
		printk(KERN_ERR "seeprom: read error.\n");
		memset(dat, 0xff, sizeof(dat));
	} else {
		if (strcmp(dat, "MAC") != 0)
			printk(KERN_WARNING "seeprom: bad signature.\n");
		for (i = 0, sum = 0; i < sizeof(dat); i++)
			sum += dat[i];
		if (sum)
			printk(KERN_WARNING "seeprom: bad checksum.\n");
	}
	tx4938_init_ethaddr(0, &dat[4]);
	tx4938_init_ethaddr(1, &dat[4 + 6]);
}
static int tx4938_netdev_event(struct notifier_block *this,
			       unsigned long event,
			       void *ptr)
{
	struct net_device *dev = (struct net_device *)ptr;
	if (event == NETDEV_REGISTER) {
		int ch = -1;
		if (dev->irq == txx9_irq_to_irq(TX4938_IR_ETH0))
			ch = 0;
		else if (dev->irq == txx9_irq_to_irq(TX4938_IR_ETH1))
			ch = 1;
		if (ch >= 0) {
			memcpy(dev->dev_addr, tx4938_ethaddr[ch], 6);
		}
	}
	return NOTIFY_DONE;
}

static struct notifier_block tx4938_netdev_notifier = {
	.notifier_call = tx4938_netdev_event,
};

void __init tx4938_setup_eth(void)
{
	if (tx4938_ccfgptr->pcfg & (TX4938_PCFG_ETH0_SEL|TX4938_PCFG_ETH1_SEL))
		register_netdevice_notifier(&tx4938_netdev_notifier);
}
#else
void __init tx4938_init_ethaddr(int ch, const unsigned char *addr) {}
void __init tx4938_init_ethaddr_by_seeprom(int busid, int chipid) {}
void __init tx4938_setup_eth(void) {}
#endif

#ifdef CONFIG_SPI
/* default cs_func for txx9spi */
static int tx4938_spi_cs_func(struct spi_device *spi, int on)
{
	unsigned int bit = spi->chip_select;
	unsigned long flags;
	if (spi->mode & SPI_CS_HIGH)
		on = !on;
	spin_lock_irqsave(&txx9_pio_lock, flags);
	if (on)
		tx4938_pioptr->dout &= ~(1 << bit);
	else
		tx4938_pioptr->dout |= (1 << bit);
	iob();
	spin_unlock_irqrestore(&txx9_pio_lock, flags);
	return 0;
}
void __init tx4938_spi_setup(int busid,
			     int (*cs_func)(struct spi_device *spi, int on))
{
	if (!cs_func)
		cs_func = tx4938_spi_cs_func;
	txx9_spi_init(busid, TX4938_SPI_REG & 0xfffffffffULL,
		      txx9_irq_to_irq(TX4938_IR_SPI), cs_func);
}
#else
void __init tx4938_spi_setup(int busid,
			     int (*cs_func)(struct spi_device *spi, int on)) {}
#endif

void __init tx4938_setup_ata(unsigned int irq, unsigned int gap, int tune)
{
#if defined(CONFIG_BLK_DEV_IDE_TX4938) || defined(CONFIG_BLK_DEV_IDE_TX4938_MODULE)
	if ((tx4938_ccfgptr->pcfg & (TX4938_PCFG_ATA_SEL | TX4938_PCFG_NDF_SEL))
	    == TX4938_PCFG_ATA_SEL) {
		struct resource res[] = {
			{ .start = irq, .flags = IORESOURCE_IRQ, },
			{ .start = gap, .flags = IORESOURCE_IRQ, },
			{ .start = tune, .flags = IORESOURCE_IRQ, },
		};
		platform_device_register_simple("tx4938ide", -1,
						res, ARRAY_SIZE(res));
	}
#endif
}

void __init tx4938_setup_ndfmc(unsigned int hold, unsigned int spw, int wp)
{
	struct ndfmc_platform_data plat_data = {
#ifdef __BIG_ENDIAN
		.addr = TX4938_NDFMC_REG + 4,
#else
		.addr = TX4938_NDFMC_REG,
#endif
		.shift = 1,
		.hold = hold,
		.spw = spw,
		.ch_mask = 1,
		.wp_mask = !!wp,
		.dmanr = -1,
	};
	if ((tx4938_ccfgptr->pcfg &
	     (TX4938_PCFG_ATA_SEL|TX4938_PCFG_ISA_SEL|TX4938_PCFG_NDF_SEL)) ==
	    TX4938_PCFG_NDF_SEL)
		register_txboard_nand(&plat_data);
}

static struct sysdev_class tx4927sramc_sysdev_class;
static struct sys_device device_tx4927sramc = {
	.cls	= &tx4927sramc_sysdev_class,
};

static ssize_t tx4927_sram_read(struct kobject *kobj, char *buf,
				loff_t pos, size_t size)
{
	unsigned int ramsize =
		tx4927_sram_resource.end - tx4927_sram_resource.start + 1;
	phys_t base = tx4927_sram_resource.start;
	void __iomem *p;

	if (pos >= ramsize)
		return 0;
	if (pos + size > ramsize)
		size = ramsize - pos;
	p = ioremap(base, ramsize);
	if (!p)
		return 0;
	memcpy_fromio(buf, p + pos, size);
	iounmap(p);
	return size;
}

static ssize_t tx4927_sram_write(struct kobject *kobj, char *buf,
				 loff_t pos, size_t size)
{
	unsigned int ramsize =
		tx4927_sram_resource.end - tx4927_sram_resource.start + 1;
	phys_t base = tx4927_sram_resource.start;
	void __iomem *p;

	if (pos >= ramsize)
		return 0;
	if (pos + size > ramsize)
		size = ramsize - pos;
	p = ioremap(base, size);
	if (!p)
		return 0;
	memcpy_toio(p + pos, buf, size);
	iounmap(p);
	return size;
}

static struct bin_attribute sramc_bindata_attr = {
	.attr = {
		.name = "bindata",
		.mode = 0600,
		.owner = THIS_MODULE,
	},
	.read = tx4927_sram_read,
	.write = tx4927_sram_write,
};

static int __init tx4927sramc_init_sysdev(void)
{
	int error;

	if (!tx4927_sram_resource.start)
		return -ENODEV;
	sramc_bindata_attr.size =
		tx4927_sram_resource.end - tx4927_sram_resource.start + 1;
	sprintf(tx4927sramc_sysdev_class.kset.kobj.name,
		"%ssramc", txx9_pcode_str);
	error = sysdev_class_register(&tx4927sramc_sysdev_class);
	if (!error) {
		error = sysdev_register(&device_tx4927sramc);
		if (!error)
			error = sysfs_create_bin_file(&device_tx4927sramc.kobj,
						      &sramc_bindata_attr);
	}
	return error;
}

#ifdef CONFIG_PM
struct tx4927_sysdev_state {
	struct tx4927_dma_state dma_state[2];
	struct txx9_tmr_state tmr_state[TX4927_NR_TMR];
	struct txx9_pio_state pio_state;
	unsigned long long ebccr[8];
	unsigned long long pcfg;
	unsigned long long clkctr;
};
static void tx4927_sysdev_save_state(struct tx4927_sysdev_state *state)
{
	int i;

	if (txx9_pcode == 0x4927)
		tx4927_dma_save_state(&state->dma_state[0], tx4927_dmaptr);
	else {
		for (i = 0; i < ARRAY_SIZE(state->dma_state); i++)
			tx4927_dma_save_state(&state->dma_state[i],
					      tx4938_dmaptr(i));
	}
	for (i = 0; i < ARRAY_SIZE(state->tmr_state); i++)
		txx9_tmr_save_state(&state->tmr_state[i], tx4927_tmrptr(i));
	txx9_pio_save_state(&state->pio_state, tx4927_pioptr);
	for (i = 0; i < ARRAY_SIZE(state->ebccr); i++)
		state->ebccr[i] = __txx9_read64(&tx4927_ebuscptr->cr[i]);
	state->pcfg = __txx9_read64(&tx4927_ccfgptr->pcfg);
	state->clkctr = __txx9_read64(&tx4927_ccfgptr->clkctr);
}

static void tx4927_sysdev_restore_state(struct tx4927_sysdev_state *state)
{
	int i;

	__txx9_write64(state->clkctr, &tx4927_ccfgptr->clkctr);
	__txx9_write64(state->pcfg, &tx4927_ccfgptr->pcfg);
	for (i = 0; i < ARRAY_SIZE(state->ebccr); i++)
		__txx9_write64(state->ebccr[i], &tx4927_ebuscptr->cr[i]);
	txx9_pio_restore_state(&state->pio_state, tx4927_pioptr);
	for (i = 0; i < ARRAY_SIZE(state->tmr_state); i++)
		txx9_tmr_restore_state(&state->tmr_state[i], tx4927_tmrptr(i));
	if (txx9_pcode == 0x4927)
		tx4927_dma_restore_state(&state->dma_state[0], tx4927_dmaptr);
	else {
		for (i = 0; i < ARRAY_SIZE(state->dma_state); i++)
			tx4927_dma_restore_state(&state->dma_state[i],
						 tx4938_dmaptr(i));
	}
}

static struct tx4927_sysdev_state tx4927_sysdev_state;
static int tx4927_sysdev_suspend(struct sys_device *dev, pm_message_t state)
{
	tx4927_sysdev_save_state(&tx4927_sysdev_state);
	return 0;
}
static int tx4927_sysdev_resume(struct sys_device *dev)
{
	tx4927_sysdev_restore_state(&tx4927_sysdev_state);
	return 0;
}
#endif

static struct sys_device tx4927_sysdev = {
	.cls = &tx4927_sysdev_class,
};

static int tx4927_init_sysdev(void)
{
	int error;

#ifdef CONFIG_PM
	tx4927_sysdev_class.suspend = tx4927_sysdev_suspend;
	tx4927_sysdev_class.resume = tx4927_sysdev_resume;
#endif
	error = sysdev_class_register(&tx4927_sysdev_class);
	if (!error)
		error = sysdev_register(&tx4927_sysdev);
	return error;
}

static int __init tx4927_init_sysfs(void)
{
	if (tx4927_sysdev_class.kset.kobj.name[0])
		tx4927_init_sysdev();
	tx4927sramc_init_sysdev();
	return 0;
}
subsys_initcall(tx4927_init_sysfs);

static int __init tx4927_late_init_sysfs(void)
{
	if (tx4927_sysdev_class.kset.kobj.name[0])
		tx4927_stop_unused_modules();
	return 0;
}
late_initcall(tx4927_late_init_sysfs);
