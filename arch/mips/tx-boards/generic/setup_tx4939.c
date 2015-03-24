/*
 * linux/arch/mips/tx-boards/generic/setup_tx4939.c
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
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/netdevice.h>
#include <linux/notifier.h>
#include <linux/ethtool.h>
#include <linux/rtc.h>
#include <asm/param.h>
#include <asm/reboot.h>
#include <asm/signal.h>
#include <asm/time.h>
#include <asm/traps.h>
#include <asm/tx-boards/generic.h>
#include <asm/tx-boards/irq.h>
#include <asm/tx-boards/dma.h>
#include <asm/tx-boards/pci.h>
#include <asm/tx-boards/pmon.h>
#include <asm/tx-boards/tx4939.h>
#include <asm/tx-boards/ndfmc.h>

extern void tx4927_be_init(void);
extern void tx4927_machine_restart(char *command);
extern void tx4927_wdr_init(void);

static struct resource tx4939_sdram_resource[4];
extern struct resource tx4927_sram_resource;
#define tx4939_sram_resource tx4927_sram_resource
#define TX4939_SRAM_SIZE 0x800

static struct sysdev_class tx4939_sysdev_class;

void __init tx4939_setup(void)
{
	int i;
	__u32 divmode;
	__u64 pcfg;
	int cpuclk = 0;
	extern int mips_config_cwfon;

	txx9_reg_res_init(TX4939_REV_PCODE(), TX4939_REG_BASE, TX4939_REG_SIZE);
	strcpy(tx4939_sysdev_class.kset.kobj.name, txx9_pcode_str);

	if (mips_config_cwfon)
		set_c0_config(TX49_CONF_CWFON);
	else
		clear_c0_config(TX49_CONF_CWFON);

	/* SDRAMC,EBUSC are configured by PROM */
	for (i = 0; i < 4; i++) {
		if (!(tx4939_ebuscptr->cr[i] & 0x8))
			continue;	/* disabled */
		tx_ce_res[i].start = (unsigned long)TX4927_EBUSC_BA(i);
		tx_ce_res[i].end = tx_ce_res[i].start + TX4927_EBUSC_SIZE(i) - 1;
		request_resource(&iomem_resource, &tx_ce_res[i]);
	}

	/* clocks */
	if (txx9_master_clock) {
		/* calculate cpu_clock from master_clock */
		divmode = (__u32)__txx9_read64(&tx4939_ccfgptr->ccfg) & TX4939_CCFG_MULCLK_MASK;
		cpuclk = txx9_master_clock * 20 / 2;
		switch (divmode) {
		case TX4939_CCFG_MULCLK_8:
			cpuclk = cpuclk / 3 * 4 /* / 6 *  8 */; break;
		case TX4939_CCFG_MULCLK_9:
			cpuclk = cpuclk / 2 * 3 /* / 6 *  9 */; break;
		case TX4939_CCFG_MULCLK_10:
			cpuclk = cpuclk / 3 * 5 /* / 6 * 10 */; break;
		case TX4939_CCFG_MULCLK_11:
			cpuclk = cpuclk / 6 * 11; break;
		case TX4939_CCFG_MULCLK_12:
			cpuclk = cpuclk * 2 /* / 6 * 12 */; break;
		case TX4939_CCFG_MULCLK_13:
			cpuclk = cpuclk / 6 * 13; break;
		case TX4939_CCFG_MULCLK_14:
			cpuclk = cpuclk / 3 * 7 /* / 6 * 14 */; break;
		case TX4939_CCFG_MULCLK_15:
			cpuclk = cpuclk / 2 * 5 /* / 6 * 15 */; break;
		}
		txx9_cpu_clock = cpuclk;
	} else {
		if (txx9_cpu_clock == 0) {
			/* use PMON's clkfreq variable */
			txx9_cpu_clock = get_pmon_clkfreq();
		}
		if (txx9_cpu_clock == 0) {
			txx9_cpu_clock = 400000000;	/* 400MHz */
		}
		/* calculate master_clock from cpu_clock */
		cpuclk = txx9_cpu_clock;
		divmode = (__u32)__txx9_read64(&tx4939_ccfgptr->ccfg) & TX4939_CCFG_MULCLK_MASK;
		switch (divmode) {
		case TX4939_CCFG_MULCLK_8:
			txx9_master_clock = cpuclk * 6 / 8; break;
		case TX4939_CCFG_MULCLK_9:
			txx9_master_clock = cpuclk * 6 / 9; break;
		case TX4939_CCFG_MULCLK_10:
			txx9_master_clock = cpuclk * 6 / 10; break;
		case TX4939_CCFG_MULCLK_11:
			txx9_master_clock = cpuclk * 6 / 11; break;
		case TX4939_CCFG_MULCLK_12:
			txx9_master_clock = cpuclk * 6 / 12; break;
		case TX4939_CCFG_MULCLK_13:
			txx9_master_clock = cpuclk * 6 / 13; break;
		case TX4939_CCFG_MULCLK_14:
			txx9_master_clock = cpuclk * 6 / 14; break;
		case TX4939_CCFG_MULCLK_15:
			txx9_master_clock = cpuclk * 6 / 15; break;
		}
		txx9_master_clock /= 10; /* * 2 / 20 */
	}
	/* calculate gbus_clock from cpu_clock */
	divmode = (__u32)__txx9_read64(&tx4939_ccfgptr->ccfg) & TX4939_CCFG_YDIVMODE_MASK;
	txx9_gbus_clock = txx9_cpu_clock;
	switch (divmode) {
	case TX4939_CCFG_YDIVMODE_2:
		txx9_gbus_clock /= 2; break;
	case TX4939_CCFG_YDIVMODE_3:
		txx9_gbus_clock /= 3; break;
	case TX4939_CCFG_YDIVMODE_5:
		txx9_gbus_clock /= 5; break;
	case TX4939_CCFG_YDIVMODE_6:
		txx9_gbus_clock /= 6; break;
	}
	/* change default value to udelay/mdelay take reasonable time */
	loops_per_jiffy = txx9_cpu_clock / HZ / 2;

	/* CCFG */
	tx4927_wdr_init();
	/* clear BusErrorOnWrite flag (W1C) */
	tx4939_ccfg_set(TX4939_CCFG_WDRST | TX4939_CCFG_BEOW);
	/* enable Timeout BusError */
	if (tx_ccfg_toeon)
		tx4939_ccfg_set(TX4939_CCFG_TOE);

	/* DMA selection */
	txx9_clear64(&tx4939_ccfgptr->pcfg, TX4939_PCFG_DMASEL_ALL);

	/* Use external clock for external arbiter */
	if (!(__txx9_read64(&tx4939_ccfgptr->ccfg) & TX4939_CCFG_PCIARB))
		txx9_clear64(&tx4939_ccfgptr->pcfg, TX4939_PCFG_PCICLKEN_ALL);

	printk("%s -- %dMHz(M%dMHz,G%dMHz) CRIR:%08x CCFG:%llx PCFG:%llx\n",
	       txx9_pcode_str,
	       (cpuclk + 500000) / 1000000,
	       (txx9_master_clock + 500000) / 1000000,
	       (txx9_gbus_clock + 500000) / 1000000,
	       (__u32)__txx9_read64(&tx4939_ccfgptr->crir),
	       (unsigned long long)__txx9_read64(&tx4939_ccfgptr->ccfg),
	       (unsigned long long)__txx9_read64(&tx4939_ccfgptr->pcfg));

	printk("%s DDRC -- EN:%08x", txx9_pcode_str,
	       (__u32)__txx9_read64(&tx4939_ddrcptr->winen));
	for (i = 0; i < 4; i++) {
		__u64 win = __txx9_read64(&tx4939_ddrcptr->win[i]);
		if (!((__u32)__txx9_read64(&tx4939_ddrcptr->winen) & (1 << i)))
			continue;	/* disabled */
		printk(" #%d:%016llx", i, (unsigned long long)win);
		tx4939_sdram_resource[i].name = "DDR SDRAM";
		tx4939_sdram_resource[i].start =
			(unsigned long)(win >> 48) << 20;
		tx4939_sdram_resource[i].end =
			((((unsigned long)(win >> 32) & 0xffff) + 1) << 20) - 1;
		tx4939_sdram_resource[i].flags = IORESOURCE_MEM;
		request_resource(&iomem_resource, &tx4939_sdram_resource[i]);
	}
	printk("\n");

	/* SRAM */
	if (tx4939_sramcptr->cr & 1) {
		unsigned int size = TX4939_SRAM_SIZE;
		tx4939_sram_resource.name = "SRAM";
		tx4939_sram_resource.start =
			(tx4939_sramcptr->cr >> (39-11)) & ~(size - 1);
		tx4939_sram_resource.end =
			tx4939_sram_resource.start + TX4939_SRAM_SIZE - 1;
		tx4939_sram_resource.flags = IORESOURCE_MEM;
		request_resource(&iomem_resource, &tx4939_sram_resource);
	}

	/* IRC */
	/* disable interrupt control */
	tx4939_ircptr->den.r = 0;
	tx4939_ircptr->maskint.r = 0;
	tx4939_ircptr->maskext.r = 0;

	/* TMR */
	/* disable all timers */
	for (i = 0; i < TX4939_NR_TMR; i++) {
		tx4939_tmrptr(i)->tcr = TXx9_TMTCR_CRE;
		tx4939_tmrptr(i)->tisr = 0;
		tx4939_tmrptr(i)->cpra = 0xffffffff;
		tx4939_tmrptr(i)->itmr = 0;
		tx4939_tmrptr(i)->ccdr = 0;
		tx4939_tmrptr(i)->pgmr = 0;
	}

	/* DMA */
	tx4939_dma_init();

	/* set PCIC1 reset (required to prevent hangup on BIST) */
	txx9_set64(&tx4939_ccfgptr->clkctr, TX4939_CLKCTR_PCI1RST);
	pcfg = __txx9_read64(&tx4939_ccfgptr->pcfg);
	if (pcfg & (TX4939_PCFG_ET0MODE | TX4939_PCFG_ET1MODE)) {
		mdelay(1);	/* at least 128 cpu clock */
		/* clear PCIC1 reset */
		txx9_clear64(&tx4939_ccfgptr->clkctr, TX4939_CLKCTR_PCI1RST);
	} else {
		printk("%s: stop PCIC1\n", txx9_pcode_str);
		/* stop PCIC1 */
		txx9_set64(&tx4939_ccfgptr->clkctr, TX4939_CLKCTR_PCI1CKD);
	}
	if (!(pcfg & TX4939_PCFG_ET0MODE)) {
		printk("%s: stop ETH0\n", txx9_pcode_str);
		txx9_set64(&tx4939_ccfgptr->clkctr, TX4939_CLKCTR_ETH0RST);
		txx9_set64(&tx4939_ccfgptr->clkctr, TX4939_CLKCTR_ETH0CKD);
	}
	if (!(pcfg & TX4939_PCFG_ET1MODE)) {
		printk("%s: stop ETH1\n", txx9_pcode_str);
		txx9_set64(&tx4939_ccfgptr->clkctr, TX4939_CLKCTR_ETH1RST);
		txx9_set64(&tx4939_ccfgptr->clkctr, TX4939_CLKCTR_ETH1CKD);
	}

	board_be_init = tx4927_be_init;
	_machine_restart = tx4927_machine_restart;
}

/* default time_init routine */
void __init tx4939_time_init(unsigned int cpu_irq_base, unsigned int tmrnr)
{
	/* We use onchip r4k counter or TMR timer as our system wide timer
	 * interrupt running at HZ. */
	if (__txx9_read64(&tx4939_ccfgptr->ccfg) & TX4939_CCFG_TINTDIS) {
		printk("Tick timer: use %s timer%d interrupt.\n",
		       txx9_pcode_str, tmrnr);
		/* use R4k timer as HPT and TXX9 timer as timer IRQ source */
		txx9_time_init(tx4939_tmrptr(tmrnr));
		txboard_timer_irqno = tx4939_irq_to_irq(TX4939_IR_TMR(tmrnr));
	} else {
		printk("Tick timer: use r4k counter interrupt.\n");
		/* we are using the cpu counter for timer interrupts */
		mips_hpt_frequency = txx9_cpu_clock / 2;
		txboard_timer_irqno = cpu_irq_base + 7;
	}
}

static void __init tx4939_stop_unused_modules(void)
{
	__u64 pcfg, rst = 0, ckd = 0;
	unsigned long flags;
	char buf[128];
	buf[0] = '\0';
	local_irq_save(flags);
	pcfg = __txx9_read64(&tx4939_ccfgptr->pcfg);
	if ((pcfg & TX4939_PCFG_I2SMODE_MASK) !=
	    TX4939_PCFG_I2SMODE_ACLC) {
		rst |= TX4939_CLKCTR_ACLRST;
		ckd |= TX4939_CLKCTR_ACLCKD;
		strcat(buf, " ACLC");
	}
	if ((pcfg & TX4939_PCFG_I2SMODE_MASK) !=
	    TX4939_PCFG_I2SMODE_I2S &&
	    (pcfg & TX4939_PCFG_I2SMODE_MASK) !=
	    TX4939_PCFG_I2SMODE_I2S_ALT) {
		rst |= TX4939_CLKCTR_I2SRST;
		ckd |= TX4939_CLKCTR_I2SCKD;
		strcat(buf, " I2S");
	}
	if (!(pcfg & TX4939_PCFG_ATA0MODE)) {
		rst |= TX4939_CLKCTR_ATA0RST;
		ckd |= TX4939_CLKCTR_ATA0CKD;
		strcat(buf, " ATA0");
	}
	if (!(pcfg & TX4939_PCFG_ATA1MODE)) {
		rst |= TX4939_CLKCTR_ATA1RST;
		ckd |= TX4939_CLKCTR_ATA1CKD;
		strcat(buf, " ATA1");
	}
	if (pcfg & TX4939_PCFG_SPIMODE) {
		rst |= TX4939_CLKCTR_SPIRST;
		ckd |= TX4939_CLKCTR_SPICKD;
		strcat(buf, " SPI");
	}
	if (!(pcfg & (TX4939_PCFG_VSSMODE | TX4939_PCFG_VPSMODE))) {
		rst |= TX4939_CLKCTR_VPCRST;
		ckd |= TX4939_CLKCTR_VPCCKD;
		strcat(buf, " VPC");
	}
	if (!pmon_vector) {
		/* Do not stop SIO2,SIO3 while PMON use them */
		if ((pcfg & TX4939_PCFG_SIO2MODE_MASK) !=
		    TX4939_PCFG_SIO2MODE_SIO2) {
			rst |= TX4939_CLKCTR_SIO2RST;
			ckd |= TX4939_CLKCTR_SIO2CKD;
			strcat(buf, " SIO2");
		}
		if (pcfg & TX4939_PCFG_SIO3MODE) {
			rst |= TX4939_CLKCTR_SIO3RST;
			ckd |= TX4939_CLKCTR_SIO3CKD;
			strcat(buf, " SIO3");
		}
	}
	if (rst | ckd) {
		txx9_set64(&tx4939_ccfgptr->clkctr, rst);
		txx9_set64(&tx4939_ccfgptr->clkctr, ckd);
	}
	local_irq_restore(flags);
	if (buf)
		printk(KERN_INFO "%s: stop%s\n", txx9_pcode_str, buf);
}

#ifdef CONFIG_PCI
extern irqreturn_t tx4927_pcierr_interrupt(int irq, void *dev_id);

void __init tx4939_setup_pcierr_irq(void)
{
	request_irq(tx4939_irq_to_irq(TX4939_IR_PCIERR),
		    tx4927_pcierr_interrupt,
		    IRQF_DISABLED, "PCI error", NULL);
}
#endif /* CONFIG_PCI */

void __init tx4939_setup_serial(unsigned int sclk, int irq_base, unsigned int cts_mask, unsigned int ch_mask)
{
#ifdef CONFIG_SERIAL_TXX9
	int i;
	struct uart_port req;
	__u64 pcfg = __txx9_read64(&tx4939_ccfgptr->pcfg);

	cts_mask |= ~1;	/* only SIO0 have RTS/CTS */
	if ((pcfg & TX4939_PCFG_SIO2MODE_MASK) != TX4939_PCFG_SIO2MODE_SIO0)
		cts_mask |= 1 << 0; /* disable SIO0 RTS/CTS by PCFG setting */
	if ((pcfg & TX4939_PCFG_SIO2MODE_MASK) != TX4939_PCFG_SIO2MODE_SIO2)
		ch_mask |= 1 << 2; /* disable SIO2 by PCFG setting */
	if (pcfg & TX4939_PCFG_SIO3MODE)
		ch_mask |= 1 << 3; /* disable SIO3 by PCFG setting */
	for(i = 0; i < 4; i++) {
		if ((1 << i) & ch_mask)
			continue;
		memset(&req, 0, sizeof(req));
		req.line = i;
		req.iotype = UPIO_MEM;
		req.membase = (__force unsigned char __iomem *)TX4939_SIO_REG(i);
		req.mapbase = TX4939_SIO_REG(i) & 0xfffffffffULL;
		req.irq = irq_base + TX4939_IR_SIO(i);
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

void __init tx4939_setup_ata(void)
{
#if defined(CONFIG_BLK_DEV_IDE_TX4939) || defined(CONFIG_BLK_DEV_IDE_TX4939_MODULE)
	__u64 pcfg = txx9_read64(&tx4939_ccfgptr->pcfg);
	if (pcfg & (TX4939_PCFG_ATA0MODE | TX4939_PCFG_ATA1MODE)) {
		struct resource res[2];
		int i;
		memset(res, 0, sizeof(res));
		for (i = 0; i < 2; i++) {
			if (i == 0 &&
			    !(pcfg & TX4939_PCFG_ATA0MODE))
				continue;
			if (i == 1 &&
			    (pcfg & (TX4939_PCFG_ATA1MODE | TX4939_PCFG_ET1MODE | TX4939_PCFG_ET0MODE)) !=
			    TX4939_PCFG_ATA1MODE)
				continue;
			res[0].start = TX4939_ATA_REG(i) & 0xfffffffffULL;
			res[0].end = res[0].start + 0x1000 - 1;
			res[0].flags = IORESOURCE_MEM;
			res[0].parent = &txx9_reg_res;
			res[1].start = tx4939_irq_to_irq(TX4939_IR_ATA(i));
			res[1].flags = IORESOURCE_IRQ;
			platform_device_register_simple("tx4939ide", i,
							res, ARRAY_SIZE(res));
		}
	}
#endif
}

#if defined(CONFIG_TC35815_1) || defined(CONFIG_TC35815_1_MODULE)
static unsigned char tx4939_ethaddr[2][6];
void __init tx4939_init_ethaddr(int ch, const unsigned char *addr)
{
	memcpy(tx4939_ethaddr[ch], addr, 6);
}
static int tx4939_get_eth_speed(struct net_device *dev)
{
	struct ethtool_cmd cmd = { ETHTOOL_GSET };
	int speed = 100;	/* default 100Mbps */
	int err;
	if (!dev->ethtool_ops || !dev->ethtool_ops->get_settings)
		return speed;
	err = dev->ethtool_ops->get_settings(dev, &cmd);
	if (err < 0)
		return speed;
	speed = cmd.speed == SPEED_100 ? 100 : 10;
	return speed;
}
static int tx4939_netdev_event(struct notifier_block *this,
			       unsigned long event,
			       void *ptr)
{
	struct net_device *dev = (struct net_device *)ptr;
	if (event == NETDEV_CHANGE && netif_carrier_ok(dev)) {
		__u64 bit = 0;
		if (dev->irq == tx4939_irq_to_irq(TX4939_IR_ETH(0)))
			bit = TX4939_PCFG_SPEED0;
		else if (dev->irq == tx4939_irq_to_irq(TX4939_IR_ETH(1)))
			bit = TX4939_PCFG_SPEED1;
		if (bit) {
			unsigned long flags;
			int speed = tx4939_get_eth_speed(dev);
			local_irq_save(flags);
			if (speed == 100)
				txx9_set64(&tx4939_ccfgptr->pcfg, bit);
			else
				txx9_clear64(&tx4939_ccfgptr->pcfg, bit);
			local_irq_restore(flags);
		}
	}
	if (event == NETDEV_REGISTER) {
		int ch = -1;
		if (dev->irq == tx4939_irq_to_irq(TX4939_IR_ETH(0)))
			ch = 0;
		else if (dev->irq == tx4939_irq_to_irq(TX4939_IR_ETH(1)))
			ch = 1;
		if (ch >= 0) {
			memcpy(dev->dev_addr, tx4939_ethaddr[ch], 6);
		}
	}
	return NOTIFY_DONE;
}

static struct notifier_block tx4939_netdev_notifier = {
	.notifier_call = tx4939_netdev_event,
	.priority = 1,
};

void __init tx4939_setup_eth(void)
{
	register_netdevice_notifier(&tx4939_netdev_notifier);
}
#else
void __init tx4939_setup_eth(void) {}
#endif

void __init tx4939_setup_ndfmc(unsigned int hold, unsigned int spw,
			       unsigned char wp_mask, unsigned char ch_mask,
			       unsigned char wide_mask)
{
	struct ndfmc_platform_data plat_data = {
		.addr = TX4939_NDFMC_REG,
		.shift = 1,
		.hold = hold,
		.spw = spw,
		.flags = NDFMC_PLAT_FLAG_NO_RSTR | NDFMC_PLAT_FLAG_NO_SPWADD,
		.ch_mask = ch_mask,
		.wp_mask = wp_mask,
		.wide_mask = wide_mask,
		.dmanr = -1,
	};
	register_txboard_nand(&plat_data);
}

#ifdef CONFIG_RTC_CLASS
static spinlock_t tx4939_rtc_lock = SPIN_LOCK_UNLOCKED;

static int tx4939_rtc_cmd(int cmd)
{
	int i = 0;

	tx4939_rtcptr->ctl = cmd;
	/* this might take 30us (next 32.768KHz clock) */
	while (tx4939_rtcptr->ctl & TX4939_RTCCTL_BUSY) {
		if (i++ > 200 * 30 * 3)
			return -EBUSY;
	}
	return 0;
}

static int
tx4939_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long sec;
	int i;
	unsigned char buf[6];
	int ret;

	rtc_tm_to_time(tm, &sec);
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = sec;
	buf[3] = sec >> 8;
	buf[4] = sec >> 16;
	buf[5] = sec >> 24;
	spin_lock_irq(&tx4939_rtc_lock);
	tx4939_rtcptr->adr = 0;
	for (i = 0; i < 6; i++)
		tx4939_rtcptr->dat = buf[i];
	ret = tx4939_rtc_cmd(TX4939_RTCCTL_COMMAND_SETTIME |
			     (tx4939_rtcptr->ctl & TX4939_RTCCTL_ALME));
	spin_unlock_irq(&tx4939_rtc_lock);
	return ret;
}

static int
tx4939_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	int i, ret;
	unsigned char buf[6];
	unsigned long sec;

	spin_lock_irq(&tx4939_rtc_lock);
	ret = tx4939_rtc_cmd(TX4939_RTCCTL_COMMAND_GETTIME |
			     (tx4939_rtcptr->ctl & TX4939_RTCCTL_ALME));
	if (ret) {
		spin_unlock_irq(&tx4939_rtc_lock);
		return ret;
	}
	tx4939_rtcptr->adr = 2;
	for (i = 2; i < 6; i++)
		buf[i] = tx4939_rtcptr->dat;
	spin_unlock_irq(&tx4939_rtc_lock);
	sec = (buf[5] << 24) | (buf[4] << 16) | (buf[3] << 8) | buf[2];
	rtc_time_to_tm(sec, tm);
	return 0;
}

static int
tx4939_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	unsigned long sec;
	int i;
	unsigned char buf[6];
	int ret;

	if (alrm->time.tm_sec < 0 ||
	    alrm->time.tm_min < 0 ||
	    alrm->time.tm_hour < 0 ||
	    alrm->time.tm_mday < 0 ||
	    alrm->time.tm_mon < 0 ||
	    alrm->time.tm_year < 0)
		return -EINVAL;
	rtc_tm_to_time(&alrm->time, &sec);
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = sec;
	buf[3] = sec >> 8;
	buf[4] = sec >> 16;
	buf[5] = sec >> 24;
	spin_lock_irq(&tx4939_rtc_lock);
	tx4939_rtcptr->adr = 0;
	for (i = 0; i < 6; i++)
		tx4939_rtcptr->dat = buf[i];
	ret = tx4939_rtc_cmd(TX4939_RTCCTL_COMMAND_SETALARM |
			     (alrm->enabled ? TX4939_RTCCTL_ALME : 0));
	spin_unlock_irq(&tx4939_rtc_lock);
	return ret;
}

static int
tx4939_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	int i, ret;
	unsigned char buf[6];
	unsigned long sec;

	spin_lock_irq(&tx4939_rtc_lock);
	ret = tx4939_rtc_cmd(TX4939_RTCCTL_COMMAND_GETALARM |
			     (tx4939_rtcptr->ctl & TX4939_RTCCTL_ALME));
	if (ret) {
		spin_unlock_irq(&tx4939_rtc_lock);
		return ret;
	}
	tx4939_rtcptr->adr = 2;
	for (i = 2; i < 6; i++)
		buf[i] = tx4939_rtcptr->dat;
	alrm->enabled = (tx4939_rtcptr->ctl & TX4939_RTCCTL_ALME) ? 1 : 0;
	alrm->pending = (tx4939_rtcptr->ctl & TX4939_RTCCTL_ALMD) ? 1 : 0;
	spin_unlock_irq(&tx4939_rtc_lock);
	sec = (buf[5] << 24) | (buf[4] << 16) | (buf[3] << 8) | buf[2];
	rtc_time_to_tm(sec, &alrm->time);
	return 0;
}

static irqreturn_t
tx4939_rtc_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	unsigned long events = RTC_IRQF;

	spin_lock(&tx4939_rtc_lock);
	if (tx4939_rtcptr->ctl & TX4939_RTCCTL_ALMD) {
		events |= RTC_AF;
		tx4939_rtc_cmd(TX4939_RTCCTL_COMMAND_NOP);
	}
	spin_unlock(&tx4939_rtc_lock);
	rtc_update_irq(&rtc->class_dev, 1, events);
	return IRQ_HANDLED;
}

static void
tx4939_rtc_release(struct device *dev)
{
	spin_lock_irq(&tx4939_rtc_lock);
	tx4939_rtc_cmd(TX4939_RTCCTL_COMMAND_NOP);
	spin_unlock_irq(&tx4939_rtc_lock);
}

static int
tx4939_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	struct rtc_time tm;
	struct rtc_wkalrm alarm;
	void __user *uarg = (void __user *) arg;

	switch (cmd) {
	case RTC_ALM_SET:
		if (copy_from_user(&alarm.time, uarg, sizeof(tm)))
			return -EFAULT;
		alarm.enabled = 0;
		alarm.pending = 0;
		/* keep all date/time in alarm.time */
		return rtc_set_alarm(&rtc->class_dev, &alarm);
	case RTC_AIE_OFF:
		spin_lock_irq(&tx4939_rtc_lock);
		tx4939_rtc_cmd(TX4939_RTCCTL_COMMAND_NOP);
		spin_unlock_irq(&tx4939_rtc_lock);
		break;
	case RTC_AIE_ON:
		spin_lock_irq(&tx4939_rtc_lock);
		tx4939_rtc_cmd(TX4939_RTCCTL_COMMAND_NOP |
			       TX4939_RTCCTL_ALME);
		spin_unlock_irq(&tx4939_rtc_lock);
		break;
	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

static const struct rtc_class_ops tx4939_rtc_ops = {
	.read_time	= tx4939_rtc_read_time,
	.set_time	= tx4939_rtc_set_time,
	.read_alarm	= tx4939_rtc_read_alarm,
	.set_alarm	= tx4939_rtc_set_alarm,
	.release	= tx4939_rtc_release,
	.ioctl		= tx4939_rtc_ioctl,
};

static ssize_t
tx4939_rtc_nvram_read(struct kobject *kobj, char *buf, loff_t pos, size_t size)
{
	ssize_t count;

	spin_lock_irq(&tx4939_rtc_lock);
	for (count = 0; size > 0 && pos < TX4939_RTC_REG_RAMSIZE;
	     count++, size--) {
		tx4939_rtcptr->adr = pos++;
		*buf++ = tx4939_rtcptr->dat;
	}
	spin_unlock_irq(&tx4939_rtc_lock);
	return count;
}

static ssize_t
tx4939_rtc_nvram_write(struct kobject *kobj, char *buf, loff_t pos, size_t size)
{
	ssize_t count;

	spin_lock_irq(&tx4939_rtc_lock);
	for (count = 0; size > 0 && pos < TX4939_RTC_REG_RAMSIZE;
	     count++, size--) {
		tx4939_rtcptr->adr = pos++;
		tx4939_rtcptr->dat = *buf++;
	}
	spin_unlock_irq(&tx4939_rtc_lock);
	return count;
}

static struct bin_attribute tx4939_rtc_nvram_attr = {
	.attr = {
		.name = "nvram",
		.mode = S_IRUGO | S_IWUGO,
		.owner = THIS_MODULE,
	},
	.size = TX4939_RTC_REG_RAMSIZE,
	.read = tx4939_rtc_nvram_read,
	.write = tx4939_rtc_nvram_write,
};

static int __init tx4939_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;
	int irq = tx4939_irq_to_irq(TX4939_IR_RTC);
	int ret;

	tx4939_rtc_cmd(TX4939_RTCCTL_COMMAND_NOP);
	if (request_irq(irq, tx4939_rtc_interrupt, IRQF_DISABLED | IRQF_SHARED,
			pdev->name, pdev) < 0)
		return -EBUSY;
	rtc = rtc_device_register(pdev->name, &pdev->dev,
				  &tx4939_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		free_irq(irq, pdev);
		return PTR_ERR(rtc);
	}
	platform_set_drvdata(pdev, rtc);
	ret = sysfs_create_bin_file(&pdev->dev.kobj, &tx4939_rtc_nvram_attr);
	if (ret) {
		rtc_device_unregister(rtc);
		free_irq(irq, pdev);
	}
	return ret;
}

static struct platform_driver tx4939_rtc_driver = {
	.probe		= tx4939_rtc_probe,
	.driver		= {
		.name	= "tx4939rtc",
		.owner	= THIS_MODULE,
	},
};

/* must be called after rtc class's initcall */
static int __init tx4939rtc_init(void)
{
	return platform_driver_register(&tx4939_rtc_driver);
}
device_initcall(tx4939rtc_init);
#endif /* CONFIG_RTC_CLASS */

#ifdef CONFIG_PM
struct tx4939_sysdev_state {
	struct tx4927_dma_state dma_state[2];
	struct txx9_tmr_state tmr_state[TX4939_NR_TMR];
	unsigned long long ebccr[4];
	unsigned long long pcfg;
	unsigned long long clkctr;
};
static void tx4939_sysdev_save_state(struct tx4939_sysdev_state *state)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(state->dma_state); i++)
		tx4927_dma_save_state(&state->dma_state[i], tx4939_dmaptr(i));
	for (i = 0; i < ARRAY_SIZE(state->tmr_state); i++)
		txx9_tmr_save_state(&state->tmr_state[i], tx4939_tmrptr(i));
	for (i = 0; i < ARRAY_SIZE(state->ebccr); i++)
		state->ebccr[i] = __txx9_read64(&tx4939_ebuscptr->cr[i]);
	state->pcfg = __txx9_read64(&tx4939_ccfgptr->pcfg);
	state->clkctr = __txx9_read64(&tx4939_ccfgptr->clkctr);
}

static void tx4939_sysdev_restore_state(struct tx4939_sysdev_state *state)
{
	int i;

	__txx9_write64(state->clkctr, &tx4939_ccfgptr->clkctr);
	__txx9_write64(state->pcfg, &tx4939_ccfgptr->pcfg);
	for (i = 0; i < ARRAY_SIZE(state->ebccr); i++)
		__txx9_write64(state->ebccr[i], &tx4939_ebuscptr->cr[i]);
	for (i = 0; i < ARRAY_SIZE(state->tmr_state); i++)
		txx9_tmr_restore_state(&state->tmr_state[i], tx4939_tmrptr(i));
	for (i = 0; i < ARRAY_SIZE(state->dma_state); i++)
		tx4927_dma_restore_state(&state->dma_state[i],
					 tx4939_dmaptr(i));
}

struct tx4939_sysdev_state tx4939_sysdev_state;
static int tx4939_sysdev_suspend(struct sys_device *dev, pm_message_t state)
{
	tx4939_sysdev_save_state(&tx4939_sysdev_state);
	return 0;
}
static int tx4939_sysdev_resume(struct sys_device *dev)
{
	tx4939_sysdev_restore_state(&tx4939_sysdev_state);
	return 0;
}
#endif

static struct sys_device tx4939_sysdev = {
	.cls	= &tx4939_sysdev_class,
};

static int tx4939_init_sysdev(void)
{
	int error;
#ifdef CONFIG_PM
	tx4939_sysdev_class.suspend = tx4939_sysdev_suspend;
	tx4939_sysdev_class.resume = tx4939_sysdev_resume;
#endif
	error = sysdev_class_register(&tx4939_sysdev_class);
	if (!error)
		error = sysdev_register(&tx4939_sysdev);
	return error;
}

static int __init tx4939_init_sysfs(void)
{
	if (tx4939_sysdev_class.kset.kobj.name[0])
		tx4939_init_sysdev();
	return 0;
}
subsys_initcall(tx4939_init_sysfs);

static int __init tx4939_late_init_sysfs(void)
{
	if (tx4939_sysdev_class.kset.kobj.name[0])
		tx4939_stop_unused_modules();
	return 0;
}
late_initcall(tx4939_late_init_sysfs);
