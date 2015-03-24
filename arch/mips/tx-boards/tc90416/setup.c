/*
 * linux/arch/mips/tx-boards/tc90416/setup.c
 *
 * Setup pointers to hardware-dependent routines.
 *
 * based on arch/mips/tx-boards/tc90411/setup.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2005-2008
 * All Rights Reserved.
 *
 */
#include <linux/init.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <asm/bootinfo.h>
#include <asm/reboot.h>
#include <asm/time.h>
#include <asm/traps.h>
#include <asm/irq_regs.h>
#include <asm/tx-boards/generic.h>
#include <asm/tx-boards/pci.h>
#include <asm/tx-boards/pmon.h>
#include <asm/tx-boards/ndfmc.h>
#include <asm/tx-boards/dma.h>
#include <asm/tx-boards/tsb-generic.h>
#if defined(CONFIG_SERIAL_TXX9)
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#endif
#if defined(CONFIG_MTD_PHYSMAP_LEN) && CONFIG_MTD_PHYSMAP_LEN == 0
#include <linux/mtd/physmap.h>
#endif

extern void tc90416_irq_setup(void) __init;
extern void show_registers(struct pt_regs *regs);

static void __init tc90416_time_init(void)
{
#ifdef CONFIG_TICK_BY_TXTMR0
	printk("Tick timer: use TC90416 timer0 interrupt.\n");
	/* use R4k timer as HPT and TXX9 timer as timer IRQ source */
	txx9_time_init(tc90416_tmrptr(0));
	txboard_timer_irqno = txx9_64_irq_to_irq(TC90416_IR_TMR(0));
#else
	printk("Tick timer: use r4k counter interrupt.\n");
	/* we are using the cpu counter for timer interrupts */
	mips_hpt_frequency = txx9_cpu_clock / 2;
	txboard_timer_irqno = TC90416_IRQ_LOCAL + 7;
#endif
}

static int tc90416_be_handler(struct pt_regs *regs, int is_fixup)
{
	extern void breakpoint(void);
	int data = regs->cp0_cause & 4;
	console_verbose();
	printk("%cBE exception at 0x%08lx\n",
	       data ? 'D' : 'I', regs->cp0_epc);
	printk("ccfg:%llx, gbusea:0x%016llx\n",
	       tc90416_ccfgptr->ccfg, tc90416_ccfgptr->gbusea);
	show_registers(regs);
	printk("BusError!\n");
#ifdef CONFIG_KGDB
	breakpoint();
#endif
	pmon_halt();	/* never return */
}
static void __init tc90416_be_init(void)
{
	board_be_handler = tc90416_be_handler;
}

struct tc90416_serial_initsetting {
	signed int index;
	unsigned int uartclk;
	upf_t flags;
	unsigned int kdbg;
};
static void __init tc90416_setup_serial(int irq_base)
{
#ifdef CONFIG_SERIAL_TXX9
	int i;
	struct tc90416_serial_initsetting serial_setting[4];
	struct uart_port req;

	switch (mips_machtype) {
	case MACH_TC90416_PROTO1:
	case MACH_TC90417_PROTO1:
		{
		/*----- TC90416 Proto Board
		  UART 0: Debug Serial (ttyS0)
		  UART 1: Debug Serial (ttyS1)
		  UART 2: N.C.
		  UART 3: N.C.
		-------*/
		struct tc90416_serial_initsetting proto1_setting[4] =
			/* index, uartclk    , flags                 , kdbg */
			{ {  0  , TXX9_IMCLK , 0                     ,1  },
			  {  1  , TXX9_IMCLK , 0                     ,1  },
			  { -1  , 0          , 0                     ,0  },
			  { -1  , 0          , 0                     ,0  } };
		memcpy(serial_setting, proto1_setting, sizeof(proto1_setting));
		break;
		}
	case MACH_TC90417_PROTOJP:
	case MACH_TC90417_CELLTV1:
	case MACH_TC90417_JPTVH1:
	case MACH_TC90417_STBF1:
	case MACH_TC90417_STBR1:
	case MACH_TC90417_STBF2:
	case MACH_TC90417_STB2:
	case MACH_TC90417_JPTVH2:
		{
		/*----- TC90417 ProtoJP Board
		  UART 0: Debug Serial (ttyS0)
		  UART 1: Modem (ttyS1)
		  UART 2: CAS
		  UART 3: TV ucom  (uat_ucom)
		-------*/
		struct tc90416_serial_initsetting protojp_setting[4] =
			/* index, uartclk    , flags                 , kdbg */
			{ {  0  , TXX9_IMCLK , 0                     ,1  },
			  {  1  , TXX9_IMCLK , UPF_TXX9_HAVE_CTS_LINE,0  },
			  { -1  , 0          , 0                     ,0  },
			  { -1  , 0          , 0                     ,0  } };
		memcpy(serial_setting, protojp_setting, sizeof(protojp_setting));
		break;
		}
	case MACH_TC90417_STBR2:
	case MACH_TC90417_JPTVZR1:
		{
#if CONFIG_TOSHIBA_UART_TYPE==0
		/*        TYPE 0
		  UART 0: Debug Serial (ttyS0)
		  UART 1: Modem (ttyS1)
		  UART 2: CAS
		  UART 3: TV ucom  (uat_ucom)
		-------*/
		struct tc90416_serial_initsetting protojp_setting[4] =
			/* index, uartclk    , flags                 , kdbg */
			{ {  0  , TXX9_IMCLK , 0                     ,1  },
			  {  1  , TXX9_IMCLK , UPF_TXX9_HAVE_CTS_LINE,0  },
			  { -1  , 0          , 0                     ,0  },
			  { -1  , 0          , 0                     ,0  } };
#elif CONFIG_TOSHIBA_UART_TYPE==1
		/*        TYPE 0
		  UART 0: Debug Serial (ttyS0)
		  UART 1: CAS
		  UART 2: CAS
		  UART 3: TV ucom  (uat_ucom)
		-------*/
		struct tc90416_serial_initsetting protojp_setting[4] =
			/* index, uartclk    , flags                 , kdbg */
			{ {  0  , TXX9_IMCLK , 0                     ,1  },
			  { -1  , 0          , 0                     ,0  },
			  { -1  , 0          , 0                     ,0  },
			  { -1  , 0          , 0                     ,0  } };
#elif CONFIG_TOSHIBA_UART_TYPE==2
		/*        TYPE 0
		  UART 0: Debug Serial (ttyS0)
		  UART 1: Modem (ttyS1)
		  UART 2: CAS
		  UART 3: CAC
		-------*/
		struct tc90416_serial_initsetting protojp_setting[4] =
			/* index, uartclk    , flags                 , kdbg */
			{ {  0  , TXX9_IMCLK , 0                     ,1  },
			  {  1  , TXX9_IMCLK , UPF_TXX9_HAVE_CTS_LINE,0  },
			  { -1  , 0          , 0                     ,0  },
			  { -1  , 0          , 0                     ,0  } };
#else
#error "Unknown type. Support 0,1,2."
#endif
		memcpy(serial_setting, protojp_setting, sizeof(protojp_setting));
		break;
		}
	case MACH_TC90417_PROTOEU:
	case MACH_TC90417_JPTVZF1:
	case MACH_TC90417_JPTVAO1:
	case MACH_TC90417_CELLPN2:
	case MACH_TC90417_CELLTV2:
	case MACH_TC90417_CELLTV3:
		{
		/*----- TC90417 ProtoEU Board
		  UART 0: Debug Serial (ttyS0)
		  UART 1: N.C.
		  UART 2: N.C.
		  UART 3: TV ucom  (uat_ucom)
		-------*/
		struct tc90416_serial_initsetting protoeu_setting[4] =
			/* index, uartclk    , flags                 , kdbg */
			{ {  0  , TXX9_IMCLK , 0                     ,1  },
			  { -1  , 0          , 0                     ,0  },
			  { -1  , 0          , 0                     ,0  },
			  { -1  , 0          , 0                     ,0  } };
		memcpy(serial_setting, protoeu_setting, sizeof(protoeu_setting));
		break;
		}
	case MACH_TC90417_EUTV1:
	case MACH_TC90417_CELLPN1:
		{
		/*----- TC90417 EU Board
		  UART 0: Debug Serial (ttyS0)
		  UART 1: Vaddis (ttyS1)
		  UART 2: N.C.
		  UART 3: TV ucom  (uat_ucom)
		-------*/
		struct tc90416_serial_initsetting eu1_setting[4] =
			/* index, uartclk    , flags                 , kdbg */
			{ {  0  , TXX9_IMCLK , 0                     ,1  },
			  {  1  , TXX9_IMCLK , 0                     ,1  },
			  { -1  , 0          , 0                     ,0  },
			  { -1  , 0          , 0                     ,0  } };
		memcpy(serial_setting, eu1_setting, sizeof(eu1_setting));
		break;
		}
	}

	for(i = 0 ; i < 4 ; i++) {
		if (serial_setting[i].index >= 0) {
			memset(&req, 0, sizeof(req));

			req.line    = serial_setting[i].index;
			req.iotype  = UPIO_MEM;
			req.membase = (__force unsigned char __iomem *)TC90416_SIO_REG(i);
			req.mapbase = TC90416_SIO_REG(i);
			req.irq     = irq_base + TC90416_IR_SIO(i);
			req.uartclk = serial_setting[i].uartclk;
			req.flags  |= serial_setting[i].flags;

			early_serial_txx9_setup(&req);
#ifdef CONFIG_KGDB
			if (serial_setting[i].kdbg == 1)
				early_serial_txx9_kgdb_setup(&req);
#endif
		}
	}
#endif /* CONFIG_SERIAL_TXX9 */
}

#ifdef CONFIG_SPI
#include <linux/spi/spi.h>
static struct spi_board_info spi_info = {
	.modalias = "spioctl",
	.max_speed_hz = 2000000, /* 2.0Mbps */
	.bus_num = 0,
	.chip_select = 0,
	/* Mode 3 (CPHA=CPOL=1), Low Avtive CS */
	.mode = SPI_MODE_3,
};

#define SPI_CS_SELF_CONTROL

#define PIO_SPI_CS 59
static int tc90416_spi_cs_func(struct spi_device *spi, int on)
{
#ifdef SPI_CS_SELF_CONTROL
	int mask = 1 << (PIO_SPI_CS - 32);
	if(on)
		tc90416_pioptr->dout1r = mask; /* to low */
	else
		tc90416_pioptr->dout1s = mask; /* to high */
#endif
	return 0;
}

void __init tc90416_spi_setup(int busid,
			     int (*cs_func)(struct spi_device *spi, int on))
{
#ifdef SPI_CS_SELF_CONTROL
	tc90416_ccfgptr->pcfg &= ~0x0000200001000000ULL; /* SPI_CS = GPIO */
	tc90416_pioptr->dir1 |= 1 << (PIO_SPI_CS-32); /* SPI_CS to Output */
#endif
	if (!cs_func)
		cs_func = tc90416_spi_cs_func;
	txx9_spi_init(busid, TC90416_SPI_PHYS_REG,
		      txx9_64_irq_to_irq(TC90417_IR_SPI), cs_func);
}

#else
static inline void tc90416_spi_setup(void) {}
#endif

// for default setting ( when read maker code & device code )
#define TXX9_8BIT_NAND_RW_HOLD_TIME        15 // [ns] max number of Read / Write Hold Time of supported NANDs
#define TXX9_8BIT_NAND_RW_PLUS_WIDTH       35 // [ns] max number of Read / Write Pulse Width of supported NANDs
#define TXX9_8BIT_NAND_ALE_DELAY           50 // [ns] max number of Read / Write ALE Delay of supported NANDs
static void __init tc90416_setup_ndfmc(void)
{
	struct ndfmc_platform_data plat_data = {
#ifdef __BIG_ENDIAN
		.addr = TC90416_NDFMC_REG + 4,
#else
		.addr = TC90416_NDFMC_REG,
#endif
		.shift = 1,
		.hold = TXX9_8BIT_NAND_RW_HOLD_TIME,
		.spw = TXX9_8BIT_NAND_RW_PLUS_WIDTH,
		.aledelay = TXX9_8BIT_NAND_ALE_DELAY,
		.rrtime = 20,
		.flags = NDFMC_PLAT_FLAG_USE_ALEDELAY | NDFMC_PLAT_FLAG_USE_RRTIME,
		.ch_mask = 1,
		.dmanr = 1,
		.dmamode = TXx9_DMA_CCR_EXTRQ,
	};
	register_txboard_nand(&plat_data);
}

extern void __init
tx4927_dma_init_sub(int dmacno, struct tx4927_dma_reg *dmaptr, int dmairq);

static void __init
tc90416_arch_init(void)
{
}

#if defined(CONFIG_TXSSETH) || defined(CONFIG_TXSSETH_MODULE)
static struct resource txss_eth_resources[] = {
	{
		.start = TC90417_EMAC_PHYS_REG,
		.end = TC90417_EMAC_PHYS_REG+0x10000-1,
		.flags = IORESOURCE_MEM,
	},
	{
		.flags = IORESOURCE_IRQ,
	},
};
#endif

#if defined(CONFIG_MTD_PHYSMAP_LEN) && CONFIG_MTD_PHYSMAP_LEN == 0
static struct physmap_flash_data tc90416_flash_data = {
	.width		= 2,
};

static struct resource tc90416_flash_resource = {
	.flags		= IORESOURCE_MEM
};

static struct platform_device tc90416_flash = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
		.platform_data	= &tc90416_flash_data,
	},
	.num_resources	= 1,
	.resource	= &tc90416_flash_resource,
};

static void __init tc90416_setup_physmap(void)
{
	unsigned long addr;
	addr = (unsigned long)(tc90416_ebuscptr->ccr[0] >> 28);
	tc90416_flash_resource.start = addr;
	tc90416_flash_resource.end   = 0x20000000 - 1;
	platform_device_register(&tc90416_flash);
}
#endif

static void __init
tc90416_device_init(void)
{
	extern struct platform_device *register_platform_resource(int,char*,int,int,int);
#if defined(CONFIG_USB)
	/* USB */
	struct platform_device *pdev;
	static u64 dmamask = 0x000000001fffffffULL;
	(void)register_platform_resource(0,"tc90416-usbhcb", 0, TC90416_USBHC_PHYS_REG, 0x100);
	pdev = register_platform_resource(0,"tc90416-ohci", txx9_64_irq_to_irq(TC90416_IR_USBHC), TC90416_USBHC_PHYS_REG+0x1000, 0x100);
	if (pdev) {
		pdev->dev.dma_mask = &dmamask;
		pdev->dev.coherent_dma_mask = dmamask;
	}
	pdev = register_platform_resource(0,"tc90416-ehci", txx9_64_irq_to_irq(TC90416_IR_USBHC), TC90416_USBHC_PHYS_REG+0x1100, 0x100);
	if (pdev) {
		pdev->dev.dma_mask = &dmamask;
		pdev->dev.coherent_dma_mask = dmamask;
	}
#endif
#if defined(CONFIG_TOSHIBA_SDCARD_DRV) || defined(CONFIG_TOSHIBA_SDCARD_DRV_MODULE)
	/* SDHC for TC90416/TC90417 */
	{
		struct resource r[] = {
			{
				.start = TC90416_SDHC_PHYS_REG + 0x200,
				.end = TC90416_SDHC_PHYS_REG + 0x200 +
					0x200 - 1,
				.flags = IORESOURCE_MEM,
			}, {
				.start = txx9_64_irq_to_irq(TC90416_IR_SDHCCTR),
				.flags = IORESOURCE_IRQ,
			}, {
				.start = TC90416_SDHC_PHYS_REG,
				.end = TC90416_SDHC_PHYS_REG + 0x200 - 1,
				.flags = IORESOURCE_MEM,
#ifdef CONFIG_TOSHIBA_SDCARDCORE_TC904XX_DMA
			}, {
				.start = 2,	/* dma read */
				.flags = IORESOURCE_DMA,
			}, {
				.start = 3,	/* dma write */
				.flags = IORESOURCE_DMA,
#endif
			},
		};
		platform_device_register_simple("sdcard", 0, r, ARRAY_SIZE(r));
	}
	/* enable DMA synchronous read for reading into RAM */
	tc90416_ccfgptr->gbs2srtrg = 0x188030b800000000ULL; /* DMCSR2 */
	tc90416_ccfgptr->gbssren |= 1<<2; /* enable SREN2 */
#endif
#if defined(CONFIG_TXSSETH) || defined(CONFIG_TXSSETH_MODULE)
	if((tc90416_ccfgptr->revid >> 16) == 0x417){
		/* Gigabit Ether in TC90417 */
		txss_eth_resources[1].start = txx9_64_irq_to_irq(TC90417_IR_EMAC);
		platform_device_register_simple("txss-eth",0,txss_eth_resources,sizeof(txss_eth_resources)/sizeof(struct resource));
	}
#endif
	pmon_setup_default_etheraddr();
	tc90416_setup_ndfmc();
#ifdef CONFIG_SPI
	spi_register_board_info(&spi_info, 1);
	tc90416_spi_setup(0, 0);
#endif
#if defined(CONFIG_MTD_PHYSMAP_LEN) && CONFIG_MTD_PHYSMAP_LEN == 0
	tc90416_setup_physmap();
#endif
}

static struct sysdev_class tc90416_sysdev_class;

void __init tc90416_setup(void)
{
	int i;

	printk("%s\n", get_system_type());

	txx9_reg_res_init(TC90416_REV_PCODE(), TC90416_REG_PBASE, TC90416_REG_SIZE);
	strcpy(txx9_pcode_str, "tc90416");
	strcpy(tc90416_sysdev_class.kset.kobj.name, txx9_pcode_str);

	/* SDRAMC,EBUSC are configured by PROM */
	for (i = 0; i < 8; i++) {
		if (!(tc90416_ebuscptr->ccr[i] & 0x8))
			continue;	/* disabled */
		tx_ce_res[i].start = (tc90416_ebuscptr->ccr[i] >> 48) << 20;
		tx_ce_res[i].end = tx_ce_res[i].start +
			(0x00100000 << ((tc90416_ebuscptr->ccr[i] >> 8) & 0xf)) - 1;
		request_resource(&iomem_resource, &tx_ce_res[i]);
	}

	/* clocks */
	/* round master clock so that hpt_frequency is multiple of HZ */
	/* (i.e. cpu_clock should multiple of (HZ * 2)) */
	txx9_master_clock = ((txx9_master_clock / 2) / (HZ * 2))
			     * (HZ * 2) * 2;
	txx9_gbus_clock = (txx9_master_clock / 6);
	txx9_cpu_clock  = (txx9_master_clock / 2);

	/* change default value to udelay/mdelay take reasonable time */
	loops_per_jiffy = txx9_cpu_clock / HZ / 2;

	/* CCFG */
	/* enable Timeout BusError */
	if (tx_ccfg_toeon)
		tc90416_ccfgptr->ccfg |= TC90416_CCFG_TOE;

	printk("TC90%3x -- %dMHz(M%dMHz) REVID:%08x CCFG:%016llx\n",
	       (int)((tc90416_ccfgptr->revid >>16) & 0xfff),
	       (txx9_cpu_clock + 500000) / 1000000,
	       (txx9_master_clock + 500000) / 1000000,
	       tc90416_ccfgptr->revid, tc90416_ccfgptr->ccfg);

	/* IRC */
	/* disable interrupt control */
	tc90416_ircptr->cer = 0;

	/* TMR */
	/* disable all timers */
	for (i = 0; i < TC90416_NR_TMR; i++) {
		tc90416_tmrptr(i)->tcr = TXx9_TMTCR_CRE;
		tc90416_tmrptr(i)->tisr = 0;
		tc90416_tmrptr(i)->cpra = 0xffffffff;
		tc90416_tmrptr(i)->itmr = 0;
		tc90416_tmrptr(i)->ccdr = 0;
		tc90416_tmrptr(i)->pgmr = 0;
	}

	board_be_init = tc90416_be_init;

	tx4927_dma_init_sub(0, tc90416_gdmaptr, TC90416_IR_GDMA(0));
	for (i = 0; i < 4; i++)
		tc90416_gdmaptr->mcr |= TX4927_DMA_MCR_FIFUM(i);

	board_time_init = tc90416_time_init;
	txboard_irq_setup = tc90416_irq_setup;

	tc90416_setup_serial(TC90416_IRQ_IRC);

	txboard_arch_init = tc90416_arch_init;
	txboard_device_init = tc90416_device_init;
}

#ifdef CONFIG_SOFTWARE_SUSPEND
/*
 * tc904xx have its own method for suspend-to-mem.  Use this only if
 * suspend-to-disk enabled.  (i.e. use CONFIG_SOFTWARE_SUSPEND instead
 * of CONFIG_PM)
 */
struct tc90416_pio_state {
	u32 dout0;
	u32 dir0;
	u32 od0;
	u32 dout1;
	u32 dir1;
	u32 od1;
};

static inline void tc90416_pio_save_state(struct tc90416_pio_state *state,
					  struct tc90416_pio_reg *pioptr)
{
	state->dout0 = pioptr->dout0;
	state->dir0 = pioptr->dir0;
	state->od0 = pioptr->od0;
	state->dout1 = pioptr->dout1;
	state->dir1 = pioptr->dir1;
	state->od1 = pioptr->od1;
}

static inline void tc90416_pio_restore_state(struct tc90416_pio_state *state,
					     struct tc90416_pio_reg *pioptr)
{
	pioptr->dout0 = state->dout0;
	pioptr->od0 = state->od0;
	pioptr->dir0 = state->dir0;
	pioptr->dout1 = state->dout1;
	pioptr->od1 = state->od1;
	pioptr->dir1 = state->dir1;
}

struct tc90416_sysdev_state {
	struct tx4927_dma_state dma_state;
	struct txx9_tmr_state tmr_state[TC90416_NR_TMR];
	struct tc90416_pio_state pio_state;
	unsigned long long ebccr[8];
	unsigned long long ccfg;
	unsigned long long pcfg;
	unsigned long long clkrstctr;
	unsigned long long gbs2srtrg;
	unsigned long long gbssren;
};
static void tc90416_sysdev_save_state(struct tc90416_sysdev_state *state)
{
	int i;

	tx4927_dma_save_state(&state->dma_state, tc90416_gdmaptr);
	for (i = 0; i < ARRAY_SIZE(state->tmr_state); i++)
		txx9_tmr_save_state(&state->tmr_state[i], tc90416_tmrptr(i));
	tc90416_pio_save_state(&state->pio_state, tc90416_pioptr);
	for (i = 0; i < ARRAY_SIZE(state->ebccr); i++)
		state->ebccr[i] = tc90416_ebuscptr->ccr[i];
	state->gbssren = tc90416_ccfgptr->gbssren;
	state->gbs2srtrg = tc90416_ccfgptr->gbs2srtrg;
	state->ccfg = tc90416_ccfgptr->ccfg;
	state->pcfg = tc90416_ccfgptr->pcfg;
	state->clkrstctr = tc90416_ccfgptr->clkrstctr;
}

static void tc90416_sysdev_restore_state(struct tc90416_sysdev_state *state)
{
	int i;

	tc90416_ccfgptr->clkrstctr = state->clkrstctr;
	tc90416_ccfgptr->pcfg = state->pcfg;
	tc90416_ccfgptr->ccfg = state->ccfg;
	tc90416_ccfgptr->gbs2srtrg = state->gbs2srtrg;
	tc90416_ccfgptr->gbssren = state->gbssren;
	for (i = 0; i < ARRAY_SIZE(state->ebccr); i++) {
		/* restore EBCCR without ME bit */
		tc90416_ebuscptr->ccr[i] = state->ebccr[i] & ~0x8;
		/* restore EBCCR */
		tc90416_ebuscptr->ccr[i] = state->ebccr[i];
	}
	tc90416_pio_restore_state(&state->pio_state, tc90416_pioptr);
	for (i = 0; i < ARRAY_SIZE(state->tmr_state); i++)
		txx9_tmr_restore_state(&state->tmr_state[i], tc90416_tmrptr(i));
	tx4927_dma_restore_state(&state->dma_state, tc90416_gdmaptr);
}

static struct tc90416_sysdev_state tc90416_sysdev_state;
static int tc90416_sysdev_suspend(struct sys_device *dev, pm_message_t state)
{
	tc90416_sysdev_save_state(&tc90416_sysdev_state);
	return 0;
}
static int tc90416_sysdev_resume(struct sys_device *dev)
{
	tc90416_sysdev_restore_state(&tc90416_sysdev_state);
	return 0;
}
#else
#define tc90416_sysdev_suspend	NULL
#define tc90416_sysdev_resume	NULL
#endif

static struct sys_device tc90416_sysdev = {
	.cls = &tc90416_sysdev_class,
};

static int tc90416_init_sysdev(void)
{
	int error;

	tc90416_sysdev_class.suspend = tc90416_sysdev_suspend;
	tc90416_sysdev_class.resume = tc90416_sysdev_resume;
	error = sysdev_class_register(&tc90416_sysdev_class);
	if (!error)
		error = sysdev_register(&tc90416_sysdev);
	return error;
}

static int __init tc90416_init_sysfs(void)
{
	if (tc90416_sysdev_class.kset.kobj.name[0])
		tc90416_init_sysdev();
	return 0;
}
subsys_initcall(tc90416_init_sysfs);
