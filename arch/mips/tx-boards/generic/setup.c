/*
 * linux/arch/mips/tx-boards/generic/setup.c
 *
 * Setup pointers to hardware-dependent routines.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2007
 * All Rights Reserved.
 *
 */
#include <linux/console.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/ioport.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <asm/mipsregs.h>
#include <asm/bootinfo.h>
#include <asm/cpu.h>
#include <asm/reboot.h>
#include <asm/time.h>
#include <asm/fpu_emulator.h>
#include <asm/tx-boards/generic.h>
#include <asm/tx-boards/pmon.h>
#include <asm/tx-boards/flash.h>
#include <asm/tx-boards/ndfmc.h>
#include <asm/tx-boards/txx9.h>

static void __init txboard_cache_report(void)
{
	/* enable cache here */
	switch (current_cpu_data.processor_id & 0xff00) {
#ifdef CONFIG_CPU_TX49XX
	case PRID_IMP_TX49:	/* TX49 core */
	{
		unsigned int conf = read_c0_config();
		if (conf & TX49_CONF_IC)
			printk("TX49XX I-Cache disabled.\n");
		if (conf & TX49_CONF_DC)
			printk("TX49XX D-Cache disabled.\n");
		if (conf & TX49_CONF_CWFON)
			printk("TX49XX CWF enabled.\n");
		break;
	}
#endif
	}
}

static int panic_event(struct notifier_block *this, unsigned long event,
                      void *ptr)
{
	extern void breakpoint(void);
	/* called when panic happens. enter to monitor. */
#ifdef CONFIG_KGDB
	breakpoint();
#endif
	if (!panic_timeout)
		pmon_halt();
	return NOTIFY_DONE;
}

static struct notifier_block panic_block = {
	.notifier_call = panic_event,
};

static void txboard_machine_halt(void)
{
	local_irq_disable();
	clear_c0_status(ST0_IM);
	printk("Halted.\n");
	while (1) {
		if (cpu_wait) {
			(*cpu_wait)();
			if (cpu_has_counter) {
				/*
				 * Clear counter interrupt while it
				 * breaks WAIT instruction even if
				 * masked.
				 */
				write_c0_compare(0);
			}
		}
	}
}

int txboard_timer_irqno __initdata;
void __init plat_timer_setup(struct irqaction *irq)
{
	setup_irq(txboard_timer_irqno, irq);
}

/* EBUSC settings of TX4927, TX4925, etc. */
struct resource tx_ce_res[8];
EXPORT_SYMBOL(tx_ce_res);
static char tx_ce_res_name[8][4];	/* "CEn" */

void (*txboard_mach_setup)(void) __initdata;
void (*txboard_scache_setup)(void) __initdata;

void __init plat_mem_setup(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(tx_ce_res); i++) {
		sprintf(tx_ce_res_name[i], "CE%d", i);
		tx_ce_res[i].flags = IORESOURCE_MEM;
		tx_ce_res[i].name = tx_ce_res_name[i];
	}

	clear_c0_status(ST0_RE);

	atomic_notifier_chain_register(&panic_notifier_list, &panic_block);

	/* setup resource limits */
	ioport_resource.start = 0;
	ioport_resource.end = ~0UL;	/* no limit */
	iomem_resource.start = 0;
	iomem_resource.end = ~0UL;	/* no limit */

	_machine_restart = (void (*)(char *))txboard_machine_halt;
	_machine_halt = txboard_machine_halt;
	pm_power_off = txboard_machine_halt;

	if (!txboard_mach_setup)
		panic("Unsupported type");
	txboard_mach_setup();
	printk(KERN_INFO "IO port base: %08lx\n", mips_io_port_base);

	/* If we have the secondary cache on our cpu, we have to call setup function. */
	if (txboard_scache_setup)
		txboard_scache_setup();

	txboard_cache_report();
}

static ssize_t mathemu_show(struct sysdev_class *class, char *buf)
{
	char *p = buf;

	p += sprintf(p, "emulated : %u\n", fpuemustats.emulated);
	p += sprintf(p, "loads    : %u\n", fpuemustats.loads);
	p += sprintf(p, "stores   : %u\n", fpuemustats.stores);
	p += sprintf(p, "cp1ops   : %u\n", fpuemustats.cp1ops);
	p += sprintf(p, "cp1xops  : %u\n", fpuemustats.cp1xops);
	p += sprintf(p, "errors   : %u\n", fpuemustats.errors);
	return p - buf;
}

static SYSDEV_CLASS_ATTR(mathemu, 0444, mathemu_show, NULL)

static ssize_t kernel_show(struct sysdev_class *class, char *buf)
{
	extern unsigned long unaligned_instructions;
	return sprintf(buf, "unaligned : %lu\n", unaligned_instructions);
}

static SYSDEV_CLASS_ATTR(kernel, 0444, kernel_show, NULL)

static ssize_t pmon_bp_store(struct sysdev_class *class, const char *buf,
			     size_t size)
{
	pmon_breakpoint();
	return size;
}

static SYSDEV_CLASS_ATTR(pmon_bp, 0200, NULL, pmon_bp_store)

#define MAX_PMON_INTERPRET_OUTLEN	2048
static char *pmon_interpret_outbuf;
static int pmon_interpret_outlen;
static ssize_t pmon_interp_show(struct sysdev_class *class, char *buf)
{
	if (!pmon_interpret_outbuf)
		return 0;
	memcpy(buf, pmon_interpret_outbuf, pmon_interpret_outlen);
	return pmon_interpret_outlen;
}
static ssize_t pmon_interp_store(struct sysdev_class *class,
				 const char *buf, size_t size)
{
	if (!pmon_interpret_outbuf) {
		pmon_interpret_outbuf = kmalloc(MAX_PMON_INTERPRET_OUTLEN,
						GFP_KERNEL);
		if (!pmon_interpret_outbuf)
			return -ENOMEM;
	}
	pmon_interpret_outlen =
		pmon_interpret(buf, pmon_interpret_outbuf,
			       MAX_PMON_INTERPRET_OUTLEN);
	if (pmon_interpret_outlen < 0) {
		pmon_interpret_outlen = 0;
		return -ENODEV;
	}
	return size;
}

static SYSDEV_CLASS_ATTR(pmon_interp, 0600,
			 pmon_interp_show, pmon_interp_store);

static struct sysdev_class tx_sysdev_class = {
	set_kset_name("tx-board"),
};
static struct sys_device tx_sys_device = {
	.cls = &tx_sysdev_class,
};
int txboard_sysdev_create_file(struct sysdev_attribute *a)
{
	return sysdev_create_file(&tx_sys_device, a);
}
int txboard_sysdev_class_create_file(struct sysdev_class_attribute *a)
{
	return sysdev_class_create_file(&tx_sysdev_class, a);
}

/* board specific routines */
void (*txboard_device_init)(void) __initdata;
void (*txboard_arch_init)(void) __initdata;

static int __init
_txboard_arch_init(void)
{
	if (txboard_arch_init)
		txboard_arch_init();
	return 0;
}
arch_initcall(_txboard_arch_init);

static int __init
_txboard_device_init(void)
{
	if (sysdev_class_register(&tx_sysdev_class) == 0) {
		sysdev_register(&tx_sys_device);
		sysdev_class_create_file(&tx_sysdev_class, &attr_mathemu);
		sysdev_class_create_file(&tx_sysdev_class, &attr_kernel);
		sysdev_class_create_file(&tx_sysdev_class, &attr_pmon_bp);
		sysdev_class_create_file(&tx_sysdev_class, &attr_pmon_interp);
	}

	if (txboard_device_init)
		txboard_device_init();
	return 0;
}
device_initcall(_txboard_device_init);

int txboard_ob_ether_disable __initdata;
#ifdef CONFIG_NETDEVICES
static int __init ob_ether_setup(char *str)
{
	if (strcmp(str, "off") == 0)
		txboard_ob_ether_disable = 1;
	else if (strcmp(str, "on") == 0)
		txboard_ob_ether_disable = 0;
	else
		txboard_ob_ether_disable = (simple_strtoul(str, NULL, 0) == 0);
	return 1;
}
__setup("ob_ether=", ob_ether_setup);
#endif

/* pcode, internal register */
unsigned int txx9_pcode;
char txx9_pcode_str[8];
EXPORT_SYMBOL(txx9_pcode);
EXPORT_SYMBOL(txx9_pcode_str);
struct resource txx9_reg_res = {
	.name = txx9_pcode_str,
	.flags = IORESOURCE_MEM,
};
void __init
txx9_reg_res_init(unsigned int pcode, unsigned long base, unsigned long size)
{
	txx9_pcode = pcode;
	sprintf(txx9_pcode_str, "TX%x", pcode);
	if (base) {
		txx9_reg_res.start = base & 0xfffffffffULL;
		txx9_reg_res.end = (base & 0xfffffffffULL) + (size - 1);
		request_resource(&iomem_resource, &txx9_reg_res);
	}
}

/* clocks */
unsigned int txx9_master_clock;
unsigned int txx9_cpu_clock;
unsigned int txx9_gbus_clock;
EXPORT_SYMBOL(txx9_gbus_clock);

DEFINE_SPINLOCK(txx9_pio_lock);
EXPORT_SYMBOL(txx9_pio_lock);

struct platform_device * __init
register_txboard_flash(int no, unsigned long addr, unsigned long size,
		       int bankwidth, struct resource *parent)
{
#if defined(CONFIG_MTD_TXBOARD) || defined(CONFIG_MTD_TXBOARD_MODULE)
	struct {
		struct platform_device pdev;
		struct resource resource;
		struct flash_platform_data flash_data;
	} *pobj;
	int ret;
	pobj = kzalloc(sizeof(*pobj), GFP_KERNEL);
	if (!pobj)
		return ERR_PTR(-ENOMEM);
	pobj->pdev.name = "txflash";
	pobj->pdev.id = no;
	pobj->pdev.resource = &pobj->resource;
	pobj->pdev.num_resources = 1;
	pobj->pdev.dev.platform_data = &pobj->flash_data;
	pobj->resource.start = addr;
	pobj->resource.end = addr + size - 1;
	pobj->resource.flags = IORESOURCE_MEM;
	pobj->resource.parent = parent ? parent : &iomem_resource;
	pobj->flash_data.width = bankwidth;
	ret = platform_device_register(&pobj->pdev);
	if (ret) {
		kfree(pobj);
		return ERR_PTR(ret);
	}
	return &pobj->pdev;
#else
	return ERR_PTR(-ENODEV);
#endif
}

struct platform_device * __init
register_txboard_nand(const struct ndfmc_platform_data *plat_data)
{
#if defined(CONFIG_MTD_NAND_TXX9NDFMC) || defined(CONFIG_MTD_NAND_TXX9NDFMC_MODULE)
	struct {
		struct platform_device pdev;
		struct ndfmc_platform_data plat_data;
	} *pobj;
	int ret;
	pobj = kzalloc(sizeof(*pobj), GFP_KERNEL);
	if (!pobj)
		return ERR_PTR(-ENOMEM);
	pobj->pdev.name = "txx9ndfmc";
	pobj->pdev.id = -1;
	pobj->pdev.dev.platform_data = &pobj->plat_data;
	pobj->plat_data = *plat_data;
	ret = platform_device_register(&pobj->pdev);
	if (ret) {
		kfree(pobj);
		return ERR_PTR(ret);
	}
	return &pobj->pdev;
#else
	return ERR_PTR(-ENODEV);
#endif
}

void __init
register_txboard_led(unsigned long ioaddr, struct resource *parent,
		     unsigned int num, int lowactive)
{
#if defined(CONFIG_LEDS_TXBOARD) || defined(CONFIG_LEDS_TXBOARD_MODULE)
	struct resource res[] = {
		{
			.start	= ioaddr,
			.end	= ioaddr,
			.flags	= IORESOURCE_MEM,
			.parent	= parent,
		}, {
			.start	= num,
			.flags	= IORESOURCE_IRQ,
		}, {
			.start	= lowactive,
			.flags	= IORESOURCE_IRQ,
		},
	};
	platform_device_register_simple("tx-led", -1,
					res, ARRAY_SIZE(res));
#endif
}

void __init
register_rs5c348_info(u16 bus_num, u16 chip_select)
{
#if defined(CONFIG_RTC_DRV_RS5C348) || defined(CONFIG_RTC_DRV_RS5C348_MODULE)
	struct spi_board_info srtc_info = {
		.modalias = "rs5c348",
		.max_speed_hz = 1000000, /* 1.0Mbps @ Vdd 2.0V */
		.bus_num = bus_num,
		.chip_select = chip_select,
		/* Mode 1 (High-Active, Shift-Then-Sample), High Avtive CS  */
		.mode = SPI_MODE_1 | SPI_CS_HIGH,
	};
	spi_register_board_info(&srtc_info, 1);
#endif
}

void __init
register_dallas_rtc_device(char *name, unsigned long base, int irq)
{
#if defined(CONFIG_RTC_DRV_DS1742) || defined(CONFIG_RTC_DRV_DS1742_MODULE)
	unsigned int size = 0x800;
	struct resource res[] = {
		{
			.start	= base,
			.end	= base + size - 1,
			.flags	= IORESOURCE_MEM,
		}, {
			.start	= irq,
			.flags	= IORESOURCE_IRQ,
		},
	};
	platform_device_register_simple(name, -1,
					res, irq >= 0 ? 2 : 1);
#endif
}

void __init
register_txboard_obne_device(unsigned long ioaddr, int irq)
{
#ifdef CONFIG_TXBOARD_OBNE_NET
	if (!txboard_ob_ether_disable) {
		struct resource res[] = {
			{
				.start	= ioaddr,
				.end	= ioaddr + 0x20 - 1,
				.flags	= IORESOURCE_IO,
			}, {
				.start	= irq,
				.flags	= IORESOURCE_IRQ,
			}
		};
		platform_device_register_simple("obne", -1,
						res, ARRAY_SIZE(res));
	}
#endif
}

#ifdef CONFIG_CPU_HAS_OOO_MEMACCESS
#ifdef CONFIG_TOSHIBA_TC90412
#include <asm/tx-boards/tc90412-generic.h>
#endif
#ifdef CONFIG_TOSHIBA_TC90416
#include <asm/tx-boards/tc90416-generic.h>
#endif
void __wmb(void)
{
#ifdef CONFIG_TOSHIBA_TC90412
	tc90412_wmb();
#elif defined(CONFIG_TOSHIBA_TC90416)
	tc90416_wmb();
#else
	fast_wmb();
#endif
}
EXPORT_SYMBOL(__wmb);
void __rmb(void)
{
#ifdef CONFIG_TOSHIBA_TC90412
	tc90412_rmb();
#else
	fast_rmb();
#endif
}
EXPORT_SYMBOL(__rmb);
void __mb(void)
{
#ifdef CONFIG_TOSHIBA_TC90412
	tc90412_wmb();
	tc90412_rmb();
#elif defined(CONFIG_TOSHIBA_TC90416)
	tc90416_wmb();
#else
	fast_mb();
#endif
}
EXPORT_SYMBOL(__mb);
#endif /* CONFIG_CPU_HAS_OOO_MEMACCESS */

/*
  register platform resource
    device name: name + no.  ex. "foo0" if name="foo" & no=0.
    memory area: use parent if size equal 0.
    irq        : don't set if irq equal 0.
 */

struct platform_device * __init
register_platform_resource(int no, char* name, int irq, unsigned long addr,
		      unsigned long size)
{
	struct resource r[2];
	int num = 0;
	memset(r, 0, sizeof(r));
	r[num].flags = IORESOURCE_MEM;
	if(size){
		r[num].start = addr;
		r[num].end = addr + size - 1;
		num++;
	}
	if(irq){
		r[num].start = irq;
		r[num].flags = IORESOURCE_IRQ;
		num++;
	}
	if(!num)
		return ERR_PTR(-EINVAL);
	return platform_device_register_simple(name,no,r,num);
}
