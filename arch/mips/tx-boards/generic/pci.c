/*
 * linux/arch/mips/tx-boards/generic/pci.c
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
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <asm/cpu.h>
#include <asm/mipsregs.h>
#include <asm/io.h>
#include <asm/tx-boards/generic.h>
#include <asm/tx-boards/pci.h>
#include <asm/tx-boards/irq.h>

static int __init
early_read_config_word(struct pci_controller *hose,
		       int top_bus, int bus, int devfn, int offset, u16 *value)
{
	struct pci_dev fake_dev;
	struct pci_bus fake_bus;

	fake_dev.bus = &fake_bus;
	fake_dev.sysdata = hose;
	fake_dev.devfn = devfn;
	fake_bus.number = bus;
	fake_bus.sysdata = hose;
	fake_bus.ops = hose->pci_ops;

	if(bus != top_bus)
		/* Fake a parent bus structure. */
		fake_bus.parent = &fake_bus;
	else
		fake_bus.parent = NULL;

	return pci_read_config_word(&fake_dev, offset, value);
}

int __init txboard_pci66_check(struct pci_controller *hose, int top_bus, int current_bus)
{
	u32 pci_devfn;
	unsigned short vid;
	int cap66 = -1;
	u16 stat;

	/* It seems SLC90E66 needs some time after PCI reset... */
	mdelay(80);

	printk("PCI: Checking 66MHz capabilities...\n");

	for (pci_devfn = 0; pci_devfn < 0xff; pci_devfn++) {
		if (PCI_FUNC(pci_devfn))
			continue;
		if (early_read_config_word(hose, top_bus, current_bus, pci_devfn,
					   PCI_VENDOR_ID, &vid) !=
		    PCIBIOS_SUCCESSFUL)
			continue;
		if (vid == 0xffff) continue;

		/* check 66MHz capability */
		if (cap66 < 0)
			cap66 = 1;
		if (cap66) {
			early_read_config_word(hose, top_bus, current_bus, pci_devfn,
					       PCI_STATUS, &stat);
			if (!(stat & PCI_STATUS_66MHZ)) {
				printk(KERN_DEBUG "PCI: %02x:%02x not 66MHz capable.\n",
				       current_bus, pci_devfn);
				cap66 = 0;
				break;
			}
		}
	}
	return cap66 > 0;
}

int (*txboard_pci_map_irq)(struct pci_dev *dev, u8 slot, u8 pin);
void (*txboard_pci_fixup)(struct pci_dev *dev);
char * (*txboard_pcibios_setup)(char *str) __initdata;
int (*txboard_pci_onboard)(struct pci_bus *bus, unsigned int devfn);
int (*txboard_pci_scan_hook)(struct pci_dev *dev);
int (*txboard_pci_plat_dev_init)(struct pci_dev *dev);

int pcibios_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	return txboard_pci_map_irq(dev, slot, pin);
}

/* Do platform specific device initialization at pci_enable_device() time */
int pcibios_plat_dev_init(struct pci_dev *dev)
{
	int ret = 0;

	if (txboard_pci_plat_dev_init)
		ret = txboard_pci_plat_dev_init(dev);

	return ret;
}

static struct resource primary_pci_mem_res[2] = {
	{ .name = "PCI MEM" },
	{ .name = "PCI MMIO" }, /* XTX needs this */
};
static struct resource primary_pci_io_res = { .name = "PCI IO" };
struct pci_controller txboard_primary_pcic = {
	.mem_resource = &primary_pci_mem_res[0],
	.io_resource = &primary_pci_io_res,
};

#ifdef CONFIG_64BIT
int txboard_pci_mem_high __initdata = 1;
#else
int txboard_pci_mem_high __initdata;
#endif

extern struct pci_controller *hose_head;

/*
 * allocate pci_controller and resources.
 * mem_base, io_base: physical addresss.  0 for auto assignment.
 * mem_size and io_size means max size on auto assignment.
 * pcic must be &txboard_primary_pcic or NULL.
 */
struct pci_controller * __init
txboard_alloc_pci_controller(struct pci_controller *pcic,
			     unsigned long mem_base, unsigned long mem_size,
			     unsigned long io_base, unsigned long io_size)
{
	struct pcic {
		struct pci_controller c;
		struct resource r_mem[2];
		struct resource r_io;
		char mem_name[2][12], io_name[12];
	} *new = NULL;
	int min_size = 0x10000;
	int n;

	if (!pcic) {
		struct pci_controller *p;
		new = kzalloc(sizeof(*new), GFP_KERNEL);
		if (!new)
			return NULL;

		strcpy(new->mem_name[0], "PCI MEM");
		strcpy(new->mem_name[1], "PCI MMIO");
		strcpy(new->io_name, "PCI IO");
		for (n = 0, p = hose_head; p; p = p->next, n++)
			;
		if (n) {
			char buf[4];
			sprintf(buf, " %d", n);
			strcat(new->mem_name[0], buf);
			strcat(new->mem_name[1], buf);
			strcat(new->io_name, buf);
		}
		new->r_mem[0].name = new->mem_name[0];
		new->r_mem[1].name = new->mem_name[1];
		new->r_io.name = new->io_name;
		new->c.mem_resource = new->r_mem;
		new->c.io_resource = &new->r_io;
		pcic = &new->c;
	} else {
		BUG_ON(pcic != &txboard_primary_pcic);
	}
	pcic->io_resource->flags = IORESOURCE_IO;

	/*
	 * for auto assignment, first search a (big) region for PCI
	 * MEM, then search a region for PCI IO.
	 */
	if (mem_base) {
		pcic->mem_resource[0].start = mem_base;
		pcic->mem_resource[0].end = mem_base + mem_size - 1;
		if (request_resource(&iomem_resource, &pcic->mem_resource[0]))
			goto free_and_exit;
	} else {
		unsigned long min = 0, max = 0x20000000; /* low 512MB */
		if (!mem_size) {
			/* default size for auto assignment */
			if (txboard_pci_mem_high)
				mem_size = 0x20000000;	/* mem:512M(max) */
			else
				mem_size = 0x08000000;	/* mem:128M(max) */
		}
		if (txboard_pci_mem_high) {
			min = 0x20000000;
			max = 0xe0000000;
		}
		/* search free region for PCI MEM */
		for (; mem_size >= min_size; mem_size /= 2) {
			if (allocate_resource(&iomem_resource,
					      &pcic->mem_resource[0],
					      mem_size, min, max,
					      mem_size, NULL, NULL) == 0)
				break;
		}
		if (mem_size < min_size)
			goto free_and_exit;
	}

	pcic->mem_resource[1].flags = IORESOURCE_MEM | IORESOURCE_BUSY;
	if (io_base) {
		pcic->mem_resource[1].start = io_base;
		pcic->mem_resource[1].end = io_base + io_size - 1;
		if (request_resource(&iomem_resource, &pcic->mem_resource[1]))
			goto release_and_exit;
	} else {
		if (!io_size) {
			/* default size for auto assignment */
			io_size = 0x01000000;	/* io:16M(max) */
		}
		/* search free region for PCI IO in low 512MB */
		for (; io_size >= min_size; io_size /= 2) {
			if (allocate_resource(&iomem_resource,
					      &pcic->mem_resource[1],
					      io_size, 0, 0x20000000,
					      io_size, NULL, NULL) == 0)
				break;
		}
		if (io_size < min_size)
			goto release_and_exit;
		io_base = pcic->mem_resource[1].start;
	}

	pcic->mem_resource[0].flags = IORESOURCE_MEM;
	if (pcic == &txboard_primary_pcic &&
	    mips_io_port_base == (unsigned long)-1) {
		/* map ioport 0 to PCI I/O space address 0 */
		set_io_port_base(IO_BASE + pcic->mem_resource[1].start);
		pcic->io_resource->start = 0;
		pcic->io_offset = 0;	/* busaddr == ioaddr */
		pcic->io_map_base = IO_BASE + pcic->mem_resource[1].start;
	} else {
		/* physaddr to ioaddr */
		pcic->io_resource->start = io_base - (mips_io_port_base - IO_BASE);
		pcic->io_offset = io_base - (mips_io_port_base - IO_BASE);
		pcic->io_map_base = mips_io_port_base;
	}
	pcic->io_resource->end = pcic->io_resource->start + io_size - 1;

	pcic->mem_offset = 0;	/* busaddr == physaddr */

	printk(KERN_INFO "PCI: IO 0x%08llx-0x%08llx MEM 0x%08llx-0x%08llx\n",
	       (unsigned long long)pcic->mem_resource[1].start,
	       (unsigned long long)pcic->mem_resource[1].end,
	       (unsigned long long)pcic->mem_resource[0].start,
	       (unsigned long long)pcic->mem_resource[0].end);

	return pcic;
 release_and_exit:
	release_resource(&pcic->mem_resource[0]);
 free_and_exit:
	kfree(new);
	printk(KERN_ERR "PCI: Failed to allocate resources.\n");
	return NULL;
}

static int __init
txboard_arch_pci_init(void)
{
	struct pci_controller *p;

	PCIBIOS_MIN_IO = 0x8000;	/* reseve legacy I/O space */

	/*
	 * txboard_alloc_pci_controller calls request_resource for PCI
	 * MEM.  Now release the resource and then pcibios_init will
	 * call request_resource again.
	 */
	for (p = hose_head; p; p = p->next)
		release_resource(p->mem_resource);
	return 0;
}
/* should be called after _txboard_arch_init */
arch_initcall(txboard_arch_pci_init);

/* IRQ/IDSEL mapping */
int txboard_pci_option =
#ifdef CONFIG_PICMG_PCI_BACKPLANE_DEFAULT
	TXBOARD_PCI_OPT_PICMG |
#endif
#ifdef CONFIG_PCI_NOPROBE_OFFBOARD
	TXBOARD_PCI_OPT_NOOFFBOARD |
#endif
	TXBOARD_PCI_OPT_CLK_AUTO;

enum txboard_pci_err_action txboard_pci_err_action = TXBOARD_PCI_ERR_REPORT;

/* replaces same function in arch/mips/pci/pci.c */
char * __init pcibios_setup(char *str)
{
	if (txboard_pcibios_setup && !txboard_pcibios_setup(str))
		return NULL;
	if (!strcmp(str, "off")) {
		static struct resource dummy_resource;
		/* request dummy resource to prevent IO port probing */
		dummy_resource.name = "dummy";
		dummy_resource.start = 0;
		dummy_resource.end = 0xffff;
		dummy_resource.flags |= IORESOURCE_BUSY;
		request_resource(&ioport_resource, &dummy_resource);
		hose_head = NULL;
		txboard_pci_option &= ~TXBOARD_PCI_OPT_CLK_MASK;
		txboard_pci_option |= TXBOARD_PCI_OPT_CLK_OFF;
		txboard_pci_option |= TXBOARD_PCI_OPT_OFF;
		return NULL;
	} else if (!strcmp(str, "picmg")) {
		/* PICMG compliant backplane (TOSHIBA JMB-PICMG-ATX
                   (5V or 3.3V), JMB-PICMG-L2 (5V only), etc.) */
		txboard_pci_option |= TXBOARD_PCI_OPT_PICMG;
		return NULL;
	} else if (!strcmp(str, "nopicmg")) {
		/* non-PICMG compliant backplane (TOSHIBA
                   RBHBK4100,RBHBK4200, Interface PCM-PCM05, etc.) */
		txboard_pci_option &= ~TXBOARD_PCI_OPT_PICMG;
		return NULL;
	} else if (!strcmp(str, "noparity")) {
		txboard_pci_option |= TXBOARD_PCI_OPT_NOPARITY;
		return NULL;
	} else if (!strcmp(str, "nooffboard")) {
		txboard_pci_option |= TXBOARD_PCI_OPT_NOOFFBOARD;
		return NULL;
	} else if (!strcmp(str, "offboard")) {
		txboard_pci_option &= ~TXBOARD_PCI_OPT_NOOFFBOARD;
		return NULL;
#ifdef CONFIG_REMOTE_PCI
	} else if (!strcmp(str, "noremote")) {
		txboard_pci_option |= TXBOARD_PCI_OPT_NOREMOTE;
		return NULL;
#endif
	} else if (!strncmp(str, "clk=", 4)) {
		char *val = str + 4;
		txboard_pci_option &= ~TXBOARD_PCI_OPT_CLK_MASK;
		if (strcmp(val, "33") == 0)
			txboard_pci_option |= TXBOARD_PCI_OPT_CLK_33;
		else if (strcmp(val, "66") == 0)
			txboard_pci_option |= TXBOARD_PCI_OPT_CLK_66;
		else if (strcmp(val, "off") == 0 || strcmp(val, "0") == 0)
			txboard_pci_option |= TXBOARD_PCI_OPT_CLK_OFF;
		else /* "auto" */
			txboard_pci_option |= TXBOARD_PCI_OPT_CLK_AUTO;
		return NULL;
	} else if (!strncmp(str, "err=", 4)) {
		if (!strcmp(str + 4, "panic"))
			txboard_pci_err_action = TXBOARD_PCI_ERR_PANIC;
		else if (!strcmp(str + 4, "ignore"))
			txboard_pci_err_action = TXBOARD_PCI_ERR_IGNORE;
		return NULL;
	}
	return str;
}

static void final_fixup(struct pci_dev *dev)
{
	unsigned char bist;

	/* Do build-in self test */
	if (pci_read_config_byte(dev, PCI_BIST, &bist) == PCIBIOS_SUCCESSFUL &&
	    (bist & PCI_BIST_CAPABLE)) {
		unsigned long timeout;
		pci_set_power_state(dev, PCI_D0);
		printk(KERN_INFO "PCI: %02x:%02x BIST...",
		       dev->bus->number, dev->devfn);
		pci_write_config_byte(dev, PCI_BIST, PCI_BIST_START);
		timeout = jiffies + HZ * 2;	/* timeout after 2 sec */
		do {
			pci_read_config_byte(dev, PCI_BIST, &bist);
			if (time_after(jiffies, timeout))
				break;
		} while (bist & PCI_BIST_START);
		if (bist & (PCI_BIST_CODE_MASK | PCI_BIST_START))
			printk("failed. (0x%x)\n", bist);
		else
			printk("OK.\n");
	}

	/* Do board specific fixups */
	if (txboard_pci_fixup)
		txboard_pci_fixup(dev);
}

static void enable_fixup(struct pci_dev *dev)
{
	extern u8 pci_cache_line_size;
	/* PCI_LATENCY_TIMER will be initialized in
	 * pcibios_set_master() */
	pci_write_config_byte(dev, PCI_CACHE_LINE_SIZE, pci_cache_line_size);
}

DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, final_fixup);
#ifndef CONFIG_REMOTE_PCI
DECLARE_PCI_FIXUP_RESUME(PCI_ANY_ID, PCI_ANY_ID, final_fixup);
#endif
DECLARE_PCI_FIXUP_ENABLE(PCI_ANY_ID, PCI_ANY_ID, enable_fixup);

/* debugging... */

/* pci_get_device can not be used in interrupt. This is an alternative. */
static struct pci_dev *
pci_get_all_device(struct pci_dev *from)
{
	struct list_head *n;
	struct pci_dev *dev;
	extern struct rw_semaphore pci_bus_sem;

	if (!down_read_trylock(&pci_bus_sem)) {
		pci_dev_put(from);
		return NULL;
	}
	n = from ? from->global_list.next : pci_devices.next;

	if (n && (n != &pci_devices))
		dev = pci_dev_g(n);
	else
		dev = NULL;
	pci_dev_put(from);
	dev = pci_dev_get(dev);
	up_read(&pci_bus_sem);
	return dev;
}
void txboard_dump_pci_config(void)
{
	struct pci_dev *dev;
	unsigned short w;
	int i, j;

	dev = NULL;
	while ((dev = pci_get_all_device(dev)) != NULL) {
		printk("PCICONFIG: %02x:%02x\n",
		       dev->bus->number, dev->devfn);
		for (i = 0; i < 0x40; i+=0x10) {
			printk("%02x:", i);
			for (j = 0; j < 0x10; j+=2) {
				pci_read_config_word(dev, i + j, &w);
				printk(" %04x", w);
			}
			printk("\n");
		}
	}
}
