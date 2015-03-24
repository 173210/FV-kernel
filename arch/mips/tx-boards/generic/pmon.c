/*
 * linux/arch/mips/tx-boards/generic/pmon.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2006
 * All Rights Reserved.
 *
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/smp.h>	/* for smp_processor_id() */
#ifdef CONFIG_EARLY_PRINTK
#include <linux/console.h>
#endif /* CONFIG_EARLY_PRINTK */
#include <asm/cpu.h>
#include <asm/mipsregs.h>
#include <asm/tx-boards/pmon.h>
#if defined(CONFIG_TOSHIBA_OPBD4939) && defined(CONFIG_TOSHIBA_TC90411)
#include <asm/tx-boards/opbd4939.h>
#endif

struct pmon_vector *pmon_vector;

/* this function is executed 32-bit kernel mode even on mips64 */
void __init pmon_setup_vector(void)
{
#ifdef CONFIG_YAMON_BOOT
	pmon_vector = (struct pmon_vector *)NULL;
#else
	int i;
	unsigned long vec;
	unsigned long vec2;

	pmon_vector = NULL;
	vec = PMON_VECTOR_R4K;
#if defined(CONFIG_TOSHIBA_OPBD4939) && defined(CONFIG_TOSHIBA_TC90411)
	if ((TX4939_REV_PCODE() & 0xffff) == 0x4939)
		vec = PMON_VECTOR_OPBD4939;
#endif
	vec2 = vec & 0xdffc0000;

	/* simple check for PMON vector */
	for (i = 0; i < sizeof(*pmon_vector) / sizeof(void *); i++) {
		__u32 p = *((__u32 *)vec + i);
		if ((p & 0xdff80003) != vec2 && p != 0) {
			return;
		}
	}
	pmon_vector = (struct pmon_vector *)vec;
#endif
}

void pmon_halt(void)
{
	local_irq_disable();
	if (pmon_vector && pmon_vector->exit)
		pmon_vector->exit(0);
	while (1)
		;
}

extern asmlinkage void pmon_trap_low(void);
void *pmon_user_exception_handler[32];
static int pmon_debug_initialized;
void __init set_pmon_debug_traps(void)
{
	unsigned long flags;
	int tt = 9;	/* beakpoint only */

	if (!pmon_vector)
		return;
	local_irq_save(flags);
	/* save original handlers (for user-mode exceptions) */
	pmon_user_exception_handler[tt] = set_except_vector(tt, pmon_trap_low);
	local_irq_restore(flags);
	pmon_debug_initialized = 1;
}

void pmon_breakpoint(void)
{
	if (!pmon_debug_initialized)
		return;
	pmon_printf("entering PMON.  Type \"c\" to continue.\n");

	__asm__ __volatile__(
		".globl	pmon_breakinst\n\t"
		".set	noreorder\n\t"
		"nop\n"
		"pmon_breakinst:\tbreak\n\t"
		"nop\n\t"
		".set	reorder"
		);
}

char __init *pmon_getenv(const char *name)
{
	unsigned long flags;
	char *s;
	if (!pmon_vector)
		return NULL;
	local_irq_save(flags);
	s = pmon_vector->getenv((char *)name);
	local_irq_restore(flags);
	return s;
}

int pmon_interpret(const char *cmd, char *outbuf, int len)
{
	unsigned long flags;
	int ret;
	if (!pmon_vector || !pmon_vector->interpret)
		return -1;
	local_irq_save(flags);
	ret = pmon_vector->interpret(cmd, outbuf, len);
	local_irq_restore(flags);
	return ret;
}

unsigned int __init get_pmon_clkfreq(void)
{
	unsigned int cpu_clock = 0;
	char *s, *endp;
	if ((s = pmon_getenv("clkfreq")) != NULL) {
		cpu_clock = simple_strtoul(s, &endp, 10) * 1000000;
		if (*endp == '.') {
			unsigned int sub;
			s = endp + 1;
			sub = simple_strtoul(s, &endp, 10);
			while (endp++ < s + 6)
				sub *= 10;
			cpu_clock += sub;
		}
	}
	return cpu_clock;
}

#ifndef CONFIG_YAMON_BOOT
/* promlib support routine */
void prom_putchar(char c)
{
	unsigned long flags;
	if (!pmon_vector)
		return;
	local_irq_save(flags);
	pmon_vector->write(1/*STDOUT*/, &c, 1);
	local_irq_restore(flags);
}
#else /* CONFIG_YAMON_BOOT */
/* promlib support routine */
void prom_putchar(char c)
{
	unsigned long flags;

	char str[2];
	typedef int (*yamon_print_func_t)(unsigned long port, const char *s);
	yamon_print_func_t yamon_print;
	yamon_print = (yamon_print_func_t)*(unsigned long *)(0xbfc00500 + 0x34);

	str[0] = c;
	str[1] = '\0';

	local_irq_save(flags);
	yamon_print(0, str);
	local_irq_restore(flags);
}
#endif

#ifdef CONFIG_EARLY_PRINTK
static void __init prom_console_write(struct console *con, const char *s,
				      unsigned int c)
{
	int i;
	for (i = 0; i < c; i++) {
		if (*s == '\n')
			prom_putchar('\r');
		prom_putchar(*s++);
	}
}

static struct console promcons __initdata = {
	.name	= "prom",
	.write	= prom_console_write,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
};

static int promcons_output __initdata = 0;

void __init register_prom_console(void)
{
	if (!promcons_output) {
		promcons_output = 1;
		register_console(&promcons);
	}
}

void __init unregister_prom_console(void)
{
	if (promcons_output) {
		unregister_console(&promcons);
		promcons_output = 0;
	}
}
#endif /* CONFIG_EARLY_PRINTK */

#ifdef CONFIG_NETDEVICES
#include <linux/netdevice.h>
#include <linux/notifier.h>
static unsigned char default_ethaddr[6];
static int ethaddr_netdev_event(struct notifier_block *this,
				unsigned long event,
				void *ptr)
{
	struct net_device *dev = (struct net_device *)ptr;
	if (event == NETDEV_REGISTER) {
		if (((dev->dev_addr[0] & dev->dev_addr[1] &
		      dev->dev_addr[2] & dev->dev_addr[3] &
		      dev->dev_addr[4] & dev->dev_addr[5]) == 0xff) ||
		    ((dev->dev_addr[0] | dev->dev_addr[1] |
		      dev->dev_addr[2] | dev->dev_addr[3] |
		      dev->dev_addr[4] | dev->dev_addr[5]) == 0)) {
			memcpy(dev->dev_addr, default_ethaddr, 6);
			memcpy(dev->perm_addr, default_ethaddr, 6);
		}
	}
	return NOTIFY_DONE;
}
static struct notifier_block ethaddr_netdev_notifier = {
	.notifier_call = ethaddr_netdev_event,
	.priority = 1,
};

void __init pmon_setup_default_etheraddr(void)
{
	int i;
	char *tmpstr = pmon_getenv("etheraddr");
	if (tmpstr) {
		for (i = 0; i < 6; i++) {
			default_ethaddr[i] =
				simple_strtoul(tmpstr, &tmpstr, 16);
			if (*tmpstr == ':')
				tmpstr++;
		}
	}
	register_netdevice_notifier(&ethaddr_netdev_notifier);
}
#else
void __init pmon_setup_default_etheraddr(void) {}
#endif
