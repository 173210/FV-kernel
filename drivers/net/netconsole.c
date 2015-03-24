/*
 *  linux/drivers/net/netconsole.c
 *
 *  Copyright (C) 2001  Ingo Molnar <mingo@redhat.com>
 *
 *  Copyright (C) 2007  TOSHIBA CORPORATION
 *
 *  This file contains the implementation of an IRQ-safe, crash-safe
 *  kernel console implementation that outputs kernel messages to the
 *  network.
 *
 * Modification history:
 *
 * 2001-09-17    started by Ingo Molnar.
 * 2003-08-11    2.6 port by Matt Mackall
 *               simplified options
 *               generic card hooks
 *               works non-modular
 * 2003-09-07    rewritten with netpoll api
 */

/****************************************************************
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2, or (at your option)
 *      any later version.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with this program; if not, write to the Free Software
 *      Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ****************************************************************/

#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/tty_driver.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/sysrq.h>
#include <linux/smp.h>
#include <linux/netpoll.h>
#ifdef CONFIG_NETCONSOLE_TTY
#ifdef CONFIG_MAGIC_SYSRQ
#define SUPPORT_SYSRQ
#endif
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#define NETTTY_TTY_NAME "ttyNET"
#define NETTTY_TTY_MINOR_START	250
#define NETTTY_TTY_MAJOR	204
#endif

MODULE_AUTHOR("Maintainer: Matt Mackall <mpm@selenic.com>");
MODULE_DESCRIPTION("Console driver for network interfaces");
MODULE_LICENSE("GPL");

static char config[256];
module_param_string(netconsole, config, 256, 0);
MODULE_PARM_DESC(netconsole, " netconsole=[src-port]@[src-ip]/[dev],[tgt-port]@<tgt-ip>/[tgt-macaddr]\n");

static struct netpoll np = {
	.name = "netconsole",
	.dev_name = "eth0",
	.local_port = 6665,
	.remote_port = 6666,
	.remote_mac = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
};
static int configured = 0;

#define MAX_PRINT_CHUNK 1000

static void write_msg(struct console *con, const char *msg, unsigned int len)
{
	int frag, left;
	unsigned long flags;

	if (!np.dev)
		return;

	local_irq_save(flags);

	for(left = len; left; ) {
		frag = min(left, MAX_PRINT_CHUNK);
		netpoll_send_udp(&np, msg, frag);
		msg += frag;
		left -= frag;
	}

	local_irq_restore(flags);
}

static struct console netconsole = {
	.name = "netcon",
	.flags = CON_ENABLED | CON_PRINTBUFFER,
	.write = write_msg
};

static int option_setup(char *opt)
{
	configured = !netpoll_parse_options(&np, opt);
	return 1;
}

__setup("netconsole=", option_setup);

static ssize_t netconsole_show(struct subsystem *subsys, char *page)
{
	return sprintf(page, "%d:%s\n", configured, config);
}

static ssize_t netconsole_store(struct subsystem *subsys, const char *buf,
				size_t count)
{
	int err;

	if (!configured) {
		strcpy(config, buf);
		option_setup(config);
		if (configured) {
			err = netpoll_setup(&np);
			if (err) {
				memset(config, 0, sizeof(config));
				configured = 0;
				printk(KERN_ERR "netconsole: netpoll_setup error\n");
			}
			else {
				printk(KERN_INFO "netconsole: network logging started\n");
			}
		}
		else {
			memset(config, 0, sizeof(config));
			printk(KERN_WARNING "netconsole: not configured, aborting\n");
		}
	}
	return count;
}

static struct subsys_attribute netconsole_attr =
	__ATTR(netconsole, 0644, netconsole_show, netconsole_store);

static int init_netconsole(void)
{
	int err;

	if (subsys_create_file(&kernel_subsys, &netconsole_attr))
		printk(KERN_WARNING "netconsole: could not create file\n");

	register_console(&netconsole);

	if(strlen(config))
		option_setup(config);

	if(!configured) {
		printk("netconsole: not configured, aborting\n");
		return 0;
	}

	err = netpoll_setup(&np);
	if (err)
		return err;

	printk(KERN_INFO "netconsole: network logging started\n");
	return 0;
}

static void cleanup_netconsole(void)
{
	/* see subsys_remove_file in lib/kobject.c */
	if (subsys_get(&kernel_subsys)) {
		sysfs_remove_file(&kernel_subsys.kset.kobj,
			&netconsole_attr.attr);
		subsys_put(&kernel_subsys);
	}

	unregister_console(&netconsole);
	netpoll_cleanup(&np);
}

#ifdef CONFIG_NETCONSOLE_TTY
struct uart_nettty_port {
	struct uart_port	port;
	int tx_enable;
	int rx_enable;
#ifdef SUPPORT_SYSRQ
	int sysrq_char;
	struct tasklet_struct sysrq_tlet;
#endif
};
static struct uart_nettty_port nettty_port;

static void nettty_do_work(struct work_struct *work);
static DECLARE_WORK(nettty_work, nettty_do_work);

static void nettty_stop_tx(struct uart_port *port)
{
	struct uart_nettty_port *up = (struct uart_nettty_port *)port;
	up->tx_enable = 0;
}

static void nettty_start_tx(struct uart_port *port)
{
	struct uart_nettty_port *up = (struct uart_nettty_port *)port;
	up->tx_enable = 1;
	schedule_work(&nettty_work);
}

static void nettty_stop_rx(struct uart_port *port)
{
	struct uart_nettty_port *up = (struct uart_nettty_port *)port;
	up->rx_enable = 0;
}

static void nettty_transmit_chars(struct uart_nettty_port *up)
{
	struct circ_buf *xmit = &up->port.info->xmit;
	int count;

	if (!up->tx_enable)
		return;
	if (up->port.x_char) {
		netpoll_send_udp(&np, &up->port.x_char, 1);
		up->port.icount.tx++;
		up->port.x_char = 0;
		goto end;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)) {
		nettty_stop_tx(&up->port);
		goto end;
	}

	count = xmit->tail < xmit->head ? xmit->head - xmit->tail : UART_XMIT_SIZE - xmit->tail;
	count = min(count, MAX_PRINT_CHUNK);
	netpoll_send_udp(&np, &xmit->buf[xmit->tail], count);
	xmit->tail = (xmit->tail + count) & (UART_XMIT_SIZE - 1);
	up->port.icount.tx += count;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	if (uart_circ_empty(xmit))
		nettty_stop_tx(&up->port);
 end:
	if (up->tx_enable)
		schedule_work(&nettty_work);
}

static void nettty_do_work(struct work_struct *work)
{
	struct uart_nettty_port *up = &nettty_port;
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);
	nettty_transmit_chars(up);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

#ifdef SUPPORT_SYSRQ
static void nettty_sysrq_action(unsigned long data)
{
	struct uart_nettty_port *up = (struct uart_nettty_port *)data;
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);
	if (up->sysrq_char) {
		handle_sysrq(up->sysrq_char, up->port.info->tty);
		up->sysrq_char = 0;
	}
	spin_unlock_irqrestore(&up->port.lock, flags);
}
#endif

static int nettty_startup(struct uart_port *port)
{
	struct uart_nettty_port *up = (struct uart_nettty_port *)port;
	up->rx_enable = 1;
#ifdef SUPPORT_SYSRQ
	tasklet_init(&up->sysrq_tlet, nettty_sysrq_action, (unsigned long)up);
#endif
	return 0;
}

static void nettty_shutdown(struct uart_port *port)
{
	struct uart_nettty_port *up = (struct uart_nettty_port *)port;
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);
	up->rx_enable = 0;
	up->tx_enable = 0;
	spin_unlock_irqrestore(&up->port.lock, flags);
	flush_scheduled_work();
#ifdef SUPPORT_SYSRQ
	tasklet_kill(&up->sysrq_tlet);
#endif
}

static void nettty_enable_ms(struct uart_port *port) {}
static unsigned int nettty_tx_empty(struct uart_port *port) { return 1; }
static unsigned int nettty_get_mctrl(struct uart_port *port) { return 0; }
static void nettty_set_mctrl(struct uart_port *port, unsigned int mctrl) {}
static void nettty_break_ctl(struct uart_port *port, int break_state) {}
static void nettty_set_termios(struct uart_port *port, struct ktermios *termios,
	struct ktermios *old) {}
static void nettty_release_port(struct uart_port *port) {}
static int nettty_request_port(struct uart_port *port) { return 0; }
static const char *nettty_type(struct uart_port *port) { return "net"; }

static struct uart_ops nettty_pops = {
	.tx_empty	= nettty_tx_empty,
	.set_mctrl	= nettty_set_mctrl,
	.get_mctrl	= nettty_get_mctrl,
	.stop_tx	= nettty_stop_tx,
	.start_tx	= nettty_start_tx,
	.stop_rx	= nettty_stop_rx,
	.enable_ms	= nettty_enable_ms,
	.break_ctl	= nettty_break_ctl,
	.startup	= nettty_startup,
	.shutdown	= nettty_shutdown,
	.set_termios	= nettty_set_termios,
	.type		= nettty_type,
	.release_port	= nettty_release_port,
	.request_port	= nettty_request_port,
};

static struct uart_driver nettty_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "nettty",
	.dev_name		= NETTTY_TTY_NAME,
	.major			= NETTTY_TTY_MAJOR,
	.minor			= NETTTY_TTY_MINOR_START,
	.nr			= 1,
	.cons			= &netconsole,
};

static void nettty_rx_hook(struct netpoll *np, int port, char *msg, int len)
{
	struct uart_nettty_port *up = &nettty_port;
	unsigned long flags;

	if (!up->port.info)
		return;
	spin_lock_irqsave(&up->port.lock, flags);
	if (up->rx_enable) {
		struct tty_struct *tty = up->port.info->tty;
		while (len--) {
			unsigned char ch = *msg++;
#ifdef SUPPORT_SYSRQ
			if (ch == 0) {
				/* Ctl-@ */
				up->port.icount.brk++;
				if (uart_handle_break(&up->port))
					continue;
			}
			/* delayed uart_handle_sysrq_char */
			if (up->port.sysrq) {
				if (ch &&
				    time_before(jiffies, up->port.sysrq)) {
					up->sysrq_char = ch;
					tasklet_schedule(&up->sysrq_tlet);
					up->port.sysrq = 0;
					continue;
				}
				up->port.sysrq = 0;
			}
#endif
			up->port.icount.rx++;
			tty_insert_flip_char(tty, ch, TTY_NORMAL);
		}
		spin_unlock(&up->port.lock);
		tty_flip_buffer_push(tty);
		spin_lock(&up->port.lock);
	}
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static int __init nettty_init(void)
{
	int ret;

	np.rx_hook = nettty_rx_hook;
	strcpy(netconsole.name, NETTTY_TTY_NAME);
	netconsole.device = uart_console_device;
	netconsole.data = &nettty_reg;
	if (init_netconsole())
		return -EINVAL;

	nettty_port.port.ops = &nettty_pops;
	nettty_port.port.type = PORT_NETTTY;
	/* call this to initialize port->lock */
	uart_set_options(&nettty_port.port, &netconsole, 0, 'n', 8, 0);
	ret = uart_register_driver(&nettty_reg);
	if (ret >= 0)
		uart_add_one_port(&nettty_reg, &nettty_port.port);
	return ret;
}

static void __exit nettty_exit(void)
{
	uart_remove_one_port(&nettty_reg, &nettty_port.port);
	uart_unregister_driver(&nettty_reg);
	cleanup_netconsole();
}

late_initcall(nettty_init);
module_exit(nettty_exit);
#else
module_init(init_netconsole);
module_exit(cleanup_netconsole);
#endif
