/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2007
 * All Rights Reserved.
 *
 */
#ifndef __ASM_TX_BOARDS_PMON_H
#define __ASM_TX_BOARDS_PMON_H

struct pmon_vector {
	int (*read)(int fd, char *buf, int size);
	int (*write)(int fd, char *buf, int size);
	int (*open)(char *name, int mode);
	int (*close)(int fd);
	int (*ioctl)(int fd, int request, void* arg);
	int (*printf)(char *fmstr, ...);
	int (*vsprintf)(char *dst, char *fmtstr, ...);
	int (*ttctl)(int fd, int op, int ap1, int ap2);
	void (*exit)(int status);
	char *(*getenv)(char *name);
	void (*onintr)(int code, int *dat);
	void (*flush_cache)(int cache);
	void (*_exception)(void);
	int (*_fpstatesz)(void);
	void (*_fpinit)(void);
	void *(*_fpstate)(void);
	void (*cop1)(void);
	int (*adr2symoff)(char *dst, int value, int width);
	int (*sym2adr)(int *v, char *label);
	int (*getclkfreq)(void);
	void (*_clkinit)(void);
	int (*interpret)(const char *cmd, char *outbuf, int len);
};

extern struct pmon_vector *pmon_vector;

/* from promlib.c */
extern void prom_printf(char *fmt, ...);
/* pmonlib support routine */
extern void prom_putchar(char c);

#define pmon_printf prom_printf

extern void pmon_setup_vector(void) __init;

extern void pmon_halt(void) __attribute__((noreturn));
extern void set_pmon_debug_traps(void) __init;
extern void pmon_breakpoint(void);

extern char *pmon_getenv(const char *name) __init;
extern int pmon_interpret(const char *cmd, char *outbuf, int len);

extern unsigned int get_pmon_clkfreq(void) __init;

extern void register_prom_console(void) __init;
extern void unregister_prom_console(void) __init;

extern void pmon_setup_default_etheraddr(void) __init;

#ifdef CONFIG_PMON_RAMBOOT
#ifdef	CONFIG_PMON_RAMBOOT_PMONADDR
#define PMON_VECTOR_BASE	CONFIG_PMON_RAMBOOT_PMONADDR
#else
#define PMON_VECTOR_BASE	0x80100000
#endif
#else
#define PMON_VECTOR_BASE	0xbfc00000
#endif
#define PMON_VECTOR	(PMON_VECTOR_BASE + 0x200)
#define PMON_VECTOR_R4K	(PMON_VECTOR_BASE + 0x500) /* with R4KEXCEPTIONS */

#endif /* __ASM_TX_BOARDS_PMON_H */
