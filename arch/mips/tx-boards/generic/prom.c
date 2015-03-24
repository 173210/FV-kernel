/*
 * linux/arch/mips/tx-boards/generic/prom.c
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
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/smp.h>	/* for smp_processor_id() */
#include <asm/mipsregs.h>
#include <asm/io.h>
#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/sections.h>
#include <asm/system.h>
#include <asm/tx-boards/tsb-generic.h>
#include <asm/tx-boards/generic.h>
#include <asm/tx-boards/pmon.h>
#include <asm/tx-boards/yamon.h>
#include <asm/tx-boards/pci.h>

extern int rbtx4927_prom_init(int argc, char **argv, char **envp);
extern int rbtx4938_prom_init(int argc, char **argv, char **envp);
extern int rbtx4939_prom_init(int argc, char **argv, char **envp);
extern int opbd4938_prom_init(int argc, char **argv, char **envp);
extern int opbd4939_prom_init(int argc, char **argv, char **envp);
extern int tc90411_prom_init(int argc, char **argv, char **envp);
extern int tc90412_prom_init(int argc, char **argv, char **envp);
extern int tc90416_prom_init(int argc, char **argv, char **envp);

extern void (*txboard_mach_setup)(void) __initdata;
extern void (*txboard_scache_setup)(void) __initdata;
extern void rbtx4927_setup(void);
extern void rbtx4938_setup(void);
extern void rbtx4939_setup(void);
extern void opbd4938_setup(void);
extern void opbd4939_setup(void);
extern void tc90411_setup(void);
extern void tc90412_setup(void);
extern void tc90416_setup(void);

#ifdef CONFIG_TXBOARD_CPU_SCACHE
/* Secondary cache setup functions */
extern void tc90412_scache_setup(void);
#endif

static struct {
	int prid;
	int revid;
	const char *system;
	unsigned long group;
	unsigned long type; /* default type */
	int (*prom_init)(int argc, char **argv, char **envp);
	void (*setup)(void);
	int base_clock;
} board_defs[] __initdata = {
#ifdef CONFIG_TX_RBTX4927
	{ PRID_IMP_TX49|0x22, 0x4927, "Toshiba TX", MACH_GROUP_TX,
	  MACH_TX_RBTX4927, rbtx4927_prom_init, rbtx4927_setup },
	{ PRID_IMP_TX49|0x30, 0x4937, "Toshiba TX", MACH_GROUP_TX,
	  MACH_TX_RBTX4927, rbtx4927_prom_init, rbtx4927_setup },
#endif
#ifdef CONFIG_TX_RBTX4938
	{ PRID_IMP_TX49|0x30, 0x4938, "Toshiba TX", MACH_GROUP_TX,
	  MACH_TX_RBTX4938, rbtx4938_prom_init, rbtx4938_setup },
#endif
#ifdef CONFIG_TX_RBTX4939
	{ PRID_IMP_TX49|0x40, 0x4939, "Toshiba TX", MACH_GROUP_TX,
	  MACH_TX_RBTX4939, rbtx4939_prom_init, rbtx4939_setup },
#endif
#ifdef CONFIG_TOSHIBA_OPBD4938
	{ PRID_IMP_TX49|0x30, 0x4938, "Toshiba TX", MACH_GROUP_TX,
	  MACH_TX_OPBD493X, opbd4938_prom_init, opbd4938_setup },
#endif
#ifdef CONFIG_TOSHIBA_OPBD4939
	{ PRID_IMP_TX49|0x40, 0x4939, "Toshiba TX", MACH_GROUP_TX,
	  MACH_TX_OPBD493X, opbd4939_prom_init, opbd4939_setup },
#endif
#ifdef CONFIG_TOSHIBA_TC90411
	{ PRID_IMP_TX49|0x40, 0x0411, "Toshiba TC90411", MACH_GROUP_TC90411,
	  MACH_TC90411_PROTO1, tc90411_prom_init, tc90411_setup, 33333300 },
#endif
#ifdef CONFIG_TOSHIBA_TC90412
	{ PRID_IMP_TX49|0xC0, 0x0412, "Toshiba TC90412", MACH_GROUP_TC90412,
	  MACH_TC90412_PROTO, tc90412_prom_init, tc90412_setup, 33333300 },
	{ PRID_IMP_TX49|0x48, 0x0413, "Toshiba TC90413", MACH_GROUP_TC90412,
	  MACH_TC90412_PROTO, tc90412_prom_init, tc90412_setup, 33333300 },
	{ PRID_IMP_TX49|0xC0, 0x0415, "Toshiba TC90415", MACH_GROUP_TC90412,
	  MACH_TC90412_PROTO, tc90412_prom_init, tc90412_setup, 33333300 },
#endif
#ifdef CONFIG_TOSHIBA_TC90416
	{ PRID_IMP_24KE|0x50, 0x0416, "Toshiba TC90416", MACH_GROUP_TC90416,
	  MACH_TC90416_PROTO1, tc90416_prom_init, tc90416_setup, 0 },
	{ PRID_IMP_24KE|0x50, 0x0417, "Toshiba TC90417", MACH_GROUP_TC90416,
	  MACH_TC90417_PROTO1, tc90416_prom_init, tc90416_setup, 0 },
#endif
};
static char tx_system_str[32];

char pmon_version[80];

static int mips_ic_disable __initdata;
static int mips_dc_disable __initdata;
int mips_config_cwfon __initdata = 1;
int tx_ccfg_toeon __initdata = 1;
static char txboard_name[16];

extern int enable_initrd;

static int tx49xx_get_revid(void)
{
	unsigned int revid;
#ifdef CONFIG_CPU_TX49XX
	revid = *(volatile unsigned int*)0xff1fe008;
	if(!revid)
		revid = *(volatile unsigned int*)0xff1fe00c;
#elif defined(CONFIG_TOSHIBA_TC90416)
	revid = *(volatile unsigned int*)0xb880e00c; /* tc90416_ccfgptr->revid*/
#else
	revid = 0;
#endif
	return (revid >> 16) & 0xffff;
}

static void __init txboard_cache_setup(void)
{
#ifdef CONFIG_CPU_TX49XX
	unsigned int conf;
	unsigned int cbits = TX49_CONF_IC | TX49_CONF_DC;
#endif
	/* if cache enabled, flush and disable here */
	switch (current_cpu_data.processor_id & 0xff00) {
#ifdef CONFIG_CPU_TX49XX
	case PRID_IMP_TX49:	/* TX49 core */
		conf = read_c0_config();
		if (((conf & cbits) != cbits) &&
		    pmon_vector) {
			pmon_vector->flush_cache(1/*DCACHE*/);
			pmon_vector->flush_cache(0/*ICACHE*/);
			/* disable L1 cache first */
			conf |= cbits;
			write_c0_config(conf);
			__asm__ __volatile__(".set push\n\t"
					     ".set noreorder\n\t"
					     "b 1f\n\t"
					     " nop\n\t"
					     "1:\n\t"
					     ".set pop"); /* stop streaming */
		}
		break;
#endif
	default:
		if (pmon_vector) {
			pmon_vector->flush_cache(1/*DCACHE*/);
			pmon_vector->flush_cache(0/*ICACHE*/);
		}
	}
#ifdef CONFIG_MIPS_UNCACHED
	change_c0_config(CONF_CM_CMASK, CONF_CM_DEFAULT);
#endif

	/* enable cache here */
#ifdef CONFIG_CPU_TX49XX
	switch (current_cpu_data.processor_id & 0xff00) {
	case PRID_IMP_TX49:	/* TX49 core */
		conf = read_c0_config();
		conf &= ~cbits;
		/* enable L1 cache */
		conf |= mips_ic_disable ? TX49_CONF_IC : 0;
		conf |= mips_dc_disable ? TX49_CONF_DC : 0;
		write_c0_config(conf);
		break;
	}
#endif
}

#ifdef CONFIG_EARLY_PRINTK
void __init disable_early_printk(void)
{
	unregister_prom_console();
}
#endif /* CONFIG_EARLY_PRINTK */

static void __init clear_initrd_header(void)
{
#ifdef CONFIG_BLK_DEV_INITRD
	unsigned long tmp;
	unsigned int* initrd_header;
	/* see arch/mips/kernel/setup.c */
	tmp = (((unsigned long)&_end + PAGE_SIZE-1) & PAGE_MASK) - 8;
	if (tmp < (unsigned long)&_end)
		tmp += PAGE_SIZE;
	initrd_header = (unsigned int *)tmp;
	/* clear initrd_header to avoid unexpected discovery */
	initrd_header[0] = 0;
#endif
}

char __init *yamon_getenv(const char *name)
{
	const struct yamon_env *env = (struct yamon_env *)fw_arg2;
	if (pmon_vector || !env)
		return NULL;
	while (env->name && env->var) {
		if (!strcmp((char *)(unsigned long)env->name, name))
			return (char *)(unsigned long)env->var;
		env++;
	}
	return NULL;
}

void __init prom_init(void)
{
	int i;
	int (*arch_prom_init)(int argc, char **argv, char **envp) = NULL;
	char *envstr = NULL;
	char *tmp_argv[32 /* max argc */];
	char *builtin_cmdline = (char *)&_text + 128; /* 0x80020080 */
	int argc = (int)fw_arg0;
	char **argv = (char **)fw_arg1;
	char **envp = (char **)fw_arg2;
	int use_mon_console = 0;
#ifdef CONFIG_IP_PNP
	int use_mon_ipaddr = 0;
#endif
#ifdef CONFIG_EARLY_PRINTK
	int do_early_printk = 0;
#endif
	int disable_console = 0;
	int prid;
	char bankdevmap[2] = {'a', 'b'};

	pmon_setup_vector();

	/*
	 * set machgroup & machtype
	 */
	prid = current_cpu_data.processor_id & 0xffff;
#ifdef CONFIG_TOSHIBA_TC90416
	if ((prid & 0xff00) == PRID_IMP_24KE) {
		/* ignore patch level (bit [0:1]) */
		prid &= ~0x3;
	}
#endif
	for (i = 0; i < ARRAY_SIZE(board_defs); i++) {
		if ((prid==board_defs[i].prid) &&
		    (tx49xx_get_revid()==board_defs[i].revid)){
			mips_machgroup=board_defs[i].group;
			mips_machtype=board_defs[i].type;
			strcpy(tx_system_str, board_defs[i].system);
			arch_prom_init = board_defs[i].prom_init;
			txboard_mach_setup = board_defs[i].setup;
			if (board_defs[i].base_clock)
				txx9_master_clock = board_defs[i].base_clock;
			break;
		}
	}
	if (mips_machgroup == MACH_GROUP_UNKNOWN) {
		pmon_printf("unknown arch. (PRid %x)\n", current_cpu_data.processor_id);
		pmon_halt();
	}
#ifdef CONFIG_TOSHIBA_TC90416
	if (mips_machgroup == MACH_GROUP_TC90416) {
		if ((tc90416_ccfgptr->revid & 0xffff0000) == 0x04170000 &&
		    (tc90416_ccfgptr->revid & 0xff) >= 0x20 &&
		    (tc90416_ccfgptr->ccfg & TC90416_CCFG_CCFG)) {
			/* USBCLK */
			txx9_master_clock = 12000000*88;    /* 1056MHz */
		} else {
			/* STCCLKI */
			txx9_master_clock = 27000000*79/2;  /* 1066.5MHz */
		}
	}
#endif
#ifdef CONFIG_TXBOARD_CPU_SCACHE
	if (mips_machgroup == MACH_GROUP_TC90412){
		/* TC90412 has the secondary cache in TXMBR. */
		txboard_scache_setup = tc90412_scache_setup;
	}
#endif
	/* next, determine by "machtype" envvar */
	if (pmon_vector)
		envstr = pmon_getenv("machtype");
	if (envstr){
		mips_machtype = simple_strtoul(envstr, NULL, 10);
	}
	/*
	else{
		pmon_printf("'machtype' IS NOT FOUND. SET DEFAULT TYPE(0).\n");
	}
	*/

#if defined(CONFIG_TOSHIBA_OPBD4938) || defined(CONFIG_TOSHIBA_OPBD4939)
	/* OPBD's builtin_cmdline is _text + 512 */
	if (mips_machtype==MACH_TX_OPBD493X) {
		builtin_cmdline = (char *)&_text + 512; /* 0x80040200 */
	}
#endif
#if defined(CONFIG_TOSHIBA_TC90416)
	/* builtin_cmdline is _text + 512 */
	if (mips_machtype==MACH_TC90417_JPTVZF1) {
		builtin_cmdline = (char *)&_text + 512; /* 0x80040200 */
	}
#endif

	/* use CONFIG_CMDLINE if exists */
	if (!builtin_cmdline[0] && arcs_cmdline[0])
		strcpy(builtin_cmdline, arcs_cmdline);

	/* process builtin_cmdline */
	if (builtin_cmdline[0]) {
		if (builtin_cmdline[0] == '+') {
			/* append to original arguments */
			for (i = 0; i < argc; i++)
				tmp_argv[i] = argv[i];
			argv = tmp_argv;
			builtin_cmdline++;
		} else {
			/* ignore argc and argv */
			argv = tmp_argv;
			argv[0] = "linux";
			argc = 1;
		}
		while ((argv[argc] = strsep(&builtin_cmdline, " ")) != NULL) {
			if (strcmp(argv[argc], "nopmon") == 0) {
				argc--;
				pmon_vector = NULL;
			}
			argc++;
		}
	} else {
		/* always use tmp_argv so that we can append argv later. */
		for (i = 0; i < argc; i++)
			tmp_argv[i] = argv[i];
		argv = tmp_argv;
	}

	if (pmon_vector && (argc <= 0 || strncmp(argv[0], "HCP", 3) == 0))
		pmon_vector = NULL;
	if (pmon_vector) {
		envp = NULL;	/* PMON does not pass env */
		fw_arg2 = 0;
	}

	/* If we can not add memory region here, board specific
           prom_init must do it. */
	if ((envstr = pmon_getenv("clienttop")) != NULL) {
		/* "mem=" option can override this */
		unsigned long eaddr = simple_strtoul(envstr, NULL, 16);
		add_memory_region(0, __pa(eaddr), BOOT_MEM_RAM);
	} else if ((envstr = yamon_getenv("memsize")) != NULL) {
		unsigned long size = simple_strtoul(envstr, NULL, 16);
		add_memory_region(0, size, BOOT_MEM_RAM);
	}

	/*
	 *  Get pmon version
	 */
	if ((envstr = pmon_getenv("version")) != NULL) {
		strncpy(pmon_version, envstr, strnlen(envstr,sizeof(pmon_version)-1));
		pmon_version[sizeof(pmon_version)-1] = '\0';
	} else {
		pmon_version[0]='\0';
	}

	pmon_printf("pmon version: %s\n", pmon_version);

	if (((envstr = pmon_getenv("disconsole")) != NULL) &&
	    strncmp(envstr, "1", 1) == 0) {
		disable_console = 1;
		use_mon_console = -2;
	}

	/* command from ROM Monitor */
	arcs_cmdline[0] = '\0';
	pmon_printf("boot:");
	/* argv[0] = "g" (go command) */
	for (i = 1; i < argc; i++) {
		char *str = argv[i];
		pmon_printf(" %s", str);
		/* check board specific options */
		if (arch_prom_init && arch_prom_init(-1, &str, NULL)) {
			continue;	/* skip (do not pass to "init") */
		}
		if (strncmp(str, "board=", 6) == 0) {
			strcpy(txboard_name, str+6);
			continue;
		} else if (strncmp(str, "machtype=", 9) == 0) {
			mips_machtype = simple_strtoul(str + 9, NULL, 10);
			continue;
		} else if (strncmp(str, "masterclk=", 10) == 0) {
			txx9_master_clock = simple_strtoul(str + 10, NULL, 10);
			continue;
		} else if (strcmp(str, "icdisable") == 0) {
			mips_ic_disable = 1;
			continue;
		} else if (strcmp(str, "dcdisable") == 0) {
			mips_dc_disable = 1;
			continue;
		} else if (strcmp(str, "cwfoff") == 0) {
			mips_config_cwfon = 0;
			continue;
		} else if (strcmp(str, "cwfon") == 0) {
			mips_config_cwfon = 1;
			continue;
		} else if (strcmp(str, "toeoff") == 0) {
			tx_ccfg_toeon = 0;
			continue;
		} else if (strcmp(str, "toeon") == 0) {
			tx_ccfg_toeon = 1;
			continue;
#ifdef CONFIG_PCI
		} else if (strcmp(str, "pcimem=high") == 0) {
			txboard_pci_mem_high = 1;
			continue;
		} else if (strcmp(str, "pcimem=low") == 0) {
			txboard_pci_mem_high = 0;
			continue;
#endif
		} else if (strncmp(str, "CONSOLE=", 8) == 0) {
			if ( disable_console == 1) {
				continue;
			}
		} else if (strncmp(str, "console=", 8) == 0) {
			if ( disable_console == 1) {
				continue;
			}
			if (strcmp(str + 8, "pmon") == 0 ||
			    strcmp(str + 8, "yamon") == 0) {
				use_mon_console = 1;
				continue;
			} else {
				use_mon_console = -1;
			}
#ifdef CONFIG_IP_PNP
		} else if (strncmp(str, "ip=", 3) == 0) {
			if (strcmp(str + 3, "pmon") == 0 ||
			    strcmp(str + 3, "yamon") == 0) {
				use_mon_ipaddr = 1;
				continue;
			} else {
				use_mon_ipaddr = -1;
			}
#ifdef CONFIG_ROOT_NFS
		} else if (strncmp(str, "nfsroot=", 8) == 0) {
			/* use monitor's ipaddr if not specified */
			if (!use_mon_ipaddr)
				use_mon_ipaddr = 1;
#endif
#endif
#ifdef CONFIG_EARLY_PRINTK
		} else if (strcmp(str, "early_printk") == 0) {
			do_early_printk = 1;
			continue;
#endif
		} else if (strncmp(str, "bankdevmap=", 11) == 0) {
			bankdevmap[0] = str[11];
			bankdevmap[1] = str[12];
			continue;
		}
		if (arcs_cmdline[0])
			strcat(arcs_cmdline, " ");
		strcat(arcs_cmdline, str);
	}
	pmon_printf("\n");
	
	pmon_printf("prid: 0x%04x mips_machgroup:%d mips_machtype:%d\n",
		    current_cpu_data.processor_id, mips_machgroup, mips_machtype);

#ifdef CONFIG_EARLY_PRINTK
	if (do_early_printk)
		register_prom_console();
#endif

	/* handle "console=pmon" */
#ifndef CONFIG_VT_CONSOLE
	/* use monitor's console if not specified */
	if (use_mon_console == 0) {
		use_mon_console = 1;
	}
#endif /* !CONFIG_VT_CONSOLE */
	if (use_mon_console > 0) {
		int ttyno = 0, baud = 38400;
		char *ttyname = NULL;
		char *con;
		char stty_out[128];
		if (((con = pmon_getenv("console")) != NULL ||
		     (con = yamon_getenv("bootserport")) != NULL) &&
		    strncmp(con, "tty", 3) == 0) {
			if (con[3] >= '0' && con[3] <= '9')
				ttyno = con[3] - '0';
		}
		if (pmon_interpret("stty", stty_out, sizeof(stty_out)) > 0) {
			char *p;
			if ((p = strstr(stty_out, "baud=")) != NULL)
				baud = simple_strtoul(p + 5, NULL, 10);
			if ((p = strstr(stty_out, "driver=")) != NULL) {
#if defined(CONFIG_SERIAL_TXX9_CONSOLE) && !defined(CONFIG_SERIAL_TXX9_STDSERIAL)
				if (!ttyname &&
				    strncmp(p + 7, "sio_txx9", 8) == 0)
					ttyname = "ttyTX";
#endif
			}
		} else {
			char buf[128];
			sprintf(buf, "modetty%d", ttyno);
			if ((con = yamon_getenv(buf)) != NULL) {
				baud = simple_strtoul(con, NULL, 10);
				if (baud <= 0)
					baud = 38400;
			}
#if defined(CONFIG_SERIAL_TXX9_CONSOLE) && !defined(CONFIG_SERIAL_TXX9_STDSERIAL)
			ttyname = "ttyTX";
#endif
		}
		if (!ttyname)
			ttyname = "ttyS";
		if (arcs_cmdline[0])
			strcat(arcs_cmdline, " ");
		sprintf(arcs_cmdline + strlen(arcs_cmdline),
			"console=%s%d,%d", ttyname, ttyno, baud);
	} 
	if ( use_mon_console == -2) {
		if (arcs_cmdline[0])
			strcat(arcs_cmdline, " ");
		strcat(arcs_cmdline + strlen(arcs_cmdline),
			"console=/dev/null CONSOLE=/dev/null");
	}
#ifdef CONFIG_KGDB
	for (i = 1; i < argc; i++) {
		if (strncmp(argv[i], "kgdb=", 5) == 0)
			break;
	}
	if (i >= argc) {
		/* no "kgdb=" specified.  Disable kgdb. */
		if (arcs_cmdline[0])
			strcat(arcs_cmdline, " ");
		strcat(arcs_cmdline, "nokgdb");
	}
#endif /* CONFIG_KGDB */
#ifdef CONFIG_IP_PNP
	if (use_mon_ipaddr > 0) {
		char *s;
		if ((s = pmon_getenv("ipaddr")) != NULL ||
		    (s = yamon_getenv("ipaddr")) != NULL) {
			if (arcs_cmdline[0])
				strcat(arcs_cmdline, " ");
			strcat(arcs_cmdline, "ip=");
			if (strcmp(s, "0.0.0.0") == 0) {
				strcat(arcs_cmdline, "dhcp");
			} else {
				strcat(arcs_cmdline, s);
				strcat(arcs_cmdline, "::");
				if ((s = yamon_getenv("gateway")) != NULL &&
				    strcmp(s, "0.0.0.0") != 0)
					strcat(arcs_cmdline, s);
				strcat(arcs_cmdline, ":");
				if ((s = yamon_getenv("subnetmask")) != NULL)
					strcat(arcs_cmdline, s);
				strcat(arcs_cmdline, "::eth0:");
			}
		}
	}
#endif /* CONFIG_IP_PNP */
	/*
	 *  add board name to cmdline.
	 */
	if (!txboard_name[0] && pmon_vector) {
		envstr=pmon_getenv("board");
		if(envstr)
			strcpy(txboard_name,envstr);
	}
	if (txboard_name[0]) {
		strcat(arcs_cmdline, " BOARD=");
		strcat(arcs_cmdline, txboard_name);
	}

	/* add load_from value */
	envstr = pmon_getenv("load_from");
	if (envstr) {
		char *s, *p;
		if (arcs_cmdline[0])
			strcat(arcs_cmdline, " ");
		strcat(arcs_cmdline, "load_from=");
		strcat(arcs_cmdline, envstr);
		/* check load_from optional parameters */
		p = s = strchr(envstr, ':');
		if (s) {
			int bank = s[1] - '1';
			/*
			 * replace a 'N' char in root= option.  if
			 * bankdevmap=ab, 'N' is replaced with 'a' for
			 * bank 0 and 'b' for bank 1.
			 */
			s = strstr(arcs_cmdline, "root=/dev/");
			if ((bank == 0 || bank == 1) && s) {
				for (s += 10; *s && *s != ' '; s++) {
					if (*s == 'N') {
						*s = bankdevmap[bank];
						break;
					}
				}
			}
			s = strchr(&p[1], ':');
			if (strncmp(envstr, "NAND", 4)
			    || (s && strcmp(&s[1], "none")))
				enable_initrd = 1;
		}
	}

	if (arch_prom_init)
		arch_prom_init(argc, argv, envp);

	txboard_cache_setup();
	clear_initrd_header();
}

char * __init prom_getcmdline(void)
{
	return &(arcs_cmdline[0]);
}

extern void free_init_pages(char *what, unsigned long begin, unsigned long end);
unsigned long __init prom_free_prom_memory(void)
{
	unsigned long saddr = PAGE_SIZE;
#ifdef CONFIG_XIP_KERNEL
	unsigned long eaddr = __pa_symbol(&_sdata);
#else
	unsigned long eaddr = __pa_symbol(&_text);
#endif
#ifdef CONFIG_FREE_PROM_MEMORY
	/* disable pmon */
	pmon_vector = NULL;
#else
	char *envstr;
	if ((envstr = pmon_getenv("heaptop")) != NULL) {
		saddr = simple_strtoul(envstr, NULL, 16);
		saddr = __pa(saddr);
	}
#endif
	if (saddr < eaddr)
		free_init_pages("prom memory", saddr, eaddr);
	return 0;
}

const char *get_system_type(void)
{
	return tx_system_str;
}
