/*
 * ==========================================================================
 *
 *       Filename:  mx53_fv40_proc.c
 *
 *    Description:  
 *
 *        Version:  0.01
 *        Created:  2012年06月27日 09时52分01秒
 *
 *         Author:  xyxu (), 
 *        Company:  GSL
 *
 * ==========================================================================
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/fsl_devices.h>
#include "../../../drivers/char/mxc_iim.h"
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/gsl_version.h>
#include <asm/uaccess.h>
#include <linux/ctype.h>

extern inline void mxc_iim_disable_irq(void);
extern u32 sense_fuse(u32 bank, u32 row, u32 bit);
extern struct mxc_iim_platform_data *iim_data;

static int uboot_ver_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len=0;
	char *string;
	char *p = NULL;
	char tmp[40];
	int i = 0;
	string = saved_command_line;
	p =strstr(string, "uboot=" );
	if ( p != NULL) {
		p=p+6;
		while((*p != ' ') && (p<string+strlen(string)))
		{
			tmp[i]=*p;
			i++;
			if(i>=40)
				break;
			p++;
		}
		tmp[i]='\0';
		len = sprintf(page,tmp);
	}

	return len;
}
static int kernel_ver_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;
	len = sprintf(page, GSL_VERSION);
	return len;
}
static int __init fv40_proc_init(void)
{
	struct proc_dir_entry *res;
	struct proc_dir_entry *proc_fv40_id_root;
	struct proc_dir_entry *proc_fv40_gslver_root;
	proc_fv40_id_root = proc_mkdir("id", 0);
	proc_fv40_gslver_root = proc_mkdir("gsl_ver", 0);

	/* system uboot version */
	res = create_proc_entry("uboot_ver", 0644, proc_fv40_gslver_root);
	if(res) {
		res->read_proc = uboot_ver_read_proc;
		res->write_proc = NULL;
		res->data = NULL;
	}
	/* system kernel version */
	res = create_proc_entry("kernel_ver", 0644, proc_fv40_gslver_root);
	if(res) {
		res->read_proc = kernel_ver_read_proc;
		res->write_proc = NULL;
		res->data = NULL;
	}

	return 0;
}
late_initcall(fv40_proc_init);

