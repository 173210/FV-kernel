/******************************************************************************

  User Direct Driver
  COPYRIGHT (C) 2002-2010 TOSHIBA CORPORATION   ALL RIGHTS RESERVED

  VERSION HISTORY
  VER.  DATE
  0.01  2002-11-12
  2.00  2004-12-02
  2.01  2005-01-28 modified for TC90411
  2.02  2007-04-05 modified for OPBD4939

******************************************************************************/

#include <asm/tx-boards/tsb-generic.h>
#include "udirect.h"

#include <linux/module.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/smp_lock.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <asm/bitops.h>

#include <asm/system.h>
#include <asm/processor.h>
#include <asm/bootinfo.h>

#define THIS_DRV_VERSION "2.02"

static DECLARE_WAIT_QUEUE_HEAD(int_wait);

static UD_TYPE_T ena_ircmask = 0;
static UD_TYPE_T reg_ircmask = 0;
static UD_TYPE_T swstatus = 0;

static struct reqirq_data irqtable[UD_MAXIRCNUM];

#define irc2irq(irc) ( (irc)+ UD_BASE_IRC )
#define irq2irc(irq) ( (irq)- UD_BASE_IRC )

static struct udirect_mmap udirect_mmap_tbl[MAX_UDIRECT_MMAP];
static int max_table;

#define pio_in(off) (*(volatile unsigned long*)(UD_PIO_BASE+(off)))
#define pio_out(off,val) (*(volatile unsigned long*)(UD_PIO_BASE+(off))=(val))

#if (UD_MAXIRCNUM == 32)
#define set_bit64(nr, addrptr) set_bit(nr, addrptr)
#define clear_bit64(nr, addrptr) clear_bit(nr, addrptr)
#elif (UD_MAXIRCNUM == 64)
#ifdef __MIPSEB__
#define set_bit64(nr, addrptr) \
do{ \
	if((nr) >= 32){ \
		set_bit(nr - 32, (volatile unsigned long *)(addrptr)); \
	}else{ \
		set_bit(nr, (volatile unsigned long *)(addrptr) + 1); \
	} \
}while(0)
#define clear_bit64(nr, addrptr) \
do{ \
	if((nr) >= 32){ \
		clear_bit(nr - 32, (volatile unsigned long *)(addrptr)); \
	}else{ \
		clear_bit(nr, (volatile unsigned long *)(addrptr) + 1); \
	} \
}while(0)
#else /* __MIPSEB__ */
#define set_bit64(nr, addr) \
do{ \
	if((nr) >= 32){ \
		set_bit(nr - 32, (volatile unsigned long *)(addrptr) + 1); \
	}else{ \
		set_bit(nr, (volatile unsigned long *)(addrptr)); \
	} \
}while(0)
#define clear_bit64(nr, addr) \
do{ \
	if((nr) >= 32){ \
		clear_bit(nr - 32, (volatile unsigned long *)(addrptr) + 1); \
	}else{ \
		clear_bit(nr, (volatile unsigned long *)(addrptr)); \
	} \
}while(0)
#endif /* __MIPSEB__ */
#else
#error "must be set UD_MAXIRCNUM = 32 or 64"
#endif

/**********************************************************************

   Function: udirect Interrupt Handler

**********************************************************************/

static irqreturn_t intdrv_interrupt(int irq, void *dev_id)
{
	wake_up_interruptible(&int_wait);
	disable_irq_nosync(irq);
	set_bit64(irq2irc(irq), &swstatus);
	return IRQ_HANDLED;
}

/**********************************************************************

   Function: udirect_open

**********************************************************************/
static int udirect_open(struct inode *inode, struct file *file)
{
	if (UDIRET_CAPABILITY_CHECK)
		return -EPERM;
	return 0;
}

/**********************************************************************

   Function: udirect_mmap

***********************************************************************/
static struct page *udirect_nopage(struct vm_area_struct *vma,
				   unsigned long address, int *type)
{
	struct page *page;
	unsigned long off, end;
	unsigned long phys_addr = 0;
	int i;

	off = vma->vm_pgoff << PAGE_SHIFT;

	for (i = 0; i < max_table; i++) {
		end = udirect_mmap_tbl[i].offset + udirect_mmap_tbl[i].size;
		if (off >= udirect_mmap_tbl[i].offset && (off < end)) {
			phys_addr = off - udirect_mmap_tbl[i].offset +
				(udirect_mmap_tbl[i].physaddr << 4);
			break;
		}
	}

	if (i == max_table) {
		printk(KERN_ERR "%s: ERROR couldn't find mmap table entry\n",
			__func__);
		return NOPAGE_SIGBUS;
	}

	if (address - vma->vm_start + PAGE_SIZE > udirect_mmap_tbl[i].size) {
		printk(KERN_ERR "%s: ERROR invalid address\n",
			__func__);
		return NOPAGE_SIGBUS;
	}
	phys_addr += address - vma->vm_start;
	page = pfn_to_page(phys_addr >> PAGE_SHIFT);
	get_page(page);
	if (type)
		*type = VM_FAULT_MINOR;
	return page;
}

static struct vm_operations_struct udirect_vm_ops = {
	.nopage = udirect_nopage,
};

static int udirect_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long off,size,end;
	unsigned long phys_addr = 0;
	int i;

	if (UDIRET_CAPABILITY_CHECK)
		return -EPERM;

	off = vma->vm_pgoff << PAGE_SHIFT;
	size = vma->vm_end - vma->vm_start;

	for (i = 0; i < max_table; i++) {
		end = udirect_mmap_tbl[i].offset + udirect_mmap_tbl[i].size;
		if (off >= udirect_mmap_tbl[i].offset && (off < end)) {
			if ((off + size) > end) {
			    printk(KERN_ERR "udirect_mmap: ERROR invalid size %lx\n", size);
			    return -EINVAL;
			}
			phys_addr = off - udirect_mmap_tbl[i].offset +
				(udirect_mmap_tbl[i].physaddr << 4);
			break;
		}
	}

	if (i == max_table) {
		printk(KERN_ERR "udiret_mmap: ERROR couldn't find mmap table entry\n");
		return -EINVAL;
	}

	if (!udirect_mmap_tbl[i].cache)
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (phys_addr + size <= __pa(high_memory)) {
		vma->vm_flags |= VM_RESERVED;
		vma->vm_ops = &udirect_vm_ops;
		return 0;
	}
	if (io_remap_pfn_range(vma, vma->vm_start, phys_addr >> PAGE_SHIFT,
				vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static unsigned long pending_irc(void)
{
	UD_TYPE_T pending_bits;

	pending_bits = ena_ircmask & swstatus;

	/* pending_bits must be checked if not zero before finding bit */

	if (!pending_bits)
		return 0;

#if (UD_MAXIRCNUM == 32)
	return __ffs(pending_bits);
#elif (UD_MAXIRCNUM == 64)
        {
		unsigned long low_half, high_half;
		low_half = (unsigned long) (0xffffffff & pending_bits);
		high_half = (unsigned long) (pending_bits >> 32);
		if (low_half)
			return __ffs(low_half);
		else
			return __ffs(high_half) + 32;
	}
#else
#error "must be set UD_MAXIRCNUM = 32 or 64"
#endif
}
static unsigned long test_and_clear_irc(void)
{
	unsigned long irc;
	unsigned long flags;
	local_irq_save(flags);
	irc = pending_irc();
	if (irc)
		clear_bit64(irc, &swstatus);
	local_irq_restore(flags);
	return irc;
}

static int udirect_ioctl(struct inode *inodep, struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct reqirq_data data;
	unsigned long irc, irq;

	if (UDIRET_CAPABILITY_CHECK)
		return -EPERM;

	switch (cmd) {
	case IOCTL_INTDRV_INTSET:
		if (copy_from_user(&data, (struct reqirq_data __user *) arg, sizeof(data)))
			return -EFAULT;
		irc = data.irc;
		irq = irc2irq(irc);
		strcpy(irqtable[irc].name, data.name);
		/* inthdr_irq is irq number */
		set_bit64(irc, &ena_ircmask);
		set_bit64(irc, &reg_ircmask);
		if (request_irq(irq, intdrv_interrupt, IRQF_SHARED, irqtable[irc].name, (void *) &irqtable[irc])) {
			printk(KERN_ERR "intdrv: cannot register IRQ %ld\n", irq);
			clear_bit64(irc, &ena_ircmask);
			return -EIO;
		}
		break;
	case IOCTL_INTDRV_INTUNSET:
		if (get_user(irc, (unsigned long __user *) arg))
			return -EFAULT;
		irq = irc2irq(irc);
		free_irq(irq, (void *) &irqtable[irc]);
		clear_bit64(irc, &ena_ircmask);
		clear_bit64(irc, &reg_ircmask);
		break;
	case IOCTL_INTDRV_INTWAIT:
		{
			static int restart_flag = 0;
			if (get_user(irc, (unsigned long __user *) arg))
				return -EFAULT;
			if(restart_flag == 0 && irc != INTDRV_NONEEDTOACK && irc >= 0 && irc < UD_MAXIRCNUM){
				irq = irc2irq(irc);
				enable_irq(irq);
			}
			restart_flag = 0;

			if (wait_event_interruptible(int_wait,
						     (irc = test_and_clear_irc()) != 0)) {
				restart_flag = 1;
				return -ERESTARTSYS;
			}

			/* tell the irq number to user space */
			return put_user(irc, (unsigned long __user *) arg) ? -EFAULT : 0;
			break;
		}
	case IOCTL_INTDRV_INTENABLE:
		if (get_user(irc, (unsigned long __user *) arg))
			return -EFAULT;
		irq = irc2irq(irc);
		set_bit64(irc, &ena_ircmask);
		enable_irq(irq);
		break;
	case IOCTL_INTDRV_INTDISABLE:
		if (get_user(irc, (unsigned long __user *) arg))
			return -EFAULT;
		irq = irc2irq(irc);
		disable_irq(irq);
		clear_bit64(irc, &ena_ircmask);
		break;
	case IOCTL_GET_PMON_VERSION:
		{
			extern char pmon_version[];
			if (arg == 0)
				return -EINVAL;
			if (copy_to_user((void __user *) arg, pmon_version, strnlen(pmon_version, 39)+1)) {
				return -EFAULT;
			}
			break;
		}
#if (defined(CONFIG_TOSHIBA_OPBD4938) || defined(CONFIG_TOSHIBA_OPBD4939)) && defined(CONFIG_TOSHIBA_TC90411)
	case IOCTL_OPBD_ALOADDONEBIT:
		{
			int flag;
			if (mips_machgroup == MACH_GROUP_TX)
				return -ENODEV;
			if (arg == 0)
				return -EINVAL;
			if (copy_from_user(&flag, (void __user *) arg, sizeof(int))) {
				return -EFAULT;
			}
			if (flag != 0) {
				*OPBD493X_PIOFLAG_ADDR = OPBD493X_ALOADDONEBIT;
			}
			if (*OPBD493X_PIOFLAG_ADDR & OPBD493X_ALOADDONEBIT) {
				put_user(0, (int __user *) arg);	/* not loaded */
			} else {
				put_user(1, (int __user *) arg);	/* loaded */
			}
			break;
		}
#endif
#if defined(CONFIG_TOSHIBA_OPBD4939) && defined(CONFIG_TOSHIBA_TC90411)
	case IOCTL_OPBD_KLOADDONEBIT:
		{
			int flag;
			if (mips_machgroup == MACH_GROUP_TX)
				return -ENODEV;
			if (arg == 0)
				return -EINVAL;
			if (copy_from_user(&flag, (void __user *) arg, sizeof(int))) {
				return -EFAULT;
			}
			if (flag != 0) {
				*OPBD493X_PIOFLAG_ADDR = OPBD493X_KLOADDONEBIT;
			}
			if (*OPBD493X_PIOFLAG_ADDR & OPBD493X_KLOADDONEBIT) {
				put_user(0, (int __user *) arg);	/* not loaded */
			} else {
				put_user(1, (int __user *) arg);	/* loaded */
			}
			break;
		}
	case IOCTL_OPBD_WAITPBOOT:
		/* check PBOOTBIT status
		 * arguement: 4byte integer
		 *   input  -  timeout [msec]
		 *   output -  0: not boot,  1: booted
		 */
		{
			int flag,wcnt;
			if (arg == 0)
				return -EINVAL;
			if (copy_from_user(&flag, (void __user *) arg, sizeof(int))) {
				return -EFAULT;
			}
			wcnt = flag/100;
			while(wcnt-- && (tc90411_pioptr->flag[1] & OPBD493X_PBOOTBIT))
					msleep(100);
			if (tc90411_pioptr->flag[1] & OPBD493X_PBOOTBIT) {
				put_user(0, (int __user *) arg);	/* not boot */
			} else {
				put_user(1, (int __user *) arg);	/* booted */
			}
			break;
		}
	case IOCTL_OPBD_WAITKBOOT:
		/* check KBOOTBIT status
		 * arguement: 4byte integer
		 *   input  -  timeout [msec]
		 *   output -  0: not boot,  1: booted
		 */
		{
			int flag,wcnt;
			if (arg == 0)
				return -EINVAL;
			if (copy_from_user(&flag, (void __user *) arg, sizeof(int))) {
				return -EFAULT;
			}
			wcnt = flag/100;
			while(wcnt-- && (tc90411_pioptr->flag[1] & OPBD493X_KBOOTBIT))
				msleep(100);
			if (tc90411_pioptr->flag[1] & OPBD493X_KBOOTBIT) {
				put_user(0, (int __user *) arg);	/* not boot */
			} else {
				put_user(1, (int __user *) arg);	/* booted */
			}
			break;
		}
#endif
	case IOCTL_PIO_BITWRITE:
		{
			unsigned long flags;
			unsigned long value;
			struct req_bitwrite_data        req_data;

			if(copy_from_user(&req_data, (struct req_bitwrite_data __user *)arg, sizeof(req_data)))
				return -EFAULT;

			if (req_data.bit_no > 31 || req_data.write_data > 1 || req_data.offset > 0x38)
				return -EINVAL;

			local_irq_save(flags);

			value = pio_in(req_data.offset);
			value = value & ~(1 << req_data.bit_no);
			value = value | ((unsigned long)req_data.write_data << req_data.bit_no );
			pio_out(req_data.offset, value);

			local_irq_restore(flags);
			break;
		}
	case IOCTL_REGPHYS:
		{
			int i;
			struct udirect_mmap udm_tmp;

			if (copy_from_user(&udm_tmp, (void __user *) arg, sizeof(struct udirect_mmap))) {
				return -EFAULT;
			}

			/*
			 *  Align Check
			 */

			if ((udm_tmp.size & (PAGE_SIZE -1)) ||
			    (udm_tmp.physaddr & ((PAGE_SIZE -1) >> 4))) {
				return -EINVAL;
			}

			/*
			 *  check registered table
			 */

			for (i = 0; i < max_table; i++) {
				if (strncmp(udm_tmp.name,
					    udirect_mmap_tbl[i].name, 8))
					continue;
				if (udm_tmp.physaddr != udirect_mmap_tbl[i].physaddr)
					continue;
				if (udm_tmp.size != udirect_mmap_tbl[i].size)
					continue;
				if (udm_tmp.cache != udirect_mmap_tbl[i].cache)
					continue;
				break;
			}

			if (i < max_table) {
				/* find same entry */
				if (copy_to_user((void __user *) arg, (void *) &udirect_mmap_tbl[i],
						 sizeof(struct udirect_mmap))) {
					return -EFAULT;
				}
				return 0;
			}

			if (i == MAX_UDIRECT_MMAP) {
				return -EINVAL;
			}

			/*
			 *  register
			 */
			memcpy(&udirect_mmap_tbl[i], &udm_tmp, sizeof(struct udirect_mmap));
			if (i) {
				udirect_mmap_tbl[i].offset = udirect_mmap_tbl[i-1].offset +
					udirect_mmap_tbl[i-1].size;
			} else {
				udirect_mmap_tbl[i].offset = 0;
			}
			max_table++;

			if (copy_to_user((void __user *) arg, (void *) &udirect_mmap_tbl[i],
							 sizeof(struct udirect_mmap))) {
				return -EFAULT;
			}

		}
		break;
	default:
		printk(KERN_ERR "udirect: unknown command %x\n", cmd);
		return -EINVAL;
		break;
	}
	return 0;
}

/*********************************************************************

   Function: udirect_release

**********************************************************************/
static int udirect_release(struct inode *inode, struct file *file)
{
	int i, irq;
	UD_TYPE_T bits = reg_ircmask;

	if (UDIRET_CAPABILITY_CHECK)
		return -EPERM;

	for (i = 0; i < UD_MAXIRCNUM; i++, bits >>= 1) {
		if (0x1 & bits) {
			irq = irc2irq(i);
			free_irq(irq, (void *) &irqtable[i]);
		}
	}

	reg_ircmask = 0;
	ena_ircmask = 0;
	swstatus    = 0;

	return 0;
}

#ifdef CONFIG_PROC_FS
/*********************************************************************

   Function: PROC_FS

**********************************************************************/
static int udirect_read_proc(char *page, char **start, off_t off,
			      int count, int *eof, void *data)
{
	char *p;
	int i;

	p = page;
	p += sprintf(p, "    name, phys_addr      size   cache     offset\n");

	for (i=0; i < max_table; i++) {
		p += sprintf(p, "%8s,  %08lx, %08lx, %6s,  %08lx\n",
			     udirect_mmap_tbl[i].name,
			     udirect_mmap_tbl[i].physaddr,
			     udirect_mmap_tbl[i].size,
			     udirect_mmap_tbl[i].cache ? "YES" : "NO",
			     udirect_mmap_tbl[i].offset);
	}
	return (int) (p - page);
}
#endif

static struct file_operations udirect_fops = {
      .owner = THIS_MODULE,
      .open = udirect_open,
      .mmap = udirect_mmap,
      .ioctl = udirect_ioctl,
      .release = udirect_release,
};

static struct miscdevice udirect_miscdev = {
      .minor = UDIRECT_MISC_MINOR,
      .name = "udirect",
      .fops = &udirect_fops,
};

static int __init udirect_init(void)
{
	int rc;

	if (IS_OPBD493X)
		return -ENODEV;

	printk(KERN_INFO "User Direct Driver, v%s\n", THIS_DRV_VERSION);

	rc = misc_register(&udirect_miscdev);

	if (rc) {
		printk(KERN_ERR "udirect: misc_register failed\n");
		return rc;
	}

	if (!create_proc_read_entry("driver/udirect", 0, NULL, udirect_read_proc, NULL)) {
		printk(KERN_ERR "udirect: fail to regist proc ");
		misc_deregister(&udirect_miscdev);
		return -ENOMEM;
	}

	return 0;
}

static void __exit udirect_exit(void)
{
	misc_deregister(&udirect_miscdev);
}

module_init(udirect_init);
module_exit(udirect_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("User Direct driver");
