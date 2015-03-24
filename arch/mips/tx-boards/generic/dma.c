/*
 * linux/arch/mips/tx-boards/generic/dma.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2007
 * All Rights Reserved.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <asm/tx-boards/dma.h>

struct txx9_dma_ops *txx9_dma_ops;
EXPORT_SYMBOL(txx9_dma_ops);

/* DMA channel allocator.  Based on kernel/dma.c */

struct txx9_dma_chan {
	int  lock;
	const char *device_id;
};

static struct txx9_dma_chan txx9_dma_chan_busy[MAX_TXX9_DMA_CHANNELS];

int request_txx9_dma(unsigned int dmanr, const char * device_id)
{
	if (!txx9_dma_ops)
		return -ENODEV;
	if (dmanr >= MAX_TXX9_DMA_CHANNELS)
		return -EINVAL;

	if (xchg(&txx9_dma_chan_busy[dmanr].lock, 1) != 0)
		return -EBUSY;

	txx9_dma_chan_busy[dmanr].device_id = device_id;

	/* old flag was 0, now contains 1 to indicate busy */
	return 0;
} /* request_txx9_dma */


void free_txx9_dma(unsigned int dmanr)
{
	if (dmanr >= MAX_TXX9_DMA_CHANNELS) {
		printk(KERN_WARNING "Trying to free DMA%d\n", dmanr);
		return;
	}

	if (xchg(&txx9_dma_chan_busy[dmanr].lock, 0) == 0) {
		printk(KERN_WARNING "Trying to free free DMA%d\n", dmanr);
		return;
	}

} /* free_txx9_dma */

#ifdef CONFIG_PROC_FS

static int proc_txx9_dma_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0 ; i < MAX_TXX9_DMA_CHANNELS ; i++) {
		if (txx9_dma_chan_busy[i].lock) {
		    seq_printf(m, "%2d: %s\n", i,
			       txx9_dma_chan_busy[i].device_id);
		}
	}
	return 0;
}

static int proc_txx9_dma_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_txx9_dma_show, NULL);
}

static const struct file_operations proc_txx9_dma_operations = {
	.open		= proc_txx9_dma_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_txx9_dma_init(void)
{
	struct proc_dir_entry *e;

	if (!txx9_dma_ops)
		return -ENODEV;
	e = create_proc_entry("txx9_dma", 0, NULL);
	if (e)
		e->proc_fops = &proc_txx9_dma_operations;

	return 0;
}

__initcall(proc_txx9_dma_init);
#endif

EXPORT_SYMBOL(request_txx9_dma);
EXPORT_SYMBOL(free_txx9_dma);
