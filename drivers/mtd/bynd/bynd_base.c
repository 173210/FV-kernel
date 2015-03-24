/*
 *
 * Based on SSFDC_RO drivers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * (C) Copyright TOSHIBA CORPORATION 2009-2010
 * All Rights Reserved.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/hdreg.h>
#include <linux/list.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/blktrans.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>

#include "bynd.h"

#define BYND_VERSION "1.0"

static int bynd_eraseblock(struct mtd_info *, unsigned long);
static int bynd_update_blockoob(
	struct bynd_record *, uint8_t *, const uint8_t, unsigned int);

enum data_status {
	STATUS_ASSIGN_FREE = 0xff,
	STATUS_ERASE = 0x33
};

static struct bynd_layout bynd_oobinfo_512 = {
	.logaddr       = { 3, { 0, 1, 2 } },
	.blockversion  = { 1, { 3 } },
	.datastatus    = { 1, { 4 } },
	.blockstatus   = { 1, { 5 } },
	.start         = { 1, { 6 } },
	.end           = { 1, { 7 } },
	.eccr          = { 2, { 11, 12 } },
	.ecc           = { 6, { 14, 13, 15, 9, 8, 10 } }
};

static struct bynd_layout bynd_oobinfo_2048 = {
#if defined(CONFIG_MTD_NAND_TXX9NDFMC_OOB_HWLAYOUT)
	.logaddr       = { 3, { 2, 3, 4 } },
	.blockversion  = { 1, { 5 } },
	.datastatus    = { 1, { 6 } },
	.blockstatus   = { 1, { 0 } },
	.start         = { 1, { 11 } },
	.end           = { 1, { 12 } },
	.eccr          = { 2, { 16, 17 } },
	.ecc           = { 24, 
			   { 14, 13, 15,  9,  8, 10, 30, 29,
			     31, 25, 24, 26, 46, 45, 47, 41,
			     40, 42, 62, 61, 63, 57, 56, 58 }}
#else
	.logaddr       = { 3, { 2, 3, 4 } },
	.blockversion  = { 1, { 5 } },
	.datastatus    = { 1, { 6 } },
	.blockstatus   = { 1, { 0 } },
	.start         = { 1, { 7 } },
	.end           = { 1, { 8 } },
	.eccr          = { 2, { 9, 10 } },
	.ecc           = { 24, 
			   {40, 41, 42, 43, 44, 45, 46, 47,
			    48, 49, 50, 51, 52, 53, 54, 55,
			    56, 57, 58, 59, 60, 61, 62, 63}}
#endif
};

struct rewrite_args {
	char *buf;
	unsigned long num;
};

#define BYND_MAX_DEVICES 32
#define BYND_PARAM_LEN_MAX 64

static int mtd_devs;
static int mtd_dev_param[BYND_MAX_DEVICES];
static struct bynd_record *bynd_table[BYND_MAX_DEVICES];
static int max_logic_blk_per_zone = CONFIG_BYND_MAX_LOGIC_BLK_PER_ZONE;
static int bynd_write_retries = CONFIG_BYND_WRITE_RETRIES;
static int bynd_erase_retries = CONFIG_BYND_ERASE_RETRIES;

#define BYND_MAJOR		241

/* #define READ_ONLY */
#define SECTOR_SIZE		4096
#define SECTOR_SHIFT		12
#define BYND_PARTN_BITS	0

#define MAX_PHYS_BLK_PER_ZONE	1024

#define NO_ASSIGN_BLOCK	0xffffff

/* mask to update a oob */
#define LOG_ADDR_UP	0x0007
#define BLOCK_VER_UP	0x0008
#define DATA_STATUS_UP	0x0010
#define BLOCK_STATUS_UP	0x0020
#define START_UP	0x0040
#define END_UP		0x0080
#define OOB_ECC		0x1800

/* errors */
#define OOB_CORRECTED_ERROR	0x01
#define OOB_UNCORRECTABLE_ERROR	0x02

#define DISABLE_OOB_ECC	0x00
#define ENABLE_OOB_ECC	0x01

/* proc filename */
#define PROCNAME	"driver/bynd"

/* for debug of BYND */
/* #define BYND_DEBUG */
#ifdef BYND_DEBUG
static int bynd_debug_write_retries;
static int bynd_debug_erase_retries;
static int bynd_debug_compare_head_oob;
static int bynd_debug_compare_tail_oob;
#endif

static inline unsigned long bynd_logicalsize(unsigned long phys_size)
{
	return ((unsigned long long)phys_size * max_logic_blk_per_zone)
			 / MAX_PHYS_BLK_PER_ZONE;
}

static inline int bynd_check_version(struct bynd_record *bynd,
				uint8_t old, uint8_t new)
{
	uint8_t check = (old - new);
	if (check & 0x80)
		return 1;
	else
		return 0;
}

static inline int bynd_getlogicaladdr(struct bynd_record *bynd, uint8_t *oob)
{
	int i, ret = 0;
	for (i = 0; i < bynd->layout->logaddr.size; i++) {
		ret <<= 8;
		ret += oob[bynd->layout->logaddr.pos[i]];
	}
	return ret;
}

static inline void bynd_setlogicaladdr(struct bynd_record *bynd,
				uint8_t *oob, int logaddr)
{
	int i;
	for (i = 0; i < bynd->layout->logaddr.size; i++) {
		oob[bynd->layout->logaddr.pos[bynd->layout->logaddr.size-i-1]] =
			 (uint8_t)(logaddr & 0xff);
		logaddr >>= 8;
	}
}

static inline uint8_t bynd_getblockversion(struct bynd_record *bynd,
						uint8_t *oob)
{
	int i;
	uint8_t ret = 0;
	for (i = 0; i < bynd->layout->blockversion.size; i++) {
		ret <<= 8;
		ret += oob[bynd->layout->blockversion.pos[i]];
	}
	return ret;
}

static inline void bynd_setblockversion(struct bynd_record *bynd,
				uint8_t *oob, uint8_t blockversion)
{
	int i;
	for (i = 0; i < bynd->layout->blockversion.size; i++) {
		oob[bynd->layout->blockversion.pos[
			bynd->layout->blockversion.size-i-1]] =
			(uint8_t)(blockversion & 0xff);
		blockversion >>= 8;
	}
}

static inline uint8_t bynd_getdatastatus(struct bynd_record *bynd,
						uint8_t *oob)
{
	return (uint8_t)oob[bynd->layout->datastatus.pos[0]];
}

static inline void bynd_setdatastatus(struct bynd_record *bynd,
				uint8_t *oob, uint8_t status)
{
	oob[bynd->layout->datastatus.pos[0]] = status;
}

static int bynd_put_oldversion(struct bynd_record *bynd, unsigned int physnum)
{
	struct bynd_list_entry *tmp = kzalloc(sizeof(struct bynd_list_entry),
					GFP_KERNEL);
	if (!tmp) {
		printk(KERN_ERR "BYND: bynd_put_erase\n");
		return -ENOMEM;
	}
	tmp->physnum = physnum;
	list_add_tail(&tmp->list, &bynd->oldversion);

	return 0;
}

static int bynd_put_erase(struct bynd_record *bynd, unsigned int physnum)
{
	struct bynd_list_entry *tmp = kzalloc(sizeof(struct bynd_list_entry),
					GFP_KERNEL);
	if (!tmp) {
		printk(KERN_ERR "BYND: bynd_put_erase\n");
		return -ENOMEM;
	}
	tmp->physnum = physnum;
	list_add_tail(&tmp->list, &bynd->erase);

	return 0;
}

static int bynd_put_onebiterror(struct bynd_record *bynd,
			unsigned int physnum
	)
{
	struct bynd_list_entry *tmp = kzalloc(sizeof(struct bynd_list_entry),
					 GFP_KERNEL);
	if (!tmp) {
		printk(KERN_ERR "BYND: bynd_put_onebiterror\n");
		return -ENOMEM;
	}
	tmp->physnum = physnum;
	list_add_tail(&tmp->list, &bynd->onebiterror);

	return 0;
}

static struct bynd_list_entry *bynd_get_erase(struct bynd_record *bynd)
{
	struct bynd_list_entry *tmp = NULL;
	if (!list_empty(&bynd->erase))
		tmp = list_entry(bynd->erase.next,
			struct bynd_list_entry, list);

	return tmp;
}

static struct bynd_list_entry *bynd_get_onebiterror(struct bynd_record *bynd)
{
	struct bynd_list_entry *tmp = NULL;
	if (!list_empty(&bynd->onebiterror))
		tmp = list_entry(bynd->onebiterror.next,
			 struct bynd_list_entry, list);

	return tmp;
}

/* Read physical sector (wrapper to MTD_READ) */
static int read_physical_sector(struct mtd_info *mtd, uint8_t *sect_buf,
							int sect_no)
{
	int ret;
	size_t retlen;
	loff_t offs = (loff_t)sect_no << SECTOR_SHIFT;

	ret = mtd->read(mtd, offs, SECTOR_SIZE, &retlen, sect_buf);
	if (retlen != SECTOR_SIZE)
		ret = -EIO;

	if (ret < 0) {
		printk(KERN_ERR "BYND: mtd->read error %x %x\n", ret, retlen);
		return ret;
	}

	return 0;
}

/* Read redundancy area (wrapper to MTD_READ_OOB */
static int read_raw_oob(struct mtd_info *mtd, loff_t offs, uint8_t *buf)
{
	struct mtd_oob_ops ops;
	int ret;

	ops.mode = MTD_OOB_RAW;
	ops.ooboffs = 0;
	ops.ooblen = mtd->oobsize;
	ops.oobbuf = buf;
	ops.datbuf = NULL;

	ret = mtd->read_oob(mtd, offs, &ops);

	if (ret < 0)
		return ret;
	if (ops.oobretlen != mtd->oobsize)
		return -EIO;

	return 0;
}

/* Write redundancy area (wrapper to MTD_WRITE_OOB */
static int write_raw_oob(struct mtd_info *mtd, loff_t offs, uint8_t *buf)
{
	struct mtd_oob_ops ops;
	int ret;

	ops.mode = MTD_OOB_RAW;
	ops.ooboffs = 0;
	ops.ooblen = mtd->oobsize;
	ops.oobbuf = buf;
	ops.datbuf = NULL;

	ret = mtd->write_oob(mtd, offs, &ops);

	if (ret < 0)
		return ret;
	if (ops.oobretlen != mtd->oobsize)
		return -EIO;

	return 0;
}

static int bynd_read_oob(struct bynd_record *bynd, loff_t offs, uint8_t *buf,
							int error_correct)
{
	int ret;
	struct mtd_info *mtd = bynd->mbd.mtd;

	ret = read_raw_oob(mtd, offs, buf);
	if (ret < 0)
		return ret;

	/* Check for the signature bits in the address field (MSBits) */
	if (error_correct) {
		ret = correct_oob_data(bynd, buf);

		if ((ret == 1) || (ret == 2))
			ret = OOB_CORRECTED_ERROR;
		else if (ret == 3)
			ret = OOB_UNCORRECTABLE_ERROR;
	}

	return ret;
}

static int bynd_markbad(struct mtd_info *mtd, loff_t offs)
{
	bynd_table[mtd->index]->badblocks++;
	return mtd->block_markbad(mtd, offs);
}

static int bynd_markerase(struct bynd_record *bynd, unsigned int physnum)
{
	int ret;
	uint8_t *oob_buf;
	struct mtd_info *mtd = bynd->mbd.mtd;

	oob_buf = kmalloc(mtd->oobsize, GFP_KERNEL);
	if (!oob_buf)
		return -ENOMEM;

	memset(oob_buf, 0xff, sizeof(uint8_t) * mtd->oobsize);
	bynd_setdatastatus(bynd, oob_buf, STATUS_ERASE);

	ret = bynd_update_blockoob(bynd, oob_buf, DATA_STATUS_UP, physnum);
	if (ret < 0) {
		if (bynd_eraseblock(mtd, physnum * mtd->erasesize)) {
			printk(KERN_WARNING "BYND: erase error at %x.\n",
				physnum * mtd->erasesize);
			bynd_markbad(mtd, physnum * mtd->erasesize);
			kfree(oob_buf);
			return 0;
		}
	}

	if (bynd_put_erase(bynd, physnum) < 0) {
		kfree(oob_buf);
		return -ENOMEM;
	}

	kfree(oob_buf);
	return 0;
}

static int bynd_read_blockoob(struct bynd_record *bynd, unsigned int physnum,
								uint8_t *buf)
{
	int i, ret;
	struct mtd_info *mtd = bynd->mbd.mtd;
	loff_t head_offs = physnum * mtd->erasesize;
	loff_t tail_offs = head_offs + mtd->erasesize - mtd->writesize;

	ret = bynd_read_oob(bynd, tail_offs, buf, ENABLE_OOB_ECC);
	if (ret < 0)
		return ret;

	if (ret == OOB_UNCORRECTABLE_ERROR) {
		printk(KERN_ERR
			" %06x : Uncorrectable ECC error has occurred.!!\n",
			physnum);
		/* read duplicate oob data */
		ret = bynd_read_oob(bynd, head_offs, buf, ENABLE_OOB_ECC);
		if (ret < 0)
			return ret;
		if ((ret == OOB_UNCORRECTABLE_ERROR)) {
			printk(KERN_ERR
				" %06x : duplicate uncorrectable ecc error has "
				"occurred.\n", physnum);
			return -EIO;
		}
		ret = OOB_CORRECTED_ERROR;
	} else if (ret == OOB_CORRECTED_ERROR) {
		printk(KERN_WARNING
			" %06x : 1bit oob ecc error!! OOB data recorverd "
			"with ECC.\n", physnum);
	}

	/* data ecc clear */
	for (i = 0; i < bynd->layout->ecc.size; i++)
		buf[bynd->layout->ecc.pos[i]] = 0xff;

	return ret;
}

static int bynd_update_blockoob(struct bynd_record *bynd,
	uint8_t *update_data, const uint8_t update_mask, unsigned int physnum)
{
	int ret;
	loff_t head_offs, tail_offs;
	uint8_t *oob_buf, *compare_oob_buf;
	struct mtd_info *mtd = bynd->mbd.mtd;
	uint16_t oob_mask;

	oob_buf = kmalloc(mtd->oobsize, GFP_KERNEL);
	if (!oob_buf)
		return -ENOMEM;
	compare_oob_buf = kmalloc(mtd->oobsize, GFP_KERNEL);
	if (!compare_oob_buf) {
		kfree(oob_buf);
		return -ENOMEM;
	}

	oob_mask = (uint16_t)update_mask | OOB_ECC;

	head_offs = physnum * mtd->erasesize;
	tail_offs = (physnum + 1) * mtd->erasesize - mtd->writesize;

	/* read oob */
	ret = bynd_read_blockoob(bynd, physnum, oob_buf);
	if (ret < 0)
		return ret;

	/* create oob_buf */
	oob_buf[bynd->layout->datastatus.pos[0]] = update_data[bynd->layout->datastatus.pos[0]];
	calc_ecc_for_oob(bynd, oob_buf);
	memset(update_data,0xff,mtd->oobsize);
	update_data[bynd->layout->datastatus.pos[0]] = oob_buf[bynd->layout->datastatus.pos[0]];
	update_data[bynd->layout->eccr.pos[0]] = oob_buf[bynd->layout->eccr.pos[0]];
	update_data[bynd->layout->eccr.pos[1]] = oob_buf[bynd->layout->eccr.pos[1]];
	memcpy(oob_buf,update_data,mtd->oobsize);

	/* update head page oob in block */
	ret = write_raw_oob(mtd, head_offs, oob_buf);
	if (ret < 0) {
		printk(KERN_ERR
		      "BYND: mtd write_oob() failed at %x\n",
		      physnum * mtd->erasesize);
		goto out_err;
	}

	/* update tail page oob in block */
	ret = write_raw_oob(mtd, tail_offs, oob_buf);
	if (ret < 0) {
		printk(KERN_ERR
		      "BYND: mtd write_oob() failed at %x\n",
		      physnum * mtd->erasesize);
		goto out_err;
	}

	/* compare */
	ret = read_raw_oob(mtd, head_offs, compare_oob_buf);
	if (ret < 0)
		goto out_err;
#ifdef BYND_DEBUG
	if (bynd_debug_compare_head_oob) {
		compare_oob_buf[bynd->layout->eccr.pos[0]] =
				~compare_oob_buf[bynd->layout->eccr.pos[0]];
		bynd_debug_compare_head_oob = false;
		printk(KERN_INFO "set different data, in head oob\n");
	}
#endif

	if ( ( compare_oob_buf[bynd->layout->datastatus.pos[0]] != 
	       oob_buf[bynd->layout->datastatus.pos[0]] ) || 
	     ( compare_oob_buf[bynd->layout->eccr.pos[0]] != 
	       oob_buf[bynd->layout->eccr.pos[0]] ) ||
	     ( compare_oob_buf[bynd->layout->eccr.pos[1]] != 
	       oob_buf[bynd->layout->eccr.pos[1]] ) ) {
		ret = -EIO;
		printk(KERN_INFO "head oob compare error!!\n");
		goto out_err;
	}

	ret = read_raw_oob(mtd, tail_offs, compare_oob_buf);
	if (ret < 0)
		goto out_err;
#ifdef BYND_DEBUG
	if (bynd_debug_compare_tail_oob) {
		compare_oob_buf[bynd->layout->eccr.pos[0]] =
				~compare_oob_buf[bynd->layout->eccr.pos[0]];
		bynd_debug_compare_tail_oob = false;
		printk(KERN_INFO "set different data, in tail oob\n");
	}
#endif

	if ( ( compare_oob_buf[bynd->layout->datastatus.pos[0]] != 
	       oob_buf[bynd->layout->datastatus.pos[0]] ) || 
	     ( compare_oob_buf[bynd->layout->eccr.pos[0]] != 
	       oob_buf[bynd->layout->eccr.pos[0]] ) ||
	     ( compare_oob_buf[bynd->layout->eccr.pos[1]] != 
	       oob_buf[bynd->layout->eccr.pos[1]] ) ) {
		ret = -EIO;
		printk(KERN_INFO "head oob compare error!!\n");
		goto out_err;
	}

	ret = 0;
out_err:
	kfree(oob_buf);
	kfree(compare_oob_buf);
	return ret;
}

/* Write physical page */
static int write_physical_page(struct mtd_info *mtd, loff_t offs, uint8_t *buf,
							uint8_t *dat_buf)
{
	uint8_t *oob_buf;
	struct mtd_oob_ops ops;
	int ret;

	oob_buf = kmalloc(mtd->oobsize, GFP_KERNEL);
	if (!oob_buf)
		return -ENOMEM;

	ops.mode	= MTD_OOB_PLACE;
	ops.len		= mtd->writesize;
	ops.retlen	= 0;
	ops.ooblen	= mtd->oobsize;
	ops.oobretlen	= 0;
	ops.ooboffs	= 0;
	ops.oobbuf	= oob_buf;
	ops.datbuf	= dat_buf;

	if (buf != NULL)
		memcpy(oob_buf, buf, mtd->oobsize);
	else
		memset(oob_buf, 0xff, sizeof(uint8_t) * mtd->oobsize);

	ret = mtd->write_oob(mtd, offs, &ops);
	if ((ops.oobretlen != mtd->oobsize) || (ops.retlen != mtd->writesize))
		ret = -EIO;

	if (ret < 0) {
		printk(KERN_ERR "BYND: mtd->write_oob error %x %x %x \n",
			ret, ops.oobretlen, ops.retlen);
		kfree(oob_buf);
		return ret;
	}

	kfree(oob_buf);
	if (ret < 0)
		return ret;
	if (ops.oobretlen != mtd->oobsize)
		return -EIO;

	return 0;
}

/* Read physical block */
static int read_physical_block(struct bynd_record *bynd, unsigned int physnum,
								int skip_sect)
{
	int i, ret, page_offs;
	struct mtd_info *mtd = bynd->mbd.mtd;
	size_t retlen;
	size_t len = mtd->writesize;
	loff_t offs = physnum * mtd->erasesize;
	int pages_per_block = mtd->erasesize/mtd->writesize;

	/* read */
	for (i = 0; i < pages_per_block; i++) {
		retlen = 0;
		page_offs = i * mtd->writesize;

		ret = mtd->read(mtd, offs + page_offs, len, &retlen,
					&bynd->page_buf[page_offs]);

		if (retlen != len)
			ret = -EIO;

		if (ret < 0) {
			if (ret == -EUCLEAN) {
				printk(KERN_WARNING
					"BYND: read ecc corrected "
					"(physical block number %x)"
					"(physical page addr %llx) %x.\n",
					physnum, (offs + page_offs), ret);
			} else {
				printk(KERN_ERR
					"BYND: mtd->read error %x %x\n"
					"(physical page addr %llx)\n",
					ret, retlen, (offs + page_offs));
				memset(&bynd->page_buf[page_offs], 0xff,
						sizeof(char) * mtd->writesize);
			}
		}
	}

	return 0;
}

/* Write physical block */
static int write_physical_block(struct bynd_record *bynd,
				unsigned int physnum, uint8_t *oob_buf)
{
	uint8_t *arg_oob_buf;
	int i, ret, page_offs;
	struct mtd_info *mtd = bynd->mbd.mtd;
	int pages_per_block = mtd->erasesize/mtd->writesize;
	loff_t offs = physnum * mtd->erasesize;

	for (i = 0; i < pages_per_block; i++) {
		arg_oob_buf = (i == 0 || i == (pages_per_block - 1)) ?
			 oob_buf : NULL;
		page_offs = i * mtd->writesize;

		ret = write_physical_page(mtd, offs + page_offs, arg_oob_buf,
				&bynd->page_buf[page_offs]);
		if (ret < 0) {
			printk(KERN_ERR
				"BYND: write page error "
				"(physical page addr %llx) %x.\n",
				 offs + page_offs, ret);
			if (bynd_eraseblock(mtd, offs)) {
				printk(KERN_WARNING
					"BYND: erase error at %llx.\n",
					offs);
				bynd_markbad(mtd, offs);
			} else if (bynd_put_erase(bynd, physnum) < 0)
				return -ENOMEM;
			return ret;
		}
	}
	return 0;
}

#ifdef CONFIG_BYND_WRITE_CACHE
/*
 args           buf       num
---------------------------------------------
 write         !NULL      log_sect_no
 move           NULL      physical block num
 cache flush    NULL      ULONG_MAX
*/
static int bynd_cached_write(struct bynd_record *bynd,
			struct rewrite_args *args)
{
	int ret, physnum, cache_physnum;
	struct mtd_info *mtd = bynd->mbd.mtd;
	uint8_t *oob_buf;
	loff_t offs = -1;
	unsigned int retries = 0;
	unsigned long logaddr, new_physnum;
	struct bynd_list_entry *new;
#ifdef BYND_DEBUG
	int block_alloc_retry = 0;
#endif

	oob_buf = kmalloc(mtd->oobsize, GFP_KERNEL);
	if (!oob_buf)
		return -ENOMEM;

	if (args->buf) {
		int sectors_per_block;

		sectors_per_block = mtd->erasesize >> SECTOR_SHIFT;
		offs = (int)(args->num % sectors_per_block);
		logaddr = (int)(args->num / sectors_per_block);
		physnum = bynd->log_block_map[logaddr];
	} else 	{
		if (args->num == ULONG_MAX) {
			if (bynd->cache_logaddr > sizeof(bynd->log_block_map[0])
							* bynd->map_size) {
				kfree(oob_buf);
				return -EIO;
			}
			logaddr = args->num;
			physnum = bynd->log_block_map[bynd->cache_logaddr];
		} else {
			ret = bynd_read_blockoob(bynd, args->num, oob_buf);
			if (ret < 0) {
				kfree(oob_buf);
				return ret;
			}

			logaddr = bynd_getlogicaladdr(bynd, oob_buf);
			if (logaddr == NO_ASSIGN_BLOCK) {
				kfree(oob_buf);
				return 0;
			}
			if (logaddr > (sizeof(bynd->log_block_map[0])
							* bynd->map_size)) {
				kfree(oob_buf);
				return -EIO;
			}
			physnum = args->num;
		}
	}

	if (bynd->cache_state == STATE_DIRTY &&
					bynd->cache_logaddr != logaddr) {
		cache_physnum = bynd->log_block_map[bynd->cache_logaddr];
write_retry:
		new = bynd_get_erase(bynd);
		if (!new) {
			printk(KERN_ERR "BYND: no freeblocks found.\n");
			kfree(oob_buf);
			return -ENOSPC;
		}

		new_physnum = new->physnum;
		list_del(&new->list);
		kfree(new);

		if (bynd_eraseblock(mtd, (new_physnum * mtd->erasesize))) {
			printk(KERN_WARNING
				"BYND: erase error at %lx.\n",
				(new_physnum * mtd->erasesize));
			bynd_markbad(mtd, (new_physnum * mtd->erasesize));

			if (retries++ < bynd_write_retries) {
#ifdef BYND_DEBUG
				block_alloc_retry = retries;
#endif
				printk(KERN_INFO "write retry!!\n");
				goto write_retry;
			} else {
				kfree(oob_buf);
				return -EIO;
			}
		}

		/* write data & oob */
		memset(oob_buf, 0xff, sizeof(uint8_t) * mtd->oobsize);
		bynd_setlogicaladdr(bynd, oob_buf, bynd->cache_logaddr);
		bynd_setblockversion(bynd, oob_buf,
			bynd->log_block_version[bynd->cache_logaddr]-1);
		calc_ecc_for_oob(bynd, oob_buf);

		ret = write_physical_block(bynd, new_physnum, oob_buf);
#ifdef BYND_DEBUG
		if (ret == 0 && bynd_debug_write_retries) {
			ret = bynd_markerase(bynd, new_physnum);
			if (ret < 0) {
				kfree(oob_buf);
				return ret;
			}
			if ((retries - block_alloc_retry) ==
						 (bynd_debug_write_retries - 1))
				bynd_debug_write_retries = 0;
			ret = -EIO;
		}
#endif
		if (ret < 0) {
			if ((ret == -EIO) && (retries++ < bynd_write_retries)) {
				printk(KERN_INFO "write retry!!\n");
				goto write_retry;
			} else {
				kfree(oob_buf);
				return ret;
			}
		}

		/* update block version */
		bynd->log_block_map[bynd->cache_logaddr] = new_physnum;
		bynd->log_block_version[bynd->cache_logaddr]--;

		bynd->cache_state = STATE_EMPTY;

		/* update oob in erase block */
		if (cache_physnum >= 0) {
			ret = bynd_markerase(bynd, cache_physnum);
			if (ret < 0) {
				kfree(oob_buf);
				return ret;
			}
		}

		/* when call from close() return */
		if (args->num == ULONG_MAX) {
			kfree(oob_buf);
			return 0;
		}
	}

	if (physnum >= bynd->map_len) {
		printk(KERN_ERR	"BYND: physical block number %x is too big.\n",
								physnum);
		kfree(oob_buf);
		return -EIO;
	}

	/* update cache */
	if (bynd->cache_state == STATE_EMPTY || bynd->cache_logaddr !=
								logaddr) {
		if (physnum < 0) {
			memset(&bynd->page_buf[0], 0xff, sizeof(char) *
								mtd->erasesize);
		} else {
			ret = read_physical_block(bynd, physnum, offs);
			if (ret < 0) {
				printk(KERN_ERR
					"BYND: read error (physical block "
					"number %x) %x.\n", physnum, ret);
				kfree(oob_buf);
				return ret;
			}
		}
		bynd->cache_logaddr = logaddr;
		bynd->cache_state = STATE_DIRTY;
	}

	if (args->buf)
		memcpy(&bynd->page_buf[offs<<SECTOR_SHIFT], args->buf,
								SECTOR_SIZE);

	kfree(oob_buf);
	return 0;
}

#else
static int bynd_rewrite_block(struct bynd_record *bynd,
			struct rewrite_args *args, unsigned int move_flag)
{
	int ret, physnum;
	struct mtd_info *mtd = bynd->mbd.mtd;
	uint8_t *oob_buf;
	loff_t offs = -1;
	unsigned int retries = 0;
	unsigned long logaddr, new_physnum;
	struct bynd_list_entry *new;
#ifdef BYND_DEBUG
	int block_alloc_retry = 0;
#endif


	oob_buf = kmalloc(mtd->oobsize, GFP_KERNEL);
	if (!oob_buf)
		return -ENOMEM;

	if (!move_flag) {
		int sectors_per_block;

		sectors_per_block = mtd->erasesize >> SECTOR_SHIFT;
		offs = (int)(args->num % sectors_per_block);
		logaddr = (int)(args->num / sectors_per_block);
		physnum = bynd->log_block_map[logaddr];
	} else {
		ret = bynd_read_blockoob(bynd, args->num, oob_buf);
		if (ret < 0) {
			kfree(oob_buf);
			return ret;
		}

		logaddr = bynd_getlogicaladdr(bynd, oob_buf);
		if (logaddr == NO_ASSIGN_BLOCK) {
			kfree(oob_buf);
			return 0;
		}
		if (logaddr > (sizeof(bynd->log_block_map[0])
							* bynd->map_size)) {
			kfree(oob_buf);
			return -EIO;
		}
		physnum = args->num;
	}

	if (physnum < 0) {
		memset(&bynd->page_buf[0], 0xff, sizeof(char) * mtd->erasesize);
		goto move_retry;
	}

	if (physnum >= bynd->map_len) {
		printk(KERN_ERR	"BYND: physical block number %x is too big.\n",
								physnum);
		kfree(oob_buf);
		return -EIO;
	}

	ret = read_physical_block(bynd, physnum, offs);
	if (ret < 0) {
		printk(KERN_ERR
			"BYND: read error (physical block number %x) %x.\n",
			physnum, ret);
		kfree(oob_buf);
		return ret;
	}

move_retry:
	new = bynd_get_erase(bynd);
	if (!new) {
		printk(KERN_ERR "BYND: no freeblocks found.\n");
		kfree(oob_buf);
		return -ENOSPC;
	}

	new_physnum = bynd->log_block_map[logaddr] = new->physnum;
	list_del(&new->list);
	kfree(new);

	if (!move_flag)
		memcpy(&bynd->page_buf[offs<<SECTOR_SHIFT], args->buf,
			SECTOR_SIZE);

	if (bynd_eraseblock(mtd, (new_physnum * mtd->erasesize))) {
		printk(KERN_WARNING
			"BYND: erase error at %lx.\n",
			(new_physnum * mtd->erasesize));
		bynd_markbad(mtd, (new_physnum * mtd->erasesize));

		if (retries++ < bynd_write_retries) {
#ifdef BYND_DEBUG
			block_alloc_retry = retries;
#endif
			printk(KERN_INFO "write retry!!\n");
			goto move_retry;
		} else {
			kfree(oob_buf);
			if (move_flag)
				bynd->log_block_map[logaddr] = physnum;
			return -EIO;
		}
	}

	/* write data & oob */
	if (!move_flag) {
		memset(oob_buf, 0xff, sizeof(uint8_t) * mtd->oobsize);
		bynd_setlogicaladdr(bynd, oob_buf, logaddr);
	}
	bynd_setblockversion(bynd, oob_buf, bynd->log_block_version[logaddr]-1);
	calc_ecc_for_oob(bynd, oob_buf);

	ret = write_physical_block(bynd, new_physnum, oob_buf);
#ifdef BYND_DEBUG
	if (ret == 0 && bynd_debug_write_retries) {
		ret = bynd_markerase(bynd, new_physnum);
		if (ret < 0) {
			kfree(oob_buf);
			return ret;
		}
		if ((retries - block_alloc_retry) ==
						 (bynd_debug_write_retries - 1))
			bynd_debug_write_retries = 0;
		ret = -EIO;
	}
#endif
	if (ret < 0) {
		if ((ret == -EIO) && (retries++ < bynd_write_retries)) {
			printk(KERN_INFO "write retry!!\n");
			goto move_retry;
		} else {
			kfree(oob_buf);
			if (move_flag)
				bynd->log_block_map[logaddr] = physnum;
			return ret;
		}
	}

	/* update block version */
	bynd->log_block_version[logaddr]--;

	/* update oob in erase block */
	if (physnum >= 0) {
		ret = bynd_markerase(bynd, physnum);
		if (ret < 0) {
			kfree(oob_buf);
			return ret;
		}
	}

	if (move_flag)
		printk(KERN_INFO
			"replace physical block from No. %x to No. %lx\n",
			physnum, new_physnum);

	kfree(oob_buf);
	return 0;
}
#endif

static int bynd_move_block(struct bynd_record *bynd, unsigned long physnum)
{
	struct rewrite_args args = {NULL, physnum};

#ifdef CONFIG_BYND_WRITE_CACHE
	return bynd_cached_write(bynd, &args);
#else
	return bynd_rewrite_block(bynd, &args, 1);
#endif
}

/* Build the logic block map */
static int build_logical_block_map(struct bynd_record *bynd)
{
	int phys_block, ret;
	struct mtd_info *mtd = bynd->mbd.mtd;
	uint8_t *oob_buf;
	unsigned int logaddr;
	unsigned int fix_block_flag;

	oob_buf = kmalloc(mtd->oobsize, GFP_KERNEL);
	if (!oob_buf)
		return -ENOMEM;
	INIT_LIST_HEAD(&bynd->onebiterror);
	INIT_LIST_HEAD(&bynd->oldversion);

	for (phys_block = 0; phys_block < bynd->map_len; phys_block++) {
		fix_block_flag = 0;

		/* skip bad blocks */
		if (mtd->block_isbad(mtd, (unsigned long)phys_block *
				mtd->erasesize)) {
			printk("%06x: bad block\n", phys_block);
			bynd->badblocks++;
			continue;
		}

		/* get logical address */
		ret = bynd_read_blockoob(bynd, phys_block, oob_buf);
		if (ret < 0) {
			ret = bynd_markerase(bynd, phys_block);
			if (ret < 0) {
				kfree(oob_buf);
				return ret;
			}
			continue;
		}
		else if (ret == OOB_CORRECTED_ERROR)
			fix_block_flag = 1;

		logaddr = bynd_getlogicaladdr(bynd, oob_buf);

		switch (bynd_getdatastatus(bynd, oob_buf)) {
		case STATUS_ASSIGN_FREE:
			if (logaddr == NO_ASSIGN_BLOCK) {
				if (bynd_put_erase(bynd, phys_block) < 0) {
					kfree(oob_buf);
					return -ENOMEM;
				}
			} else {
				if (logaddr > (sizeof(bynd->log_block_map[0])
							* bynd->map_size)) {
					ret = bynd_markerase(bynd, phys_block);
					if (ret < 0) {
						kfree(oob_buf);
						return ret;
					}
					continue;
				}
				if (bynd->log_block_map[logaddr] < 0) {
					bynd->log_block_map[logaddr]     =
						phys_block;
					bynd->log_block_version[logaddr] =
						bynd_getblockversion(bynd,
								 oob_buf);
				} else {
					int erase_block;
					uint8_t block_version =
						 bynd_getblockversion(bynd,
								oob_buf);
					if (bynd_check_version(bynd,
							block_version,
							bynd->log_block_version
							[logaddr])) {
						erase_block =
							bynd->log_block_map
							[logaddr];
						bynd->log_block_map[logaddr] =
							phys_block;
						bynd->log_block_version
							[logaddr] =
							block_version;
					} else {
						erase_block = phys_block;
						fix_block_flag = 0;
					}

					if (bynd_put_oldversion(bynd,
							erase_block) < 0) {
						kfree(oob_buf);
						return -ENOMEM;
					}
				}

				if (fix_block_flag) {
					printk("%s %d %d\n",__FUNCTION__,__LINE__,phys_block);
					if (bynd_put_onebiterror(bynd,
							 phys_block) < 0) {
						kfree(oob_buf);
						return -ENOMEM;
					}
				}
			}

			break;
		case STATUS_ERASE:
			if (bynd_put_erase(bynd, phys_block) < 0) {
				kfree(oob_buf);
				return -ENOMEM;
			}
			break;
		default:
			printk(KERN_INFO "   invalid data status\n");
			ret = bynd_markerase(bynd, phys_block);
			if (ret < 0) {
				kfree(oob_buf);
				return ret;
			}
			break;
		}
	}

	/* replace 1bit error block */
	{
		struct bynd_list_entry *replace;
		int replace_block;

		while ((replace = bynd_get_onebiterror(bynd))) {
			struct bynd_list_entry *tmp = NULL;
			replace_block = replace->physnum;
			list_del(&replace->list);
			kfree(replace);
			list_for_each_entry(tmp, &bynd->oldversion, list)
				if (tmp->physnum == replace_block)
					break;
			if (!tmp || (tmp->physnum != replace_block))
				bynd_move_block(bynd, replace_block);

		}

		if (bynd->cache_state == STATE_DIRTY) {
			struct rewrite_args args = {NULL, ULONG_MAX};
			bynd_cached_write(bynd, &args);
			bynd->cache_state = STATE_EMPTY;
		}

	}

	/* put into erase list head */
	while (!list_empty(&bynd->oldversion)) {
		struct bynd_list_entry *tmp = NULL;
		tmp = list_entry(bynd->oldversion.next,
				 struct bynd_list_entry, list);
		list_del(&tmp->list);
		list_add(&tmp->list, &bynd->erase);
	}

	kfree(oob_buf);
	return 0;
}

static void bynd_add_mtd(struct mtd_blktrans_ops *tr, struct mtd_info *mtd)
{
	int i = 0;
	struct bynd_record *bynd;

	if (bynd_write_retries < 0)
		bynd_write_retries = 5;

	if (bynd_erase_retries < 0)
		bynd_erase_retries = 5;

	for (i = 0; i < mtd_devs; i++) {
		if (mtd->index == mtd_dev_param[i])
			break;
	}
	if (i == mtd_devs)
		return;

	/* Check for small page NAND flash */
	if (mtd->type != MTD_NANDFLASH)
		return;

	bynd = kzalloc(sizeof(struct bynd_record), GFP_KERNEL);
	if (!bynd) {
		printk(KERN_ERR "BYND: out of memory for data structures\n");
		return;
	}

	bynd_table[mtd->index] = bynd;

	if (mtd->writesize == 512)
		bynd->layout = &bynd_oobinfo_512;
	else if ((mtd->writesize == 2048))
		bynd->layout = &bynd_oobinfo_2048;
	else {
		printk(KERN_WARNING
		       "BYND: not support writesize\n");
		goto out_err;
	}

	bynd->mbd.mtd = mtd;
	bynd->mbd.devnum = mtd->index;
	bynd->mbd.tr = tr;
#if defined(READ_ONLY)
	bynd->mbd.readonly = 1;
#endif /* READ_ONLY */

	bynd->map_len = mtd->size / mtd->erasesize;
	bynd->map_size = bynd_logicalsize(bynd->map_len);

	if (bynd->map_len == 0) {
		printk(KERN_ERR "BYND: map len is zero\n");
		goto out_err;
	}

	bynd->page_buf = kzalloc(mtd->erasesize, GFP_KERNEL);
	if (!bynd->page_buf) {
		printk(KERN_ERR
			"BYND: out of memory for data structures\n");
		goto out_err;
	}

#ifdef CONFIG_BYND_WRITE_CACHE
	printk(KERN_INFO "BYND write cache support\n");
	bynd->cache_state = STATE_EMPTY;
	bynd->cache_size = mtd->erasesize;
#endif

	/* Set geometry */
	bynd->heads = 16;
	bynd->sectors = 32;
	bynd->cylinders = (unsigned short)((bynd_logicalsize(mtd->size)
			>> SECTOR_SHIFT) /
			((long)bynd->sectors * (long)bynd->heads));

	bynd->mbd.size = bynd_logicalsize(mtd->size) / SECTOR_SIZE;
	bynd->badblocks = 0;

	/* Allocate logical block map */
	INIT_LIST_HEAD(&bynd->erase);
	bynd->log_block_map = kmalloc(sizeof(bynd->log_block_map[0]) *
					 bynd->map_size, GFP_KERNEL);
	if (!bynd->log_block_map) {
		printk(KERN_ERR
			"BYND: out of memory for data structures\n");
		goto out_err;
	}
	memset(bynd->log_block_map, 0xff, sizeof(bynd->log_block_map[0]) *
		bynd->map_size);

	bynd->log_block_version = kmalloc(sizeof(bynd->log_block_version[0]) *
					 bynd->map_size, GFP_KERNEL);
	if (!bynd->log_block_version) {
		printk(KERN_ERR
			"BYND: out of memory for data structures\n");
		goto out_err;
	}

	memset(bynd->log_block_version, 0xff,
		sizeof(bynd->log_block_version[0]) * bynd->map_size);

	/* Build logical block map */
	if (build_logical_block_map(bynd) < 0)
		goto out_err;

	/* Register device + partitions */
	if (add_mtd_blktrans_dev(&bynd->mbd))
		goto out_err;

	return;

out_err:
	kfree(bynd->log_block_map);
	kfree(bynd->log_block_version);
	kfree(bynd->page_buf);
	kfree(bynd);
}

static void bynd_remove_dev(struct mtd_blktrans_dev *dev)
{
	struct bynd_record *bynd = (struct bynd_record *)dev;
	struct bynd_list_entry *free_e;

	DEBUG(MTD_DEBUG_LEVEL1, "BYND: remove_dev (i=%d)\n", dev->devnum);

	del_mtd_blktrans_dev(dev);
	while ((free_e = bynd_get_erase(bynd)))	{
		list_del(&free_e->list);
		kfree(free_e);
	}
	kfree(bynd->log_block_map);
	kfree(bynd->log_block_version);
	kfree(bynd->page_buf);
	kfree(bynd);
}

#ifdef CONFIG_BYND_WRITE_CACHE
static int bynd_release(struct mtd_blktrans_dev *dev)
{
	int ret = 0;
	struct bynd_record *bynd = (struct bynd_record *)dev;

	mutex_lock(&dev->lock);

	if (bynd->cache_state == STATE_DIRTY) {
		struct rewrite_args args = {NULL, ULONG_MAX};
		ret = bynd_cached_write(bynd, &args);
	}

	if (ret == 0)
		bynd->cache_state = STATE_EMPTY;

	mutex_unlock(&dev->lock);

	return 0;
}

static int bynd_flush(struct mtd_blktrans_dev *dev)
{
	int ret = 0;
	struct bynd_record *bynd = (struct bynd_record *)dev;

	mutex_lock(&dev->lock);

	if (bynd->cache_state == STATE_DIRTY) {
		struct rewrite_args args = {NULL, ULONG_MAX};
		ret = bynd_cached_write(bynd, &args);
	}

	if (ret == 0)
		bynd->cache_state = STATE_EMPTY;

	mutex_unlock(&dev->lock);

	return 0;
}
#endif

#ifdef CONFIG_SUSPEND_TO_MTD
/* flush cache and rebuild mapping */
static int bynd_rescan(struct mtd_blktrans_dev *dev)
{
	int ret = 0;
	struct bynd_record *bynd = (struct bynd_record *)dev;

	mutex_lock(&dev->lock);

#ifdef CONFIG_BYND_WRITE_CACHE
	if (bynd->cache_state == STATE_DIRTY) {
		struct rewrite_args args = {NULL, ULONG_MAX};
		ret = bynd_cached_write(bynd, &args);
	}

	if (ret == 0)
		bynd->cache_state = STATE_EMPTY;
#endif
	if (ret == 0) {
		struct bynd_list_entry *free_e;

		/* onebiterror list should be empty here */
		BUG_ON(!list_empty(&bynd->onebiterror));
		/* cleanup erase list */
		while ((free_e = bynd_get_erase(bynd)))	{
			list_del(&free_e->list);
			kfree(free_e);
		}
		bynd->badblocks = 0;
		memset(bynd->log_block_map, 0xff,
		       sizeof(bynd->log_block_map[0]) * bynd->map_size);
		memset(bynd->log_block_version, 0xff,
		       sizeof(bynd->log_block_version[0]) * bynd->map_size);
		ret = build_logical_block_map(bynd);
	}

	mutex_unlock(&dev->lock);

	return ret;
}
#endif /* CONFIG_SUSPEND_TO_MTD */

static int bynd_readsect(struct mtd_blktrans_dev *dev,
				unsigned long log_sect_no, char *buf)
{
	struct bynd_record *bynd = (struct bynd_record *)dev;
	int sectors_per_block, offs, block_addr = 0, logaddr;
	unsigned long sect_no;
	int ret;

	sectors_per_block = bynd->mbd.mtd->erasesize >> SECTOR_SHIFT;
	offs = (int)(log_sect_no % sectors_per_block);
	logaddr = (int)(log_sect_no / sectors_per_block);

	block_addr = bynd->log_block_map[logaddr];

	if (block_addr >= bynd->map_len) {
		printk(KERN_ERR "BYND: block_addr %x is too big.\n",
						block_addr);
		return -EIO;
	}

#ifdef CONFIG_BYND_WRITE_CACHE
	if (bynd->cache_state == STATE_DIRTY && bynd->cache_logaddr ==
								logaddr) {
		memcpy(buf, bynd->page_buf + (offs<<SECTOR_SHIFT), SECTOR_SIZE);
		return 0;
	}
#endif

	if (block_addr < 0) {
		memset(&buf[0], 0xff, sizeof(char) * SECTOR_SIZE);
		return 0;
	}

	sect_no = (unsigned long)block_addr * sectors_per_block + offs;
	ret = read_physical_sector(bynd->mbd.mtd, buf, sect_no);
	if (ret < 0) {
		if (ret == -EUCLEAN) {
			printk(KERN_WARNING
				"BYND: read ecc corrected (sect_no %lx) %x.\n",
				sect_no, ret);
			bynd_move_block(bynd, block_addr);
			return 0;
		} else {
			printk(KERN_ERR "BYND: read error (sect_no %lx) %x.\n",
				sect_no, ret);
			return ret;
		}
	}

	return 0;
}

static void bynd_erase_callback(struct erase_info *instr)
{
	wake_up((wait_queue_head_t *)instr->priv);
}

static int bynd_eraseblock(struct mtd_info *mtd, unsigned long addr)
{
	int ret = 0, retries = 0;
	struct erase_info *erase;

	erase = kmalloc(sizeof(struct erase_info), GFP_KERNEL);
	if (!erase) {
		ret = -ENOMEM;
	} else {
		wait_queue_head_t waitq;
		DECLARE_WAITQUEUE(wait, current);

retry:
		init_waitqueue_head(&waitq);
		memset(erase, 0, sizeof(struct erase_info));

		erase->mtd = mtd;
		erase->callback = bynd_erase_callback;
		erase->priv = (unsigned long)&waitq;
		erase->addr = addr;
		erase->len  = mtd->erasesize;
		ret = mtd->erase(mtd, erase);
		if (!ret) {
			set_current_state(TASK_UNINTERRUPTIBLE);
			add_wait_queue(&waitq, &wait);
			if (erase->state != MTD_ERASE_DONE &&
			    erase->state != MTD_ERASE_FAILED) {
				schedule();
			}
			remove_wait_queue(&waitq, &wait);
			set_current_state(TASK_RUNNING);

#ifdef BYND_DEBUG
			if (erase->state == MTD_ERASE_FAILED ||
				bynd_debug_erase_retries) {
				if (retries == (bynd_debug_erase_retries - 1))
					bynd_debug_erase_retries = 0;
#else
			if (erase->state == MTD_ERASE_FAILED) {
#endif
				if (retries++ < bynd_erase_retries) {
					yield();
					printk(KERN_INFO "erase retry!!\n");
					goto retry;
				}
#ifdef BYND_DEBUG
				bynd_debug_erase_retries -=
						(bynd_erase_retries + 1);
				bynd_debug_erase_retries =
						(bynd_debug_erase_retries > 0) ?
						bynd_debug_erase_retries : 0;
#endif
				ret = -EIO;
			}
		}
		kfree(erase);
	}
	return ret;
}

static int bynd_writesect(struct mtd_blktrans_dev *dev,
				unsigned long log_sect_no, char *buf)
{
	struct rewrite_args args = {buf, log_sect_no};

#ifdef CONFIG_BYND_WRITE_CACHE
	return bynd_cached_write((struct bynd_record *)dev, &args);
#else
	return bynd_rewrite_block((struct bynd_record *)dev, &args, 0);
#endif
}

static int bynd_getgeo(struct mtd_blktrans_dev *dev,  struct hd_geometry *geo)
{
	struct bynd_record *bynd = (struct bynd_record *)dev;

	DEBUG(MTD_DEBUG_LEVEL1, "BYND: bynd_getgeo() C=%d, H=%d, S=%d\n",
			bynd->cylinders, bynd->heads, bynd->sectors);

	geo->heads = bynd->heads;
	geo->sectors = bynd->sectors;
	geo->cylinders = bynd->cylinders;

	return 0;
}

#ifdef BYND_DEBUG
static int proc_write(struct file *filp, const char __user *buf,
						unsigned long len, void *data)
{
	char change_config;
	int change_value;
	char mod_buf[BYND_PARAM_LEN_MAX];

	if (len >= BYND_PARAM_LEN_MAX) {
		printk(KERN_WARNING "proc_write len = %lu\n", len);
		return -ENOSPC;
	}

	if (copy_from_user(mod_buf, buf, len))
		return -EFAULT;

	sscanf(mod_buf, "%c=%d", &change_config, &change_value);

	if (change_value < 0)
		return -ENOSPC;

	switch (change_config) {
	case 'c':
		switch (change_value) {
		case 0:
			bynd_debug_compare_head_oob = true;
			printk(KERN_INFO "test compare head oob update\n");
			break;
		case 1:
			bynd_debug_compare_tail_oob = true;
			printk(KERN_INFO "test compare tail oob update\n");
			break;
		default:
			printk(KERN_INFO "c=0: head oob, c=1: tail oob\n");
			break;
		}
		break;
	case 'e':
		switch (change_value) {
		case 0:
			bynd_debug_erase_retries = 1;
			break;
		case 1:
			bynd_debug_erase_retries = bynd_erase_retries;
			break;
		case 2:
			bynd_debug_erase_retries = bynd_erase_retries + 1;
			break;
		case 3:
			bynd_debug_erase_retries = (bynd_erase_retries + 1)
							* bynd_write_retries;
			break;
		case 4:
			bynd_debug_erase_retries = (bynd_erase_retries + 1)
						* (bynd_write_retries + 1);
			break;
		default:
			break;
		}

		printk(KERN_INFO
				"erase retry test: fails in erasing %d times\n",
				bynd_debug_erase_retries);
		break;
	case 'w':
		switch (change_value) {
		case 0:
			bynd_debug_write_retries = 1;
			break;
		case 1:
			bynd_debug_write_retries = bynd_write_retries;
			break;
		case 2:
			bynd_debug_write_retries = bynd_write_retries + 1;
			break;
		default:
			break;
		}

		printk(KERN_INFO
				"write retry test: fails in writing %d times\n",
				bynd_debug_write_retries);
		break;
	default:
		return -EINVAL;
		break;
	}

	return len;
}
#endif

static int bynd_proc_info(char *buf, int i)
{
	int erase_blocks = 0;
	struct list_head *p;
	struct bynd_record *this = bynd_table[mtd_dev_param[i]];

	if (!this)
		return 0;

	list_for_each(p, &this->erase) {
		erase_blocks++;
	}

	return sprintf(buf, "bynd%-2d: mtd%-2d    %-9.8x    %-15d    "
		"%-14d    %-11d    %-10d    "
		"%-10d    %-8d    %-8d    %4d/%4d\n",
		mtd_dev_param[i], this->mbd.devnum, this->mbd.mtd->erasesize,
		this->map_len, this->map_size, erase_blocks, this->badblocks,
		SECTOR_SIZE, bynd_write_retries, bynd_erase_retries,
		max_logic_blk_per_zone, MAX_PHYS_BLK_PER_ZONE);
}

static int proc_read(char *page, char **start, off_t offset, int count,
						int *eof, void *data)
{
	int len, l, i;
	off_t begin = 0;

	len = sprintf(page, "dev   : mtd      erasesize    physical_blocks    "
		"logical_blocks    free_blocks    bad_blocks    "
		"sector_size    wretries    eretries    logical_size\n");
	for (i = 0; i < BYND_MAX_DEVICES; i++) {
		l = bynd_proc_info(page + len, i);
		len += l;
		if (len + begin > offset + count)
			goto done;
		if (len + begin < offset) {
			begin += len;
			len = 0;
		}
	}

	*eof = 1;

done:
	if (offset >= len + begin)
		return 0;
	*start = page + (offset - begin);
	return ((count < begin + len - offset) ? count : begin + len - offset);
}

/****************************************************************************
 *
 * Module stuff
 *
 ****************************************************************************/

static struct mtd_blktrans_ops bynd_tr = {
	.name		= "bynd",
	.major		= BYND_MAJOR,
	.part_bits	= BYND_PARTN_BITS,
	.blksize	= SECTOR_SIZE,
	.getgeo		= bynd_getgeo,
#ifdef CONFIG_BYND_WRITE_CACHE
	.flush		= bynd_flush,
	.release	= bynd_release,
#endif
#ifdef CONFIG_SUSPEND_TO_MTD
	.rescan		= bynd_rescan,
#endif
	.readsect	= bynd_readsect,
	.writesect	= bynd_writesect,
	.add_mtd	= bynd_add_mtd,
	.remove_dev	= bynd_remove_dev,
	.owner		= THIS_MODULE,
};

static int part_bits = BYND_PARTN_BITS;
module_param(part_bits, int, 0444);
MODULE_PARM_DESC(part_bits, "bits for partition of a bynd block device");

static int __init init_bynd(void)
{
	int i = 0, ret;
	struct proc_dir_entry *entry;

	while (i < BYND_MAX_DEVICES)
		bynd_table[i++] = NULL;

	printk(KERN_INFO "BYND version %s read / write Flash Translation "
			"layer\n", BYND_VERSION);
	bynd_tr.part_bits = part_bits;
	ret = register_mtd_blktrans(&bynd_tr);

	/* create proc file */
#ifdef BYND_DEBUG
	entry = create_proc_entry(PROCNAME, 0666, NULL);
#else
	entry = create_proc_entry(PROCNAME, 0444, NULL);
#endif
	if (entry) {
#ifdef BYND_DEBUG
		entry->write_proc = proc_write;
#endif
		entry->read_proc = proc_read;
		entry->owner = THIS_MODULE;
	} else {
		printk(KERN_ERR "create_proc_entry failed\n");
		return -EBUSY;
	}

#ifdef BYND_DEBUG
	bynd_debug_write_retries = bynd_debug_erase_retries = 0;
	bynd_debug_compare_head_oob = bynd_debug_compare_tail_oob = false;
#endif

	return ret;
}

static void __exit cleanup_bynd(void)
{
	deregister_mtd_blktrans(&bynd_tr);
	remove_proc_entry(PROCNAME, NULL);
}

static int __init bynd_param_parse(const char *val, struct kernel_param *kp)
{
	int len;

	if (!val)
		return -EINVAL;

	if (mtd_devs == BYND_MAX_DEVICES) {
		printk(KERN_ERR "BYND error: too many parameters, max. is %d\n",
		       BYND_MAX_DEVICES);
		return -EINVAL;
	}

	len = strnlen(val, BYND_PARAM_LEN_MAX);
	if (len == BYND_PARAM_LEN_MAX) {
		printk(KERN_ERR "BYND error: parameter \"%s\" is too long, "
		       "max. is %d\n", val, BYND_PARAM_LEN_MAX);
		return -EINVAL;
	}

	if (len == 0) {
		printk(KERN_WARNING "BYND warning: empty 'mtd=' parameter - "
		       "ignored\n");
		return 0;
	}

	mtd_dev_param[mtd_devs] = simple_strtoul(val, NULL, 0);

	mtd_devs += 1;
	return 0;
}

static int __init bynd_max_logic_blk_per_zone_param_parse(const char *val,
						struct kernel_param *kp)
{
	int len;

	if (!val)
		return -EINVAL;

	len = strnlen(val, BYND_PARAM_LEN_MAX);
	if (len == BYND_PARAM_LEN_MAX) {
		printk(KERN_ERR "BYND error: parameter \"%s\" is too long, "
		       "max. is %d\n", val, BYND_PARAM_LEN_MAX);
		return -EINVAL;
	}

	if (len == 0) {
		printk(KERN_WARNING
			"BYND warning: empty 'ls=' parameter - "
			"set default max_logic_blk_per_zone %d\n",
			max_logic_blk_per_zone);
		return 0;
	}

	max_logic_blk_per_zone = simple_strtol(val, NULL, 0);

	if (max_logic_blk_per_zone > MAX_PHYS_BLK_PER_ZONE ||
						max_logic_blk_per_zone < 0) {
		printk(KERN_ERR
			"BYND error: max_logic_blk_per_zone is bad value.\n");
		return -EINVAL;
	}
	

	return 0;
}

static int __init bynd_write_retry_param_parse(const char *val,
						struct kernel_param *kp)
{
	int len;

	if (!val)
		return -EINVAL;

	len = strnlen(val, BYND_PARAM_LEN_MAX);
	if (len == BYND_PARAM_LEN_MAX) {
		printk(KERN_ERR "BYND error: parameter \"%s\" is too long, "
		       "max. is %d\n", val, BYND_PARAM_LEN_MAX);
		return -EINVAL;
	}

	if (len == 0) {
		printk(KERN_WARNING
			"BYND warning: empty 'wretry=' parameter - "
			"set default bynd_write_retries %d\n",
			bynd_write_retries);
		return 0;
	}

	bynd_write_retries = simple_strtol(val, NULL, 0);

	return 0;
}

static int __init bynd_erase_retry_param_parse(const char *val,
						struct kernel_param *kp)
{
	int len;

	if (!val)
		return -EINVAL;

	len = strnlen(val, BYND_PARAM_LEN_MAX);
	if (len == BYND_PARAM_LEN_MAX) {
		printk(KERN_ERR "BYND error: parameter \"%s\" is too long, "
		       "max. is %d\n", val, BYND_PARAM_LEN_MAX);
		return -EINVAL;
	}

	if (len == 0) {
		printk(KERN_WARNING
			"BYND warning: empty 'eretry=' parameter - "
			"set default bynd_erase_retries %d\n",
			bynd_erase_retries);
		return 0;
	}

	bynd_erase_retries = simple_strtol(val, NULL, 0);

	return 0;
}

module_param_call(mtd, bynd_param_parse, NULL, NULL, 000);
module_param_call(ls, bynd_max_logic_blk_per_zone_param_parse, NULL, NULL, 000);
module_param_call(wretry, bynd_write_retry_param_parse, NULL, NULL, 000);
module_param_call(eretry, bynd_erase_retry_param_parse, NULL, NULL, 000);
MODULE_PARM_DESC(mtd, "MTD devices to attach. ");
MODULE_PARM_DESC(ls, "BYND logical blocks per physical blocks ratio.");
MODULE_PARM_DESC(wretry, "BYND write retry time. ");
MODULE_PARM_DESC(eretry, "BYND erase retry time. ");

module_init(init_bynd);
module_exit(cleanup_bynd);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("TOSHIBA");
MODULE_DESCRIPTION("BYND");
