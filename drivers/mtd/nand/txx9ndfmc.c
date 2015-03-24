/*
 * drivers/mtd/nand/txx9ndfmc.c
 *
 * Based on spia.c by Steven J. Hill
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2007
 * All Rights Reserved.
 *
 *  Overview:
 *   This is a device driver for the NAND flash device connected to
 *   TX4925/TX4938/TX4939 internal NAND Memory Controller.
 *
 *  Revision History:
 *	1.00	New NAND API. (Unified from tx4925ndfmc.c and tx4938ndfmc.c)
 *	1.01	TX4939 support.
 *	1.02	Kernel 2.6.18
 *	1.03	Use platform_device.  PM support.  Fix BSPRT setting.
 */
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mm.h>	/* for high_memory */
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <asm/io.h>
#include <asm/tx-boards/generic.h>
#include <asm/tx-boards/ndfmc.h>
#include <asm/tx-boards/dma.h>

/* TXX9 NDFMC Registers */
#define TXX9_NDFDTR	0x00
#define TXX9_NDFMCR	0x04
#define TXX9_NDFSR	0x08
#define TXX9_NDFISR	0x0c
#define TXX9_NDFIMR	0x10
#define TXX9_NDFSPR	0x14
#define TXX9_NDFRSTR	0x18	/* not TX4939 */
#define TXX9_NDFECTR	0x38
#define TXX9_NDFFRR	0x2c	/* TC90412 */
#define TXX9_NDFFRTR	0x30	/* TC90412 */
#if defined(CONFIG_TOSHIBA_TC90416)
#define TXX9_NDFWRR	0x3C
#define HAVE_TXX9_NDFWRR
#define NO_DELAY_FOR_DMAPOLL
#endif
#if defined(CONFIG_TOSHIBA_TC90412) || defined(CONFIG_TOSHIBA_TC90416)
#define TXX9_NDFDBR	0x24
#define TXX9_NDFDBR_LBLK	0x04
#define HAVE_TXX9_NDFDBR
#endif

/* NDFMCR : NDFMC Mode Control */
#define TXX9_NDFMCR_WE	0x80
#define TXX9_NDFMCR_ECC_ALL	0x60
#define TXX9_NDFMCR_ECC_RESET	0x60
#define TXX9_NDFMCR_ECC_READ	0x40
#define TXX9_NDFMCR_ECC_ON	0x20
#define TXX9_NDFMCR_ECC_OFF	0x00
#define TXX9_NDFMCR_CE	0x10
#define TXX9_NDFMCR_BSPRT	0x04	/* TX4925/TX4926 only */
#define TXX9_NDFMCR_ALE	0x02
#define TXX9_NDFMCR_CLE	0x01
/* TX4939 only */
#define TXX9_NDFMCR_X16	0x0400
#define TXX9_NDFMCR_DMAREQ_MASK	0x0300
#define TXX9_NDFMCR_DMAREQ_NODMA	0x0000
#define TXX9_NDFMCR_DMAREQ_128	0x0100
#define TXX9_NDFMCR_DMAREQ_256	0x0200
#define TXX9_NDFMCR_DMAREQ_512	0x0300
#define TXX9_NDFMCR_CS_MASK	0x0c
#define TXX9_NDFMCR_CS(ch)	((ch) << 2)

/* NDFMCR : NDFMC Status */
#define TXX9_NDFSR_BUSY	0x80
/* TX4939 only */
#define TXX9_NDFSR_DMARUN	0x40

/* NDFMCR : NDFMC Reset */
#define TXX9_NDFRSTR_RST	0x01

/* TC90412 or later */
#define TXX9_NDFMCR_DMAE_DIS	0x0000
#define TXX9_NDFMCR_DMAE_ENA	0x0100
#define TXX9_NDFMCR_DMAE_SEQENA	0x0300
#define TXX9_NDFMCR_DMAE_MASK	0x0300
/* NDFISR : NDFMC Interrupt Status */
#define TXX9_NDFISR_FIFRE	0x08
#define TXX9_NDFISR_ECC		0x02

#define DYNAMIC_TIMING_CONFIGURATION

#ifdef CONFIG_MTD_NAND_TXX9NDFMC_OOB_HWLAYOUT
static struct nand_ecclayout txx9ndfmc_oobinfo_512 = {
	.eccbytes = 6,
	.eccpos = { 14, 13, 15, 9, 8, 10 },
	.oobfree = {{.offset = 0, .length = 4},
		    {.offset = 6, .length = 2},
		    {.offset = 11, .length = 2}}
};
static struct nand_ecclayout txx9ndfmc_oobinfo_2048 = {
	.eccbytes = 24,
	.eccpos = { 14, 13, 15, 9, 8, 10,
		    30, 29, 31, 25, 24, 26,
		    46, 45, 47, 41, 40, 42,
		    62, 61, 63, 57, 56, 58 },
	.oobfree = {{.offset = 1, .length = 7},
		    {.offset = 11, .length = 2},
		    {.offset = 16, .length = 8},
		    {.offset = 27, .length = 2},
		    {.offset = 32, .length = 8},
		    {.offset = 43, .length = 2},
		    {.offset = 48, .length = 8},
		    {.offset = 59, .length = 2}}
};
#endif

struct txx9ndfmc_priv {
	unsigned int shift;
	int cs;
	char mtdname[16];
	struct ndfmc_platform_data *plat;
	struct platform_device *dev;
};

static int use_dma = 1;
module_param(use_dma, bool, 0444);
MODULE_PARM_DESC(use_dma, "enable DMA for data transfering from NAND HW");
static int use_dmaint = 1;
module_param(use_dmaint, bool, 0444);
MODULE_PARM_DESC(use_dmaint, "use DMA interrupt");

static inline void __iomem *ndregaddr(struct nand_chip *this, unsigned int reg)
{
	struct txx9ndfmc_priv *txx9_priv = this->priv;
	return this->IO_ADDR_R + (reg << txx9_priv->shift);
}

#define ndread(this, reg)	__raw_readl(ndregaddr(this, reg))
#define ndwrite(this, val, reg)	__raw_writel(val, ndregaddr(this, reg))

#define MAX_TXX9NDFMC_DEV	4
struct txx9ndfmc_drvdata {
	struct mtd_info *mtds[MAX_TXX9NDFMC_DEV];
	unsigned char hold;	/* in gbusclock */
	unsigned char spw;	/* in gbusclock */
	unsigned char aledelay;	/* in gbusclock */
	unsigned char rrtime;	/* in gbusclock */
	int dmanr;
	int dmamode;
	int dmairq;
	u64 dmareg;
	struct completion cmp;
	void (*nand_command_org)(struct mtd_info *mtd, unsigned int command,
				 int column, int page_addr);
	int start_page_addr;
	int cur_page_addr;
};


/*
 * Define partitions for flash device
 */
#define flush_wb()	(void)ndread(this, TXX9_NDFMCR)

static uint8_t txx9ndfmc_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	// return readl(ncp->IO_ADDR);
	return ndread(this, TXX9_NDFDTR);
}

static void txx9ndfmc_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip *this = mtd->priv;
	int i;
	u32 mcr = ndread(this, TXX9_NDFMCR);
	ndwrite(this, mcr | TXX9_NDFMCR_WE, TXX9_NDFMCR);
	for (i=0; i<len; i++)
		ndwrite(this, buf[i], TXX9_NDFDTR);
	ndwrite(this, mcr, TXX9_NDFMCR);
	flush_wb();

}

#ifdef HAVE_TXX9_NDFWRR
static void txx9ndfmc_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *this = mtd->priv;
	int i,_len = (len % 4);
	u32 *buf32 = (u32 *)buf;
	for (i=0; i<(len/4); i++)
		buf32[i] = ndread(this, TXX9_NDFWRR);
	for (i=(len-_len); i<len; i++)
		buf[i] = ndread(this, TXX9_NDFDTR);
}

static int txx9ndfmc_verify_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip *this = mtd->priv;
	int i,_len = (len % 4);
	u32 *buf32 = (u32 *)buf;
	for (i=0; i<(len/4); i++)
		if (buf32[i] != ndread(this, TXX9_NDFWRR))
			return -EFAULT;
	for (i=(len-_len); i<len; i++)
		if (buf[i] != (uint8_t)ndread(this, TXX9_NDFDTR))
			return -EFAULT;
	return 0;
}

#ifdef CONFIG_MTD_NAND_VERIFY_WRITE_SUBPAGE
static int txx9ndfmc_verify_buf_column(struct mtd_info *mtd, const uint8_t *buf, int len,
				       int column, int verify_size)
{
	struct nand_chip *this = mtd->priv;
	int i;
	u32 *buf32 = (u32 *)buf;
	if ((column % 4) || (verify_size % 4) || ((column + verify_size) > len))
		BUG();
	for (i=0; i<((column + verify_size)/4); i++) {
		if (i >= (column/4)) {
			if (buf32[i] != ndread(this, TXX9_NDFWRR))
				return -EFAULT;
		} else
			ndread(this, TXX9_NDFWRR);
	}
	return 0;
}
#endif

#else
static void txx9ndfmc_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *this = mtd->priv;
	int i;
	for (i=0; i<len; i++)
		buf[i] = ndread(this, TXX9_NDFDTR);
}

static int txx9ndfmc_verify_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip *this = mtd->priv;
	int i;
	for (i=0; i<len; i++)
		if (buf[i] != (uint8_t)ndread(this, TXX9_NDFDTR))
			return -EFAULT;
	return 0;
}

#ifdef CONFIG_MTD_NAND_VERIFY_WRITE_SUBPAGE
static int txx9ndfmc_verify_buf_column(struct mtd_info *mtd, const uint8_t *buf, int len,
				       int column, int verify_size)
{
	struct nand_chip *this = mtd->priv;
	int i;
	if ((column + verify_size) > len)
		BUG();
	for (i=0; i<(column + verify_size); i++) {
		if (i >= column) {
			if (buf[i] != (uint8_t)ndread(this, TXX9_NDFDTR))
				return -EFAULT;
		} else
			ndread(this, TXX9_NDFDTR);
	}
	return 0;
}
#endif

#endif

static void txx9ndfmc_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *this = mtd->priv;
	struct txx9ndfmc_priv *txx9_priv = this->priv;
	if (ctrl & NAND_CTRL_CHANGE) {
		u32 mcr = ndread(this, TXX9_NDFMCR);
		mcr &= ~(TXX9_NDFMCR_CLE | TXX9_NDFMCR_ALE | TXX9_NDFMCR_CE);
		mcr |= ctrl & NAND_CLE ? TXX9_NDFMCR_CLE : 0;
		mcr |= ctrl & NAND_ALE ? TXX9_NDFMCR_ALE : 0;
		/* TXX9_NDFMCR_CE bit is 0:high 1:low */
		mcr |= ctrl & NAND_NCE ? TXX9_NDFMCR_CE : 0;
		if (txx9_priv->cs >= 0 && (ctrl & NAND_NCE)) {
			mcr &= ~TXX9_NDFMCR_CS_MASK;
			mcr |= TXX9_NDFMCR_CS(txx9_priv->cs);
		}
		ndwrite(this, mcr, TXX9_NDFMCR);
	}
	if (cmd != NAND_CMD_NONE)
		ndwrite(this, cmd & 0xff, TXX9_NDFDTR);
	flush_wb();
}
static int txx9ndfmc_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	flush_wb();
	return !(ndread(this, TXX9_NDFSR) & TXX9_NDFSR_BUSY);
}
static struct nand_errstat_cmd *txx9ndfmc_errstat[FL_STATE_MAX];
static int txx9ndfmc_do_errstat(struct mtd_info *mtd, struct nand_chip *this, int state, int status, int page)
{
	struct nand_errstat_cmd *esc = txx9ndfmc_errstat[state];
	struct nand_chip *chip = mtd->priv;

	if (esc == NULL)
		return status;

	DEBUG(MTD_DEBUG_LEVEL0, "txx9ndfmc_errstat: "
	      "additional error status check enabled, state=0x%x\n", state);

	for ( ; esc->maf_id != -1; esc++) {
		if ((this->maf_id != esc->maf_id) || (this->dev_id != esc->dev_id))
			continue;

		switch (esc->command) {
		case NAND_CMD_STATUS:
		case NAND_CMD_STATUS_MULTI:
			status = this->waitfunc(mtd, this);
			DEBUG(MTD_DEBUG_LEVEL3, "cmd: %02X, status: %02X\n",
			      esc->command, status);
			break;

		default:
			DEBUG(MTD_DEBUG_LEVEL3,
			      "cmd: %02X, col: %d, page: %s\n",
			      esc->command, esc->column,
			      esc->page_addr ? "yes" : "no");
			this->cmdfunc(mtd, esc->command, esc->column,
				      esc->page_addr ? page & chip->pagemask : -1);
			break;
		}
	}
	
	return status;
}
static int txx9ndfmc_get_errstat(struct mtd_info *mtd, int state, struct nand_errstat_cmd **esc, size_t *count)
{
	struct nand_errstat_cmd *c;

	if ((state < 0) || (FL_STATE_MAX <= state) || (count == NULL))
		return -EINVAL;

	if (txx9ndfmc_errstat[state] == NULL) {
		*count = 0;
		return 0;
	}

	for (c = txx9ndfmc_errstat[state]; c->maf_id != -1; c++);
	*count = c - txx9ndfmc_errstat[state];

	if (esc != NULL) {
		*esc = txx9ndfmc_errstat[state];
	}

	return 0;
}
static int txx9ndfmc_set_errstat(struct mtd_info *mtd, int state, struct nand_errstat_cmd *esc, size_t count)
{
	int i;

	if (state == -1) {
		for (i = 0; i < FL_STATE_MAX; i++) {
			if (txx9ndfmc_errstat[i] != NULL) {
				kfree(txx9ndfmc_errstat[i]);
				txx9ndfmc_errstat[i] = NULL;
			}
		}
		return 0;
	}

	if ((state < 0) || (FL_STATE_MAX <= state))
		return -EINVAL;

	if (txx9ndfmc_errstat[state] != NULL) {
		kfree(txx9ndfmc_errstat[state]);
		txx9ndfmc_errstat[state] = NULL;
	}

	if ((count == 0) && (esc == NULL)) {
		return 0;
	}

	txx9ndfmc_errstat[state] = esc;

	return 0;
}
static int txx9ndfmc_read_ecc(struct nand_chip *this, int eccbytes,
			      uint8_t *ecc_code)
{
	u32 mcr = ndread(this, TXX9_NDFMCR);
	mcr &= ~TXX9_NDFMCR_ECC_ALL;
	ndwrite(this, mcr | TXX9_NDFMCR_ECC_OFF, TXX9_NDFMCR);
	ndwrite(this, mcr | TXX9_NDFMCR_ECC_READ, TXX9_NDFMCR);
	for (; eccbytes > 0; eccbytes -= 3) {
		ecc_code[1] = ndread(this, TXX9_NDFDTR);
		ecc_code[0] = ndread(this, TXX9_NDFDTR);
		ecc_code[2] = ndread(this, TXX9_NDFDTR);
		ecc_code += 3;
	}
	ndwrite(this, mcr | TXX9_NDFMCR_ECC_OFF, TXX9_NDFMCR);
	return 0;
}
static int txx9ndfmc_calculate_ecc(struct mtd_info *mtd, const uint8_t *dat, uint8_t *ecc_code)
{
	struct nand_chip *this = mtd->priv;
	return txx9ndfmc_read_ecc(this, this->ecc.bytes, ecc_code);
}
static void txx9ndfmc_enable_hwecc(struct mtd_info *mtd, int mode)
{
	struct nand_chip *this = mtd->priv;
	u32 mcr = ndread(this, TXX9_NDFMCR);
	mcr &= ~TXX9_NDFMCR_ECC_ALL;
	ndwrite(this, mcr | TXX9_NDFMCR_ECC_RESET, TXX9_NDFMCR);
	ndwrite(this, mcr | TXX9_NDFMCR_ECC_OFF, TXX9_NDFMCR);
	ndwrite(this, mcr | TXX9_NDFMCR_ECC_ON, TXX9_NDFMCR);
}

static int txx9ndfmc_correct_data(struct mtd_info *mtd, unsigned char *buf,
		unsigned char *read_ecc, unsigned char *calc_ecc)
{
	struct nand_chip *chip = mtd->priv;
	int eccsize;
	int corrected = 0;
	int stat;

	for (eccsize = chip->ecc.size; eccsize > 0; eccsize -= 256) {
		stat = nand_correct_data(mtd, buf, read_ecc, calc_ecc);
		if (stat < 0)
			return stat;
		corrected += stat;
		buf += 256;
		read_ecc += 3;
		calc_ecc += 3;
	}
	return corrected;
}

static void txx9ndfmc_nand_command(struct mtd_info *mtd, unsigned int command,
				   int column, int page_addr)
{
	struct nand_chip *chip = mtd->priv;
	struct txx9ndfmc_priv *txx9_priv = chip->priv;
	struct txx9ndfmc_drvdata *drvdata =
		platform_get_drvdata(txx9_priv->dev);

	if (command == NAND_CMD_READ0) {
		drvdata->start_page_addr = page_addr;
		drvdata->cur_page_addr = page_addr;
	}
	drvdata->nand_command_org(mtd, command, column, page_addr);
}

static int txx9ndfmc_dma_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *this = mtd->priv;
	struct txx9ndfmc_priv *txx9_priv = this->priv;
	struct txx9ndfmc_drvdata *drvdata =
		platform_get_drvdata(txx9_priv->dev);
	dma_addr_t dmabuf;
	unsigned int stat;

	/* vmalloc-ed buffer? */
	if ((void *)buf >= high_memory)
		goto fallback;
	dmabuf = dma_map_single(NULL, buf, len, DMA_FROM_DEVICE);
	if (dma_mapping_error(dmabuf))
		goto fallback;
	if (drvdata->dmamode & TXx9_DMA_CCR_EXTRQ) {
		u32 mcr;
		ndwrite(this, TXX9_NDFISR_FIFRE | TXX9_NDFISR_ECC, TXX9_NDFISR);
		mcr = ndread(this, TXX9_NDFMCR);
		mcr &= ~TXX9_NDFMCR_DMAE_MASK;
		if (mtd->writesize > 512 ||
		    drvdata->start_page_addr == drvdata->cur_page_addr)
			mcr |= TXX9_NDFMCR_DMAE_ENA;
		else
			mcr |= TXX9_NDFMCR_DMAE_SEQENA;
		ndwrite(this, mcr, TXX9_NDFMCR);
		set_txx9_dma_inc(drvdata->dmanr, 0, 8);
	} else
		set_txx9_dma_inc(drvdata->dmanr, 0, 1);
	set_txx9_dma_addr64(drvdata->dmanr, drvdata->dmareg, dmabuf);
	set_txx9_dma_count(drvdata->dmanr, len);
	clear_txx9_dma_status(drvdata->dmanr);
	if (use_dmaint) {
		set_txx9_dma_mode(drvdata->dmanr,
				  drvdata->dmamode |
				  TXx9_DMA_CCR_INTENE | TXx9_DMA_CCR_INTENC);
		INIT_COMPLETION(drvdata->cmp);
		enable_txx9_dma(drvdata->dmanr);
		if (!wait_for_completion_timeout(&drvdata->cmp, HZ))
			pr_debug("TXx9NDFMC DMA timeout\n");
		stat = get_txx9_dma_status(drvdata->dmanr);
	} else {
		unsigned long end_time = jiffies + HZ;
		enable_txx9_dma(drvdata->dmanr);
#ifdef NO_DELAY_FOR_DMAPOLL
		udelay(((drvdata->dmamode & TXx9_DMA_CCR_EXTRQ) ? 20 : 40) *
		       (len / 256));
#endif
		do {
			stat = get_txx9_dma_status(drvdata->dmanr);
			if (!(stat & TXx9_DMA_STATUS_CHNACT))
				break;
#ifdef NO_DELAY_FOR_DMAPOLL
			cpu_relax();
#else
			udelay(2);
#endif
		} while (time_after_eq(end_time, jiffies));
	}
	dma_unmap_single(NULL, dmabuf, len, DMA_FROM_DEVICE);
	if (!(stat & TXx9_DMA_STATUS_NTRNFC)) {
		printk(KERN_ERR "TXx9NDFMC DMA error %x\n", stat);
		disable_txx9_dma(drvdata->dmanr);
		goto fallback;
	}
	if (drvdata->dmamode & TXx9_DMA_CCR_EXTRQ) {
		u32 mcr;
		u32 isr;
		unsigned long end_time = jiffies + HZ;
		/* wait for completion of ECC calculation */
		do {
			isr = ndread(this, TXX9_NDFISR);
			if (isr & TXX9_NDFISR_FIFRE)
				break;
			cpu_relax();
		} while (time_after_eq(end_time, jiffies));
		mcr = ndread(this, TXX9_NDFMCR);
		mcr = (mcr & ~TXX9_NDFMCR_DMAE_MASK) | TXX9_NDFMCR_DMAE_DIS;
		ndwrite(this, mcr, TXX9_NDFMCR);
		if (!(isr & TXX9_NDFISR_ECC))
			return 0;	/* ECC check OK */
	}
	return -1;	/* need to check ECC */
fallback:
	txx9ndfmc_read_buf(mtd, buf, len);
	return -1;
}

static irqreturn_t txx9ndfmc_dma_irq(int irq, void *dev_id)
{
	struct platform_device *dev = dev_id;
	struct txx9ndfmc_drvdata *drvdata = platform_get_drvdata(dev);
	unsigned int stat = get_txx9_dma_status(drvdata->dmanr);

	if (stat & TXx9_DMA_STATUS_CHNACT)
		return IRQ_NONE;
	/* disable INTENE, INTENC interrupt */
	set_txx9_dma_mode(drvdata->dmanr, drvdata->dmamode);
	complete(&drvdata->cmp);
	return IRQ_HANDLED;
}

static int txx9ndfmc_dma_read_page(struct mtd_info *mtd, struct nand_chip *chip,
				   uint8_t *buf)
{
	struct txx9ndfmc_priv *txx9_priv = chip->priv;
	struct txx9ndfmc_drvdata *drvdata =
		platform_get_drvdata(txx9_priv->dev);
	int i, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint8_t *ecc_code = chip->buffers->ecccode;
	uint32_t *eccpos = chip->ecc.layout->eccpos;

	txx9ndfmc_enable_hwecc(mtd, NAND_ECC_READ);
	if (txx9ndfmc_dma_read_buf(mtd, buf, mtd->writesize) == 0)
		goto end;
	txx9ndfmc_read_ecc(mtd->priv, chip->ecc.total, ecc_calc);
	if (drvdata->dmamode & TXx9_DMA_CCR_EXTRQ) {
		/* set page address before reading OOB */
		nand_wait_ready(mtd);
		txx9ndfmc_nand_command(mtd, NAND_CMD_READOOB,
				       0, drvdata->cur_page_addr);
	}
	txx9ndfmc_read_buf(mtd, chip->oob_poi, mtd->oobsize);

	for (i = 0; i < chip->ecc.total; i++)
		ecc_code[i] = chip->oob_poi[eccpos[i]];

	for (i = 0 ; eccsteps; eccsteps--, i += eccbytes, buf += eccsize) {
		int stat;

		stat = txx9ndfmc_correct_data(mtd, buf,
					      &ecc_code[i], &ecc_calc[i]);
		if (stat == -1)
			mtd->ecc_stats.failed++;
		else
			mtd->ecc_stats.corrected += stat;
	}
end:
	drvdata->cur_page_addr = (drvdata->cur_page_addr + 1) & chip->pagemask;
	return 0;
}

static void txx9ndfmc_initialize(struct nand_chip *this)
{
	struct txx9ndfmc_priv *txx9_priv = this->priv;
	struct ndfmc_platform_data *plat = txx9_priv->dev->dev.platform_data;
	struct txx9ndfmc_drvdata *drvdata =
		platform_get_drvdata(txx9_priv->dev);
	int tmout = 10000;
	if (plat->flags & NDFMC_PLAT_FLAG_NO_RSTR) {
		/* no NDFRSTR.  Write to NDFSPR resets the NDFMC. */
	} else {
		/* reset NDFMC */
		ndwrite(this, ndread(this, TXX9_NDFRSTR) | TXX9_NDFRSTR_RST,
			TXX9_NDFRSTR);
		while (ndread(this, TXX9_NDFRSTR) & TXX9_NDFRSTR_RST) {
			if (--tmout == 0) {
				printk(KERN_ERR "TXX9 NDFMC: reset failed.\n");
				break;
			}
		}
	}
	/* setup Hold Time, Strobe Pulse Width */
	ndwrite(this, drvdata->hold << 4 | drvdata->spw, TXX9_NDFSPR);
	ndwrite(this,
		(plat->flags & NDFMC_PLAT_FLAG_USE_BSPRT) ?
		TXX9_NDFMCR_BSPRT : 0, TXX9_NDFMCR);
	if (plat->flags & NDFMC_PLAT_FLAG_USE_ALEDELAY)
		ndwrite(this, drvdata->aledelay, TXX9_NDFECTR);
	if (plat->flags & NDFMC_PLAT_FLAG_USE_RRTIME)
		ndwrite(this, (0x3f << 10) | (0x3f << 4) | drvdata->rrtime,
			TXX9_NDFFRTR);
}

#define TXX9NDFMC_NS_TO_CYC(ns) \
	(((ns) * ((TXX9_GBUSCLK + 500) / 1000) + 1000000 - 1) / 1000000)

static int txx9ndfmc_nand_scan(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct txx9ndfmc_priv *txx9_priv = chip->priv;
	struct txx9ndfmc_drvdata *drvdata =
		platform_get_drvdata(txx9_priv->dev);
	int ret;

	ret = nand_scan_ident(mtd, 1);
	if (!ret) {
		if (drvdata->dmanr >= 0) {
			drvdata->nand_command_org = chip->cmdfunc;
			chip->cmdfunc = txx9ndfmc_nand_command;
		}

		if (mtd->writesize >= 512) {
			if (mtd->writesize != 512 && mtd->writesize != 2048) {
				printk(KERN_WARNING
				       "TXX9 NDFMC: unsupported writesize %d\n",
				       mtd->writesize);
				return -EINVAL;
			}
#ifdef HAVE_TXX9_NDFDBR
			if (mtd->writesize > 512)
				ndwrite(chip, TXX9_NDFDBR_LBLK, TXX9_NDFDBR);
#endif
#ifdef CONFIG_MTD_NAND_TXX9NDFMC_RW_NOSUBPAGE
			chip->ecc.size = mtd->writesize;
			chip->ecc.bytes = 3 * (mtd->writesize / 256);
#endif
#ifdef CONFIG_MTD_NAND_TXX9NDFMC_OOB_HWLAYOUT
			if (mtd->writesize == 512)
				chip->ecc.layout = &txx9ndfmc_oobinfo_512;
			else if (mtd->writesize == 2048)
				chip->ecc.layout = &txx9ndfmc_oobinfo_2048;
			else {
				printk(KERN_WARNING
				       "TXX9 NDFMC: no layout for %d\n",
				       mtd->writesize);
				return -EINVAL;
			}
#endif
		}
		ret = nand_scan_tail(mtd);
	}
	return ret;
}

/*
 * Main initialization routine
 */
static int __init txx9ndfmc_probe(struct platform_device *dev)
{
	struct ndfmc_platform_data *plat = dev->dev.platform_data;
	struct nand_chip *this;
#ifdef CONFIG_MTD_PARTITIONS
	int mtd_parts_nb = 0;
	struct mtd_partition *mtd_parts = NULL;
	static const char *probes[] = { "cmdlinepart", NULL };
#endif
	int hold, spw;
	struct mtd_info *mtd;
	struct txx9ndfmc_priv *txx9_priv;
	int i;
	int do_reset = 1;
	struct txx9ndfmc_drvdata *drvdata;

	drvdata = kzalloc(sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;
	platform_set_drvdata(dev, drvdata);

	hold = plat->hold ?: 20;
	spw = plat->spw ?: 75;

	/* convert to gbusclk cycles */
	hold = TXX9NDFMC_NS_TO_CYC(hold);
	spw = TXX9NDFMC_NS_TO_CYC(spw);
	if (plat->flags & NDFMC_PLAT_FLAG_NO_SPWADD)
		hold -= 1;	/* actual hold time : (HOLD + 1) BUSCLK */
	else
		spw -= 1;	/* actual wait time : (SPW + 1) BUSCLK */
	drvdata->hold = min(max(hold, 1), 15);
	drvdata->spw = min(max(spw, 1), 15);
	printk(KERN_DEBUG "TXX9 NDFMC: HOLD:%d SPW:%d.\n", hold, spw);
	if (plat->flags & NDFMC_PLAT_FLAG_USE_ALEDELAY)
		drvdata->aledelay = TXX9NDFMC_NS_TO_CYC(plat->aledelay);
	if (plat->flags & NDFMC_PLAT_FLAG_USE_RRTIME)
		drvdata->rrtime = TXX9NDFMC_NS_TO_CYC(plat->rrtime);

	drvdata->dmanr = -1;
	if (use_dma && plat->dmanr >= 0) {
		drvdata->dmanr = plat->dmanr + TXX9_DMA_CHANNEL_START;
#ifndef CONFIG_MTD_NAND_TXX9NDFMC_OOB_HWLAYOUT
		if (plat->dmamode & TXx9_DMA_CCR_EXTRQ) {
			printk(KERN_WARNING "TXX9 NDFMC: "
			       "DMA can be used for HWLAYOUT only\n");
			drvdata->dmanr = -1;
		}
#endif
	}
	if (drvdata->dmanr >= 0) {
		if (request_txx9_dma(drvdata->dmanr, dev->name) < 0)
			drvdata->dmanr = -1;
	}
	if (drvdata->dmanr >= 0) {
		init_txx9_dma(drvdata->dmanr);
		if (plat->addr >= (unsigned long)(int)0xff000000)
			drvdata->dmareg = (u64)(int)plat->addr & 0xfffffffffull;
		else
			drvdata->dmareg = plat->addr - IO_BASE;
		drvdata->dmareg &= ~0xffful;
		drvdata->dmamode = plat->dmamode;
		if (drvdata->dmamode & TXx9_DMA_CCR_EXTRQ) {
			drvdata->dmamode |= TXx9_DMA_CCR_XFSZ(3);
			drvdata->dmareg += TXX9_NDFFRR << plat->shift;
		} else {
			drvdata->dmamode |= TXx9_DMA_CCR_XFSZ(0);
			drvdata->dmareg += TXX9_NDFDTR << plat->shift;
#ifdef __BIG_ENDIAN
			drvdata->dmareg += (sizeof(u32) << plat->shift) - 1;
#endif
		}
		set_txx9_dma_mode(drvdata->dmanr, drvdata->dmamode);
		drvdata->dmairq = -1;
		if (use_dmaint)
			drvdata->dmairq = get_txx9_dma_irqno(drvdata->dmanr);
		if (drvdata->dmairq >= 0) {
			if (request_irq(drvdata->dmairq, txx9ndfmc_dma_irq,
					IRQF_DISABLED, dev->name, dev) < 0) {
				printk(KERN_WARNING
				       "TXX9 NDFMC: dma irq not available.\n");
				drvdata->dmairq = -1;
			}
		}
		printk(KERN_DEBUG
		       "TXX9 NDFMC: DMAREG:%#llx DMAMODE:%#x, DMAIRQ:%d\n",
		       drvdata->dmareg, drvdata->dmamode, drvdata->dmairq);
		init_completion(&drvdata->cmp);
	}
	for (i = 0; i < MAX_TXX9NDFMC_DEV; i++) {
		if (!(plat->ch_mask & (1 << i)))
			continue;
		/* Allocate memory for MTD device structure and private data */
		mtd = kzalloc (sizeof(struct mtd_info) +
			       sizeof(struct nand_chip) +
			       sizeof(struct txx9ndfmc_priv),
			       GFP_KERNEL);
		if (!mtd) {
			printk(KERN_ERR "TXX9 NDFMC: Unable to allocate "
			       "TXX9 NDFMC MTD device structure.\n");
			continue;
		}
		mtd->owner = THIS_MODULE;

		/* Get pointer to private data */
		this = (struct nand_chip *)(mtd + 1);
		txx9_priv = (struct txx9ndfmc_priv *)(this + 1);

		/* Link the private data with the MTD structure */
		mtd->priv = this;

		/* Set address of NAND IO lines */
		this->read_byte = txx9ndfmc_read_byte;
		this->read_buf = txx9ndfmc_read_buf;
		this->write_buf = txx9ndfmc_write_buf;
		this->verify_buf = txx9ndfmc_verify_buf;
#ifdef CONFIG_MTD_NAND_VERIFY_WRITE_SUBPAGE
		this->verify_buf_column = txx9ndfmc_verify_buf_column;
#endif
		this->cmd_ctrl = txx9ndfmc_hwcontrol;
		this->dev_ready = txx9ndfmc_dev_ready;
		this->errstat = txx9ndfmc_do_errstat;
		this->errstat_get = txx9ndfmc_get_errstat;
		this->errstat_set = txx9ndfmc_set_errstat;
		this->ecc.calculate = txx9ndfmc_calculate_ecc;
		this->ecc.correct = txx9ndfmc_correct_data;
		this->ecc.hwctl = txx9ndfmc_enable_hwecc;
		this->ecc.mode = NAND_ECC_HW;
		this->ecc.size = 256;
		this->ecc.bytes = 3;
		this->chip_delay = 100;
		if (drvdata->dmanr >= 0)
			this->ecc.read_page = txx9ndfmc_dma_read_page;

		this->priv = txx9_priv;
		this->IO_ADDR_R = (void __iomem *)plat->addr;
		txx9_priv->shift = plat->shift;
		txx9_priv->dev = dev;

		if (plat->ch_mask != 1)
			txx9_priv->cs = i;
		else
			txx9_priv->cs = -1;
		if (plat->wide_mask & (1 << i))
			this->options |= NAND_BUSWIDTH_16;

		if (do_reset) {
			do_reset = 0;
			txx9ndfmc_initialize(this);
		}

		/* Scan to find existance of the device */
		if (txx9ndfmc_nand_scan(mtd)) {
			kfree (mtd);
			continue;
		}

#ifdef DYNAMIC_TIMING_CONFIGURATION
		{
			struct nand_access_timing *tm;
			for (tm = nand_access_timing_ids; tm->maf_id; tm++) {
				if (tm->maf_id == this->maf_id &&
				    tm->dev_id == this->dev_id &&
				    (tm->extid3 == -1 ||
				     tm->extid3 == this->extid3) &&
				    (tm->extid4 == -1 ||
				     tm->extid4 == this->extid4))
					break;
			}
			if (tm->maf_id) {
				spw = TXX9NDFMC_NS_TO_CYC(tm->pulse_width);
				hold = TXX9NDFMC_NS_TO_CYC(tm->hold_time);
				if (plat->flags & NDFMC_PLAT_FLAG_NO_SPWADD)
					hold -= 1;	/* actual hold time : (HOLD + 1) BUSCLK */
				else
					spw -= 1;	/* actual wait time : (SPW + 1) BUSCLK */
				drvdata->hold = min(max(hold, 1), 15);
				drvdata->spw = min(max(spw, 1), 15);
				if (plat->flags & NDFMC_PLAT_FLAG_USE_ALEDELAY)
					drvdata->aledelay = TXX9NDFMC_NS_TO_CYC(tm->ale_delay);
				txx9ndfmc_initialize(this);
#ifdef HAVE_TXX9_NDFDBR
				if (mtd->writesize > 512)
					ndwrite(this, TXX9_NDFDBR_LBLK,
						TXX9_NDFDBR);
#endif
			}
		}
#endif

		/* override mtd name */
		if (plat->ch_mask != 1)
			sprintf(txx9_priv->mtdname, "txx9ndfmc%u", i);
		else
			strcpy(txx9_priv->mtdname, "txx9ndfmc");
		mtd->name = txx9_priv->mtdname;

		if (plat->wp_mask & (1 << i)) {
			printk(KERN_INFO "TXX9 NDFMC: write protected.\n");
			mtd->flags &= ~MTD_WRITEABLE;
		}

		add_mtd_device(mtd);
#ifdef CONFIG_MTD_PARTITIONS
		mtd_parts_nb = parse_mtd_partitions(mtd, probes,
						    &mtd_parts, 0);
		if (mtd_parts_nb > 0)
			add_mtd_partitions(mtd, mtd_parts, mtd_parts_nb);
#endif
		drvdata->mtds[i] = mtd;
	}

	for (i = 0; i < FL_STATE_MAX; i++) {
		txx9ndfmc_errstat[i] = NULL;
	}

	return 0;
}

/*
 * Clean up routine
 */
static int __devexit txx9ndfmc_remove(struct platform_device *dev)
{
	struct txx9ndfmc_drvdata *drvdata = platform_get_drvdata(dev);
	struct nand_chip *this;
	int i;

	platform_set_drvdata(dev, NULL);
	if (!drvdata)
		return 0;
	for (i = 0; i < FL_STATE_MAX; i++) {
		if (txx9ndfmc_errstat[i] != NULL) {
			kfree(txx9ndfmc_errstat[i]);
			txx9ndfmc_errstat[i] = NULL;
		}
	}
	for (i = 0; i < MAX_TXX9NDFMC_DEV; i++) {
		struct mtd_info *mtd = drvdata->mtds[i];
		if (!mtd)
			continue;
		this = mtd->priv;
		/* Unregister the device */
#ifdef CONFIG_MTD_PARTITIONS
		del_mtd_partitions(mtd);
#endif
		del_mtd_device(mtd);

		/* Free the MTD device structure */
		kfree(mtd);
	}
	if (drvdata->dmanr >= 0) {
		if (drvdata->dmairq >= 0)
			free_irq(drvdata->dmairq, dev);
		free_txx9_dma(drvdata->dmanr);
	}
	kfree(drvdata);
	return 0;
}

#ifdef CONFIG_PM
static int txx9ndfmc_suspend(struct platform_device *dev, pm_message_t state)
{
	struct txx9ndfmc_drvdata *drvdata = platform_get_drvdata(dev);
	int i, ret = 0;

	if (!drvdata)
		return 0;
	for (i = 0; i < MAX_TXX9NDFMC_DEV; i++) {
		struct mtd_info *mtd = drvdata->mtds[i];
		if (mtd && mtd->suspend) {
			ret = mtd->suspend(mtd);
			if (ret)
				return ret;
		}
	}
	return ret;
}

static int txx9ndfmc_resume(struct platform_device *dev)
{
	struct txx9ndfmc_drvdata *drvdata = platform_get_drvdata(dev);
	int i;

	if (!drvdata)
		return 0;
	if (drvdata->dmanr >= 0)
		init_txx9_dma(drvdata->dmanr);
	for (i = 0; i < MAX_TXX9NDFMC_DEV; i++) {
		struct mtd_info *mtd = drvdata->mtds[i];
		if (mtd) {
			struct nand_chip *chip = mtd->priv;
			txx9ndfmc_initialize(chip);
#ifdef HAVE_TXX9_NDFDBR
			if (mtd->writesize > 512)
				ndwrite(chip, TXX9_NDFDBR_LBLK, TXX9_NDFDBR);
#endif
			break;
		}
	}
	for (i = 0; i < MAX_TXX9NDFMC_DEV; i++) {
		struct mtd_info *mtd = drvdata->mtds[i];
		if (mtd && mtd->resume)
			mtd->resume(mtd);
	}
	return 0;
}
#endif

static struct platform_driver txx9ndfmc_driver = {
	.probe		= txx9ndfmc_probe,
	.remove		= __devexit_p(txx9ndfmc_remove),
#ifdef CONFIG_PM
	.suspend	= txx9ndfmc_suspend,
	.resume		= txx9ndfmc_resume,
#endif
	.driver		= {
		.name	= "txx9ndfmc",
		.owner	= THIS_MODULE,
	},
};

static int __init txx9ndfmc_init(void)
{
	return platform_driver_register(&txx9ndfmc_driver);
}

static void __exit txx9ndfmc_exit(void)
{
	platform_driver_unregister(&txx9ndfmc_driver);
}

module_init(txx9ndfmc_init);
module_exit(txx9ndfmc_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Board-specific glue layer for NAND flash on TXX9 NDFMC");
