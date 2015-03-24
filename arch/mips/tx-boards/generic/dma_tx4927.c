/*
 * linux/arch/mips/tx-boards/generic/dma_tx4927.c
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
#include <linux/device.h>
#include <linux/dmapool.h>
#include <asm/tx-boards/generic.h>
#include <asm/tx-boards/dma.h>
#include <asm/tx-boards/irq.h>
#include <asm/tx-boards/tx4927.h>
#include <asm/tx-boards/tx4938.h>
#include <asm/tx-boards/tx4939.h>
#include <asm/tx-boards/tsb-generic.h>

/* TX4927 stype DMA controller */

static struct tx4927_dma_reg *tx4927_dmaptrs[2];
static int tx4927_dmairqs[2];
static struct dma_pool *tx4927_dma_pool;
#define chdma_desc_to_dma(cha)	((dma_addr_t)((cha) - UNCAC_BASE))
#define chdma_dma_to_desc(dma)	((txx9_chdma_desc_t)((dma) + UNCAC_BASE))

/* reset a specific DMA channel */
static void
init_tx4927_dma(unsigned int dmanr)
{
	unsigned int ch = dmanr - TXX9_DMA_CHANNEL_START;
	struct tx4927_dma_reg *dmaptr = tx4927_dmaptrs[ch/4];
	ch %= 4;
	dmaptr->ch[ch].ccr = TX4927_DMA_CCR_CHRST;
	dmaptr->ch[ch].cha = 0ULL;
	dmaptr->ch[ch].sar = 0ULL;
	dmaptr->ch[ch].dar = 0ULL;
	dmaptr->ch[ch].cntr = 0;
	dmaptr->ch[ch].sair = 0;
	dmaptr->ch[ch].dair = 0;
	dmaptr->ch[ch].ccr = 0;
}

/* enable/disable a specific DMA channel */
static void
enable_tx4927_dma(unsigned int dmanr)
{
	unsigned int ch = dmanr - TXX9_DMA_CHANNEL_START;
	struct tx4927_dma_reg *dmaptr = tx4927_dmaptrs[ch/4];
	ch %= 4;
	dmaptr->ch[ch].ccr |= TX4927_DMA_CCR_XFACT;
}

static void
disable_tx4927_dma(unsigned int dmanr)
{
	unsigned int ch = dmanr - TXX9_DMA_CHANNEL_START;
	struct tx4927_dma_reg *dmaptr = tx4927_dmaptrs[ch/4];
	ch %= 4;
	dmaptr->ch[ch].ccr &= ~TX4927_DMA_CCR_XFACT;
}

/* set mode for a specific DMA channel */
static void
set_tx4927_dma_mode(unsigned int dmanr, unsigned int mode)
{
	unsigned int ch = dmanr - TXX9_DMA_CHANNEL_START;
	struct tx4927_dma_reg *dmaptr = tx4927_dmaptrs[ch/4];
	ch %= 4;
#ifdef __LITTLE_ENDIAN
	mode |= TX4927_DMA_CCR_LE;
#endif
	mode |= TX4927_DMA_CCR_IMMCHN;
	dmaptr->ch[ch].ccr = mode;
}

/* Set source/destination address for specific DMA channel.
 */
static void
set_tx4927_dma_addr(unsigned int dmanr,
		    dma_addr_t sa,
		    dma_addr_t da)
{
	unsigned int ch = dmanr - TXX9_DMA_CHANNEL_START;
	struct tx4927_dma_reg *dmaptr = tx4927_dmaptrs[ch/4];
	ch %= 4;
	dmaptr->ch[ch].sar = sa;
	dmaptr->ch[ch].dar = da;
}
static void
set_tx4927_dma_addr64(unsigned int dmanr,
		      dma64_addr_t sa,
		      dma64_addr_t da)
{
	unsigned int ch = dmanr - TXX9_DMA_CHANNEL_START;
	struct tx4927_dma_reg *dmaptr = tx4927_dmaptrs[ch/4];
	ch %= 4;
	dmaptr->ch[ch].sar = sa;
	dmaptr->ch[ch].dar = da;
}

/* Set transfer size for a specific DMA channel.
 */
static void
set_tx4927_dma_count(unsigned int dmanr, unsigned int count)
{
	unsigned int ch = dmanr - TXX9_DMA_CHANNEL_START;
	struct tx4927_dma_reg *dmaptr = tx4927_dmaptrs[ch/4];
	ch %= 4;
	dmaptr->ch[ch].cntr = count;
}

/* Set source/destination address increment size for a specific DMA channel.
 */
static void
set_tx4927_dma_inc(unsigned int dmanr, unsigned int sai, unsigned int dai)
{
	unsigned int ch = dmanr - TXX9_DMA_CHANNEL_START;
	struct tx4927_dma_reg *dmaptr = tx4927_dmaptrs[ch/4];
	ch %= 4;
	dmaptr->ch[ch].sair = sai;
	dmaptr->ch[ch].dair = dai;
}


/* Get DMA residue count. After a DMA transfer, this
 * should return zero. Reading this while a DMA transfer is
 * still in progress will return unpredictable results.
 */
static int
get_tx4927_dma_residue(unsigned int dmanr)
{
	unsigned int ch = dmanr - TXX9_DMA_CHANNEL_START;
	struct tx4927_dma_reg *dmaptr = tx4927_dmaptrs[ch/4];
	ch %= 4;
	return dmaptr->ch[ch].cntr;
}

/* Get DMA status.
 */
static unsigned int
get_tx4927_dma_status(unsigned int dmanr)
{
	unsigned int ch = dmanr - TXX9_DMA_CHANNEL_START;
	struct tx4927_dma_reg *dmaptr = tx4927_dmaptrs[ch/4];
	ch %= 4;
	return dmaptr->ch[ch].csr;
}

/* clear DMA status.
 */
static void
clear_tx4927_dma_status(unsigned int dmanr)
{
	unsigned int ch = dmanr - TXX9_DMA_CHANNEL_START;
	struct tx4927_dma_reg *dmaptr = tx4927_dmaptrs[ch/4];
	ch %= 4;
	dmaptr->ch[ch].csr = 0xffffffff;
}

/* get DMA completion interrupt number.
 */
static int
get_tx4927_dma_irqno(unsigned int dmanr)
{
	unsigned int ch = dmanr - TXX9_DMA_CHANNEL_START;

	if (ch >= MAX_TXX9_DMA_CHANNELS)
		return -1;
	if (txx9_pcode == 0x4939)
		return tx4939_irq_to_irq(tx4927_dmairqs[ch/4] + (ch % 4));
#if defined(CONFIG_TOSHIBA_TC90411) || defined(CONFIG_TOSHIBA_TC90412) || defined(CONFIG_TOSHIBA_TC90416)
	return txx9_64_irq_to_irq(tx4927_dmairqs[ch/4] + (ch % 4));
#else
	return txx9_irq_to_irq(tx4927_dmairqs[ch/4] + (ch % 4));
#endif
}

/*
 * chain DMA operations.
 * txx9_chdma_desc_t is nocache virtual address of the command descriptor.
 */

static void
set_tx4927_dma_chain(unsigned int dmanr, txx9_chdma_desc_t cha)
{
	unsigned int ch = dmanr - TXX9_DMA_CHANNEL_START;
	struct tx4927_dma_reg *dmaptr = tx4927_dmaptrs[ch/4];
	ch %= 4;
	dmaptr->ch[ch].cha = cha ? chdma_desc_to_dma(cha) : 0;
}

static txx9_chdma_desc_t
get_tx4927_dma_chain(unsigned int dmanr)
{
	unsigned int ch = dmanr - TXX9_DMA_CHANNEL_START;
	unsigned int cha;
	struct tx4927_dma_reg *dmaptr = tx4927_dmaptrs[ch/4];
	ch %= 4;
	cha = (unsigned int)dmaptr->ch[ch].cha;
	if (cha == 0)
		return NULL;
	return chdma_dma_to_desc(cha);
}

#define tx4927_chdma_desc(desc)	((struct tx4927_dma_ch_reg*)desc)

static void
set_tx4927_chdma_mode(txx9_chdma_desc_t desc, unsigned int mode)
{
#ifdef __LITTLE_ENDIAN
	mode |= TX4927_DMA_CCR_LE;
#endif
	tx4927_chdma_desc(desc)->ccr = mode;
}

static void
set_tx4927_chdma_addr(txx9_chdma_desc_t desc, dma_addr_t sa, dma_addr_t da)
{
	tx4927_chdma_desc(desc)->sar = sa;
	tx4927_chdma_desc(desc)->dar = da;
}

static void
get_tx4927_chdma_addr(txx9_chdma_desc_t desc, dma_addr_t *sa, dma_addr_t *da)
{
	*sa = tx4927_chdma_desc(desc)->sar;
	*da = tx4927_chdma_desc(desc)->dar;
}

static void
set_tx4927_chdma_addr64(txx9_chdma_desc_t desc, dma64_addr_t sa, dma64_addr_t da)
{
	tx4927_chdma_desc(desc)->sar = sa;
	tx4927_chdma_desc(desc)->dar = da;
}

static void
get_tx4927_chdma_addr64(txx9_chdma_desc_t desc, dma64_addr_t *sa, dma64_addr_t *da)
{
	*sa = tx4927_chdma_desc(desc)->sar;
	*da = tx4927_chdma_desc(desc)->dar;
}

static void
set_tx4927_chdma_count(txx9_chdma_desc_t desc, unsigned int count)
{
	tx4927_chdma_desc(desc)->cntr = count;
}

static void
get_tx4927_chdma_count(txx9_chdma_desc_t desc, unsigned int *count)
{
	*count = tx4927_chdma_desc(desc)->cntr;
}

static void
set_tx4927_chdma_inc(txx9_chdma_desc_t desc, unsigned int sai, unsigned int dai)
{
	tx4927_chdma_desc(desc)->sair = sai;
	tx4927_chdma_desc(desc)->dair = dai;
}

static void
set_tx4927_chdma_chain(txx9_chdma_desc_t desc, txx9_chdma_desc_t cha)
{
	tx4927_chdma_desc(desc)->cha = cha ? chdma_desc_to_dma(cha) : 0;
}

static txx9_chdma_desc_t
get_tx4927_chdma_chain(txx9_chdma_desc_t desc)
{
	unsigned int cha = (unsigned int)tx4927_chdma_desc(desc)->cha;
	if (cha == 0)
		return NULL;
	return chdma_dma_to_desc(cha);
}

static txx9_chdma_desc_t
alloc_tx4927_chdma_cmd(void)
{
	int size = sizeof(struct tx4927_dma_ch_reg);
	void *desc;
	dma_addr_t dma;
	if (!tx4927_dma_pool) {
		tx4927_dma_pool = dma_pool_create("tx4927_dma", NULL,
						  size, 32, 0);
		if (!tx4927_dma_pool)
			return NULL;
	}
	desc = dma_pool_alloc(tx4927_dma_pool, GFP_KERNEL, &dma);
	if (desc) {
		BUG_ON(chdma_desc_to_dma(desc) != dma);
		memset(desc, 0, size);
	}
	return desc;
}

static void
free_tx4927_chdma_cmd(txx9_chdma_desc_t desc)
{
	if (desc)
		dma_pool_free(tx4927_dma_pool, desc, chdma_desc_to_dma(desc));
}

static struct txx9_dma_ops tx4927_dma_ops = {
	.init_dma = init_tx4927_dma,
	.enable_dma = enable_tx4927_dma,
	.disable_dma = disable_tx4927_dma,
	.set_dma_mode = set_tx4927_dma_mode,
	.set_dma_addr = set_tx4927_dma_addr,
	.set_dma_addr64 = set_tx4927_dma_addr64,
	.set_dma_count = set_tx4927_dma_count,
	.set_dma_inc = set_tx4927_dma_inc,
	.get_dma_residue = get_tx4927_dma_residue,
	.get_dma_status = get_tx4927_dma_status,
	.clear_dma_status = clear_tx4927_dma_status,
	.get_dma_irqno = get_tx4927_dma_irqno,
	.set_dma_chain = set_tx4927_dma_chain,
	.get_dma_chain = get_tx4927_dma_chain,
	.set_chdma_mode = set_tx4927_chdma_mode,
	.set_chdma_addr = set_tx4927_chdma_addr,
	.get_chdma_addr = get_tx4927_chdma_addr,
	.set_chdma_addr64 = set_tx4927_chdma_addr64,
	.get_chdma_addr64 = get_tx4927_chdma_addr64,
	.set_chdma_count = set_tx4927_chdma_count,
	.get_chdma_count = get_tx4927_chdma_count,
	.set_chdma_inc = set_tx4927_chdma_inc,
	.set_chdma_chain = set_tx4927_chdma_chain,
	.get_chdma_chain = get_tx4927_chdma_chain,
	.alloc_chdma_cmd = alloc_tx4927_chdma_cmd,
	.free_chdma_cmd = free_tx4927_chdma_cmd
};

/* static */ /* export for other chips with tx4927-style DMAC */
void __init
tx4927_dma_init_sub(int dmacno, struct tx4927_dma_reg *dmaptr, int dmairq)
{
	int i;
	txx9_dma_ops = &tx4927_dma_ops;
	tx4927_dmaptrs[dmacno] = dmaptr;
	tx4927_dmairqs[dmacno] = dmairq;
	tx4927_dmaptrs[dmacno]->mcr = 0;
	for (i = 0; i < 4; i++)
		init_txx9_dma(TXX9_DMA_CHANNEL_START + dmacno * 4 + i);
	/* enable DMA */
	tx4927_dmaptrs[dmacno]->mcr = TX4927_DMA_MCR_MSTEN;
}
void __init tx4927_dma_init(void)
{
	tx4927_dma_init_sub(0, tx4927_dmaptr, TX4927_IR_DMA(0));
}
void __init tx4938_dma_init(void)
{
	tx4927_dma_init_sub(0, tx4938_dmaptr(0), TX4938_IR_DMA(0, 0));
	tx4927_dma_init_sub(1, tx4938_dmaptr(1), TX4938_IR_DMA(1, 0));
}
void __init tx4939_dma_init(void)
{
	tx4927_dma_init_sub(0, tx4939_dmaptr(0), TX4939_IR_DMA(0, 0));
	tx4927_dma_init_sub(1, tx4939_dmaptr(1), TX4939_IR_DMA(1, 0));
}
