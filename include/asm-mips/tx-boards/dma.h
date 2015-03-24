/*
 *  linux/arch/mips/tx-boards/generic/dma.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2006
 * All Rights Reserved.
 *
 */

#ifndef __ASM_TX_BOARDS_DMA_H
#define __ASM_TX_BOARDS_DMA_H

#include <asm/types.h>

/* TX4927/TX4925 stype DMA controllers */

/* bits for set_txx9_dma_mode() */
#define TXx9_DMA_CCR_IMMCHN	0x20000000
#define TXx9_DMA_CCR_DBINH	0x04000000
#define TXx9_DMA_CCR_SBINH	0x02000000
#define TXx9_DMA_CCR_CHRST	0x01000000
#define TXx9_DMA_CCR_RVBYTE	0x00800000
#define TXx9_DMA_CCR_ACKPOL	0x00400000
#define TXx9_DMA_CCR_REQPL	0x00200000
#define TXx9_DMA_CCR_EGREQ	0x00100000
#define TXx9_DMA_CCR_CHDN	0x00080000
#define TXx9_DMA_CCR_DNCTL	0x00060000
#define TXx9_DMA_CCR_EXTRQ	0x00010000
#define TXx9_DMA_CCR_INTRQD	0x0000e000
#define TXx9_DMA_CCR_INTENE	0x00001000
#define TXx9_DMA_CCR_INTENC	0x00000800
#define TXx9_DMA_CCR_INTENT	0x00000400
#define TXx9_DMA_CCR_CHNEN	0x00000200
#define TXx9_DMA_CCR_XFACT	0x00000100
#define TXx9_DMA_CCR_SMPCHN	0x00000020
#define TXx9_DMA_CCR_XFSZ(order)	(((order) << 2) & 0x0000001c)
#define TXx9_DMA_CCR_XFSZ_1B	TXx9_DMA_CCR_XFSZ(0)
#define TXx9_DMA_CCR_XFSZ_1W	TXx9_DMA_CCR_XFSZ(2)
/* other XFSZ are CPU dependend... */
#define TXx9_DMA_CCR_MEMIO	0x00000002
#define TXx9_DMA_CCR_SNGAD	0x00000001

/* dual address, 1 word size transfer mode */
#define TXx9_DMA_MODE_DUAL_1W	\
	(TXx9_DMA_CCR_INTENE|TXx9_DMA_CCR_INTENT|TXx9_DMA_CCR_XFSZ_1W)
/* Internal I/O, 1 word size transfer mode */
#define TXx9_DMA_MODE_IIO_1W	\
	(TXx9_DMA_CCR_EXTRQ|TXx9_DMA_CCR_INTENE|TXx9_DMA_CCR_INTENT|TXx9_DMA_CCR_XFSZ_1W)

/* bits for get_txx9_dma_status() */
#define TXx9_DMA_STATUS_CHNACT	0x00000100
#define TXx9_DMA_STATUS_ABCHC	0x00000080
#define TXx9_DMA_STATUS_NCHNC	0x00000040
#define TXx9_DMA_STATUS_NTRNFC	0x00000020
#define TXx9_DMA_STATUS_EXTDN	0x00000010
#define TXx9_DMA_STATUS_CFERR	0x00000008
#define TXx9_DMA_STATUS_CHERR	0x00000004
#define TXx9_DMA_STATUS_DESERR	0x00000002
#define TXx9_DMA_STATUS_SORERR	0x00000001

/* chained DMA descriptor */
typedef void* txx9_chdma_desc_t;

struct txx9_dma_ops {
	void (*init_dma)(unsigned int dmanr);
	void (*enable_dma)(unsigned int dmanr);
	void (*disable_dma)(unsigned int dmanr);
	void (*set_dma_mode)(unsigned int dmanr, unsigned int mode);
	void (*set_dma_addr)(unsigned int dmanr, dma_addr_t sa, dma_addr_t da);
	void (*set_dma_addr64)(unsigned int dmanr, dma64_addr_t sa, dma64_addr_t da);
	void (*set_dma_count)(unsigned int dmanr, unsigned int count);
	void (*set_dma_inc)(unsigned int dmanr, unsigned int sai, unsigned int dai);
	int (*get_dma_residue)(unsigned int dmanr);
	unsigned int (*get_dma_status)(unsigned int dmanr);
	void (*clear_dma_status)(unsigned int dmanr);
	int (*get_dma_irqno)(unsigned int dmanr);
	/* for chain operation */
	void (*set_dma_chain)(unsigned int dmanr, txx9_chdma_desc_t cha);
	txx9_chdma_desc_t (*get_dma_chain)(unsigned int dmanr);
	void (*set_chdma_mode)(txx9_chdma_desc_t desc, unsigned int mode);
	void (*set_chdma_addr)(txx9_chdma_desc_t desc, dma_addr_t sa, dma_addr_t da);
	void (*get_chdma_addr)(txx9_chdma_desc_t desc, dma_addr_t *sa, dma_addr_t *da);
	void (*set_chdma_addr64)(txx9_chdma_desc_t desc, dma64_addr_t sa, dma64_addr_t da);
	void (*get_chdma_addr64)(txx9_chdma_desc_t desc, dma64_addr_t *sa, dma64_addr_t *da);
	void (*set_chdma_count)(txx9_chdma_desc_t desc, unsigned int count);
	void (*get_chdma_count)(txx9_chdma_desc_t desc, unsigned int *count);
	void (*set_chdma_inc)(txx9_chdma_desc_t desc, unsigned int sai, unsigned int dai);
	void (*set_chdma_chain)(txx9_chdma_desc_t desc, txx9_chdma_desc_t cha);
	txx9_chdma_desc_t (*get_chdma_chain)(txx9_chdma_desc_t desc);
	txx9_chdma_desc_t (*alloc_chdma_cmd)(void);
	void (*free_chdma_cmd)(txx9_chdma_desc_t desc);
};
extern struct txx9_dma_ops *txx9_dma_ops;

#define init_txx9_dma(dmanr) \
	(*txx9_dma_ops->init_dma)(dmanr)
#define enable_txx9_dma(dmanr) \
	(*txx9_dma_ops->enable_dma)(dmanr)
#define disable_txx9_dma(dmanr) \
	(*txx9_dma_ops->disable_dma)(dmanr)
#define set_txx9_dma_mode(dmanr, mode) \
	(*txx9_dma_ops->set_dma_mode)(dmanr, mode)
#define set_txx9_dma_addr(dmanr, sa, da) \
	(*txx9_dma_ops->set_dma_addr)(dmanr, sa, da)
#define set_txx9_dma_addr64(dmanr, sa, da) \
	(*txx9_dma_ops->set_dma_addr64)(dmanr, sa, da)
#define set_txx9_dma_count(dmanr, count) \
	(*txx9_dma_ops->set_dma_count)(dmanr, count)
#define set_txx9_dma_inc(dmanr, sai, dai) \
	(*txx9_dma_ops->set_dma_inc)(dmanr, sai, dai)
#define get_txx9_dma_residue(dmanr) \
	(*txx9_dma_ops->get_dma_residue)(dmanr)
#define get_txx9_dma_status(dmanr) \
	(*txx9_dma_ops->get_dma_status)(dmanr)
#define clear_txx9_dma_status(dmanr) \
	(*txx9_dma_ops->clear_dma_status)(dmanr)
#define get_txx9_dma_irqno(dmanr) \
	(*txx9_dma_ops->get_dma_irqno)(dmanr)
/* for chain operation */
#define set_txx9_dma_chain(dmanr, cha) \
	(*txx9_dma_ops->set_dma_chain)(dmanr, cha)
#define get_txx9_dma_chain(dmanr) \
	(*txx9_dma_ops->get_dma_chain)(dmanr)
#define set_txx9_chdma_mode(desc, mode) \
	(*txx9_dma_ops->set_chdma_mode)(desc, mode)
#define set_txx9_chdma_addr(desc, sa, da) \
	(*txx9_dma_ops->set_chdma_addr)(desc, sa, da)
#define get_txx9_chdma_addr(desc, sa, da) \
	(*txx9_dma_ops->get_chdma_addr)(desc, sa, da)
#define set_txx9_chdma_addr64(desc, sa, da) \
	(*txx9_dma_ops->set_chdma_addr64)(desc, sa, da)
#define get_txx9_chdma_addr64(desc, sa, da) \
	(*txx9_dma_ops->get_chdma_addr64)(desc, sa, da)
#define set_txx9_chdma_count(desc, count) \
	(*txx9_dma_ops->set_chdma_count)(desc, count)
#define get_txx9_chdma_count(desc, count) \
	(*txx9_dma_ops->get_chdma_count)(desc, count)
#define set_txx9_chdma_inc(desc, sai, dai) \
	(*txx9_dma_ops->set_chdma_inc)(desc, sai, dai)
#define set_txx9_chdma_chain(desc, cha) \
	(*txx9_dma_ops->set_chdma_chain)(desc, cha)
#define get_txx9_chdma_chain(desc) \
	(*txx9_dma_ops->get_chdma_chain)(desc)
#define alloc_txx9_chdma_cmd() \
	(*txx9_dma_ops->alloc_chdma_cmd)()
#define free_txx9_chdma_cmd(desc) \
	(*txx9_dma_ops->free_chdma_cmd)(desc)

#define TXX9_DMA_CHANNEL_START	0
#define MAX_TXX9_DMA_CHANNELS	8

extern int request_txx9_dma(unsigned int dmanr, const char * device_id);
extern void free_txx9_dma(unsigned int dmanr);

extern void tx4925_dma_init(void);
extern void tx4927_dma_init(void);
extern void tx4938_dma_init(void);
extern void tx4939_dma_init(void);
struct tx4927_dma_reg;
extern void tx4927_dma_init_sub(int dmacno, struct tx4927_dma_reg *dmaptr, int dmairq);

#endif /* __ASM_TX_BOARDS_DMA_H */
