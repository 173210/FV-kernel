/*
 * Definitions for TX4937/TX4938
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2006
 * All Rights Reserved.
 *
 */
#ifndef __ASM_TX_BOARDS_TX4938_H
#define __ASM_TX_BOARDS_TX4938_H

#include <asm/tx-boards/tx4927.h>	/* some controllers are compatible with 4927 */

#ifdef CONFIG_64BIT
#define TX4938_REG_BASE	0xffffffffff1f0000UL /* == TX4937_REG_BASE */
#else
#define TX4938_REG_BASE	0xff1f0000 /* == TX4937_REG_BASE */
#endif
#define TX4938_REG_SIZE	0x00010000 /* == TX4937_REG_SIZE */

/* NDFMC, SRAMC, PCIC1, SPIC: TX4938 only */
#define TX4938_NDFMC_REG	(TX4938_REG_BASE + 0x5000)
#define TX4938_SRAMC_REG	(TX4938_REG_BASE + 0x6000)
#define TX4938_PCIC1_REG	(TX4938_REG_BASE + 0x7000)
#define TX4938_SDRAMC_REG	(TX4938_REG_BASE + 0x8000)
#define TX4938_EBUSC_REG	(TX4938_REG_BASE + 0x9000)
#define TX4938_DMA_REG(ch)	(TX4938_REG_BASE + 0xb000 + (ch) * 0x800)
#define TX4938_PCIC_REG		(TX4938_REG_BASE + 0xd000)
#define TX4938_CCFG_REG		(TX4938_REG_BASE + 0xe000)
#define TX4938_NR_TMR	3
#define TX4938_TMR_REG(ch)	(TX4938_REG_BASE + 0xf000 + (ch) * 0x100)
#define TX4938_NR_SIO	2
#define TX4938_SIO_REG(ch)	(TX4938_REG_BASE + 0xf300 + (ch) * 0x100)
#define TX4938_PIO_REG		(TX4938_REG_BASE + 0xf500)
#define TX4938_IRC_REG		(TX4938_REG_BASE + 0xf600)
#define TX4938_ACLC_REG		(TX4938_REG_BASE + 0xf700)
#define TX4938_SPI_REG		(TX4938_REG_BASE + 0xf800)

#include <asm/byteorder.h>

#ifdef __BIG_ENDIAN
#define endian_def_l2(e1,e2)	\
	volatile __u32 e1,e2
#else
#define endian_def_l2(e1,e2)	\
	volatile __u32 e2,e1
#endif

struct tx4938_sdramc_reg {
	volatile __u64 cr[4];
	volatile __u64 unused0[4];
	volatile __u64 tr;
	volatile __u64 unused1[2];
	volatile __u64 cmd;
	volatile __u64 sfcmd;
};

struct tx4938_ndfmc_reg {
	endian_def_l2(unused0, dtr);
	endian_def_l2(unused1, mcr);
	endian_def_l2(unused2, sr);
	endian_def_l2(unused3, isr);
	endian_def_l2(unused4, imr);
	endian_def_l2(unused5, spr);
	endian_def_l2(unused6, rstr);
};

struct tx4938_sramc_reg {
	volatile __u64 cr;
};

struct tx4938_ccfg_reg {
	volatile __u64 ccfg;
	volatile __u64 crir;
	volatile __u64 pcfg;
	volatile __u64 toea;
	volatile __u64 clkctr;
	volatile __u64 unused0;
	volatile __u64 garbc;
	volatile __u64 unused1;
	volatile __u64 unused2;
	volatile __u64 ramp;
	volatile __u64 unused3;
	volatile __u64 jmpadr;
};

#undef endian_def_l2

/*
 * NDFMC
 */

/* NDFMCR : NDFMC Mode Control */
#define TX4938_NDFMCR_WE	0x80
#define TX4938_NDFMCR_ECC_ALL	0x60
#define TX4938_NDFMCR_ECC_RESET	0x60
#define TX4938_NDFMCR_ECC_READ	0x40
#define TX4938_NDFMCR_ECC_ON	0x20
#define TX4938_NDFMCR_ECC_OFF	0x00
#define TX4938_NDFMCR_CE	0x10
#define TX4938_NDFMCR_ALE	0x02
#define TX4938_NDFMCR_CLE	0x01

/* NDFMCR : NDFMC Status */
#define TX4938_NDFSR_BUSY	0x80

/* NDFMCR : NDFMC Reset */
#define TX4938_NDFRSTR_RST	0x01

/*
 * IRC
 */

#define TX4938_IR_ECCERR	0
#define TX4938_IR_WTOERR	1
#define TX4938_NUM_IR_INT	6
#define TX4938_IR_INT(n)	(2 + (n))
#define TX4938_NUM_IR_SIO	2
#define TX4938_IR_SIO(n)	(8 + (n))
#define TX4938_NUM_IR_DMA	4
#define TX4938_IR_DMA(ch,n)	(((ch) ? 27 : 10) + (n)) /* 10-13,27-30 */
#define TX4938_IR_PIO	14
#define TX4938_IR_PDMAC	15
#define TX4938_IR_PCIC	16
#define TX4938_NUM_IR_TMR	3
#define TX4938_IR_TMR(n)	(17 + (n))
#define TX4938_IR_NDFMC	21
#define TX4938_IR_PCIERR	22
#define TX4938_IR_PCIPME	23
#define TX4938_IR_ACLC	24
#define TX4938_IR_ACLCPME	25
#define TX4938_IR_PCIC1	26
#define TX4938_IR_SPI	31
#define TX4938_NUM_IR	32
/* multiplex */
#define TX4938_IR_ETH0	TX4938_IR_INT(4)
#define TX4938_IR_ETH1	TX4938_IR_INT(3)

#define TX4938_IRC_INT	2	/* IP[2] in Status register */

/*
 * CCFG
 */
/* CCFG : Chip Configuration */
#define TX4938_CCFG_WDRST	0x0000020000000000ULL
#define TX4938_CCFG_WDREXEN	0x0000010000000000ULL
#define TX4938_CCFG_BCFG_MASK	0x000000ff00000000ULL
#define TX4938_CCFG_TINTDIS	0x01000000
#define TX4938_CCFG_PCI66	0x00800000
#define TX4938_CCFG_PCIMODE	0x00400000
#define TX4938_CCFG_PCI1_66	0x00200000
#define TX4938_CCFG_DIVMODE_MASK	0x001e0000
#define TX4938_CCFG_DIVMODE_2	(0x4 << 17)
#define TX4938_CCFG_DIVMODE_2_5	(0xf << 17)
#define TX4938_CCFG_DIVMODE_3	(0x5 << 17)
#define TX4938_CCFG_DIVMODE_4	(0x6 << 17)
#define TX4938_CCFG_DIVMODE_4_5	(0xd << 17)
#define TX4938_CCFG_DIVMODE_8	(0x0 << 17)
#define TX4938_CCFG_DIVMODE_10	(0xb << 17)
#define TX4938_CCFG_DIVMODE_12	(0x1 << 17)
#define TX4938_CCFG_DIVMODE_16	(0x2 << 17)
#define TX4938_CCFG_DIVMODE_18	(0x9 << 17)
#define TX4938_CCFG_BEOW	0x00010000
#define TX4938_CCFG_WR	0x00008000
#define TX4938_CCFG_TOE	0x00004000
#define TX4938_CCFG_PCIARB	0x00002000
#define TX4938_CCFG_PCIDIVMODE_MASK	0x00001c00
#define TX4938_CCFG_PCIDIVMODE_4	(0x1 << 10)
#define TX4938_CCFG_PCIDIVMODE_4_5	(0x3 << 10)
#define TX4938_CCFG_PCIDIVMODE_5	(0x5 << 10)
#define TX4938_CCFG_PCIDIVMODE_5_5	(0x7 << 10)
#define TX4938_CCFG_PCIDIVMODE_8	(0x0 << 10)
#define TX4938_CCFG_PCIDIVMODE_9	(0x2 << 10)
#define TX4938_CCFG_PCIDIVMODE_10	(0x4 << 10)
#define TX4938_CCFG_PCIDIVMODE_11	(0x6 << 10)
#define TX4938_CCFG_PCI1DMD	0x00000100
#define TX4938_CCFG_SYSSP_MASK	0x000000c0
#define TX4938_CCFG_ENDIAN	0x00000004
#define TX4938_CCFG_HALT	0x00000002
#define TX4938_CCFG_ACEHOLD	0x00000001

/* PCFG : Pin Configuration */
#define TX4938_PCFG_ETH0_SEL	0x8000000000000000ULL
#define TX4938_PCFG_ETH1_SEL	0x4000000000000000ULL
#define TX4938_PCFG_ATA_SEL	0x2000000000000000ULL
#define TX4938_PCFG_ISA_SEL	0x1000000000000000ULL
#define TX4938_PCFG_SPI_SEL	0x0800000000000000ULL
#define TX4938_PCFG_NDF_SEL	0x0400000000000000ULL
#define TX4938_PCFG_SDCLKDLY_MASK	0x30000000
#define TX4938_PCFG_SDCLKDLY(d)	((d)<<28)
#define TX4938_PCFG_SYSCLKEN	0x08000000
#define TX4938_PCFG_SDCLKEN_ALL	0x07800000
#define TX4938_PCFG_SDCLKEN(ch)	(0x00800000<<(ch))
#define TX4938_PCFG_PCICLKEN_ALL	0x003f0000
#define TX4938_PCFG_PCICLKEN(ch)	(0x00010000<<(ch))
#define TX4938_PCFG_SEL2	0x00000200
#define TX4938_PCFG_SEL1	0x00000100
#define TX4938_PCFG_DMASEL_ALL	0x0000000f
#define TX4938_PCFG_DMASEL0_DRQ0	0x00000000
#define TX4938_PCFG_DMASEL0_SIO1	0x00000001
#define TX4938_PCFG_DMASEL1_DRQ1	0x00000000
#define TX4938_PCFG_DMASEL1_SIO1	0x00000002
#define TX4938_PCFG_DMASEL2_DRQ2	0x00000000
#define TX4938_PCFG_DMASEL2_SIO0	0x00000004
#define TX4938_PCFG_DMASEL3_DRQ3	0x00000000
#define TX4938_PCFG_DMASEL3_SIO0	0x00000008

/* CLKCTR : Clock Control */
#define TX4938_CLKCTR_NDFCKD	0x0001000000000000ULL
#define TX4938_CLKCTR_NDFRST	0x0000000100000000ULL
#define TX4938_CLKCTR_ETH1CKD	0x80000000
#define TX4938_CLKCTR_ETH0CKD	0x40000000
#define TX4938_CLKCTR_SPICKD	0x20000000
#define TX4938_CLKCTR_SRAMCKD	0x10000000
#define TX4938_CLKCTR_PCIC1CKD	0x08000000
#define TX4938_CLKCTR_DMA1CKD	0x04000000
#define TX4938_CLKCTR_ACLCKD	0x02000000
#define TX4938_CLKCTR_PIOCKD	0x01000000
#define TX4938_CLKCTR_DMACKD	0x00800000
#define TX4938_CLKCTR_PCICKD	0x00400000
#define TX4938_CLKCTR_TM0CKD	0x00100000
#define TX4938_CLKCTR_TM1CKD	0x00080000
#define TX4938_CLKCTR_TM2CKD	0x00040000
#define TX4938_CLKCTR_SIO0CKD	0x00020000
#define TX4938_CLKCTR_SIO1CKD	0x00010000
#define TX4938_CLKCTR_ETH1RST	0x00008000
#define TX4938_CLKCTR_ETH0RST	0x00004000
#define TX4938_CLKCTR_SPIRST	0x00002000
#define TX4938_CLKCTR_SRAMRST	0x00001000
#define TX4938_CLKCTR_PCIC1RST	0x00000800
#define TX4938_CLKCTR_DMA1RST	0x00000400
#define TX4938_CLKCTR_ACLRST	0x00000200
#define TX4938_CLKCTR_PIORST	0x00000100
#define TX4938_CLKCTR_DMARST	0x00000080
#define TX4938_CLKCTR_PCIRST	0x00000040
#define TX4938_CLKCTR_TM0RST	0x00000010
#define TX4938_CLKCTR_TM1RST	0x00000008
#define TX4938_CLKCTR_TM2RST	0x00000004
#define TX4938_CLKCTR_SIO0RST	0x00000002
#define TX4938_CLKCTR_SIO1RST	0x00000001


#define tx4938_sdramcptr	((struct tx4938_sdramc_reg *)TX4938_SDRAMC_REG)
#define tx4938_ebuscptr		tx4927_ebuscptr
#define tx4938_dmaptr(ch)	((struct tx4927_dma_reg *)TX4938_DMA_REG(ch))
#define tx4938_ndfmcptr		((struct tx4938_ndfmc_reg *)TX4938_NDFMC_REG)
#define tx4938_ircptr		tx4927_ircptr
#define tx4938_pcicptr		tx4927_pcicptr
#define tx4938_pcic1ptr		((struct tx4927_pcic_reg *)TX4938_PCIC1_REG)
#define tx4938_ccfgptr		((struct tx4938_ccfg_reg *)TX4938_CCFG_REG)
#define tx4938_tmrptr(ch)	tx4927_tmrptr(ch)
#define tx4938_sioptr(ch)	tx4927_sioptr(ch)
#define tx4938_pioptr		tx4927_pioptr
#define tx4938_aclcptr		tx4927_aclcptr
#define tx4938_spiptr		((struct txx9_spi_reg *)TX4938_SPI_REG)
#define tx4938_sramcptr		((struct tx4938_sramc_reg *)TX4938_SRAMC_REG)

#define TX4938_REV_MAJ_MIN()	((__u32)txx9_read64(&tx4938_ccfgptr->crir) & 0x00ff)
#define TX4938_REV_PCODE()	((__u32)txx9_read64(&tx4938_ccfgptr->crir) >> 16)
#define TX4938_CCFG_BCFG()	((__u32)((txx9_read64(&tx4938_ccfgptr->ccfg) & TX4938_CCFG_BCFG_MASK) >> 32))

#define tx4938_ccfg_clear(bits)	tx4927_ccfg_clear(bits)
#define tx4938_ccfg_set(bits)	tx4927_ccfg_set(bits)
#define tx4938_ccfg_change(change, new)	tx4927_ccfg_change(change, new)

#endif /* __ASM_TX_BOARDS_TX4938_H */
