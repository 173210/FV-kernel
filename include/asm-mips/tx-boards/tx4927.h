/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2006
 * All Rights Reserved.
 *
 */
#ifndef __ASM_TX_BOARDS_TX4927_H
#define __ASM_TX_BOARDS_TX4927_H

#include <asm/tx-boards/txx9.h>

#ifdef CONFIG_64BIT
#define TX4927_REG_BASE	0xffffffffff1f0000UL
#else
#define TX4927_REG_BASE	0xff1f0000
#endif
#define TX4927_REG_SIZE	0x00010000

#define TX4927_SDRAMC_REG	(TX4927_REG_BASE + 0x8000)
#define TX4927_EBUSC_REG	(TX4927_REG_BASE + 0x9000)
#define TX4927_DMA_REG		(TX4927_REG_BASE + 0xb000)
#define TX4927_IRC_REG		(TX4927_REG_BASE + 0xf600)
#define TX4927_PCIC_REG		(TX4927_REG_BASE + 0xd000)
#define TX4927_CCFG_REG		(TX4927_REG_BASE + 0xe000)
#define TX4927_NR_TMR	3
#define TX4927_TMR_REG(ch)	(TX4927_REG_BASE + 0xf000 + (ch) * 0x100)
#define TX4927_NR_SIO	2
#define TX4927_SIO_REG(ch)	(TX4927_REG_BASE + 0xf300 + (ch) * 0x100)
#define TX4927_PIO_REG		(TX4927_REG_BASE + 0xf500)
#define TX4927_ACLC_REG		(TX4927_REG_BASE + 0xf700)

#include <asm/byteorder.h>

#ifdef __BIG_ENDIAN
#define endian_def_l2(e1,e2)	\
	volatile __u32 e1,e2
#else
#define endian_def_l2(e1,e2)	\
	volatile __u32 e2,e1
#endif

struct tx4927_sdramc_reg {
	volatile __u64 cr[4];
	volatile __u64 unused0[4];
	volatile __u64 tr;
	volatile __u64 unused1[2];
	volatile __u64 cmd;
};

struct tx4927_ebusc_reg {
	volatile __u64 cr[8];
};

struct tx4927_dma_reg {
	struct tx4927_dma_ch_reg {
		volatile __u64 cha;
		volatile __u64 sar;
		volatile __u64 dar;
		endian_def_l2(unused0, cntr);
		endian_def_l2(unused1, sair);
		endian_def_l2(unused2, dair);
		endian_def_l2(unused3, ccr);
		endian_def_l2(unused4, csr);
	} ch[4];
	volatile __u64 dbr[8];
	volatile __u64 tdhr;
	volatile __u64 midr;
	endian_def_l2(unused0, mcr);
};

struct tx4927_pcic_reg {
	volatile __u32 pciid;
	volatile __u32 pcistatus;
	volatile __u32 pciccrev;
	volatile __u32 pcicfg1;
	volatile __u32 p2gm0plbase;		/* +10 */
	volatile __u32 p2gm0pubase;
	volatile __u32 p2gm1plbase;
	volatile __u32 p2gm1pubase;
	volatile __u32 p2gm2pbase;		/* +20 */
	volatile __u32 p2giopbase;
	volatile __u32 unused0;
	volatile __u32 pcisid;
	volatile __u32 unused1;		/* +30 */
	volatile __u32 pcicapptr;
	volatile __u32 unused2;
	volatile __u32 pcicfg2;
	volatile __u32 g2ptocnt;		/* +40 */
	volatile __u32 unused3[15];
	volatile __u32 g2pstatus;		/* +80 */
	volatile __u32 g2pmask;
	volatile __u32 pcisstatus;
	volatile __u32 pcimask;
	volatile __u32 p2gcfg;		/* +90 */
	volatile __u32 p2gstatus;
	volatile __u32 p2gmask;
	volatile __u32 p2gccmd;
	volatile __u32 unused4[24];		/* +a0 */
	volatile __u32 pbareqport;		/* +100 */
	volatile __u32 pbacfg;
	volatile __u32 pbastatus;
	volatile __u32 pbamask;
	volatile __u32 pbabm;		/* +110 */
	volatile __u32 pbacreq;
	volatile __u32 pbacgnt;
	volatile __u32 pbacstate;
	volatile __u64 g2pmgbase[3];		/* +120 */
	volatile __u64 g2piogbase;
	volatile __u32 g2pmmask[3];		/* +140 */
	volatile __u32 g2piomask;
	volatile __u64 g2pmpbase[3];		/* +150 */
	volatile __u64 g2piopbase;
	volatile __u32 pciccfg;		/* +170 */
	volatile __u32 pcicstatus;
	volatile __u32 pcicmask;
	volatile __u32 unused5;
	volatile __u64 p2gmgbase[3];		/* +180 */
	volatile __u64 p2giogbase;
	volatile __u32 g2pcfgadrs;		/* +1a0 */
	volatile __u32 g2pcfgdata;
	volatile __u32 unused6[8];
	volatile __u32 g2pintack;
	volatile __u32 g2pspc;
	volatile __u32 unused7[12];		/* +1d0 */
	volatile __u64 pdmca;		/* +200 */
	volatile __u64 pdmga;
	volatile __u64 pdmpa;
	volatile __u64 pdmctr;
	volatile __u64 pdmcfg;		/* +220 */
	volatile __u64 pdmsts;
};

struct tx4927_ccfg_reg {
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
};

struct tx4927_aclc_reg {
	volatile __u32 acctlen;
	volatile __u32 acctldis;
	volatile __u32 acregacc;
	volatile __u32 unused0;
	volatile __u32 acintsts;
	volatile __u32 acintmsts;
	volatile __u32 acinten;
	volatile __u32 acintdis;
	volatile __u32 acsemaph;
	volatile __u32 unused1[7];
	volatile __u32 acgpidat;
	volatile __u32 acgpodat;
	volatile __u32 acslten;
	volatile __u32 acsltdis;
	volatile __u32 acfifosts;
	volatile __u32 unused2[11];
	volatile __u32 acdmasts;
	volatile __u32 acdmasel;
	volatile __u32 unused3[6];
	volatile __u32 acaudodat;
	volatile __u32 acsurrdat;
	volatile __u32 accentdat;
	volatile __u32 aclfedat;
	volatile __u32 acaudiat;
	volatile __u32 unused4;
	volatile __u32 acmodoat;
	volatile __u32 acmodidat;
	volatile __u32 unused5[15];
	volatile __u32 acrevid;
};

#undef endian_def_l2

/*
 * SDRAMC
 */

/*
 * EBUSC
 */

/*
 * DMA
 */
/* bits for MCR */
#define TX4927_DMA_MCR_EIS(ch)	(0x10000000<<(ch))
#define TX4927_DMA_MCR_DIS(ch)	(0x01000000<<(ch))
#define TX4927_DMA_MCR_RSFIF	0x00000080
#define TX4927_DMA_MCR_FIFUM(ch)	(0x00000008<<(ch))
#define TX4927_DMA_MCR_RPRT	0x00000002
#define TX4927_DMA_MCR_MSTEN	0x00000001

/* bits for CCRn */
#define TX4927_DMA_CCR_IMMCHN	0x20000000
#define TX4927_DMA_CCR_USEXFSZ	0x10000000
#define TX4927_DMA_CCR_LE	0x08000000
#define TX4927_DMA_CCR_DBINH	0x04000000
#define TX4927_DMA_CCR_SBINH	0x02000000
#define TX4927_DMA_CCR_CHRST	0x01000000
#define TX4927_DMA_CCR_RVBYTE	0x00800000
#define TX4927_DMA_CCR_ACKPOL	0x00400000
#define TX4927_DMA_CCR_REQPL	0x00200000
#define TX4927_DMA_CCR_EGREQ	0x00100000
#define TX4927_DMA_CCR_CHDN	0x00080000
#define TX4927_DMA_CCR_DNCTL	0x00060000
#define TX4927_DMA_CCR_EXTRQ	0x00010000
#define TX4927_DMA_CCR_INTRQD	0x0000e000
#define TX4927_DMA_CCR_INTENE	0x00001000
#define TX4927_DMA_CCR_INTENC	0x00000800
#define TX4927_DMA_CCR_INTENT	0x00000400
#define TX4927_DMA_CCR_CHNEN	0x00000200
#define TX4927_DMA_CCR_XFACT	0x00000100
#define TX4927_DMA_CCR_SMPCHN	0x00000020
#define TX4927_DMA_CCR_XFSZ(order)	(((order) << 2) & 0x0000001c)
#define TX4927_DMA_CCR_XFSZ_1W	TX4927_DMA_CCR_XFSZ(2)
#define TX4927_DMA_CCR_XFSZ_2W	TX4927_DMA_CCR_XFSZ(3)
#define TX4927_DMA_CCR_XFSZ_4W	TX4927_DMA_CCR_XFSZ(4)
#define TX4927_DMA_CCR_XFSZ_8W	TX4927_DMA_CCR_XFSZ(5)
#define TX4927_DMA_CCR_XFSZ_16W	TX4927_DMA_CCR_XFSZ(6)
#define TX4927_DMA_CCR_XFSZ_32W	TX4927_DMA_CCR_XFSZ(7)
#define TX4927_DMA_CCR_MEMIO	0x00000002
#define TX4927_DMA_CCR_SNGAD	0x00000001

/* bits for CSRn */
#define TX4927_DMA_CSR_CHNEN	0x00000400
#define TX4927_DMA_CSR_STLXFER	0x00000200
#define TX4927_DMA_CSR_CHNACT	0x00000100
#define TX4927_DMA_CSR_ABCHC	0x00000080
#define TX4927_DMA_CSR_NCHNC	0x00000040
#define TX4927_DMA_CSR_NTRNFC	0x00000020
#define TX4927_DMA_CSR_EXTDN	0x00000010
#define TX4927_DMA_CSR_CFERR	0x00000008
#define TX4927_DMA_CSR_CHERR	0x00000004
#define TX4927_DMA_CSR_DESERR	0x00000002
#define TX4927_DMA_CSR_SORERR	0x00000001

/*
 * IRC
 */
#define TX4927_IR_ECCERR	0
#define TX4927_IR_WTOERR	1
#define TX4927_NUM_IR_INT	6
#define TX4927_IR_INT(n)	(2 + (n))
#define TX4927_NUM_IR_SIO	2
#define TX4927_IR_SIO(n)	(8 + (n))
#define TX4927_NUM_IR_DMA	4
#define TX4927_IR_DMA(n)	(10 + (n))
#define TX4927_IR_PIO	14
#define TX4927_IR_PDMAC	15
#define TX4927_IR_PCIC	16
#define TX4927_NUM_IR_TMR	3
#define TX4927_IR_TMR(n)	(17 + (n))
#define TX4927_IR_PCIERR	22
#define TX4927_IR_PCIPME	23
#define TX4927_IR_ACLC	24
#define TX4927_IR_ACLCPME	25
#define TX4927_NUM_IR	32

#define TX4927_IRC_INT	2	/* IP[2] in Status register */

/*
 * PCIC
 */
/* bits for PCICMD */
/* see PCI_COMMAND_XXX in linux/pci.h */

/* bits for PCISTAT */
/* see PCI_STATUS_XXX in linux/pci.h */

/* bits for IOBA/MBA */
/* see PCI_BASE_ADDRESS_XXX in linux/pci.h */

/* bits for G2PSTATUS/G2PMASK */
#define TX4927_PCIC_G2PSTATUS_ALL	0x00000003
#define TX4927_PCIC_G2PSTATUS_TTOE	0x00000002
#define TX4927_PCIC_G2PSTATUS_RTOE	0x00000001

/* bits for PCIMASK (see also PCI_STATUS_XXX in linux/pci.h */
#define TX4927_PCIC_PCISTATUS_ALL	0x0000f900

/* bits for PBACFG */
#define TX4927_PCIC_PBACFG_FIXPA	0x00000008
#define TX4927_PCIC_PBACFG_RPBA	0x00000004
#define TX4927_PCIC_PBACFG_PBAEN	0x00000002
#define TX4927_PCIC_PBACFG_BMCEN	0x00000001

/* bits for PBASTATUS/PBAMASK */
#define TX4927_PCIC_PBASTATUS_ALL	0x00000001
#define TX4927_PCIC_PBASTATUS_BM	0x00000001

/* bits for G2PMnGBASE */
#define TX4927_PCIC_G2PMnGBASE_BSDIS	0x0000002000000000ULL
#define TX4927_PCIC_G2PMnGBASE_ECHG	0x0000001000000000ULL

/* bits for G2PIOGBASE */
#define TX4927_PCIC_G2PIOGBASE_BSDIS	0x0000002000000000ULL
#define TX4927_PCIC_G2PIOGBASE_ECHG	0x0000001000000000ULL

/* bits for PCICSTATUS/PCICMASK */
#define TX4927_PCIC_PCICSTATUS_ALL	0x000007b8
#define TX4927_PCIC_PCICSTATUS_PME	0x00000400
#define TX4927_PCIC_PCICSTATUS_TLB	0x00000200
#define TX4927_PCIC_PCICSTATUS_NIB	0x00000100
#define TX4927_PCIC_PCICSTATUS_ZIB	0x00000080
#define TX4927_PCIC_PCICSTATUS_PERR	0x00000020
#define TX4927_PCIC_PCICSTATUS_SERR	0x00000010
#define TX4927_PCIC_PCICSTATUS_GBE	0x00000008
#define TX4927_PCIC_PCICSTATUS_IWB	0x00000002
#define TX4927_PCIC_PCICSTATUS_E2PDONE	0x00000001

/* bits for PCICCFG */
#define TX4927_PCIC_PCICCFG_GBWC_MASK	0x0fff0000
#define TX4927_PCIC_PCICCFG_HRST	0x00000800
#define TX4927_PCIC_PCICCFG_SRST	0x00000400
#define TX4927_PCIC_PCICCFG_IRBER	0x00000200
#define TX4927_PCIC_PCICCFG_G2PMEN(ch)	(0x00000100>>(ch))
#define TX4927_PCIC_PCICCFG_G2PM0EN	0x00000100
#define TX4927_PCIC_PCICCFG_G2PM1EN	0x00000080
#define TX4927_PCIC_PCICCFG_G2PM2EN	0x00000040
#define TX4927_PCIC_PCICCFG_G2PIOEN	0x00000020
#define TX4927_PCIC_PCICCFG_TCAR	0x00000010
#define TX4927_PCIC_PCICCFG_ICAEN	0x00000008

/* bits for P2GMnGBASE */
#define TX4927_PCIC_P2GMnGBASE_TMEMEN	0x0000004000000000ULL
#define TX4927_PCIC_P2GMnGBASE_TBSDIS	0x0000002000000000ULL
#define TX4927_PCIC_P2GMnGBASE_TECHG	0x0000001000000000ULL

/* bits for P2GIOGBASE */
#define TX4927_PCIC_P2GIOGBASE_TIOEN	0x0000004000000000ULL
#define TX4927_PCIC_P2GIOGBASE_TBSDIS	0x0000002000000000ULL
#define TX4927_PCIC_P2GIOGBASE_TECHG	0x0000001000000000ULL

#define TX4927_PCIC_IDSEL_AD_TO_SLOT(ad)	((ad) - 11)
#define TX4927_PCIC_MAX_DEVNU	TX4927_PCIC_IDSEL_AD_TO_SLOT(32)

/* bits for PDMCFG */
#define TX4927_PCIC_PDMCFG_RSTFIFO	0x00200000
#define TX4927_PCIC_PDMCFG_EXFER	0x00100000
#define TX4927_PCIC_PDMCFG_REQDLY_MASK	0x00003800
#define TX4927_PCIC_PDMCFG_REQDLY_NONE	(0 << 11)
#define TX4927_PCIC_PDMCFG_REQDLY_16	(1 << 11)
#define TX4927_PCIC_PDMCFG_REQDLY_32	(2 << 11)
#define TX4927_PCIC_PDMCFG_REQDLY_64	(3 << 11)
#define TX4927_PCIC_PDMCFG_REQDLY_128	(4 << 11)
#define TX4927_PCIC_PDMCFG_REQDLY_256	(5 << 11)
#define TX4927_PCIC_PDMCFG_REQDLY_512	(6 << 11)
#define TX4927_PCIC_PDMCFG_REQDLY_1024	(7 << 11)
#define TX4927_PCIC_PDMCFG_ERRIE	0x00000400
#define TX4927_PCIC_PDMCFG_NCCMPIE	0x00000200
#define TX4927_PCIC_PDMCFG_NTCMPIE	0x00000100
#define TX4927_PCIC_PDMCFG_CHNEN	0x00000080
#define TX4927_PCIC_PDMCFG_XFRACT	0x00000040
#define TX4927_PCIC_PDMCFG_BSWAP	0x00000020
#define TX4927_PCIC_PDMCFG_XFRSIZE_MASK	0x0000000c
#define TX4927_PCIC_PDMCFG_XFRSIZE_1DW	0x00000000
#define TX4927_PCIC_PDMCFG_XFRSIZE_1QW	0x00000004
#define TX4927_PCIC_PDMCFG_XFRSIZE_4QW	0x00000008
#define TX4927_PCIC_PDMCFG_XFRDIRC	0x00000002
#define TX4927_PCIC_PDMCFG_CHRST	0x00000001

/* bits for PDMSTS */
#define TX4927_PCIC_PDMSTS_REQCNT_MASK	0x3f000000
#define TX4927_PCIC_PDMSTS_FIFOCNT_MASK	0x00f00000
#define TX4927_PCIC_PDMSTS_FIFOWP_MASK	0x000c0000
#define TX4927_PCIC_PDMSTS_FIFORP_MASK	0x00030000
#define TX4927_PCIC_PDMSTS_ERRINT	0x00000800
#define TX4927_PCIC_PDMSTS_DONEINT	0x00000400
#define TX4927_PCIC_PDMSTS_CHNEN	0x00000200
#define TX4927_PCIC_PDMSTS_XFRACT	0x00000100
#define TX4927_PCIC_PDMSTS_ACCMP	0x00000080
#define TX4927_PCIC_PDMSTS_NCCMP	0x00000040
#define TX4927_PCIC_PDMSTS_NTCMP	0x00000020
#define TX4927_PCIC_PDMSTS_CFGERR	0x00000008
#define TX4927_PCIC_PDMSTS_PCIERR	0x00000004
#define TX4927_PCIC_PDMSTS_CHNERR	0x00000002
#define TX4927_PCIC_PDMSTS_DATAERR	0x00000001
#define TX4927_PCIC_PDMSTS_ALL_CMP	0x000000e0
#define TX4927_PCIC_PDMSTS_ALL_ERR	0x0000000f

/*
 * CCFG
 */
/* CCFG : Chip Configuration */
#define TX4927_CCFG_WDRST	0x0000020000000000ULL
#define TX4927_CCFG_WDREXEN	0x0000010000000000ULL
#define TX4927_CCFG_BCFG_MASK	0x000000ff00000000ULL
#define TX4927_CCFG_TINTDIS	0x01000000
#define TX4927_CCFG_PCI66	0x00800000
#define TX4927_CCFG_PCIMODE	0x00400000
#define TX4927_CCFG_DIVMODE_MASK	0x000e0000
#define TX4927_CCFG_DIVMODE_8	(0x0 << 17)
#define TX4927_CCFG_DIVMODE_12	(0x1 << 17)
#define TX4927_CCFG_DIVMODE_16	(0x2 << 17)
#define TX4927_CCFG_DIVMODE_10	(0x3 << 17)
#define TX4927_CCFG_DIVMODE_2	(0x4 << 17)
#define TX4927_CCFG_DIVMODE_3	(0x5 << 17)
#define TX4927_CCFG_DIVMODE_4	(0x6 << 17)
#define TX4927_CCFG_DIVMODE_2_5	(0x7 << 17)
#define TX4927_CCFG_BEOW	0x00010000
#define TX4927_CCFG_WR	0x00008000
#define TX4927_CCFG_TOE	0x00004000
#define TX4927_CCFG_PCIARB	0x00002000
#define TX4927_CCFG_PCIDIVMODE_MASK	0x00001800
#define TX4927_CCFG_PCIDIVMODE_2_5	0x00000000
#define TX4927_CCFG_PCIDIVMODE_3	0x00000800
#define TX4927_CCFG_PCIDIVMODE_5	0x00001000
#define TX4927_CCFG_PCIDIVMODE_6	0x00001800
#define TX4927_CCFG_SYSSP_MASK	0x000000c0
#define TX4927_CCFG_ENDIAN	0x00000004
#define TX4927_CCFG_HALT	0x00000002
#define TX4927_CCFG_ACEHOLD	0x00000001
#define TX4927_CCFG_W1CBITS	(TX4927_CCFG_WDRST | TX4927_CCFG_BEOW)

/* PCFG : Pin Configuration */
#define TX4927_PCFG_SDCLKDLY_MASK	0x30000000
#define TX4927_PCFG_SDCLKDLY(d)	((d)<<28)
#define TX4927_PCFG_SYSCLKEN	0x08000000
#define TX4927_PCFG_SDCLKEN_ALL	0x07800000
#define TX4927_PCFG_SDCLKEN(ch)	(0x00800000<<(ch))
#define TX4927_PCFG_PCICLKEN_ALL	0x003f0000
#define TX4927_PCFG_PCICLKEN(ch)	(0x00010000<<(ch))
#define TX4927_PCFG_SEL2	0x00000200
#define TX4927_PCFG_SEL1	0x00000100
#define TX4927_PCFG_DMASEL_ALL	0x000000ff
#define TX4927_PCFG_DMASEL0_MASK	0x00000003
#define TX4927_PCFG_DMASEL1_MASK	0x0000000c
#define TX4927_PCFG_DMASEL2_MASK	0x00000030
#define TX4927_PCFG_DMASEL3_MASK	0x000000c0
#define TX4927_PCFG_DMASEL0_DRQ0	0x00000000
#define TX4927_PCFG_DMASEL0_SIO1	0x00000001
#define TX4927_PCFG_DMASEL0_ACL0	0x00000002
#define TX4927_PCFG_DMASEL0_ACL2	0x00000003
#define TX4927_PCFG_DMASEL1_DRQ1	0x00000000
#define TX4927_PCFG_DMASEL1_SIO1	0x00000004
#define TX4927_PCFG_DMASEL1_ACL1	0x00000008
#define TX4927_PCFG_DMASEL1_ACL3	0x0000000c
#define TX4927_PCFG_DMASEL2_DRQ2	0x00000000	/* SEL2=0 */
#define TX4927_PCFG_DMASEL2_SIO0	0x00000010	/* SEL2=0 */
#define TX4927_PCFG_DMASEL2_ACL1	0x00000000	/* SEL2=1 */
#define TX4927_PCFG_DMASEL2_ACL2	0x00000020	/* SEL2=1 */
#define TX4927_PCFG_DMASEL2_ACL0	0x00000030	/* SEL2=1 */
#define TX4927_PCFG_DMASEL3_DRQ3	0x00000000
#define TX4927_PCFG_DMASEL3_SIO0	0x00000040
#define TX4927_PCFG_DMASEL3_ACL3	0x00000080
#define TX4927_PCFG_DMASEL3_ACL1	0x000000c0

/* CLKCTR : Clock Control */
#define TX4927_CLKCTR_ACLCKD	0x02000000
#define TX4927_CLKCTR_PIOCKD	0x01000000
#define TX4927_CLKCTR_DMACKD	0x00800000
#define TX4927_CLKCTR_PCICKD	0x00400000
#define TX4927_CLKCTR_TM0CKD	0x00100000
#define TX4927_CLKCTR_TM1CKD	0x00080000
#define TX4927_CLKCTR_TM2CKD	0x00040000
#define TX4927_CLKCTR_SIO0CKD	0x00020000
#define TX4927_CLKCTR_SIO1CKD	0x00010000
#define TX4927_CLKCTR_ACLRST	0x00000200
#define TX4927_CLKCTR_PIORST	0x00000100
#define TX4927_CLKCTR_DMARST	0x00000080
#define TX4927_CLKCTR_PCIRST	0x00000040
#define TX4927_CLKCTR_TM0RST	0x00000010
#define TX4927_CLKCTR_TM1RST	0x00000008
#define TX4927_CLKCTR_TM2RST	0x00000004
#define TX4927_CLKCTR_SIO0RST	0x00000002
#define TX4927_CLKCTR_SIO1RST	0x00000001


#define tx4927_sdramcptr	((struct tx4927_sdramc_reg *)TX4927_SDRAMC_REG)
#define tx4927_ebuscptr		((struct tx4927_ebusc_reg *)TX4927_EBUSC_REG)
#define tx4927_dmaptr		((struct tx4927_dma_reg *)TX4927_DMA_REG)
#define tx4927_ircptr		((struct txx9_irc_reg *)TX4927_IRC_REG)
#define tx4927_pcicptr		((struct tx4927_pcic_reg *)TX4927_PCIC_REG)
#define tx4927_ccfgptr		((struct tx4927_ccfg_reg *)TX4927_CCFG_REG)
#define tx4927_tmrptr(ch)	((struct txx9_tmr_reg *)TX4927_TMR_REG(ch))
#define tx4927_sioptr(ch)	((struct txx9_sio_reg *)TX4927_SIO_REG(ch))
#define tx4927_pioptr		((struct txx9_pio_reg *)TX4927_PIO_REG)
#define tx4927_aclcptr		((struct tx4927_aclc_reg *)TX4927_ACLC_REG)

#define TX4927_REV_MAJ_MIN()	((__u32)txx9_read64(&tx4927_ccfgptr->crir) & 0x00ff)
#define TX4927_REV_PCODE()	((__u32)txx9_read64(&tx4927_ccfgptr->crir) >> 16)
#define TX4927_CCFG_BCFG()	((__u32)((txx9_read64(&tx4927_ccfgptr->ccfg) & TX4927_CCFG_BCFG_MASK) >> 32))

#define TX4927_SDRAMC_BA(ch)	((tx4927_sdramcptr->cr[(ch)] >> 49) << 21)
#define TX4927_SDRAMC_SIZE(ch)	\
	((((tx4927_sdramcptr->cr[(ch)] >> 33) & 0x7fff) + 1) << 21)

#define TX4927_EBUSC_BA(ch)	((tx4927_ebuscptr->cr[(ch)] >> 48) << 20)
#define TX4927_EBUSC_SIZE(ch)	\
	(0x00100000 << ((__u32)(tx4927_ebuscptr->cr[(ch)] >> 8) & 0xf))
#define TX4927_EBUSC_WIDTH(ch)	\
	(64 >> ((__u32)(tx4927_ebuscptr->cr[(ch)] >> 20) & 0x3))

/* These functions are not interrupt safe. */
static inline void tx4927_ccfg_clear(__u64 bits)
{
	volatile void __iomem *adr = (__force volatile void __iomem *)&tx4927_ccfgptr->ccfg;
	____raw_writeq(____raw_readq(adr) & ~(TX4927_CCFG_W1CBITS | bits), adr);
}
static inline void tx4927_ccfg_set(__u64 bits)
{
	volatile void __iomem *adr = (__force volatile void __iomem *)&tx4927_ccfgptr->ccfg;
	____raw_writeq((____raw_readq(adr) & ~TX4927_CCFG_W1CBITS) | bits, adr);
}
static inline void tx4927_ccfg_change(__u64 change, __u64 new)
{
	volatile void __iomem *adr = (__force volatile void __iomem *)&tx4927_ccfgptr->ccfg;
	____raw_writeq((____raw_readq(adr) & ~(TX4927_CCFG_W1CBITS | change)) | new, adr);
}

#ifdef CONFIG_PM
struct tx4927_dma_state {
	u32 mcr;
};

static inline void tx4927_dma_save_state(struct tx4927_dma_state *state,
					 struct tx4927_dma_reg *dmaptr)
{
	state->mcr = dmaptr->mcr;
}
static inline void tx4927_dma_restore_state(struct tx4927_dma_state *state,
					    struct tx4927_dma_reg *dmaptr)
{
	dmaptr->mcr = state->mcr;
}
#endif

#endif /* __ASM_TX_BOARDS_TX4927_H */
