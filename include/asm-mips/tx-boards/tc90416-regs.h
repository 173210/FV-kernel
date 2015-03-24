/*
 * linux/include/asm-mips/tx-boards/tc90416-regs.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2008
 * All Rights Reserved.
 *
 */
#ifndef __ASM_TX_BOARDS_TC90416_REGS_H
#define __ASM_TX_BOARDS_TC90416_REGS_H

#ifndef LANGUAGE_ASSEMBLY
#include <asm/tx-boards/tx4927.h>	/* some controllers are compatible with 4927 */
#endif

#define TC90416_REG_PBASE	0x18800000
#define TC90416_REG_BASE	(TC90416_REG_PBASE | 0xA0000000)
#define TC90416_REG_SIZE	0x00010000

#define TC90416_NR_TMR   3
#define TC90416_TMR_REG(ch)	(TC90416_REG_BASE + 0xFA00 + (ch) * 0x100)
#define TC90416_PIO_REG		(TC90416_REG_BASE + 0xF500)
#define TC90416_CI_REG		(TC90416_REG_BASE + 0xF400)
#define TC90416_I2C_PHYS_REG	(TC90416_REG_PBASE + 0xF000)
#define TC90416_I2C2_REG	(TC90416_REG_BASE + 0xF200)
#define TC90416_I2C1_REG	(TC90416_REG_BASE + 0xF100)
#define TC90416_I2C0_REG	(TC90416_REG_BASE + 0xF000)
#define TC90416_CONFIG_REG	(TC90416_REG_BASE + 0xE000)
#define TC90416_SDHC_PHYS_REG	(TC90416_REG_PBASE + 0xA000)
#define TC90416_SDHC_REG	(TC90416_REG_BASE + 0xA000)
#define TC90416_EBUSC_REG	(TC90416_REG_BASE + 0x9000)
#define TC90416_SPI_PHYS_REG	(TC90416_REG_PBASE + 0x6000)
#define TC90416_SPI_REG		(TC90416_REG_BASE + 0x6000)
#define TC90416_NDFMC_PHYS_REG	(TC90416_REG_PBASE + 0x5000)
#define TC90416_NDFMC_REG	(TC90416_REG_BASE + 0x5000)
#define TC90416_IRC_REG		(TC90416_REG_BASE + 0x4000)
#define TC90416_GDMAC_REG	(TC90416_REG_BASE + 0x3000)
#define TC90416_NR_SIO   4
#define TC90416_SIO_REG(ch)     (TC90416_REG_BASE + 0x2400 + (ch) * 0x40)
#define TC90416_SCC_REG		(TC90416_REG_BASE + 0x2000)
#define TC90416_USBHC_PHYS_REG	0x18720000
#define TC90416_USBHC_REG	(TC90416_USBHC_PHYS_REG | 0xA0000000)
#define TC90416_PCIeC_REG	(0x18710000 | 0xA0000000)
#define TC90416_CLG_REG		(0x18560000 | 0xA0000000)
#define TC90416_DXDRC_REG	(0x18490000 | 0xA0000000)
#define TC90416_SXDRC_REG_CBUS	(0x18480000 | 0xA0000000)
#define TC90416_SXDRC_REG_MBUS	(0x18470000 | 0xA0000000)

/* for TC90417 only */
#define TC90417_EMAC_PHYS_REG	0x18730000
#define TC90417_EMAC_REG	(TC90417_EMAC_PHYS_REG | 0xA0000000)
#define TC90417_DDRC_TOP_PHYS_REG	0x18474000
#define TC90417_DDRC_2ND_PHYS_REG	0x18480000
#define TC90417_DDRC_3RD_PHYS_REG	0x18484000
#define TC90417_DDRC_TOP_REG	(TC90417_DDRC_TOP_PHYS_REG | 0xA0000000)
#define TC90417_DDRC_2ND_REG	(TC90417_DDRC_2ND_PHYS_REG | 0xA0000000)
#define TC90417_DDRC_3RD_REG	(TC90417_DDRC_3RD_PHYS_REG | 0xA0000000)

#ifndef LANGUAGE_ASSEMBLY

#ifdef __BIG_ENDIAN
#define endian_def_l2(e1,e2)	\
	volatile __u32 e1,e2
#else
#define endian_def_l2(e1,e2)	\
	volatile __u32 e2,e1
#endif

/*
 * config registers define.
 */
struct tc90416_ccfg_reg {
	volatile __u64 ccfg;          /* 0x000 */
	endian_def_l2(unused0, revid);/* 0x008 */
	volatile __u64 pcfg;          /* 0x010 */
	volatile __u64 unused1;
	volatile __u64 clkrstctr;     /* 0x020 */
	volatile __u64 unused2;
	volatile __u64 gbusea;        /* 0x030 */
	volatile __u64 unused3[41];
	volatile __u64 gbssraddr;     /* 0x180 */
	volatile __u64 gbs0srtrg;     /* 0x188 */
	volatile __u64 gbs1srtrg;     /* 0x190 */
	volatile __u64 gbs2srtrg;     /* 0x198 */
	volatile __u64 gbs3srtrg;     /* 0x1A0 */
	volatile __u64 gbs4srtrg;     /* 0x1A8 */
	volatile __u64 gbs5srtrg;     /* 0x1B0 */
	volatile __u64 gbssren;       /* 0x1B8 */
	volatile __u64 gbssrcnt;      /* 0x1C0 */
	volatile __u64 gbssrcen;      /* 0x1C8 */
	volatile __u64 unused4[4];
	volatile __u64 m24kefcfg;     /* 0x200 */
};

/*
 * External Memory controller define.
 */
struct tc90416_ebusc_reg {
	volatile __u64 ccr[8];
};

/*
 * Parallel I/O port define.
 */
struct tc90416_pio_reg {
	volatile __u32 dout0;
	volatile __u32 din0;
	volatile __u32 dir0;
	volatile __u32 od0;
	volatile __u32 flag[2];
	volatile __u32 pol;
	volatile __u32 intc;
	volatile __u32 maskcpu;
	volatile __u32 unused;
	volatile __u32 dout1;
	volatile __u32 din1;
	volatile __u32 dir1;
	volatile __u32 od1;
	volatile __u32 dout2;
	volatile __u32 din2;
	volatile __u32 dir2;
	volatile __u32 od2;
	volatile __u32 reserved[14];
	volatile __u32 dout0s;
	volatile __u32 dout0r;
	volatile __u32 dout1s;
	volatile __u32 dout1r;
	volatile __u32 dout2s;
	volatile __u32 dout2r;
};

/*
 * USB HCB register define.
 */
struct tc90416_usbhcb_reg {
	volatile __u32 reserved0;
	volatile __u32 control;
	volatile __u32 framelength;
	volatile __u32 reserved1;
	volatile __u32 draddress;
	volatile __u32 drtrigger0;
	volatile __u32 drtriggermask0;
	volatile __u32 drtrigger1;
	volatile __u32 drtriggermask1;
};

#endif /* LANGUAGE_ASSEMBLY */

#define TC90416_USBHCB_CTRL_RSTSTS	(1<<8)
#define TC90416_USBHCB_CTRL_DREN	(1<<6)
#define TC90416_USBHCB_CTRL_PREFEN	(1<<5)
#define TC90416_USBHCB_CTRL_ENDIAN	(1<<4)
#define TC90416_USBHCB_CTRL_IBSWEN	(1<<3)
#define TC90416_USBHCB_CTRL_TBWEN	(1<<2)
#define TC90416_USBHCB_CTRL_OVCP	(1<<0)

/*
 *
 * controller reigsters bits define.
 *
 */
/* chip configuration register */
#define TC90416_CCFG_GTOT_MASK                 (3ULL << 26)
#define TC90416_CCFG_TOE                       (1ULL << 25)
#define TC90416_CCFG_CCFG	               (1ULL << 10) /* TC90417 ES2 or later */
#define TC90416_CCFG_BCFG_MASK                 (7ULL <<  8)
#define TC90416_CCFG_DMASEL_MASK               (7ULL <<  4)
#define TC90416_CCFG_ALEH		       (1UUL <<  0)



/* IRC */
#define TC90416_IR_WTOUT         1
#define TC90416_IR_HBUS_SERR     2
#define TC90416_IRC_FLAG         3
/* 4 reserverd */
#define TC90416_NUM_IR_I2C       2
#define TC90416_IR_I2C(n)     (5 + (n))  /* 5 - 6 */
/* 7 reserverd */
#define TC90416_NUM_IR_TMR       3
#define TC90416_IR_TMR(n)     (8 + (n))  /* 8 - 10 */
/* 11 reserverd */
#define TC90416_IRQ10           12  /* TC90416 only */
#define TC90416_IRQ11           13  /* TC90416 only */
/* 14-15 reserverd */
#define TC90416_NUM_IR_SIO       4
#define TC90416_IR_SIO(n)    (16 + (n))  /* 16 - 19 (or 30 SCC0, 31 SCC1) */
#define TC90416_IR_PCIe         20  /* TC90416 only */
#define TC90416_IR_PCIePCI      21  /* TC90416 only */
#define TC90416_IR_PCIeMSI      22  /* TC90416 only */
#define TC90416_IR_PCIeEP       23  /* TC90416 only */
/* 24-26 reserverd */
#define TC90416_IR_PCIeINTPCLK  27
#define TC90416_IRQ6            28
#define TC90416_IRQ7            29
#define TC90416_IRQ8            30
#define TC90416_IRQ9            31  /* TC90416 only */
#define TC90416_IRQ0            32
#define TC90416_IRQ1            33
#define TC90416_IRQ2            34
#define TC90416_IRQ3            35
#define TC90416_IRQ4            36
#define TC90416_IRQ5            37
#define TC90416_IR_USBHC        38
#define TC90416_IR_NDFMC        39
#define TC90416_NUM_IR_GDMA      4
#define TC90416_IR_GDMA(n)   (40 + (n))  /* 40 - 43 */
/* 44-46 reserverd */
#define TC90416_IR_CI           47
/* 48-49 reserverd */
#define TC90416_IR_SDHCCD       50
#define TC90416_IR_SDHCCTR      51
#define TC90416_IR_SCC		52
/* 53 reserverd */
#define TC90417_IR_EMAC		54  /* TC90417 only */
#define TC90417_IR_SPI		55  /* TC90417 only */
#define TC90416_IR_CRISC        56
#define TC90416_IR_TSP          57
#define TC90416_IR_ASP0         58
#define TC90416_IR_ASP1         59
#define TC90416_IR_ASP2         60
/* 61 - 63 reserverd */
#define TC90416_NUM_IR          64

#define TC90416_IRC_INT  2   /* IP[7] in Status register */

#ifndef LANGUAGE_ASSEMBLY

#define tc90416_gdmaptr          ((struct tx4927_dma_reg *)TC90416_GDMAC_REG)

#define tc90416_ccfgptr         ((struct tc90416_ccfg_reg *)TC90416_CONFIG_REG)
#define tc90416_ircptr          ((struct txx9_64_irc_reg *)TC90416_IRC_REG)
#define tc90416_ebuscptr        ((struct tc90416_ebusc_reg *)TC90416_EBUSC_REG)
#define tc90416_pioptr          ((struct tc90416_pio_reg *)TC90416_PIO_REG)
#define tc90416_sioptr(ch)      ((struct txx9_sio_reg *)TC90416_SIO_REG(ch))
#define tc90416_tmrptr(ch)      ((struct txx9_tmr_reg *)TC90416_TMR_REG(ch))

#define TC90416_REV_MAJ_MIN()   (tc90416_ccfgptr->revid & 0x00ff)
#define TC90416_REV_PCODE()     (tc90416_ccfgptr->revid >> 16)

#endif /* LANGUAGE_ASSEMBLY */

#endif /* __ASM_TX_BOARDS_TC90416_REGS_H */
