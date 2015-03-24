/*
 * Common difinitins for TX49XX builtin controllers
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2007
 * All Rights Reserved.
 *
 */
#ifndef __ASM_TX_BOARDS_TXX9_H
#define __ASM_TX_BOARDS_TXX9_H

#include <linux/spinlock.h>
#include <asm/types.h>
#include <asm/io.h>

struct txx9_tmr_reg {
	volatile __u32 tcr;
	volatile __u32 tisr;
	volatile __u32 cpra;
	volatile __u32 cprb;
	volatile __u32 itmr;
	volatile __u32 unused0[3];
	volatile __u32 ccdr;
	volatile __u32 unused1[3];
	volatile __u32 pgmr;
	volatile __u32 unused2[3];
	volatile __u32 wtmr;
	volatile __u32 unused3[43];
	volatile __u32 trr;
};

struct txx9_sio_reg {
	volatile __u32 lcr;
	volatile __u32 dicr;
	volatile __u32 disr;
	volatile __u32 cisr;
	volatile __u32 fcr;
	volatile __u32 flcr;
	volatile __u32 bgr;
	volatile __u32 tfifo;
	volatile __u32 rfifo;
};

struct txx9_pio_reg {
	volatile __u32 dout;
	volatile __u32 din;
	volatile __u32 dir;
	volatile __u32 od;
	volatile __u32 flag[2];
	volatile __u32 pol;
	volatile __u32 intc;
	volatile __u32 maskcpu;
	volatile __u32 maskext;
};

struct txx9_irc_reg {
	volatile __u32 cer;
	volatile __u32 cr[2];
	volatile __u32 unused0;
	volatile __u32 ilr[8];
	volatile __u32 unused1[4];
	volatile __u32 imr;
	volatile __u32 unused2[7];
	volatile __u32 scr;
	volatile __u32 unused3[7];
	volatile __u32 ssr;
	volatile __u32 unused4[7];
	volatile __u32 csr;
};

struct txx9_64_irc_reg {
	volatile __u64 cer;
	volatile __u64 cr[2];
	volatile __u64 unused1[1];
	volatile __u64 ilr[8];
	volatile __u64 unused2[4];
	volatile __u64 imr;
	volatile __u64 unused3[7];
	volatile __u64 scr;
	volatile __u64 unused4[7];
	volatile __u64 ssr;
	volatile __u64 ssr2;
	volatile __u64 ipnr;
	volatile __u64 unused5[5];
	volatile __u64 csr;
	volatile __u64 cipsr;
	volatile __u64 opr;
	volatile __u64 unused6[5];
	volatile __u64 dcntr[3];
};

struct txx9_spi_reg {
	volatile __u32 mcr;
	volatile __u32 cr0;
	volatile __u32 cr1;
	volatile __u32 fs;
	volatile __u32 unused1;
	volatile __u32 sr;
	volatile __u32 dr;
	volatile __u32 unused2;
};

/*
 * TMR
 */
/* TMTCR : Timer Control */
#define TXx9_TMTCR_TCE	0x00000080
#define TXx9_TMTCR_CCDE	0x00000040
#define TXx9_TMTCR_CRE	0x00000020
#define TXx9_TMTCR_ECES	0x00000008
#define TXx9_TMTCR_CCS	0x00000004
#define TXx9_TMTCR_TMODE_MASK	0x00000003
#define TXx9_TMTCR_TMODE_ITVL	0x00000000
#define TXx9_TMTCR_TMODE_PGEN	0x00000001
#define TXx9_TMTCR_TMODE_WDOG	0x00000002

/* TMTISR : Timer Int. Status */
#define TXx9_TMTISR_TPIBS	0x00000004
#define TXx9_TMTISR_TPIAS	0x00000002
#define TXx9_TMTISR_TIIS	0x00000001

/* TMITMR : Interval Timer Mode */
#define TXx9_TMITMR_TIIE	0x00008000
#define TXx9_TMITMR_TZCE	0x00000001

/* TMWTMR : Watchdog Timer Mode */
#define TXx9_TMWTMR_TWIE	0x00008000
#define TXx9_TMWTMR_WDIS	0x00000080
#define TXx9_TMWTMR_TWC	0x00000001

/*
 * SIO
 */
/* SILCR : Line Control */
#define TXx9_SILCR_SCS_MASK	0x00000060
#define TXx9_SILCR_SCS_IMCLK	0x00000000
#define TXx9_SILCR_SCS_IMCLK_BG	0x00000020
#define TXx9_SILCR_SCS_SCLK	0x00000040
#define TXx9_SILCR_SCS_SCLK_BG	0x00000060
#define TXx9_SILCR_UEPS	0x00000010
#define TXx9_SILCR_UPEN	0x00000008
#define TXx9_SILCR_USBL_MASK	0x00000004
#define TXx9_SILCR_USBL_1BIT	0x00000000
#define TXx9_SILCR_USBL_2BIT	0x00000004
#define TXx9_SILCR_UMODE_MASK	0x00000003
#define TXx9_SILCR_UMODE_8BIT	0x00000000
#define TXx9_SILCR_UMODE_7BIT	0x00000001

/* SIDICR : DMA/Int. Control */
#define TXx9_SIDICR_TDE	0x00008000
#define TXx9_SIDICR_RDE	0x00004000
#define TXx9_SIDICR_TIE	0x00002000
#define TXx9_SIDICR_RIE	0x00001000
#define TXx9_SIDICR_SPIE	0x00000800
#define TXx9_SIDICR_CTSAC	0x00000600
#define TXx9_SIDICR_STIE_MASK	0x0000003f
#define TXx9_SIDICR_STIE_OERS		0x00000020
#define TXx9_SIDICR_STIE_CTSS		0x00000010
#define TXx9_SIDICR_STIE_RBRKD	0x00000008
#define TXx9_SIDICR_STIE_TRDY		0x00000004
#define TXx9_SIDICR_STIE_TXALS	0x00000002
#define TXx9_SIDICR_STIE_UBRKD	0x00000001

/* SIDISR : DMA/Int. Status */
#define TXx9_SIDISR_UBRK	0x00008000
#define TXx9_SIDISR_UVALID	0x00004000
#define TXx9_SIDISR_UFER	0x00002000
#define TXx9_SIDISR_UPER	0x00001000
#define TXx9_SIDISR_UOER	0x00000800
#define TXx9_SIDISR_ERI	0x00000400
#define TXx9_SIDISR_TOUT	0x00000200
#define TXx9_SIDISR_TDIS	0x00000100
#define TXx9_SIDISR_RDIS	0x00000080
#define TXx9_SIDISR_STIS	0x00000040
#define TXx9_SIDISR_RFDN_MASK	0x0000001f

/* SICISR : Change Int. Status */
#define TXx9_SICISR_OERS	0x00000020
#define TXx9_SICISR_CTSS	0x00000010
#define TXx9_SICISR_RBRKD	0x00000008
#define TXx9_SICISR_TRDY	0x00000004
#define TXx9_SICISR_TXALS	0x00000002
#define TXx9_SICISR_UBRKD	0x00000001

/* SIFCR : FIFO Control */
#define TXx9_SIFCR_SWRST	0x00008000
#define TXx9_SIFCR_RDIL_MASK	0x00000180
#define TXx9_SIFCR_RDIL_1	0x00000000
#define TXx9_SIFCR_RDIL_4	0x00000080
#define TXx9_SIFCR_RDIL_8	0x00000100
#define TXx9_SIFCR_RDIL_12	0x00000180
#define TXx9_SIFCR_RDIL_MAX	0x00000180
#define TXx9_SIFCR_TDIL_MASK	0x00000018
#define TXx9_SIFCR_TDIL_MASK	0x00000018
#define TXx9_SIFCR_TDIL_1	0x00000000
#define TXx9_SIFCR_TDIL_4	0x00000001
#define TXx9_SIFCR_TDIL_8	0x00000010
#define TXx9_SIFCR_TDIL_MAX	0x00000010
#define TXx9_SIFCR_TFRST	0x00000004
#define TXx9_SIFCR_RFRST	0x00000002
#define TXx9_SIFCR_FRSTE	0x00000001
#define TXx9_SIO_TX_FIFO	8
#define TXx9_SIO_RX_FIFO	16

/* SIFLCR : Flow Control */
#define TXx9_SIFLCR_RCS	0x00001000
#define TXx9_SIFLCR_TES	0x00000800
#define TXx9_SIFLCR_RTSSC	0x00000200
#define TXx9_SIFLCR_RSDE	0x00000100
#define TXx9_SIFLCR_TSDE	0x00000080
#define TXx9_SIFLCR_RTSTL_MASK	0x0000001e
#define TXx9_SIFLCR_RTSTL_MAX	0x0000001e
#define TXx9_SIFLCR_TBRK	0x00000001

/* SIBGR : Baudrate Control */
#define TXx9_SIBGR_BCLK_MASK	0x00000300
#define TXx9_SIBGR_BCLK_T0	0x00000000
#define TXx9_SIBGR_BCLK_T2	0x00000100
#define TXx9_SIBGR_BCLK_T4	0x00000200
#define TXx9_SIBGR_BCLK_T6	0x00000300
#define TXx9_SIBGR_BRD_MASK	0x000000ff

/*
 * PIO
 */
extern spinlock_t txx9_pio_lock;

static inline void
txx9_pio_set(struct txx9_pio_reg *pioptr, unsigned int bits)
{
	unsigned long flags;
	spin_lock_irqsave(&txx9_pio_lock, flags);
	pioptr->dout |= bits;
	spin_unlock_irqrestore(&txx9_pio_lock, flags);
}

static inline void
txx9_pio_clear(struct txx9_pio_reg *pioptr, unsigned int bits)
{
	unsigned long flags;
	spin_lock_irqsave(&txx9_pio_lock, flags);
	pioptr->dout &= ~bits;
	spin_unlock_irqrestore(&txx9_pio_lock, flags);
}

/*
 * IRC
 */
#define TXx9_IR_MAX_LEVEL	7

/* IRCER : Int. Control Enable */
#define TXx9_IRCER_ICE	0x00000001

/* IRCR : Int. Control */
#define TXx9_IRCR_LOW	0x00000000
#define TXx9_IRCR_HIGH	0x00000001
#define TXx9_IRCR_DOWN	0x00000002
#define TXx9_IRCR_UP	0x00000003
#define TXx9_IRCR_EDGE(cr)	((cr) & 0x00000002)

/* IRSCR : Int. Status Control */
#define TXx9_IRSCR_EIClrE	0x00000100
#define TXx9_IRSCR_EIClr_MASK	0x0000000f

/* IRCSR : Int. Current Status */
#define TXx9_IRCSR_IF	0x00010000
#define TXx9_IRCSR_ILV_MASK	0x00000700
#define TXx9_IRCSR_IVL_MASK	0x0000001f

/*
 * SPI
 */

/* SPMCR : SPI Master Control */
#define TXx9_SPMCR_OPMODE	0xc0
#define TXx9_SPMCR_CONFIG	0x40
#define TXx9_SPMCR_ACTIVE	0x80
#define TXx9_SPMCR_SPSTP	0x02
#define TXx9_SPMCR_BCLR	0x01

/* SPCR0 : SPI Control 0 */
#define TXx9_SPCR0_TXIFL_MASK	0xc000
#define TXx9_SPCR0_RXIFL_MASK	0x3000
#define TXx9_SPCR0_SIDIE	0x0800
#define TXx9_SPCR0_SOEIE	0x0400
#define TXx9_SPCR0_RBSIE	0x0200
#define TXx9_SPCR0_TBSIE	0x0100
#define TXx9_SPCR0_IFSPSE	0x0010
#define TXx9_SPCR0_SBOS	0x0004
#define TXx9_SPCR0_SPHA	0x0002
#define TXx9_SPCR0_SPOL	0x0001

/* SPSR : SPI Status */
#define TXx9_SPSR_TBSI	0x8000
#define TXx9_SPSR_RBSI	0x4000
#define TXx9_SPSR_TBS_MASK	0x3800
#define TXx9_SPSR_RBS_MASK	0x0700
#define TXx9_SPSR_SPOE	0x0080
#define TXx9_SPSR_IFSD	0x0008
#define TXx9_SPSR_SIDLE	0x0004
#define TXx9_SPSR_STRDY	0x0002
#define TXx9_SPSR_SRRDY	0x0001

/* SPILR : */
#define TXx9_SPILR_TXIFL_MASK	0x1f00
#define TXx9_SPILR_RXIFL_MASK	0x001f


#define TXx9_SPI_REGS	13
#define TXx9_SPMCR	0
#define TXx9_SPCR0	1
#define TXx9_SPCR1	2
#define TXx9_SPFS	3
#define TXx9_SPSR	4
#define TXx9_SPDR	5
#ifdef CONFIG_TOSHIBA_TC90416
#define TXx9_SPSSR      6
#define TXx9_SPRSR      7
#define TXx9_SPFLR      8
#define TXx9_SPILR      9
#define TXx9_SPPR      10
#define TXx9_SPLCR     11
#define TXx9_SPDER     12
#endif

/* for txx9spi driver */
struct spi_device;
struct txx9spi_platform_data {
	unsigned int offsets[TXx9_SPI_REGS];
	unsigned int baseclk;
	int (*cs_func)(struct spi_device *spi, int on);
	u32 (*calc_cr1)(struct spi_device *spi);
	u32 cr0_baseval;
	volatile void __iomem *membase;
	void *data;
};

/* for i2c-txx9pio driver */
struct txx9pio_i2c_config {
	struct txx9_pio_reg *pioptr;
	int scl_pin, sda_pin;
};

/* utilities */
static inline void txx9_clear64(volatile __u64 *adr, __u64 bits)
{
	volatile void __iomem *mem = (__force volatile void __iomem *)adr;
	____raw_writeq(____raw_readq(mem) & ~bits, mem);
}
static inline void txx9_set64(volatile __u64 *adr, __u64 bits)
{
	volatile void __iomem *mem = (__force volatile void __iomem *)adr;
	____raw_writeq(____raw_readq(mem) | bits, mem);
}
static inline __u64 txx9_read64(volatile __u64 *adr)
{
	volatile void __iomem *mem = (__force volatile void __iomem *)adr;
	return __raw_readq(mem);
}
static inline void txx9_write64(__u64 value, volatile __u64 *adr)
{
	volatile void __iomem *mem = (__force volatile void __iomem *)adr;
	__raw_writeq(value, mem);
}
static inline __u64 __txx9_read64(volatile __u64 *adr)
{
	volatile void __iomem *mem = (__force volatile void __iomem *)adr;
	return ____raw_readq(mem);
}
static inline void __txx9_write64(__u64 value, volatile __u64 *adr)
{
	volatile void __iomem *mem = (__force volatile void __iomem *)adr;
	____raw_writeq(value, mem);
}

#ifdef CONFIG_PM
struct txx9_tmr_state {
	u32 tcr;
	u32 cpra;
	u32 itmr;
	u32 ccdr;
};

static inline void txx9_tmr_save_state(struct txx9_tmr_state *state,
				       struct txx9_tmr_reg *tmrptr)
{
	state->cpra = tmrptr->cpra;
	state->itmr = tmrptr->itmr;
	state->ccdr = tmrptr->ccdr;
	state->tcr = tmrptr->tcr;
}

static inline void txx9_tmr_restore_state(struct txx9_tmr_state *state,
					  struct txx9_tmr_reg *tmrptr)
{
	tmrptr->cpra = state->cpra;
	tmrptr->itmr = state->itmr;
	tmrptr->ccdr = state->ccdr;
	tmrptr->tcr = state->tcr;
	tmrptr->tisr = 0;
}

struct txx9_pio_state {
	u32 dout;
	u32 dir;
	u32 od;
};

static inline void txx9_pio_save_state(struct txx9_pio_state *state,
				       struct txx9_pio_reg *pioptr)
{
	state->dout = pioptr->dout;
	state->dir = pioptr->dir;
	state->od = pioptr->od;
}

static inline void txx9_pio_restore_state(struct txx9_pio_state *state,
					  struct txx9_pio_reg *pioptr)
{
	pioptr->dout = state->dout;
	pioptr->od = state->od;
	pioptr->dir = state->dir;
}
#endif

#endif /* __ASM_TX_BOARDS_TXX9_H */
