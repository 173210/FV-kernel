/*
 * linux/drivers/net/txss_eth.h
 *
 * TX Sub System EtherMAC Controller driver
 *
 * (C) Copyright TOSHIBA CORPORATION SEMICONDUCTOR COMPANY 2008,2009
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <asm/io.h>

#ifdef TXSS_ETH_DEBUG
#define DBG(x...)  printk(x)
#else
#define DBG(x...)
#endif

/* independent from SWAP_IO_SPACE */
#ifdef TXSS_ETH_DEBUG_IO
u32 inline txss_readl(void __iomem * addr)
{
	u32 ret = __raw_readl(addr);
	printk("\ttxss_read (0x%08x) -> 0x%08x\n", (u32) addr, ret);
	return ret;
}

void inline txss_writel(u32 d, void __iomem * addr)
{
	__raw_writel(d, addr);
	printk("\ttxss_write(0x%08x) <- 0x%08x\n", (u32) addr, d);
}
#else
#define txss_readl(addr) __raw_readl(addr)
#define txss_writel(d, addr) __raw_writel(d, addr)
#endif

#define MII_BUS_NAME       "txss_mii_bus"
#define MII_IO_TIMEOUT     1000	/* jiffies */
#define TXSS_MSG_FLAG      (NETIF_MSG_TX_ERR|NETIF_MSG_DRV|NETIF_MSG_LINK)

#define RX_DESC_NUM    32
#define TX_DESC_NUM    32
#define TX_BUF_SIZE    2048
#define RX_BUF_MAX_SIZE    (4096 - NET_SKB_PAD)
#define RX_BUF_SIZE    (2048 - NET_SKB_PAD)
#define TXSS_MAX_MTU	RX_BUF_MAX_SIZE
#define TXSS_MIN_MTU	64

#ifdef TXSS_ETH_USE_CHECKSUM_HW
#define CSUM_LEN       2
#else
#define CSUM_LEN       0
#endif

#ifdef CONFIG_TOSHIBA_TC90417REF
#define MDIOCYCR_SET            0x35	/* 53 x 3.75ns(266MHz) = 202.5ns > 200ns */
#define EEPROM_DEVADDR_OFFSET   0x78
#endif

#ifndef BSTLMTR_SET
#define BSTLMTR_SET    0x00000800	/* default */
#endif
#ifndef MDIOCYCR_SET
#define MDIOCYCR_SET   0x00000035	/* default 200ns */
#endif

struct txss_eth_skbs {
	struct sk_buff *skb;
	dma_addr_t skb_dma;	/* have phys addr of skb->data */
};

struct txss_eth_options {
	int tx_desc_num;
	int rx_desc_num;
	int rx_buf_size;
	int pause_time;
	int phymask;
	int phyaddr;
	int speed;
};

struct txss_eth_local {
	struct net_device *dev;
	struct platform_device *pdev;
	struct mii_bus mii_bus;
	struct phy_device *phydev;
	int duplex;
	int speed;
	int link;
	struct work_struct restart_work;
#ifdef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
	spinlock_t lock;
	int sending;
#endif

	struct net_device_stats stats;
	struct {
		int tx_ints;
		int rx_ints;
		int tx_max_queue_len;
		int rx_max_queue_len;
		int tx_full;
		int tx_pause_count;
		int rx_pause_count;
		int rx_frame_overflow;
		int tx_desc_exhausted;
		int tx_fifo_underflow;
		int rx_desc_exhausted;
		int rx_fifo_overflow;
		int fci;
		int tx_timeouts;
		int rx_enable_timeouts;
		int tx_enable_timeouts;
		int frc_rx_rescheduled;
		int alloc_rxbuf_skb_failed;
	} lstats;
	u32 msg_enable;
	u32 rx_coalesce_usecs;
	u32 max_frame_size;
	u8 tx_pause;
	u8 rx_pause;
	u16 pause_time;
	u16 pause_rff;
	u16 pause_rfd;

	void *desc;
	dma_addr_t desc_dma;
	unsigned int tx_desc_num;
	unsigned int rx_desc_num;
	struct txss_eth_td *tx_desc;
	struct txss_eth_rd *rx_desc;
	struct txss_eth_td *tx_desc_dma;
	struct txss_eth_rd *rx_desc_dma;

	unsigned int txdesc_start;
	unsigned int txdesc_end;
	unsigned int rxdesc_end;

	struct txss_eth_skbs *tx_skbs, *rx_skbs;
};

/* Register Map */

struct txss_eth_regs {
	u32 emacdrr;		/* 0x0000 */
	u32 emacdrtrgr;
	u32 emacdrenr;
	u32 reserved00[5];
	u32 emacintr;		/* 0x0020 */
	u32 emacintenr;
	u32 reserved01[1014];
	u32 dmacr;		/* 0x1000 */
	u32 reserved02[1];
	u32 dmatxr;
	u32 reserved03[1];
	u32 dmarxr;
	u32 reserved04[5];
	u32 dmaintr;
	u32 reserved05[1];
	u32 dmaintenr;
	u32 reserved06[1];
	u32 errmaskr;
	u32 reserved07[1];
	u32 rxmiscntr;
	u32 reserved08[1];
	u32 txfifothr;
	u32 reserved09[1];
	u32 fifosizer;
	u32 reserved10[1];
	u32 dmarxmodr;
	u32 reserved11[1];
	u32 rxpadr;
	u32 reserved12[1];
	u32 rxfifothr;
	u32 reserved13[5];
	u32 txfifostsr;
	u32 reserved13_1[1];
	u32 rxfifostsr;
	u32 reserved13_2[29];
	u32 maccr;		/* 0x1100 */
	u32 reserved14[1];
	u32 lenlmtr;
	u32 reserved15[1];
	u32 macintr;
	u32 reserved16[1];
	u32 macintenr;
	u32 reserved17[4];
	u32 phyintr;
	u32 reserved18[9];
	u32 apftpr;
	u32 mpftpr;
	u32 pftxcntr;
	u32 pfrxcntr;
	u32 pfrtlmtr;
	u32 pfrtcntr;
	u32 reserved19[17];
	u32 macgcr;		/* 0x11b0 */
	u32 bstlmtr;
	u32 reserved20[2];
	u32 umacar;
	u32 reserved21[1];
	u32 lmacar;
	u32 reserved22[77];
	u32 tint1cntr;		/* 0x1300 */
	u32 reserved23[1];
	u32 tint2cntr;
	u32 reserved24[1];
	u32 tint3cntr;
	u32 reserved25[11];
	u32 rint1cntr;
	u32 reserved26[1];
	u32 rint2cntr;
	u32 reserved27[1];
	u32 rint3cntr;
	u32 reserved28[1];
	u32 rint4cntr;
	u32 reserved29[1];
	u32 rint5cntr;
	u32 reserved30[1];
	u32 rint6cntr;
	u32 reserved31[1];
	u32 rint7cntr;
	u32 reserved32[1];
	u32 rint8cntr;
	u32 reserved33[1];
	u32 mdiostsr;
	u32 mdiocmdr;
	u32 mdioadrr;
	u32 mdiodatr;
	u32 mdiocycr;
	u32 reserved34[27];
	u32 desccr;		/* 0x1400 */
	u32 reserved35[3];
	u32 dtxspar;
	u32 dtxcpar;
	u32 dtxlpar;
	u32 dtxdlr;
	u32 reserved36[4];
	u32 drxspar;
	u32 drxcpar;
	u32 drxlpar;
	u32 drxdlr;
	u32 reserved[48];
};

#define EMAC_DRDTRG(x)     (((x) << 16) & 0x1ffc0000)
#define EMAC_DRDMSK(x)     ((x) & 0x00001ffc)

#define EMACDRENR_DRDEN    0x00000001

#define EMACINTR_SERR      0x00000002
#define EMACINTR_DMAINT    0x00000001

#define DMACR_DE           0x00000040
#define DMACR_DL_16        0x00000030
#define DMACR_DL_32        0x00000010
#define DMACR_DL_64        0x00000020
#define DMACR_SWRT         0x00000002
#define DMACR_SWRR         0x00000001

#define DMATXR_START       0x00000003
#define DMATXR_STOP        0x00000000

#define DMARXR_START       0x00000001
#define DMARXR_STOP        0x00000000

#define DMAINTR_ALL        0xff7f07ff
#define DMAINTR_WB1        0x80000000
#define DMAINTR_WB0        0x40000000
#define DMAINTR_FTC1       0x20000000
#define DMAINTR_TUC        0x10000000
#define DMAINTR_ROC        0x08000000
#define DMAINTR_TABT       0x04000000
#define DMAINTR_RABT       0x02000000
#define DMAINTR_RFRMER     0x01000000
#define DMAINTR_MINT       0x00400000
#define DMAINTR_FTC0       0x00200000
#define DMAINTR_TDE        0x00100000
#define DMAINTR_TFE        0x00080000
#define DMAINTR_FRC        0x00040000
#define DMAINTR_RDE        0x00020000
#define DMAINTR_RFE        0x00010000
#define DMAINTR_TINT3      0x00000400
#define DMAINTR_TINT2      0x00000200
#define DMAINTR_TINT1      0x00000100
#define DMAINTR_RINT8      0x00000080
#define DMAINTR_RINT7      0x00000040
#define DMAINTR_RINT6      0x00000020
#define DMAINTR_RINT5      0x00000010
#define DMAINTR_RINT4      0x00000008
#define DMAINTR_RINT3      0x00000004
#define DMAINTR_RINT2      0x00000002
#define DMAINTR_RINT1      0x00000001
#define DMAINTR_ERR        (DMAINTR_RFRMER|DMAINTR_TDE|DMAINTR_TFE|DMAINTR_RDE|DMAINTR_RFE)

#define TXFIFOTHR_SF       0x00000000
#define TXFIFOTHR_MAX      0x00000400

#define FIFOSIZER_DFL      0x00000f0f

#define DMARXMODR_RCVMOD   0x00000001
#define DMARXMODR_DBGMOD   0x00000000

#define RXPADR_PADS(x)     (((x) << 16) & 0x001f0000)
#define RXPADR_PADP(x)     ((x) & 0x0000ffff)

#define RXFIFOTHR_RFF(x)   (((x) << 16) & 0x001f0000)
#define RXFIFOTHR_RFD(x)   ((x) >> 8)

#define RXFIFOTHR_RFF_DFL  21 /* +1 frame */
#define RXFIFOTHR_RFD_DFL  2560 /* byte */
#define PAUSE_TIME_DFL  1000; /* 512us @ 1000base */

#define TXFIFOSTSR_BUSY(x) ((x) & 0x7c0701ff)
#define RXFIFOSTSR_FAM(x)  (((x) & 0x001f0000) >> 16)

#define MACCR_TRCCM        0x04000000
#define MACCR_RCPT         0x02000000
#define MACCR_TCPT         0x01000000
#define MACCR_RCSC         0x00800000
#define MACCR_DPAD         0x00200000
#define MACCR_RZPF         0x00100000
#define MACCR_TZPF         0x00080000
#define MACCR_PFR          0x00040000
#define MACCR_RXF          0x00020000
#define MACCR_TXF          0x00010000
#define MACCR_RPE          0x00000040
#define MACCR_TPE          0x00000020
#define MACCR_DPM          0x00000002
#define MACCR_PRM          0x00000001

#define LENLMTR_LENLMT(x)  ((x) & 0x0003ffff)

#define MACINTR_ALL        0x00000019
#define MACINTR_PFRI       0x00000010
#define MACINTR_PHYI       0x00000008
#define MACINTR_FCI        0x00000001

#define PHYINTR_PHYIP_HIGH 0x00000001
#define PHYINTR_PHYIP_LOW  0x00000000

#define PFRTLMTR_UNLIMIT   0x00000000

#define MACGCR_BSE         0x00000002
#define MACGCR_SPEED       0x00000001

#define MDIOSTSR_BSY       0x00000001

#define MDIOCMDR_READ      0x00000002
#define MDIOCMDR_WRITE     0x00000001

#define MDIOADRR_PHYADR(x) (((x) << 5) & 0x000003e0)
#define MDIOADRR_PHYREG(x) ((x) & 0x0000001f)

#define DESCCR_ENT         0x00000002
#define DESCCR_ENR         0x00000001

#define DTXDLR_DTXDL       0x00000001

#define DRXDLR_DRXDL       0x00000001

struct txss_eth_td {
	u32 td0;
	u32 td1;
	u32 td2;
	u32 dummy;
};

struct txss_eth_rd {
	u32 rd0;
	u32 rd1;
	u32 rd2;
	u32 dummy;
};

#define TD0_TACT            0x80000000
#define TD0_TDL             0x40000000
#define TD0_TFP_MASK        0x30000000
#define TD0_TFP_MIDDLE      0x00000000
#define TD0_TFP_TAIL        0x10000000
#define TD0_TFP_HEAD        0x20000000
#define TD0_TFP_ALL         0x30000000
#define TD0_TFE             0x08000000
#define TD0_TWBI            0x04000000
#define TD0_TFS_MASK        0x000003ff
#define TD0_TFS_TUC         0x00000200
#define TD0_TFS_TABT        0x00000100
#define TD0_TFS_TINT3       0x00000004
#define TD0_TFS_TINT2       0x00000002
#define TD0_TFS_TINT1       0x00000001

#define TD1_TBL_MASK        0xffff0000
#define TD1_TBL_SET(x)      (((x) << 16) & TD1_TBL_MASK)

#define TD2_TBA_SET(x)      (x)

#define RD0_RACT            0x80000000
#define RD0_RDL             0x40000000
#define RD0_RFP_MASK        0x30000000
#define RD0_RFP_MIDDLE      0x00000000
#define RD0_RFP_RAIL        0x10000000
#define RD0_RFP_HEAD        0x20000000
#define RD0_RFP_ALL         0x30000000
#define RD0_RFE             0x08000000
#define RD0_PV              0x04000000
#define RD0_RFS_MASK        0x000003ff
#define RD0_RFS_ROC         0x00000200
#define RD0_RFS_RABT        0x00000100
#define RD0_RFS_RINT8       0x00000080
#define RD0_RFS_RINT7       0x00000040
#define RD0_RFS_RINT6       0x00000020
#define RD0_RFS_RINT5       0x00000010
#define RD0_RFS_RINT4       0x00000008
#define RD0_RFS_RINT3       0x00000004
#define RD0_RFS_RINT2       0x00000002
#define RD0_RFS_RINT1       0x00000001
#define RD0_RFS_ERR         (RD0_RFS_MASK & ~RD0_RFS_RINT8)

#define RD1_RBL_MASK        0xffff0000
#define RD1_RBL_SET(x)      (((x) << 16) & RD1_RBL_MASK)

#define RD1_RFL_MASK        0x0000ffff
#define RD1_RFL_GET(x)      ((x) & RD1_RFL_MASK)

#define RD2_RBA_SET(x)      (x)

extern const char txss_eth_driver_name[];
extern const char txss_eth_driver_version[];

extern u32 txss_eth_update_pftxcntr(struct net_device *dev);
extern u32 txss_eth_update_pfrxcntr(struct net_device *dev);

