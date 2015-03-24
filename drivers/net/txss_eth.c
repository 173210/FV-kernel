/*
 * linux/drivers/net/txss_eth.c
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

#undef TXSS_ETH_DEBUG
#undef TXSS_ETH_DEBUG_IO
#define TXSS_ETH_USE_CHECKSUM_HW
#define TXSS_ETH_MAX_QUEUE_LEN

#ifdef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
#define DRV_VERSION "1.5_p1_type2a-1"
#else
#define DRV_VERSION "1.5_p1"
#endif
#define DRV_DEVNAME "txss-eth"
const char txss_eth_driver_name[] = DRV_DEVNAME;
const char txss_eth_driver_version[] = DRV_VERSION;
static const char *print_version =
    "TX sub system EtherMAC controller driver: ver" DRV_VERSION;

#define NET_SKB_PAD     32	/* RD0.RBA 32byte DMA alignment */

#define NAPI_WEIGHT     16

#include <linux/version.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/gfp.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <asm/page.h>
#include "txss_eth.h"
#include "asm/tx-boards/tsb-generic.h"

static int txss_eth_open(struct net_device *dev);
static int txss_eth_stop(struct net_device *dev);
static int txss_eth_send_packet(struct sk_buff *skb, struct net_device *dev);
static void txss_eth_tx_timeout(struct net_device *dev);
static struct net_device_stats *txss_eth_get_stats(struct net_device *dev);
static void txss_eth_set_multicast_list(struct net_device *dev);
static int txss_eth_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd);
static int txss_eth_poll(struct net_device *dev, int *quota);
#ifdef CONFIG_NET_POLL_CONTROLLER
static void txss_eth_poll_controller(struct net_device *dev);
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t txss_eth_interrupt(int irq, void *dev_id, struct pt_regs *pt);
#else /* 2.6.19 and later */
static irqreturn_t txss_eth_interrupt(int irq, void *dev_id);
#endif
static int txss_eth_rx(struct net_device *dev, int limit);
static int txss_eth_tx_done(struct net_device *dev);
static void txss_eth_rx_stop(struct net_device *dev);
static void txss_eth_tx_stop(struct net_device *dev);
static void txss_eth_desc_start(struct net_device *dev);
static void txss_eth_soft_reset(struct net_device *dev);
static void txss_eth_phy_init(struct net_device *dev);
static void txss_eth_chip_reset(struct net_device *dev);
static void txss_eth_chip_init(struct net_device *dev);
static int txss_eth_mii_read(struct mii_bus *bus, int phy_id, int regnum);
static int txss_eth_mii_write(struct mii_bus *bus, int phy_id, int regnum, u16 val);
static void txss_eth_mii_link_change(struct net_device *dev);

void txss_eth_set_ethtool_ops(struct net_device *dev);

MODULE_DESCRIPTION("TX Sub System EtherMAC Controller driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

static struct txss_eth_options options;
static int phy_start_at_probe, phy_wait_link_change;
volatile static int txss_eth_rx_retry;
#ifdef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
volatile static int txss_eth_rx_frc;
#endif

u32 txss_eth_update_pftxcntr(struct net_device *dev){
	struct txss_eth_local *lp = netdev_priv(dev);
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;
	u32 cnt = txss_readl(&tr->pftxcntr);
	lp->lstats.tx_pause_count += cnt;
	return cnt;
}

u32 txss_eth_update_pfrxcntr(struct net_device *dev){
	struct txss_eth_local *lp = netdev_priv(dev);
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;
	u32 cnt = txss_readl(&tr->pfrxcntr);
	lp->lstats.rx_pause_count += cnt;
	return cnt;
}

#ifdef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
static inline void txss_eth_tx_enable(struct net_device *dev)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;	
	unsigned long flag;
	unsigned long timeout = jiffies + msecs_to_jiffies(10000); /* 10sec */

	while (txss_readl(&tr->dmarxr) != DMARXR_START) {
		u32 status = txss_readl(&tr->dmaintr);
		if (status & DMAINTR_RDE)
			break;
		if (time_after(jiffies, timeout)) {
			printk(KERN_WARNING "%s: timeout\n", __func__);
			lp->lstats.tx_enable_timeouts++;
			break;
		}
	}
	spin_lock_irqsave(&lp->lock, flag);
	txss_writel(DMATXR_START, &tr->dmatxr);	/* Tx DMA start */
	lp->sending = 1;
	spin_unlock_irqrestore(&lp->lock, flag);
}

static inline void txss_eth_rx_enable(struct net_device *dev)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;	
	unsigned long flag;
	unsigned long timeout = jiffies + msecs_to_jiffies(10000); /* 10sec */

	spin_lock_irqsave(&lp->lock, flag);
	if (txss_readl(&tr->dmatxr) != DMATXR_START) {
		if (lp->sending == 1) {
			dma_map_single(&lp->pdev->dev, &lp->tx_desc[lp->txdesc_end],
				       sizeof(struct txss_eth_td), DMA_FROM_DEVICE);
			while (lp->tx_desc[lp->txdesc_end].td0 & TD0_TACT) {
				dma_map_single(&lp->pdev->dev, &lp->tx_desc[lp->txdesc_end],
					       sizeof(struct txss_eth_td), DMA_FROM_DEVICE);
				if (time_after(jiffies, timeout)) {
					printk(KERN_WARNING "%s: timeout\n", __func__);
					lp->lstats.rx_enable_timeouts++;
					break;
				}
			}
		}
		while ((*((volatile u32 *)0xb870442c) & 0x10) != 0x0) {
			if (time_after(jiffies, timeout)) {
				printk(KERN_WARNING "%s: timeout\n", __func__);
				lp->lstats.rx_enable_timeouts++;
				break;
			}
		}
	}
	txss_writel(DMAINTR_RDE, &tr->dmaintr);	/* write to clear RDE bit*/
	txss_writel(DMARXR_START, &tr->dmarxr);	/* Rx DMA restart */
	spin_unlock_irqrestore(&lp->lock, flag);
}
#endif

/* debug function */

static ssize_t txss_eth_dump_regs(struct class_device *cdev, char *buf)
{
	struct net_device *dev =
	    container_of(cdev, struct net_device, class_dev);
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;
	unsigned int len = 0;

	len +=
	    sprintf(buf + len, "emacdrr    : 0x%08x\n",
		    txss_readl(&tr->emacdrr));
	len +=
	    sprintf(buf + len, "emacdrtrgr : 0x%08x\n",
		    txss_readl(&tr->emacdrtrgr));
	len +=
	    sprintf(buf + len, "emacdrenr  : 0x%08x\n",
		    txss_readl(&tr->emacdrenr));
	len +=
	    sprintf(buf + len, "emacintr   : 0x%08x\n",
		    txss_readl(&tr->emacintr));
	len +=
	    sprintf(buf + len, "emacintenr : 0x%08x\n",
		    txss_readl(&tr->emacintenr));
	len +=
	    sprintf(buf + len, "dmacr      : 0x%08x\n",
		    txss_readl(&tr->dmacr));
	len +=
	    sprintf(buf + len, "dmatxr     : 0x%08x\n",
		    txss_readl(&tr->dmatxr));
	len +=
	    sprintf(buf + len, "dmarxr     : 0x%08x\n",
		    txss_readl(&tr->dmarxr));
	len +=
	    sprintf(buf + len, "dmaintr    : 0x%08x\n",
		    txss_readl(&tr->dmaintr));
	len +=
	    sprintf(buf + len, "dmaintenr  : 0x%08x\n",
		    txss_readl(&tr->dmaintenr));
	len +=
	    sprintf(buf + len, "errmaskr   : 0x%08x\n",
		    txss_readl(&tr->errmaskr));
	len +=
	    sprintf(buf + len, "rxmiscntr  : 0x%08x\n",
		    txss_readl(&tr->rxmiscntr));
	len +=
	    sprintf(buf + len, "txfifothr  : 0x%08x\n",
		    txss_readl(&tr->txfifothr));
	len +=
	    sprintf(buf + len, "fifosizer  : 0x%08x\n",
		    txss_readl(&tr->fifosizer));
	len +=
	    sprintf(buf + len, "dmarxmodr  : 0x%08x\n",
		    txss_readl(&tr->dmarxmodr));
	len +=
	    sprintf(buf + len, "rxpadr     : 0x%08x\n",
		    txss_readl(&tr->rxpadr));
	len +=
	    sprintf(buf + len, "rxfifothr  : 0x%08x\n",
		    txss_readl(&tr->rxfifothr));
	len +=
	    sprintf(buf + len, "maccr      : 0x%08x\n",
		    txss_readl(&tr->maccr));
	len +=
	    sprintf(buf + len, "lenlmtr    : 0x%08x\n",
		    txss_readl(&tr->lenlmtr));
	len +=
	    sprintf(buf + len, "macintr    : 0x%08x\n",
		    txss_readl(&tr->macintr));
	len +=
	    sprintf(buf + len, "macintenr  : 0x%08x\n",
		    txss_readl(&tr->macintenr));
	len +=
	    sprintf(buf + len, "phyintr    : 0x%08x\n",
		    txss_readl(&tr->phyintr));
	len +=
	    sprintf(buf + len, "apftpr     : 0x%08x\n",
		    txss_readl(&tr->apftpr));
	len +=
	    sprintf(buf + len, "mpftpr     : 0x%08x\n",
		    txss_readl(&tr->mpftpr));
	len +=
	    sprintf(buf + len, "pftxcntr   : 0x%08x\n",
		    txss_eth_update_pftxcntr(dev));
	len +=
	    sprintf(buf + len, "pfrxcntr   : 0x%08x\n",
		    txss_eth_update_pfrxcntr(dev));
	len +=
	    sprintf(buf + len, "pfrtlmtr   : 0x%08x\n",
		    txss_readl(&tr->pfrtlmtr));
	len +=
	    sprintf(buf + len, "pfrtcntr   : 0x%08x\n",
		    txss_readl(&tr->pfrtcntr));
	len +=
	    sprintf(buf + len, "macgcr     : 0x%08x\n",
		    txss_readl(&tr->macgcr));
	len +=
	    sprintf(buf + len, "bstlmtr    : 0x%08x\n",
		    txss_readl(&tr->bstlmtr));
	len +=
	    sprintf(buf + len, "umacar     : 0x%08x\n",
		    txss_readl(&tr->umacar));
	len +=
	    sprintf(buf + len, "lmacar     : 0x%08x\n",
		    txss_readl(&tr->lmacar));
	len +=
	    sprintf(buf + len, "tint1cntr  : 0x%08x\n",
		    txss_readl(&tr->tint1cntr));
	len +=
	    sprintf(buf + len, "tint2cntr  : 0x%08x\n",
		    txss_readl(&tr->tint2cntr));
	len +=
	    sprintf(buf + len, "tint3cntr  : 0x%08x\n",
		    txss_readl(&tr->tint3cntr));
	len +=
	    sprintf(buf + len, "rint1cntr  : 0x%08x\n",
		    txss_readl(&tr->rint1cntr));
	len +=
	    sprintf(buf + len, "rint2cntr  : 0x%08x\n",
		    txss_readl(&tr->rint2cntr));
	len +=
	    sprintf(buf + len, "rint3cntr  : 0x%08x\n",
		    txss_readl(&tr->rint3cntr));
	len +=
	    sprintf(buf + len, "rint4cntr  : 0x%08x\n",
		    txss_readl(&tr->rint4cntr));
	len +=
	    sprintf(buf + len, "rint5cntr  : 0x%08x\n",
		    txss_readl(&tr->rint5cntr));
	len +=
	    sprintf(buf + len, "rint6cntr  : 0x%08x\n",
		    txss_readl(&tr->rint6cntr));
	len +=
	    sprintf(buf + len, "rint7cntr  : 0x%08x\n",
		    txss_readl(&tr->rint7cntr));
	len +=
	    sprintf(buf + len, "rint8cntr  : 0x%08x\n",
		    txss_readl(&tr->rint8cntr));
	len +=
	    sprintf(buf + len, "mdiostsr   : 0x%08x\n",
		    txss_readl(&tr->mdiostsr));
	len +=
	    sprintf(buf + len, "mdiocmdr   : 0x%08x\n",
		    txss_readl(&tr->mdiocmdr));
	len +=
	    sprintf(buf + len, "mdioadrr   : 0x%08x\n",
		    txss_readl(&tr->mdioadrr));
	len +=
	    sprintf(buf + len, "mdiodatr   : 0x%08x\n",
		    txss_readl(&tr->mdiodatr));
	len +=
	    sprintf(buf + len, "mdiocycr   : 0x%08x\n",
		    txss_readl(&tr->mdiocycr));
	len +=
	    sprintf(buf + len, "desccr     : 0x%08x\n",
		    txss_readl(&tr->desccr));
	len +=
	    sprintf(buf + len, "dtxspar    : 0x%08x\n",
		    txss_readl(&tr->dtxspar));
	len +=
	    sprintf(buf + len, "dtxcpar    : 0x%08x\n",
		    txss_readl(&tr->dtxcpar));
	len +=
	    sprintf(buf + len, "dtxlpar    : 0x%08x\n",
		    txss_readl(&tr->dtxlpar));
	len +=
	    sprintf(buf + len, "dtxdlr     : 0x%08x\n",
		    txss_readl(&tr->dtxdlr));
	len +=
	    sprintf(buf + len, "drxspar    : 0x%08x\n",
		    txss_readl(&tr->drxspar));
	len +=
	    sprintf(buf + len, "drxcpar    : 0x%08x\n",
		    txss_readl(&tr->drxcpar));
	len +=
	    sprintf(buf + len, "drxlpar    : 0x%08x\n",
		    txss_readl(&tr->drxlpar));
	len +=
	    sprintf(buf + len, "drxdlr     : 0x%08x\n",
		    txss_readl(&tr->drxdlr));

	return len;
}

static CLASS_DEVICE_ATTR(hw_regs, S_IRUGO, txss_eth_dump_regs, NULL);

static unsigned short txss_no_phy_regs[2][32]={
{
	0x1140, /* 0x0 */
	0x7849, /* 0x1 */
	0x1234, /* 0x2 */
	0x5678, /* 0x3 */
	0x01e1, /* 0x4 */
	0x01e1, /* 0x5 */
	0x0000, /* 0x6 */
	0x0000, /* 0x7 */
	0x0000, /* 0x8 */
	0x0000, /* 0x9 */
	0x0000, /* 0xA */
	0x0000, /* 0xB */
	0x0000, /* 0xC */
	0x0000, /* 0xD */
	0x0000, /* 0xE */
	0x0000, /* 0xF */
	0x0000, /* 0x10 */
	0x0000, /* 0x11 */
	0x0000, /* 0x12 */
},
{
	0x1140, /* 0x0 */
	0x7949, /* 0x1 */
	0x1234, /* 0x2 */
	0x5678, /* 0x3 */
	0x01e1, /* 0x4 */
	0x01e1, /* 0x5 */
	0x0000, /* 0x6 */
	0x0000, /* 0x7 */
	0x0000, /* 0x8 */
	0x0f00, /* 0x9 */
	0x0c00, /* 0xA */
	0x0000, /* 0xB */
	0x0000, /* 0xC */
	0x0000, /* 0xD */
	0x0000, /* 0xE */
	0x3000, /* 0xF */
	0x0000, /* 0x10 */
	0x0100, /* 0x11 */
	0x0000, /* 0x12 */
}
};

static int txss_no_phy_read(int phy_id,int regnum)
{
	int tnum = 0;
	if (options.speed == 100) {
		tnum = 0;
	} else if (options.speed == 1000) {
		tnum = 1;
	} else {
		printk("not supported speed %d\n",options.speed);
		return -EIO;
	}
	if (phy_id == options.phyaddr) {
		return txss_no_phy_regs[tnum][regnum];
	}
	return -EIO;
}
static int txss_no_phy_write(int phy_id,int regnum,int val)
{
	int tnum = 0;
	if (options.speed == 100) {
		tnum = 0;
	} else if (options.speed == 1000) {
		tnum = 1;
	} else {
		printk("not supported speed %d\n",options.speed);
		return -EIO;
	}
	if (phy_id == options.phyaddr){
		switch (regnum){
		case MII_BMCR:
			if(val & 0x200){
				txss_no_phy_regs[tnum][MII_BMSR] |= BMSR_ANEGCOMPLETE|BMSR_LSTATUS;
			}
			if(val & BMCR_RESET){
				val &= ~BMCR_RESET;
			}
			break;
		}
		txss_no_phy_regs[tnum][regnum] = val;
		return 0;
	}
	return -EIO;
}

/* PHY register access function */

static int txss_eth_mii_read(struct mii_bus *bus, int phy_id, int regnum)
{
	struct net_device *dev = bus->priv;
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;
	unsigned long timeout;

	if (options.speed) {
		int val;
		if ((val=txss_no_phy_read(phy_id,regnum))>=0){
			return val;
		}
	}
	
	timeout = jiffies + MII_IO_TIMEOUT;
	while (txss_readl(&tr->mdiostsr) & MDIOSTSR_BSY) {
		if (time_after(jiffies, timeout)) {
			printk(KERN_WARNING "%s: timeout\n", __func__);
			return -EIO;
		}
		cpu_relax();
	}
	txss_writel(MDIOADRR_PHYADR(phy_id) | MDIOADRR_PHYREG(regnum),
		    &tr->mdioadrr);
	txss_writel(MDIOCMDR_READ, &tr->mdiocmdr);
	timeout = jiffies + MII_IO_TIMEOUT;
	while (txss_readl(&tr->mdiostsr) & MDIOSTSR_BSY) {
		if (time_after(jiffies, timeout)) {
			printk(KERN_WARNING "%s: timeout\n", __func__);
			return -EIO;
		}
		cpu_relax();
	}
	return txss_readl(&tr->mdiodatr) & 0xffff;
}

static int
txss_eth_mii_write(struct mii_bus *bus, int phy_id, int regnum, u16 val)
{
	struct net_device *dev = bus->priv;
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;
	unsigned long timeout;

	if (options.speed) {
		if (!txss_no_phy_write(phy_id,regnum,val)) {
			return 0;
		}
	}
	
	timeout = jiffies + MII_IO_TIMEOUT;
	while (txss_readl(&tr->mdiostsr) & MDIOSTSR_BSY) {
		if (time_after(jiffies, timeout)) {
			printk(KERN_WARNING "%s: timeout\n", __func__);
			return -EIO;
		}
		cpu_relax();
	}
	txss_writel(MDIOADRR_PHYADR(phy_id) | MDIOADRR_PHYREG(regnum),
		    &tr->mdioadrr);
	txss_writel(val, &tr->mdiodatr);
	txss_writel(MDIOCMDR_WRITE, &tr->mdiocmdr);
	timeout = jiffies + MII_IO_TIMEOUT;
	while (txss_readl(&tr->mdiostsr) & MDIOSTSR_BSY) {
		if (time_after(jiffies, timeout)) {
			printk(KERN_WARNING "%s: timeout\n", __func__);
			return -EIO;
		}
		cpu_relax();
	}
	return 0;
}

static void txss_eth_mii_link_change(struct net_device *dev)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	struct phy_device *phydev = lp->phydev;
	int status_change = 0;

	if (phy_start_at_probe && phy_wait_link_change)
		return;

	if (phydev->link && ((lp->duplex != phydev->duplex)
			     || (lp->speed != phydev->speed))) {
		struct txss_eth_regs __iomem *tr =
		    (struct txss_eth_regs __iomem *)dev->base_addr;
		u32 maccr;
		u32 macgcr;
		maccr = txss_readl(&tr->maccr);
		maccr &= ~(MACCR_RPE | MACCR_TPE);	/* Rx/Tx disable */
		txss_writel(maccr, &tr->maccr);

		/* setting duplex */
		if (phydev->duplex == DUPLEX_FULL)
			maccr |= MACCR_DPM;
		else
			maccr &= ~MACCR_DPM;
		txss_writel(maccr, &tr->maccr);
		/* setting speed */
		macgcr = txss_readl(&tr->macgcr);
		if (phydev->speed == SPEED_1000) {
			macgcr |= MACGCR_SPEED;
		} else {
			macgcr &= ~MACGCR_SPEED;
		}
		txss_writel(macgcr, &tr->macgcr);

		maccr |= MACCR_RPE | MACCR_TPE;	/* Rx/Tx enable */
		txss_writel(maccr, &tr->maccr);

		lp->duplex = phydev->duplex;
		lp->speed = phydev->speed;
		status_change = 1;
	}

	if (phydev->link != lp->link) {
		if (phydev->link) {
			if (dev->flags & IFF_PROMISC)
				txss_eth_set_multicast_list(dev);
			netif_schedule(dev);
		} else {
			lp->duplex = -1;
			lp->speed = 0;
		}
		lp->link = phydev->link;
		status_change = 1;
	}

	if (status_change && netif_msg_link(lp))
		phy_print_status(phydev);
}

static int txss_eth_mii_probe(struct net_device *dev)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	struct phy_device *phydev = NULL;
	int phy_addr;

	/* find the first phy */
	for (phy_addr = 0; phy_addr < PHY_MAX_ADDR; phy_addr++) {
		if (lp->mii_bus.phy_map[phy_addr]) {
			if (phydev) {
				printk(KERN_ERR "%s: multiple PHYs found\n",
				       dev->name);
				return -EINVAL;
			}
			phydev = lp->mii_bus.phy_map[phy_addr];
			break;
		}
	}

	if (!phydev) {
		printk(KERN_ERR "%s: no PHY found\n", dev->name);
		return -ENODEV;
	}

	/* attach the mac to the phy */
	if ((tc90416_ccfgptr->pcfg & 0x0010000000000000ULL)
	    && (options.speed==1000 || options.speed == 0)){
		phydev = phy_connect(dev, phydev->dev.bus_id,
				     &txss_eth_mii_link_change, 0, PHY_INTERFACE_MODE_GMII);
	} else {
		phydev = phy_connect(dev, phydev->dev.bus_id,
				     &txss_eth_mii_link_change, 0, PHY_INTERFACE_MODE_MII);
	}
	if (IS_ERR(phydev)) {
		printk(KERN_ERR "%s: Could not attach to PHY\n", dev->name);
		return PTR_ERR(phydev);
	}
	printk(KERN_INFO "%s: attached PHY driver [%s] "
	       "(mii_bus:phy_addr=%s, id=%x)\n",
	       dev->name, phydev->drv->name, phydev->dev.bus_id,
	       phydev->phy_id);

	/* mask with MAC supported features */
	phydev->supported &= PHY_GBIT_FEATURES;
	if (phydev->interface == PHY_INTERFACE_MODE_GMII) {
		phydev->advertising = phydev->supported & PHY_GBIT_FEATURES;
	} else {
		phydev->advertising = phydev->supported & PHY_BASIC_FEATURES;
	}
	phydev->advertising |= SUPPORTED_Pause | SUPPORTED_Asym_Pause;

	lp->phydev = phydev;
	lp->duplex = -1;
	lp->speed = 0;
	lp->link = 0;

	return 0;
}

static int txss_eth_mii_init(struct net_device *dev)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	int err, i;

	lp->mii_bus.name = MII_BUS_NAME;
	lp->mii_bus.read = txss_eth_mii_read;
	lp->mii_bus.write = txss_eth_mii_write;
	lp->mii_bus.id = lp->pdev->id;
	lp->mii_bus.priv = dev;
	lp->mii_bus.dev = &lp->pdev->dev;
	lp->mii_bus.irq = kmalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);
	if (!lp->mii_bus.irq) {
		err = -ENOMEM;
		goto err_out;
	}

	for (i = 0; i < PHY_MAX_ADDR; i++)
		lp->mii_bus.irq[i] = PHY_POLL;

	err = mdiobus_register(&lp->mii_bus);
	if (err) {
		printk(KERN_WARNING
		       "%s: could not register mdiobus. (errno=%d)\n",
		       DRV_DEVNAME, err);
		goto err_out_free_mdio_irq;
	}
	err = txss_eth_mii_probe(dev);
	if (err) {
		printk(KERN_WARNING "%s: mii_probe failure. (errno=%d)\n",
		       DRV_DEVNAME, err);
		goto err_out_unregister_bus;
	}
	return 0;

      err_out_unregister_bus:
	mdiobus_unregister(&lp->mii_bus);
      err_out_free_mdio_irq:
	kfree(lp->mii_bus.irq);
      err_out:
	return err;
}

/* allocate sk_buff function */

static struct sk_buff *alloc_rxbuf_skb(struct net_device *dev,
				       dma_addr_t * dma_handle)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	struct sk_buff *skb;

	skb = dev_alloc_skb(lp->max_frame_size);
	if (!skb)
		return NULL;
	skb->dev = dev;
	*dma_handle = dma_map_single(&lp->pdev->dev, skb->data, lp->max_frame_size,
				     DMA_FROM_DEVICE);
	if (dma_mapping_error(*dma_handle)) {
		dev_kfree_skb_any(skb);
		return NULL;
	}
	skb_reserve(skb, NET_IP_ALIGN);	/* make IP header 4byte aligned */
	return skb;
}

static void free_rxbuf_skb(struct net_device *dev, struct sk_buff *skb,
			   dma_addr_t dma_handle)
{
	struct txss_eth_local *lp = netdev_priv(dev);

	dma_unmap_single(&lp->pdev->dev, dma_handle, lp->max_frame_size,
			 DMA_FROM_DEVICE);
	dev_kfree_skb_any(skb);
}

#ifndef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
static int txss_eth_tx_full(struct net_device *dev)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	return ((lp->txdesc_start + 1) % lp->tx_desc_num == lp->txdesc_end);
}
#endif

/* ring queue function */

static int txss_eth_init_queue(struct net_device *dev)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;
	size_t desc_size;
	int i;
	unsigned long desc_addr, desc_dma_addr;

	/* allocate descriptor ring */
	desc_size =
	    sizeof(struct txss_eth_td) * lp->tx_desc_num
	    + sizeof(struct txss_eth_rd) * lp->rx_desc_num;
	lp->desc = dma_alloc_coherent(&lp->pdev->dev, desc_size, &lp->desc_dma, GFP_ATOMIC);	/* un-cache area */
	if (lp->desc == NULL)
		return -ENOMEM;

	lp->tx_skbs =
	    kmalloc(sizeof(struct txss_eth_skbs) * lp->tx_desc_num, GFP_KERNEL);
	if (lp->tx_skbs == NULL)
		return -ENOMEM;
	lp->rx_skbs =
	    kmalloc(sizeof(struct txss_eth_skbs) * lp->rx_desc_num, GFP_KERNEL);
	if (lp->rx_skbs == NULL)
		return -ENOMEM;

	/* allocate receive buffer */
	for (i = 0; i < lp->rx_desc_num; i++) {
		lp->rx_skbs[i].skb =
		    alloc_rxbuf_skb(dev, &lp->rx_skbs[i].skb_dma);

		if (!lp->rx_skbs[i].skb) {
			while (--i >= 0) {
				free_rxbuf_skb(dev,
					       lp->rx_skbs[i].skb,
					       lp->rx_skbs[i].skb_dma);
				lp->rx_skbs[i].skb = NULL;
			}
			dma_free_coherent(&lp->pdev->dev,
					  desc_size, lp->desc, lp->desc_dma);
			lp->desc = NULL;
			return -ENOMEM;
		}
	}

	desc_addr = (unsigned long)lp->desc;
	desc_dma_addr = (unsigned long)lp->desc_dma;

	/* set receive descriptor */
	lp->rx_desc = (struct txss_eth_rd *)desc_addr;
	lp->rx_desc_dma = (struct txss_eth_rd *)desc_dma_addr;
	desc_addr += sizeof(struct txss_eth_rd) * lp->rx_desc_num;
	desc_dma_addr += sizeof(struct txss_eth_rd) * lp->rx_desc_num;
	for (i = 0; i < lp->rx_desc_num; i++) {
		lp->rx_desc[i].rd2 = RD2_RBA_SET(lp->rx_skbs[i].skb_dma);
		lp->rx_desc[i].rd1 = RD1_RBL_SET(lp->max_frame_size);
		lp->rx_desc[i].rd0 = RD0_RACT;

	}
	lp->rx_desc[lp->rx_desc_num - 1].rd0 |= RD0_RDL;
	lp->rxdesc_end = 0;

	/* clear transmit skbs */
	for (i = 0; i < lp->tx_desc_num; i++) {
		lp->tx_skbs[i].skb = NULL;
		lp->tx_skbs[i].skb_dma = 0;
	}

	/* set transmit descriptor */
	lp->tx_desc = (struct txss_eth_td *)desc_addr;
	lp->tx_desc_dma = (struct txss_eth_td *)desc_dma_addr;
	for (i = 0; i < lp->tx_desc_num; i++) {
		lp->tx_desc[i].td2 = 0x0;
		lp->tx_desc[i].td1 = 0x0;
		lp->tx_desc[i].td0 = 0x0;
	}
	lp->tx_desc[lp->tx_desc_num - 1].td0 |= TD0_TDL;
	lp->txdesc_start = lp->txdesc_end = 0;

	/* set descriptor pointer */
	dma_map_single(&lp->pdev->dev, lp->desc, desc_size, DMA_TO_DEVICE);
	txss_writel((u32) & lp->rx_desc_dma[0], &tr->drxspar);
	txss_writel((u32) & lp->rx_desc_dma[0], &tr->drxcpar);
	txss_writel((u32) & lp->rx_desc_dma[lp->rx_desc_num - 1], &tr->drxlpar);
	txss_writel(DRXDLR_DRXDL, &tr->drxdlr);
	txss_writel((u32) & lp->tx_desc_dma[0], &tr->dtxspar);
	txss_writel((u32) & lp->tx_desc_dma[0], &tr->dtxcpar);
	txss_writel((u32) & lp->tx_desc_dma[lp->tx_desc_num - 1], &tr->dtxlpar);
	txss_writel(DTXDLR_DTXDL, &tr->dtxdlr);

	return 0;
}

static void txss_eth_clear_tx_queue(struct net_device *dev)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;
	struct sk_buff *skb;
	int i;

	for (i = 0; i < lp->tx_desc_num; i++) {
		skb = lp->tx_skbs[i].skb;
		if (skb) {
			dma_unmap_single(&lp->pdev->dev, lp->tx_skbs[i].skb_dma,
					 skb->len, DMA_TO_DEVICE);
			lp->tx_skbs[i].skb = NULL;
			lp->tx_skbs[i].skb_dma = 0;
			dev_kfree_skb_any(skb);
		}
		lp->tx_desc[i].td2 = 0x0;
		lp->tx_desc[i].td1 = 0x0;
		lp->tx_desc[i].td0 = 0x0;
		dma_map_single(&lp->pdev->dev, &lp->tx_desc[i],
			       sizeof(struct txss_eth_td), DMA_FROM_DEVICE);
	}
	lp->tx_desc[lp->tx_desc_num - 1].td0 |= TD0_TDL;
	dma_map_single(&lp->pdev->dev, &lp->tx_desc[lp->tx_desc_num - 1],
		       sizeof(struct txss_eth_td), DMA_FROM_DEVICE);

	txss_writel((u32) & lp->tx_desc_dma[0], &tr->dtxspar);
	txss_writel((u32) & lp->tx_desc_dma[0], &tr->dtxcpar);
	txss_writel((u32) & lp->tx_desc_dma[lp->tx_desc_num - 1], &tr->dtxlpar);
	txss_writel(DTXDLR_DTXDL, &tr->dtxdlr);
	mb();
	lp->txdesc_start = lp->txdesc_end = 0;
}

static int txss_eth_free_queue(struct net_device *dev)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	int i;
	size_t desc_size;

	desc_size = sizeof(struct txss_eth_td) * lp->tx_desc_num
	    + sizeof(struct txss_eth_rd) * lp->rx_desc_num;

	for (i = 0; i < lp->rx_desc_num; i++) {
		free_rxbuf_skb(dev, lp->rx_skbs[i].skb, lp->rx_skbs[i].skb_dma);
		lp->rx_skbs[i].skb = NULL;
	}
	dma_free_coherent(&lp->pdev->dev, desc_size, lp->desc, lp->desc_dma);
	kfree(lp->tx_skbs);
	kfree(lp->rx_skbs);
	return 0;
}

/* print frame data for debug */

static void print_eth(const u8 * add)
{
	int i;

	printk("print_eth(%p)\n", add);
	for (i = 0; i < 6; i++)
		printk(" %2.2X", (unsigned char)add[i + 6]);
	printk(" =>");
	for (i = 0; i < 6; i++)
		printk(" %2.2X", (unsigned char)add[i]);
	printk(" : %2.2X%2.2X\n", (unsigned char)add[12],
	       (unsigned char)add[13]);
}

/* net_device function */

static int txss_eth_open(struct net_device *dev)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	static int first_call = 1;

	if (request_irq
	    (dev->irq, &txss_eth_interrupt, IRQF_SHARED, dev->name, dev))
		return -EAGAIN;

	txss_eth_chip_reset(dev);

	if (txss_eth_init_queue(dev) != 0) {
		free_irq(dev->irq, dev);
		return -EAGAIN;
	}

	netif_poll_enable(dev);
	txss_eth_chip_init(dev);

	if (phy_start_at_probe && first_call) {
		spin_lock(&lp->phydev->lock);
		if (lp->phydev->state == PHY_RUNNING)
			lp->phydev->state = PHY_CHANGELINK;
		phy_wait_link_change = 0;
		spin_unlock(&lp->phydev->lock);
		first_call = 0;
	} else
		netif_carrier_off(dev);

	phy_start(lp->phydev);
	netif_start_queue(dev);

	return 0;
}

static int txss_eth_stop(struct net_device *dev)
{
	struct txss_eth_local *lp = netdev_priv(dev);

	netif_stop_queue(dev);
	if (lp->phydev)
		phy_stop(lp->phydev);
	free_irq(dev->irq, dev);	/* disable interrupt */
	txss_eth_chip_reset(dev);
	txss_eth_free_queue(dev);
	netif_poll_disable(dev);

	return 0;
}

static int txss_eth_send_packet(struct sk_buff *skb, struct net_device *dev)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	struct txss_eth_td *tx_desc;
	unsigned int i;

	i = lp->txdesc_start;
	tx_desc = &lp->tx_desc[i];

#ifndef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
	if (txss_eth_tx_full(dev) || ((i == lp->txdesc_end) && (tx_desc->td0 & TD0_TACT))) {
		printk(KERN_WARNING "call txss_eth_send_packet when Tx desc Exhausted\n");
		return NETDEV_TX_BUSY;
	}
#endif

	if (netif_msg_pktdata(lp))
		print_eth(skb->data);

	lp->tx_skbs[i].skb = skb;
	lp->tx_skbs[i].skb_dma =
	    dma_map_single(&lp->pdev->dev, skb->data, skb->len, DMA_TO_DEVICE);

	tx_desc->td0 &= TD0_TDL;	/* clear TD0 */
	tx_desc->td2 = lp->tx_skbs[i].skb_dma;
	tx_desc->td1 = TD1_TBL_SET(skb->len);
	barrier();
	tx_desc->td0 |= TD0_TACT | TD0_TFP_ALL | TD0_TWBI;

	DBG("lp->tx_desc[%3d].td0 = 0x%08x, td1 = 0x%08x, td2 = 0x%08x\n",
	    i, tx_desc->td0, tx_desc->td1, tx_desc->td2);

	dma_map_single(&lp->pdev->dev, tx_desc,
		       sizeof(struct txss_eth_td), DMA_TO_DEVICE);
	mb();

	dev->trans_start = jiffies;

#ifndef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
{
	struct txss_eth_regs __iomem *tr =
		(struct txss_eth_regs __iomem *)dev->base_addr;
	txss_writel(DMATXR_START, &tr->dmatxr);	/* Tx DMA start */
} 
#else
	txss_eth_tx_enable(dev);
#endif

	lp->txdesc_start = (lp->txdesc_start + 1) % lp->tx_desc_num;

#ifdef TXSS_ETH_MAX_QUEUE_LEN
	lp->lstats.tx_max_queue_len = max((int)
					  ((lp->txdesc_start + lp->tx_desc_num
					    - (lp->txdesc_end))
					   % lp->tx_desc_num),
					  lp->lstats.tx_max_queue_len);
#endif

#ifndef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
	if (txss_eth_tx_full(dev)) {
		lp->lstats.tx_full++;
		if (netif_msg_tx_queued(lp))
			printk(KERN_WARNING "%s: Tx desc Exhausted.\n",
			       dev->name);
		netif_stop_queue(dev);
	}
#else
	netif_stop_queue(dev);
#endif
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void txss_eth_restart(struct net_device *dev)
{
	struct txss_eth_local *lp = netdev_priv(dev);
#else /* 2.6.20 and later */
static void txss_eth_restart(struct work_struct *work)
{
	struct txss_eth_local *lp =
		container_of(work, struct txss_eth_local, restart_work);
	struct net_device *dev = lp->dev;
#endif

	netif_poll_disable(dev);

	if (lp->phydev) {
		int timeout;

		phy_write(lp->phydev, MII_BMCR, BMCR_RESET);
		timeout = 100;
		while (--timeout) {
			if (!(phy_read(lp->phydev, MII_BMCR) & BMCR_RESET))
				break;
			udelay(1);
		}
		if (!timeout)
			printk(KERN_ERR "%s: BMCR reset failed.\n", dev->name);
	}

	txss_eth_clear_tx_queue(dev);
	txss_eth_chip_reset(dev);
	netif_poll_enable(dev);
	txss_eth_chip_init(dev);
	netif_wake_queue(dev);
}

static void txss_eth_tx_timeout(struct net_device *dev)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;

	txss_writel(0x0, &tr->emacintenr);		/* disable interrupt */
	txss_readl(&tr->emacintenr);			/* dummy read */
	iob();
	schedule_work(&lp->restart_work);
	lp->lstats.tx_timeouts++;
}

static struct net_device_stats *txss_eth_get_stats(struct net_device *dev)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;

	lp->stats.multicast += txss_readl(&tr->rint8cntr);
	lp->stats.collisions += txss_readl(&tr->tint2cntr);
	lp->stats.rx_length_errors += txss_readl(&tr->rint4cntr);
	lp->stats.rx_length_errors += txss_readl(&tr->rint3cntr);
	lp->stats.rx_crc_errors += txss_readl(&tr->rint1cntr);
	lp->stats.rx_frame_errors += txss_readl(&tr->rint5cntr);
	lp->stats.rx_missed_errors += txss_readl(&tr->rint2cntr);
	lp->stats.rx_missed_errors += txss_readl(&tr->rxmiscntr);
	lp->stats.tx_carrier_errors += txss_readl(&tr->tint3cntr);
	lp->stats.tx_window_errors += txss_readl(&tr->tint1cntr);

	txss_writel(0x0000, &tr->rxmiscntr);	/* clear */
	return &lp->stats;
}

static void txss_eth_set_multicast_list(struct net_device *dev)
{
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;
	u32 reg;

	if (dev->flags & IFF_PROMISC) {
		reg = txss_readl(&tr->maccr);
		reg |= MACCR_PRM;
		txss_writel(reg, &tr->maccr);
	} else {
		reg = txss_readl(&tr->maccr);
		reg &= ~MACCR_PRM;
		txss_writel(reg, &tr->maccr);
	};
}

static int txss_eth_nophy_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	struct mii_ioctl_data *mii_data = if_mii(ifr);
	struct mii_bus *bus = lp->phydev->bus;
	int ret = 0;

	spin_lock_bh(&bus->mdio_lock);
	switch (cmd){
	case SIOCGMIIREG:
		mii_data->val_out = bus->read(&lp->mii_bus, mii_data->phy_id, mii_data->reg_num);
		break;
	case SIOCSMIIREG:
		bus->write(&lp->mii_bus, mii_data->phy_id, mii_data->reg_num,mii_data->val_in);
		break;
	default:
		ret = -1;
	}
	spin_unlock_bh(&bus->mdio_lock);
	return ret;
}

static int txss_eth_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct txss_eth_local *lp = netdev_priv(dev);

	DBG("%s\n", __func__);

	if (options.speed) {
		if (lp->phydev) {
			if(!txss_eth_nophy_ioctl(dev, ifr, cmd))
				return 0;
		}
	}

	if (!netif_running(dev))
		return -EINVAL;
	if (!lp->phydev)
		return -ENODEV;

	return phy_mii_ioctl(lp->phydev, if_mii(ifr), cmd);
}

static int txss_eth_do_interrupt(struct net_device *dev, u32 status, int limit)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;
	u32 mac_status = txss_readl(&tr->macintr);
	int ret = -1;

	if (unlikely(mac_status)) {
		txss_writel(mac_status, &tr->macintr);
		if (netif_msg_hw(lp) && (mac_status & MACINTR_FCI)) {
			printk(KERN_WARNING "%s: False Carrier Indication\n",
			       dev->name);
			lp->lstats.fci++;
		}
	}
	if (unlikely(status & DMAINTR_ERR)) {
		if (status & DMAINTR_RFRMER) {
			if (netif_msg_rx_err(lp))
				printk(KERN_WARNING
				       "%s: Rx frame overflow. (status 0x%08x)\n",
				       dev->name, status);
			lp->lstats.rx_frame_overflow++;
			lp->stats.rx_dropped++;
		}
		if (status & DMAINTR_TDE) {
			if (netif_msg_drv(lp))
				printk(KERN_WARNING
				       "%s: Tx desc exhausted. (status 0x%08x)\n",
				       dev->name, status);
			lp->lstats.tx_desc_exhausted++;
			lp->stats.tx_dropped++;
		}
		if (status & DMAINTR_TFE) {
			if (netif_msg_drv(lp))
				printk(KERN_WARNING
				       "%s: Tx FIFO underflow. (status 0x%08x)\n",
				       dev->name, status);
			lp->lstats.tx_fifo_underflow++;
			lp->stats.tx_fifo_errors++;
		}
		if (status & DMAINTR_RDE) {
			if (netif_msg_rx_err(lp))
				printk(KERN_WARNING
				       "%s: Rx desc exhausted. (status 0x%08x)\n",
				       dev->name, status);
			lp->lstats.rx_desc_exhausted++;
			lp->stats.rx_dropped++;
		}
		if (status & DMAINTR_RFE) {
			if (netif_msg_rx_err(lp))
				printk(KERN_WARNING
				       "%s: Rx FIFO overflow. (status 0x%08x)\n",
				       dev->name, status);
			lp->lstats.rx_fifo_overflow++;
			lp->stats.rx_fifo_errors++;
		}
		ret = 0;
	}
	if (status & (DMAINTR_FRC|DMAINTR_RDE) || txss_eth_rx_retry) {
		/* got a packet(s) */
		ret = txss_eth_rx(dev, limit);
		lp->lstats.rx_ints++;
	}
	if (status & (DMAINTR_FTC1 | DMAINTR_FTC0)) {
		/* transmit complete */
		txss_eth_tx_done(dev);
		lp->lstats.tx_ints++;
		ret = 0;
	}
	return ret;
}

static int txss_eth_poll(struct net_device *dev, int *budget)
{
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;

	int limit = min(*budget, dev->quota);
	int received = 0;
	int handled;
	u32 status;

#ifdef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
	status = txss_readl(&tr->dmaintr);
	status &= ~DMAINTR_FRC;
#else
	status = txss_readl(&tr->dmaintr);
#endif
	do {
#ifdef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
		u32 clear_status = status & ~DMAINTR_RDE;
		txss_writel(clear_status, &tr->dmaintr);	/* write to clear */		
		if (txss_eth_rx_frc) {
			status |= DMAINTR_FRC;
			txss_eth_rx_frc = 0;
		}
#else
		txss_writel(status, &tr->dmaintr);	/* write to clear */
#endif
		handled = txss_eth_do_interrupt(dev, status, limit);
		if (handled >= 0) {
			received += handled;
			limit -= handled;
			if (limit <= 0)
				break;
		}
		status = txss_readl(&tr->dmaintr);
#ifdef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
		status &= ~DMAINTR_FRC;
	} while (status || txss_eth_rx_frc);
#else
	} while (status);
#endif

	dev->quota -= received;
	*budget -= received;
	if (limit <= 0)
		return 1;

	netif_rx_complete(dev);

#ifdef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
	if (txss_eth_rx_frc) {
		struct txss_eth_local *lp = netdev_priv(dev);
		lp->lstats.frc_rx_rescheduled++;
		netif_rx_reschedule(dev, 0);
	}

	/* enable interrupt */
	txss_writel(DMAINTR_FTC1 | DMAINTR_FTC0 | DMAINTR_RFRMER | DMAINTR_TDE
		    | DMAINTR_TFE | DMAINTR_FRC | DMAINTR_RDE | DMAINTR_RFE,
		    &tr->dmaintenr);
#else
	/* enable interrupt */
	txss_writel(EMACINTR_DMAINT, &tr->emacintenr);
#endif
	return 0;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void txss_eth_poll_controller(struct net_device *dev)
{
	disable_irq(dev->irq);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	txss_eth_interrupt(dev->irq, dev, NULL);
#else /* 2.6.19 and later */
	txss_eth_interrupt(dev->irq, dev);
#endif
	enable_irq(dev->irq);
}
#endif

/* interrupt handling */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t txss_eth_interrupt(int irq, void *dev_id, struct pt_regs *pt);
#else /* 2.6.19 and later */
static irqreturn_t txss_eth_interrupt(int irq, void *dev_id)
#endif
{
	struct net_device *dev = dev_id;
	struct txss_eth_local *lp = netdev_priv(dev);
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;
	u32 status;

	if (netif_msg_intr(lp))
		printk("%s: dmaintr:0x%08x macintr:0x%08x emacintr:0x%08x\n",
		       __func__, txss_readl(&tr->dmaintr),
		       txss_readl(&tr->macintr), txss_readl(&tr->emacintr));

	status = txss_readl(&tr->emacintr);
	if (status) {
		/* disable interrupt */
#ifndef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
		txss_writel(0, &tr->emacintenr);
#else
		txss_writel(DMAINTR_FRC, &tr->dmaintenr);
		{
			u32 dma_status = txss_readl(&tr->dmaintr);
			if (dma_status & DMAINTR_FRC) {
				txss_writel(DMAINTR_FRC, &tr->dmaintr);	/* write to clear */
				txss_eth_rx_enable(dev);
				txss_eth_rx_frc = 1;
			}
		}
#endif
		netif_rx_schedule(dev);
		txss_readl(&tr->emacintenr);	/* dummy read */
		iob();
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

static int txss_eth_rx(struct net_device *dev, int limit)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;
	struct txss_eth_rd *rx_desc;
	unsigned int i;
#ifdef TXSS_ETH_MAX_QUEUE_LEN
	u32 spar, lpar;
#endif
	int received = 0;

	struct sk_buff *skb_pre;
	dma_addr_t dma_handle_pre;
		
	i = lp->rxdesc_end;

	dma_map_single(&lp->pdev->dev, &lp->rx_desc[i],
		       sizeof(struct txss_eth_rd), DMA_FROM_DEVICE);

#ifdef TXSS_ETH_MAX_QUEUE_LEN
	spar = txss_readl(&tr->drxspar);
	lpar = txss_readl(&tr->drxlpar);
	rx_desc = &lp->rx_desc[i];
	if (!(lp->rx_desc[i].rd0 & RD0_RACT))	/* check write back */
		lp->lstats.rx_max_queue_len =
		    max((int)((((lpar - spar) / sizeof(struct txss_eth_rd))
			       + lp->rx_desc_num - lp->rxdesc_end)
			      % lp->rx_desc_num), lp->lstats.rx_max_queue_len);
#endif
	rx_desc = &lp->rx_desc[i];
	while (!(rx_desc->rd0 & RD0_RACT)) {
		int rx_good = (rx_desc->rd0 & RD0_RFE) ? 0 : 1;
		int rx_status = rx_desc->rd0 & RD0_RFS_MASK;
		struct sk_buff *skb;
		unsigned char *data;
		int pkt_len;

		if (netif_msg_rx_status(lp))
			printk(KERN_INFO
			       "%s: receive frame #%02d (status 0x%08x).\n",
			       dev->name, i, rx_status);

		DBG("lp->rx_desc[%3d].rd0 = 0x%08x, rd1 = 0x%08x, rd2 = 0x%08x\n", i, rx_desc->rd0, rx_desc->rd1, rx_desc->rd2);

		if (rx_good) {
			if (--limit < 0)
				break;

			skb_pre = alloc_rxbuf_skb(dev, &dma_handle_pre);

			if (!skb_pre) {
				txss_eth_rx_retry = 1;
				lp->lstats.alloc_rxbuf_skb_failed++;
				break;
			} else {
				txss_eth_rx_retry = 0;
			}
			
			skb = lp->rx_skbs[i].skb;

			dma_map_single(&lp->pdev->dev, skb->data, lp->max_frame_size,
				       DMA_FROM_DEVICE);
			pkt_len = RD1_RFL_GET(lp->rx_desc[i].rd1) - CSUM_LEN;
			data = skb_put(skb, pkt_len);

			if (netif_msg_pktdata(lp))
				print_eth(data);

#ifdef TXSS_ETH_USE_CHECKSUM_HW
			skb->csum = *(u16 *) ((u32) skb->data + pkt_len);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
			skb->ip_summed = CHECKSUM_HW;
#else /* 2.6.19 and later */
			skb->ip_summed = CHECKSUM_COMPLETE;
#endif
#endif
			skb->protocol = eth_type_trans(skb, dev);
			netif_receive_skb(skb);
			received++;

			dev->last_rx = jiffies;
			lp->stats.rx_packets++;
			lp->stats.rx_bytes += pkt_len;

			lp->rx_skbs[i].skb = skb_pre;
			lp->rx_skbs[i].skb_dma = dma_handle_pre;
			rx_desc->rd2 = RD2_RBA_SET(lp->rx_skbs[i].skb_dma);
		} else {
			lp->stats.rx_errors++;
			if (netif_msg_rx_err(lp))
				printk(KERN_DEBUG
				       "%s: Rx error (status 0x%04x)\n",
				       dev->name, rx_status);
			if (rx_status & RD0_RFS_ROC)	/* FIFO overflow frame */
				;	/* nothing to do */
			if (rx_status & RD0_RFS_RABT)	/* abort receive */
				lp->stats.rx_missed_errors++;
		}

		rx_desc->rd0 &= RD0_RDL;	/* clear RD0 */
		barrier();
		rx_desc->rd0 |= RD0_RACT;

		dma_map_single(&lp->pdev->dev, rx_desc,
			       sizeof(struct txss_eth_td), DMA_TO_DEVICE);

		lp->rxdesc_end = (lp->rxdesc_end + 1) % lp->rx_desc_num;
		i = lp->rxdesc_end;

		dma_map_single(&lp->pdev->dev, &lp->rx_desc[i],
			       sizeof(struct txss_eth_rd), DMA_FROM_DEVICE);
		rx_desc = &lp->rx_desc[i];
	}
#ifndef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
	txss_writel(DMARXR_START, &tr->dmarxr);	/* Rx DMA restart */
#else
	txss_eth_rx_enable(dev);
#endif
	return received;
}

static int txss_eth_tx_done(struct net_device *dev)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	struct txss_eth_td *tx_desc;
	unsigned int i;
#ifdef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
	unsigned int wb = 0;
#endif

	i = lp->txdesc_end;

	dma_map_single(&lp->pdev->dev, &lp->tx_desc[i],
		       sizeof(struct txss_eth_td), DMA_FROM_DEVICE);
	tx_desc = &lp->tx_desc[i];
	while ((lp->txdesc_start != lp->txdesc_end)
	       && (!(tx_desc->td0 & TD0_TACT))) {
		int tx_good = (tx_desc->td0 & TD0_TFE) ? 0 : 1;
		int tx_status = tx_desc->td0 & TD0_TFS_MASK;
		struct sk_buff *skb = lp->tx_skbs[i].skb;

#ifdef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
		wb = 1;
#endif
		if (netif_msg_tx_done(lp))
			printk(KERN_INFO "%s: complete Tx desc #%02d.\n",
			       dev->name, i);

		if (tx_good)
			lp->stats.tx_packets++;
		else {
			lp->stats.tx_errors++;
			if (netif_msg_tx_err(lp)) {
				printk(KERN_WARNING
				       "%s: Tx error (status 0x%04x)\n",
				       dev->name, tx_status);
				printk(KERN_WARNING
				       "lp->tx_desc[%d].td0=0x%08x, td1=0x%08x, td2=0x%08x\n",
				       i, tx_desc->td0, tx_desc->td1,
				       tx_desc->td2);
			}
			if (tx_status & TD0_TFS_TABT) {
				lp->stats.tx_aborted_errors++;
			}
		}

		if (skb) {
			lp->stats.tx_bytes += skb->len;
			dma_unmap_single(&lp->pdev->dev, lp->tx_skbs[i].skb_dma,
					 skb->len, DMA_TO_DEVICE);
			lp->tx_skbs[i].skb = NULL;
			lp->tx_skbs[i].skb_dma = 0;
			dev_kfree_skb_any(skb);
		}
#ifdef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
 {
		unsigned int flag;
		spin_lock_irqsave(&lp->lock, flag);
		lp->sending = 0;
#endif
		lp->txdesc_end = (lp->txdesc_end + 1) % lp->tx_desc_num;
		i = lp->txdesc_end;
#ifdef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
		spin_unlock_irqrestore(&lp->lock, flag);
 }
#endif
		dma_map_single(&lp->pdev->dev, &lp->tx_desc[i],
			       sizeof(struct txss_eth_td), DMA_FROM_DEVICE);
		tx_desc = &lp->tx_desc[i];
	}

#ifndef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
	if (netif_queue_stopped(dev) && !txss_eth_tx_full(dev))
		netif_wake_queue(dev);
#else
	if (wb == 1)
		netif_wake_queue(dev);
	else
		txss_eth_tx_done(dev);
#endif

	return 0;
}

/* initialize function */

static void txss_eth_desc_start(struct net_device *dev)
{
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;

	txss_writel(DESCCR_ENR | DESCCR_ENT, &tr->desccr);
}

#define txss_eth_wait(x)	(((x) < 1000) ? udelay(x) : schedule_timeout((x)*HZ/1000000)+1)

static void txss_eth_rx_stop(struct net_device *dev)
{
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;
	struct txss_eth_local *lp = netdev_priv(dev);
	u32 maccr;
	int xtime, mult;
	int timeout;

	mult = (lp->speed == SPEED_1000 ? 1 :
		lp->speed == SPEED_100 ? 10 : 100);

	/* MAC: Rx stop */
	maccr = txss_readl(&tr->maccr);
	maccr &= ~MACCR_RPE;
	txss_writel(maccr, &tr->maccr);

	/* Wait for 1 frame receiving */
	xtime = (((lp->max_frame_size + 12 + 8) * 8 / 1000) + 1) * mult;
	txss_eth_wait(xtime);

	/* DMA: Rx stop */
	timeout = 1000000; /* 1sec */
	while (--timeout) {
		if (RXFIFOSTSR_FAM(txss_readl(&tr->rxfifostsr)) == 0)
			break;
		udelay(1);
	}
	if (!timeout)
		printk(KERN_ERR "%s: Timeout in DMA Rx stop (rxfifostsr).\n",
		       dev->name);
}

static void txss_eth_tx_stop(struct net_device *dev)
{
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;
	struct txss_eth_local *lp = netdev_priv(dev);
	u32 maccr;
	int xtime, ytime, mult;
	int timeout;

	mult = (lp->speed == SPEED_1000 ? 1 :
		lp->speed == SPEED_100 ? 10 : 100);

	/* DMA: Tx stop */
	txss_writel(DMATXR_STOP, &tr->dmatxr);	/* Tx DMA start */
	timeout = 1000000; /* 1sec */
	while (--timeout) {
		if (txss_readl(&tr->dmatxr) == DMATXR_STOP)
			break;
		udelay(1);
	}
	if (!timeout)
		printk(KERN_ERR "%s: Timeout in DMA Tx stop.\n",
		       dev->name);
	timeout = 10000000; /* 10sec */
	while (--timeout) {
		if (TXFIFOSTSR_BUSY(txss_readl(&tr->txfifostsr)) == 0)
			break;
		udelay(1);
	}
	if (!timeout)
		printk(KERN_ERR "%s: Timeout in DMA Tx stop (fifostsr).\n",
		       dev->name);

	/* Wait for 1 frame sending */
	xtime = (((lp->max_frame_size + 12 + 8) * 8 / 1000) + 1) * mult;
	ytime = ((lp->speed != 0) && (lp->duplex == DUPLEX_HALF)) ?
		(((1+3+7+15+31+63+127+255+511+1023*6)*512 / 1000) + 1) * mult : 0;
	txss_eth_wait(xtime+ytime);

	/* MAC: Tx stop */
	maccr = txss_readl(&tr->maccr);
	maccr &= ~MACCR_TPE;
	txss_writel(maccr, &tr->maccr);
}

static void txss_eth_soft_reset(struct net_device *dev)
{
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;
	int timeout;

	/* Rx/Tx stop */
	txss_eth_rx_stop(dev);
	txss_eth_tx_stop(dev);

	/* disable interrupt */
	txss_writel(0x0, &tr->dmaintenr);
	txss_writel(0x0, &tr->macintenr);

	/* soft reset */
	txss_writel(DMACR_SWRR | DMACR_SWRT, &tr->dmacr);
	timeout = 1000;
	while (--timeout) {
		if (!(txss_readl(&tr->dmacr) & (DMACR_SWRR | DMACR_SWRT)))
			break;
		udelay(1);
	}
	if (!timeout)
		printk(KERN_ERR "%s: Controller soft reset failed.\n",
		       dev->name);
}

static void txss_eth_phy_init(struct net_device *dev)
{
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;

	txss_writel(PHYINTR_PHYIP_LOW, &tr->phyintr);
	txss_writel(MDIOCYCR_SET, &tr->mdiocycr);
}

static void txss_eth_chip_reset(struct net_device *dev)
{
	txss_eth_desc_start(dev);
	txss_eth_soft_reset(dev);
	txss_eth_phy_init(dev);
}

static void txss_eth_chip_init(struct net_device *dev)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;
	u32 reg;
	u32 uaddr, laddr;

	txss_writel(DMACR_DL_16, &tr->dmacr);
	txss_writel(0x0, &tr->errmaskr);
	txss_writel(FIFOSIZER_DFL, &tr->fifosizer);
	txss_writel(TXFIFOTHR_SF, &tr->txfifothr);
#ifndef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
	txss_writel(DMARXMODR_RCVMOD, &tr->dmarxmodr);
#else
	txss_writel(DMARXMODR_DBGMOD, &tr->dmarxmodr);
#endif
	txss_writel(RXPADR_PADS(NET_IP_ALIGN) | RXPADR_PADP(0), &tr->rxpadr);	/* for IP 4byte aligned */
	txss_writel(RXFIFOTHR_RFF(lp->pause_rff) | RXFIFOTHR_RFD(lp->pause_rfd), &tr->rxfifothr);
	txss_writel(DMAINTR_RINT8, &tr->errmaskr);
	txss_writel(DMAINTR_ALL, &tr->dmaintr);
	txss_writel(DMAINTR_FTC1 | DMAINTR_FTC0 | DMAINTR_RFRMER | DMAINTR_TDE
		    | DMAINTR_TFE | DMAINTR_FRC | DMAINTR_RDE | DMAINTR_RFE,
		    &tr->dmaintenr);

	uaddr = (dev->dev_addr[0] << 24) | (dev->dev_addr[1] << 16)
	    | (dev->dev_addr[2] << 8) | (dev->dev_addr[3] << 0);
	laddr = (dev->dev_addr[4] << 8) | (dev->dev_addr[5] << 0);
	txss_writel(uaddr, &tr->umacar);
	txss_writel(laddr, &tr->lmacar);
	txss_writel(BSTLMTR_SET, &tr->bstlmtr);
	txss_writel(LENLMTR_LENLMT(dev->mtu + ETH_HLEN + 4), &tr->lenlmtr);
	txss_writel(lp->pause_time, &tr->apftpr);
	txss_writel(0x0, &tr->pfrtlmtr);
	reg = MACCR_TRCCM | MACCR_RZPF | MACCR_TZPF;
	if (lp->rx_pause)
		reg |= MACCR_RXF;
	if (lp->tx_pause)
		reg |= MACCR_TXF;
#ifdef TXSS_ETH_USE_CHECKSUM_HW
	reg |= MACCR_RCSC;
#endif
	txss_writel(reg, &tr->maccr);
	txss_writel(MACINTR_ALL, &tr->macintr);
	txss_writel(MACINTR_FCI, &tr->macintenr);

	txss_writel(0x18470000, &tr->emacdrr);	/* default */
	txss_writel(EMAC_DRDTRG((u32) & tr->dmaintr), &tr->emacdrtrgr);
#ifndef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
	txss_writel(EMACDRENR_DRDEN, &tr->emacdrenr);
#else
	txss_writel(0x0, &tr->emacdrenr); /* dummy read disable */
#endif
	txss_writel(EMACINTR_DMAINT, &tr->emacintenr);	/* disable EMACINTR_SERR */
	txss_writel(EMACINTR_SERR, &tr->emacintr);

#ifdef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
	txss_eth_rx_enable(dev);
#else
	txss_writel(DMARXR_START, &tr->dmarxr);	/* Rx DMA start */
#endif

	reg = txss_readl(&tr->maccr);
	txss_writel(reg | MACCR_RPE | MACCR_TPE, &tr->maccr);	/* enable mac function */
}

#ifdef CONFIG_TOSHIBA_TC90417REF
static int __devinit txss_eth_init_devaddr(struct net_device *dev)
{
	struct ethtool_eeprom eeprom;
	int i, csum = 0;
	u8 data[ETH_ALEN + 1];	/* 0x78: ether address, 0x7e: checksum */

	eeprom.offset = EEPROM_DEVADDR_OFFSET;
	eeprom.len = ETH_ALEN + 1;
	if (dev->ethtool_ops->get_eeprom(dev, &eeprom, data))
		return -ENODEV;

	for (i = 0; i < ETH_ALEN + 1; i++) {
		csum += data[i];
	}

	csum = ((csum >> 8) + csum) & 0xff;

	if ((csum != 0x00) && (csum != 0xff))
		return -ENODEV;

	memcpy(dev->dev_addr, data, ETH_ALEN);

	return is_valid_ether_addr(dev->dev_addr) ? 0 : -ENODEV;
}
#else
#define txss_eth_init_devaddr(x)   (-ENODEV)
#endif

static int txss_change_mtu(struct net_device *dev, int new_mtu)
{
	struct txss_eth_local *lp = netdev_priv(dev);
	int max_frame = new_mtu + ETH_HLEN + 4;
	struct txss_eth_regs __iomem *tr =
	    (struct txss_eth_regs __iomem *)dev->base_addr;

	if ((max_frame < TXSS_MIN_MTU) ||
	    (max_frame > lp->max_frame_size)) {
		printk(KERN_WARNING "%s: Invalid MTU setting\n",DRV_DEVNAME);
		return -EINVAL;
	}
	if (max_frame<RX_BUF_SIZE)
		max_frame = RX_BUF_SIZE;

	txss_writel(LENLMTR_LENLMT(max_frame), &tr->lenlmtr);
	dev->mtu = new_mtu;

	return 0;
}

static int txss_eth_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	struct txss_eth_local *lp;
	struct resource *res;
	int ret;

	printk(KERN_INFO "%s\n", print_version);
#ifdef CONFIG_TOSHIBA_TC90416
#ifdef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
	if (TC90416_REV_MAJ_MIN() >= 0x50) {
		printk(KERN_WARNING "%s: TXSSETH_ETHERMAC_WORKAROUND is NOT required on this chip (rev %x) but defined!\n", DRV_DEVNAME, TC90416_REV_MAJ_MIN());
	}
#else
	if (TC90416_REV_MAJ_MIN() < 0x50) {
		printk(KERN_WARNING "%s: TXSSETH_ETHERMAC_WORKAROUND is required on this chip (rev %x) but not defined!\n", DRV_DEVNAME, TC90416_REV_MAJ_MIN());
	}
#endif
#endif /* CONFIG_TOSHIBA_TC90416 */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENODEV;
		goto err_out;
	}

	if (!request_mem_region
	    (res->start, res->end - res->start + 1, DRV_DEVNAME)) {
		ret = -EBUSY;
		goto err_out;
	}

	dev = alloc_etherdev(sizeof(struct txss_eth_local));
	if (!dev) {
		printk(KERN_WARNING "%s: could not allocate net_device.\n",
		       DRV_DEVNAME);
		ret = -ENODEV;
		goto err_out_free_region;
	}

	SET_MODULE_OWNER(dev);
	SET_NETDEV_DEV(dev, &pdev->dev);
	platform_set_drvdata(pdev, dev);
	lp = netdev_priv(dev);
	lp->dev = dev;
	lp->pdev = pdev;

	if (options.speed && options.speed != 100 && options.speed != 1000)
		options.speed = 100;
	if (options.phyaddr && options.phyaddr >= 32)
		options.phyaddr = 0;

	lp->tx_desc_num = TX_DESC_NUM;
	lp->rx_desc_num = RX_DESC_NUM;
	if (options.tx_desc_num)
		lp->tx_desc_num = (unsigned int)options.tx_desc_num;
	if (options.rx_desc_num)
		lp->rx_desc_num = (unsigned int)options.rx_desc_num;

	dev->open = txss_eth_open;
	dev->stop = txss_eth_stop;
	dev->hard_start_xmit = txss_eth_send_packet;
	dev->get_stats = txss_eth_get_stats;
	dev->set_multicast_list = txss_eth_set_multicast_list;
	dev->do_ioctl = txss_eth_ioctl;
	dev->poll = txss_eth_poll;
	dev->weight = NAPI_WEIGHT;
#ifdef CONFIG_NET_POLL_CONTROLLER
	dev->poll_controller = txss_eth_poll_controller;
#endif
	dev->tx_timeout = txss_eth_tx_timeout;
	dev->watchdog_timeo = msecs_to_jiffies(400);
	dev->change_mtu = txss_change_mtu;
	lp->max_frame_size = RX_BUF_SIZE;
	if (options.rx_buf_size){
		int max_frame;
		max_frame = options.rx_buf_size + ETH_HLEN + 4;
		if (max_frame > RX_BUF_SIZE && max_frame <= RX_BUF_MAX_SIZE)
			lp->max_frame_size = max_frame;
		else if (max_frame > RX_BUF_MAX_SIZE)
			lp->max_frame_size = RX_BUF_MAX_SIZE;
	}
	lp->pause_rff = RXFIFOTHR_RFF_DFL;
	lp->pause_rfd = RXFIFOTHR_RFD_DFL;
	if (options.pause_time){
		lp->pause_time = options.pause_time;
	}else{
		lp->pause_time = PAUSE_TIME_DFL;
	}
	lp->rx_pause = 1;
	lp->tx_pause = 0;
#ifdef CONFIG_TXSSETH_ETHERMAC_WORKAROUND
	lp->sending = 0;
	spin_lock_init(&lp->lock);
#endif

	dev->irq = platform_get_irq(pdev, 0);
	dev->base_addr = (unsigned long)
	    ioremap(res->start, res->end - res->start + 1);
	if (!dev->base_addr) {
		printk(KERN_WARNING "%s: could not allocate dev->base_addr.\n",
		       DRV_DEVNAME);
		ret = -ENOMEM;
		goto err_out_free_region_netdev;
	}
	lp->mii_bus.phy_mask = options.phymask;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
	INIT_WORK(&lp->restart_work, (void (*)(void *))txss_eth_restart, dev);
#else /* 2.6.20 and later */
	INIT_WORK(&lp->restart_work, txss_eth_restart);
#endif
	txss_eth_set_ethtool_ops(dev);

	txss_eth_chip_reset(dev);

	ret = register_netdev(dev);
	if (ret) {
		printk(KERN_WARNING "%s: could not register netdev.\n",
		       DRV_DEVNAME);
		goto err_out_free_region_netdev;
	}

	ret = txss_eth_mii_init(dev);
	if (ret) {
		printk(KERN_WARNING "%s: mii_init failure.\n", DRV_DEVNAME);
		goto err_out_unregister_netdev;
	}

	/* retrieve the ethernet address */
	memset(dev->dev_addr,0,sizeof(dev->dev_addr));
#if !defined(MODULE) && defined(CONFIG_TXSSETH_PMON_ETHERADDR)
 {
	int i;
	extern char *pmon_getenv(const char *name) __init;
	char *tmpstr = pmon_getenv("etheraddr");
	if (tmpstr) {
		for (i = 0; i < 6; i++) {
			dev->dev_addr[i] = simple_strtoul(tmpstr, &tmpstr, 16);
			if (*tmpstr == ':')
				tmpstr++;
		}
	}
 }
#endif
	if (!is_valid_ether_addr(dev->dev_addr)) {
		if (txss_eth_init_devaddr(dev)) {
			dev_warn(&pdev->dev, "not valid ether address\n");
			random_ether_addr(dev->dev_addr);
		}
	}

	memcpy(dev->perm_addr, dev->dev_addr, dev->addr_len);
	printk(KERN_INFO "%s: %s at 0x%lx, "
	       "%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x, "
	       "IRQ %d\n",
	       dev->name,
	       DRV_DEVNAME,
	       dev->base_addr,
	       dev->dev_addr[0], dev->dev_addr[1],
	       dev->dev_addr[2], dev->dev_addr[3],
	       dev->dev_addr[4], dev->dev_addr[5], dev->irq);

	lp->msg_enable = TXSS_MSG_FLAG;

	if ((ret = class_device_create_file(&dev->class_dev, &class_device_attr_hw_regs)) < 0){
		printk(KERN_WARNING "%s: class_device_create_file return fail, ret = %d\n", DRV_DEVNAME, ret);
		goto err_out_unregister_netdev;
	}

	if (phy_start_at_probe) {
		phy_wait_link_change = 1;
		phy_start(lp->phydev);
	}

	return 0;

      err_out_unregister_netdev:
	unregister_netdev(dev);
      err_out_free_region_netdev:
	free_netdev(dev);
      err_out_free_region:
	release_mem_region(res->start, res->end - res->start + 1);
      err_out:
	printk("%s: not found (%d).\n", DRV_DEVNAME, ret);
	return ret;
}

static int txss_eth_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct txss_eth_local *lp = netdev_priv(dev);
	struct resource *res;

	phy_disconnect(lp->phydev);
	unregister_netdev(dev);

	iounmap((void *)dev->base_addr);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res->end - res->start + 1);
	platform_set_drvdata(pdev, NULL);

	mdiobus_unregister(&lp->mii_bus);
	kfree(lp->mii_bus.irq);

	free_netdev(dev);
	return 0;
}

#ifdef CONFIG_PM
static int txss_eth_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct txss_eth_local *lp = netdev_priv(dev);

	if (!netif_running(dev)){
		goto remove_phy;
	}

	netif_device_detach(dev);

	txss_eth_stop(dev);
	
 remove_phy:
	phy_disconnect(lp->phydev);

	return 0;
}

static int txss_eth_resume(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);

	txss_eth_chip_reset(dev);

	if (txss_eth_mii_probe(dev))
		return -EIO;

	if (!netif_running(dev)){
		txss_eth_chip_reset(dev);
		return 0;
	}

	if(txss_eth_open(dev)<0){
		printk(KERN_ERR "txss_eth %s: open failed\n", dev->name);
		return -EIO;
	}

	netif_device_attach(dev);

	return 0;
}

#endif /* CONFIG_PM */

static struct platform_driver txss_eth_driver = {
	.probe = txss_eth_probe,
	.remove = txss_eth_remove,
#ifdef CONFIG_PM
	.suspend = txss_eth_suspend,
	.resume = txss_eth_resume,
#endif /* CONFIG_PM */
	.driver = {
		   .name = DRV_DEVNAME,
		   }
};

#ifndef CONFIG_TXSS_ETH_MODULE
static int __init txss_eth_txdesc_set(char *str)
{
	get_option(&str, &options.tx_desc_num);
	return 1;
}

static int __init txss_eth_rxdesc_set(char *str)
{
	get_option(&str, &options.rx_desc_num);
	return 1;
}

static int __init txss_eth_rxbufsize_set(char *str)
{
	get_option(&str, &options.rx_buf_size);
	return 1;
}

__setup("tx_desc=", txss_eth_txdesc_set);
__setup("rx_desc=", txss_eth_rxdesc_set);
__setup("rx_buf_size=", txss_eth_rxbufsize_set);
#endif

module_param_named(tx_desc, options.tx_desc_num, int, 0);
MODULE_PARM_DESC(tx_desc, "transfer descriptor ring size");
module_param_named(rx_desc, options.rx_desc_num, int, 0);
MODULE_PARM_DESC(rx_desc, "receive descriptor ring size");
module_param_named(rx_buf_size, options.rx_buf_size, int, 0);
MODULE_PARM_DESC(rx_buf_size, "receive buffer size");
module_param_named(pause_time, options.pause_time, int, 0);
MODULE_PARM_DESC(pause_time, "pause time for flow control");
module_param_named(phyaddr, options.phyaddr, int, 0);
MODULE_PARM_DESC(phyaddr,"PHY address setting");
module_param_named(speed, options.speed, int, 0);
MODULE_PARM_DESC(speed,"SPEED setting");
module_param_named(phymask, options.phymask, int, 0);
MODULE_PARM_DESC(phymask,"PHY address mask");
module_param_named(phystart, phy_start_at_probe, int, 0);
MODULE_PARM_DESC(phystart,"PHY start at probe");

static int __init txss_eth_init(void)
{
	return platform_driver_register(&txss_eth_driver);
}

static void __exit txss_eth_exit(void)
{
	platform_driver_unregister(&txss_eth_driver);
}

module_init(txss_eth_init);
module_exit(txss_eth_exit);
