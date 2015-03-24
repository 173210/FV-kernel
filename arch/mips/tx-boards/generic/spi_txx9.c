/*
 * linux/arch/mips/tx-boards/generic/spi_txx9.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright TOSHIBA CORPORATION 2004-2007
 * All Rights Reserved.
 *
 */
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/spi/spi.h>
#include <asm/tx-boards/generic.h>
#include <asm/tx-boards/txx9.h>

#ifdef CONFIG_TOSHIBA_TC90416
#define SPI_FIFO_SIZE 20
#else
#define SPI_FIFO_SIZE 4
#endif

#undef SPI_DEBUG

#ifdef SPI_DEBUG
#define DBG(a...)  printk(a)
#else
#define DBG(a...)
#endif

struct txx9spi {
	struct workqueue_struct	*workqueue;
	struct work_struct work;
	spinlock_t lock;
	struct list_head queue;
	wait_queue_head_t waitq;
	struct resource *res;
	volatile u32 __iomem *regs[TXx9_SPI_REGS];
	int irq;
	unsigned mapped:1;
	struct spi_master *master;
	int rxdone;
};

struct txx9spi_cs {
	u32 cr1;
};

static inline u32 txx9spi_rd(struct txx9spi *c, int reg)
{
	return __raw_readl(c->regs[reg]);
}
static inline void txx9spi_wr(struct txx9spi *c, u32 val, int reg)
{
	__raw_writel(val, c->regs[reg]);
}

static int
txx9spi_setup(struct spi_device *spi)
{
#ifdef CONFIG_TOSHIBA_TC90416
	struct spi_master *master = spi->master;
	struct txx9spi *c = spi_master_get_devdata(master);
#endif
	struct txx9spi_platform_data *plat =
		spi->master->cdev.dev->platform_data;
	struct txx9spi_cs *cs = spi->controller_state;
	unsigned int n;

	if (!spi->max_speed_hz)
		return -EINVAL;

	if (!cs) {
		cs = kmalloc(sizeof(*cs), GFP_KERNEL);
		if (!cs)
			return -ENOMEM;
		spi->controller_state = cs;
	}

	if (spi->bits_per_word == 0)
		spi->bits_per_word = 8;
	else if (spi->bits_per_word != 8)
		return -EINVAL;
	/* calc real speed */
#ifdef CONFIG_TOSHIBA_TC90416
	n = (plat->baseclk + spi->max_speed_hz - 1) / spi->max_speed_hz / 2;
	if (n < 1)
		n = 0;
	if (n > 0x100)
		return -EINVAL;
	n--;
	spi->max_speed_hz = plat->baseclk / (2 * (n + 1));

	if (plat->calc_cr1)
		cs->cr1 = plat->calc_cr1(spi);
	else
		cs->cr1 = (n << 8) | 0x08;	/* 8bit only */
	txx9spi_wr(c, TXx9_SPCR0_SBOS |
		   ((spi->mode & SPI_CPOL) ? TXx9_SPCR0_SPOL : 0) |
		   ((spi->mode & SPI_CPHA) ? TXx9_SPCR0_SPHA : 0) |
		   plat->cr0_baseval,
		   TXx9_SPCR0);
	txx9spi_wr(c, cs->cr1, TXx9_SPCR1);
#else 
	n = (plat->baseclk + spi->max_speed_hz - 1) / spi->max_speed_hz;
	if (n < 1)
		n = 1;
	else if (n > 0xff)
		return -EINVAL;
	spi->max_speed_hz = plat->baseclk / n;

	if (plat->calc_cr1)
		cs->cr1 = plat->calc_cr1(spi);
	else
		cs->cr1 = (n << 8) | 0x08;	/* 8bit only */
#endif 
	return 0;
}

static irqreturn_t
txx9spi_interrupt(int irq, void *dev_id)
{
	struct txx9spi *c = dev_id;
	/* disable rx intr */
	DBG("spi<%d> spsr=0x%x\n",__LINE__,txx9spi_rd(c, TXx9_SPSR));
#ifdef CONFIG_TOSHIBA_TC90416
	if(txx9spi_rd(c, TXx9_SPSR) & TXx9_SPSR_RBSI) {
		txx9spi_wr(c, txx9spi_rd(c, TXx9_SPILR) & ~TXx9_SPILR_RXIFL_MASK, TXx9_SPILR);
		c->rxdone = 1;
	}
#else 
	txx9spi_wr(c, txx9spi_rd(c, TXx9_SPCR0) & ~TXx9_SPCR0_RBSIE,
		   TXx9_SPCR0);
#endif
	wake_up(&c->waitq);
	return IRQ_HANDLED;
}

static int
txx9spi_work_one(struct txx9spi *c, struct txx9spi_platform_data *plat,
		 struct spi_message *m)
{
	struct spi_device *spi = m->spi;
	struct txx9spi_cs *cs = spi->controller_state;
	struct spi_transfer *t;
	unsigned int nsecs;
	unsigned int cs_change;
	int status;

	BUG_ON((txx9spi_rd(c, TXx9_SPMCR) & TXx9_SPMCR_OPMODE) ==
	       TXx9_SPMCR_ACTIVE);

	/* enter config mode */
	txx9spi_wr(c, TXx9_SPMCR_CONFIG | TXx9_SPMCR_BCLR, TXx9_SPMCR);
	txx9spi_wr(c, TXx9_SPCR0_SBOS |
		   ((spi->mode & SPI_CPOL) ? TXx9_SPCR0_SPOL : 0) |
		   ((spi->mode & SPI_CPHA) ? TXx9_SPCR0_SPHA : 0) |
		   plat->cr0_baseval,
		   TXx9_SPCR0);
	txx9spi_wr(c, cs->cr1, TXx9_SPCR1);
	DBG("spi<%d> cr0=0x%x,cr1=0x%x\n",__LINE__,txx9spi_rd(c, TXx9_SPCR0),cs->cr1);
	/* enter active mode */
	txx9spi_wr(c, TXx9_SPMCR_ACTIVE, TXx9_SPMCR);

	cs_change = 1;
	status = 0;
	nsecs = 100 + 1000000000 / spi->max_speed_hz / 2;
	list_for_each_entry (t, &m->transfers, transfer_list) {
		const u8 *txbuf = t->tx_buf;
		u8 *rxbuf = t->rx_buf;
		u8 data;
		unsigned int len = t->len;
		if (!txbuf && !rxbuf && len) {
			status = -EINVAL;
			break;
		}
		if (cs_change) {
			status = plat->cs_func(spi, 1);
			if (status < 0)
				break;
			ndelay(nsecs);
		}
		cs_change = t->cs_change;
		while (len) {
			unsigned int count = SPI_FIFO_SIZE;
			int i;
			u32 reg;
			if (len < count)
				count = len;
			DBG("spi<%d> count=%d\n",__LINE__,count);
			/* now tx must be idle... */
			while (!(txx9spi_rd(c, TXx9_SPSR) & TXx9_SPSR_SIDLE))
				;
#ifdef CONFIG_TOSHIBA_TC90416
			c->rxdone = 0;
			reg = txx9spi_rd(c, TXx9_SPILR);
			reg &= ~TXx9_SPILR_RXIFL_MASK;
			reg |= count;
			txx9spi_wr(c, reg, TXx9_SPILR);
			DBG("spi<%d> ilr=0x%x,spsr=0x%x\n",__LINE__,txx9spi_rd(c, TXx9_SPILR),txx9spi_rd(c, TXx9_SPSR));
#else
			reg = txx9spi_rd(c, TXx9_SPCR0);
			reg &= ~TXx9_SPCR0_RXIFL_MASK;
			reg |= (count - 1) << 12;
			/* enable rx intr */
			reg |= TXx9_SPCR0_RBSIE;
			txx9spi_wr(c, reg, TXx9_SPCR0);
#endif
			/* send */
			for (i = 0; i < count; i++)
				txx9spi_wr(c, txbuf ? *txbuf++ : 0, TXx9_SPDR);
			/* wait all rx data */
#ifdef CONFIG_TOSHIBA_TC90416
			wait_event(c->waitq, c->rxdone);
#else
			wait_event(c->waitq,
				   txx9spi_rd(c, TXx9_SPSR) & TXx9_SPSR_RBSI);
#endif
			/* receive */
			for (i = 0; i < count; i++) {
				data = txx9spi_rd(c, TXx9_SPDR);
				DBG("spi<%d> rdata=0x%x\n",__LINE__,data);
				if (rxbuf)
					*rxbuf++ = data;
			}
			len -= count;
		}
		m->actual_length += t->len;
		if (t->delay_usecs)
			udelay(t->delay_usecs);

		if (!cs_change)
			continue;
		if (t->transfer_list.next == &m->transfers)
			break;
		/* sometimes a short mid-message deselect of the chip
		 * may be needed to terminate a mode or command
		 */
		ndelay(nsecs);
		status = plat->cs_func(spi, 0);
		if (status < 0)
			break;
		ndelay(nsecs);
	}

	m->status = status;
	m->complete(m->context);

	/* normally deactivate chipselect ... unless no error and
	 * cs_change has hinted that the next message will probably
	 * be for this chip too.
	 */
	if (!(status == 0 && cs_change)) {
		ndelay(nsecs);
		plat->cs_func(spi, 0);
		ndelay(nsecs);
	}

	/* enter config mode */
	txx9spi_wr(c, TXx9_SPMCR_CONFIG | TXx9_SPMCR_BCLR, TXx9_SPMCR);
	return status;
}

static void
txx9spi_work(struct work_struct *work)
{
	struct txx9spi *c = container_of(work, struct txx9spi, work);
	struct spi_master *master = c->master;
	struct txx9spi_platform_data *plat = master->cdev.dev->platform_data;
	unsigned long flags;

	spin_lock_irqsave(&c->lock, flags);
	while (!list_empty(&c->queue)) {
		struct spi_message *m;

		m = container_of(c->queue.next, struct spi_message, queue);
		list_del_init(&m->queue);
		spin_unlock_irqrestore(&c->lock, flags);

		txx9spi_work_one(c, plat, m);

		spin_lock_irqsave(&c->lock, flags);
	}
	spin_unlock_irqrestore(&c->lock, flags);
}

static int
txx9spi_transfer(struct spi_device *spi, struct spi_message *m)
{
	struct spi_master *master = spi->master;
	struct txx9spi *c = spi_master_get_devdata(master);
	unsigned long flags;

	m->actual_length = 0;
	spin_lock_irqsave(&c->lock, flags);
	list_add_tail(&m->queue, &c->queue);
	queue_work(c->workqueue, &c->work);
	spin_unlock_irqrestore(&c->lock, flags);

	return 0;
}

static void
txx9spi_cleanup(const struct spi_device *spi)
{
	kfree(spi->controller_state);
}

static int __init
txx9spi_probe(struct platform_device *dev)
{
	struct spi_master *master;
	struct txx9spi *c;
	struct txx9spi_platform_data *plat = dev->dev.platform_data;
	int i, ret = -ENODEV;

	master = spi_alloc_master(&dev->dev, sizeof(*c));
	if (!master)
		return ret;
	c = spi_master_get_devdata(master);
	c->master = spi_master_get(master);

	INIT_WORK(&c->work, txx9spi_work);
	spin_lock_init(&c->lock);
	INIT_LIST_HEAD(&c->queue);
	init_waitqueue_head(&c->waitq);

	c->res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!c->res)
		goto put_and_exit;
	if (!plat->membase) {
		plat->membase = ioremap(c->res->start, c->res->end - c->res->start + 1);
		c->mapped = 1;
	}
	for (i = 0; i < TXx9_SPI_REGS; i++)
		c->regs[i] = plat->membase + plat->offsets[i];

	/* enter config mode */
	txx9spi_wr(c, TXx9_SPMCR_CONFIG | TXx9_SPMCR_BCLR, TXx9_SPMCR);

	c->irq = platform_get_irq(dev, 0);
	if (c->irq < 0)
		goto res_and_exit;
	ret = request_irq(c->irq, txx9spi_interrupt, 0, dev->name, c);
	if (ret)
		goto res_and_exit;

	c->workqueue = create_singlethread_workqueue(master->cdev.dev->bus_id);
	if (!c->workqueue)
		goto irq_and_exit;

	dev_info(&dev->dev, "at 0x%llx, irq %d, %dMHz\n",
		 (unsigned long long)c->res->start, c->irq,
		 (plat->baseclk + 500000) / 1000000);

	master->bus_num = dev->id;
	master->setup = txx9spi_setup;
	master->transfer = txx9spi_transfer;
	master->cleanup = txx9spi_cleanup;

	ret = spi_register_master(master);
	if (ret)
		goto wq_and_exit;
	return 0;
 wq_and_exit:
	destroy_workqueue(c->workqueue);
 irq_and_exit:
	free_irq(c->irq, master);
 res_and_exit:
	if (c->mapped)
		iounmap(plat->membase);
 put_and_exit:
	spi_master_put(master);
	return ret;
}

static struct platform_driver txx9spi_driver = {
	.probe = txx9spi_probe,
	.driver = {
		.name = "txx9spi",
		.owner= THIS_MODULE,
	},
};

#if defined(CONFIG_CPU_TX49XX) || defined(CONFIG_TOSHIBA_TC90416)
/* helper routine for TXx9 internal SPIC */
int __init
txx9_spi_init(int busid, unsigned long base, int irq,
	      int (*cs_func)(struct spi_device *spi, int on))
{
	struct {
		struct platform_device pdev;
		struct resource resource[2];
		struct txx9spi_platform_data pdata;
	} *pobj;
	int ret;
	pobj = kzalloc(sizeof(*pobj), GFP_KERNEL);
	if (!pobj)
		return -ENOMEM;
	pobj->pdev.name = "txx9spi";
	pobj->pdev.id = busid;
	pobj->pdev.resource = pobj->resource;
	pobj->pdev.dev.platform_data = &pobj->pdata;
	pobj->pdev.num_resources = 2;
	pobj->resource[0].start = base;
	pobj->resource[0].end = base + 0x20 - 1;
	pobj->resource[0].flags = IORESOURCE_MEM;
	pobj->resource[0].parent = &txx9_reg_res;
	pobj->resource[1].start = irq;
	pobj->resource[1].flags = IORESOURCE_IRQ;
	pobj->pdata.cs_func = cs_func;
	pobj->pdata.cr0_baseval = 0x08;
#ifdef CONFIG_TOSHIBA_TC90416
	pobj->pdata.baseclk = txx9_gbus_clock;
#else
	pobj->pdata.baseclk = TXX9_IMCLK / 4;
#endif
	pobj->pdata.offsets[TXx9_SPMCR] = 0x00;
	pobj->pdata.offsets[TXx9_SPCR0] = 0x04;
	pobj->pdata.offsets[TXx9_SPCR1] = 0x08;
	pobj->pdata.offsets[TXx9_SPFS] = 0x0c;
	pobj->pdata.offsets[TXx9_SPSR] = 0x14;
	pobj->pdata.offsets[TXx9_SPDR] = 0x18;
#ifdef CONFIG_TOSHIBA_TC90416
	pobj->pdata.offsets[TXx9_SPSSR] = 0x10;
	pobj->pdata.offsets[TXx9_SPRSR] = 0x1C;
	pobj->pdata.offsets[TXx9_SPFLR] = 0x20;
	pobj->pdata.offsets[TXx9_SPILR] = 0x24;
	pobj->pdata.offsets[TXx9_SPPR] = 0x28;
	pobj->pdata.offsets[TXx9_SPLCR] = 0x2C;
	pobj->pdata.offsets[TXx9_SPDER] = 0x30;
#endif
	if((base < KSEG2 && base>=KSEG1) || base>0xFF1F0000)
		pobj->pdata.membase = (volatile void __iomem *)(unsigned long)(int)base;
	
	ret = platform_device_register(&pobj->pdev);
	if (ret) {
		kfree(pobj);
		return ret;
	}
	return 0;
}
#endif /* CONFIG_CPU_TX49XX */

static int __init txx9spi_init(void)
{
	return platform_driver_register(&txx9spi_driver);
}
__initcall(txx9spi_init);
