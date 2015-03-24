/*
 * (C) Copyright TOSHIBA CORPORATION 2002-2007
 * All Rights Reserved.
 *
 * Data format for write():
 *	 [0x3a],[N],[data_1],[data_2],...,[data_N]
 */

#include <asm/tx-boards/tsb-generic.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/processor.h>

#define THIS_DRV_VERSION "1.07"

/* read() return without data if error */
#define ENABLE_ERROR_RETURN

#ifdef CONFIG_UAT_UCOM_BAUDRATE
#define UAT_DEFAULT_BAUDRATE CONFIG_UAT_UCOM_BAUDRATE
#else
#define UAT_DEFAULT_BAUDRATE 9600
#endif

/* select clock */
//#define SELECT_EXTCLK

/* ioctl */
#define UAT_SET_BAUD	2001
#define UAT_SET_WR_TIMEOUT 2002
#define UAT_SET_RD_TIMEOUT 2003
#define UAT_RESET	2004
#define UAT_GET_LASTERROR  2005
#define UAT_SET_BGR 2006

/* defalt timeout (unit 10msec) */
#define UAT_DEFAULT_WR_TIMEOUT (100 * (HZ/100))
#define UAT_DEFAULT_RD_TIMEOUT (100 * (HZ/100))

static struct {
	unsigned long irq;
	unsigned long busy;
	unsigned long baudrate;
	unsigned long wr_timeout;
	unsigned long rd_timeout;
	unsigned long rcverror;
	unsigned long lastrcverror;
} uat;

#define UAT_MISC_MINOR	241

#ifdef SELECT_EXTCLK
//#define BAUD_BASECLK	54000000	/* ExtClk */
#define BAUD_BASECLK	27000000	/* ExtClk */
#else
#define BAUD_BASECLK	TXX9_IMCLK	/* 66.66MHz */
#endif

/* UART registers */
#define UA0LCR		0x00
#define UA0DICR		0x04
#define UA0DISR		0x08
#define UA0SCISR	0x0C
#define UA0FCR		0x10
#define UA0FLCR		0x14
#define UA0BGR		0x18
#define UA0TFIFO	0x1C
#define UA0RFIFO	0x20

/* PIO registers */
#define XPIODO0		0x0
#define XPIODI0		0x4
#define XPIODIR0	0x8

#define XPIODO1		0x28
#define XPIODI1		0x2C
#define XPIODIR1	0x30

#define XPIODO2		0x38
#define XPIODI2		0x3C
#define XPIODIR2	0x40

/* Extended PIO registers */
#define XPIODO0SET	0x0080
#define XPIODO0RESET	0x0084
#define XPIODO1SET	0x0088
#define XPIODO1RESET	0x008c
#define XPIODO2SET	0x0090
#define XPIODO2RESET	0x0094

#if defined (CONFIG_TOSHIBA_TC90411)
#define UAT_UCOM_IRQ    (TC90411_IR_SIO(CONFIG_UAT_UCOM_CH) + TC90411_IRQ_IRC)
#define TXX9_SIO_REG    (TC90411_SIO_REG(CONFIG_UAT_UCOM_CH))
#define TXX9_PIO_REG    (TC90411_PIO_REG)
#elif defined (CONFIG_TOSHIBA_TC90412)
#define UAT_UCOM_IRQ    (TC90412_IR_SIO(CONFIG_UAT_UCOM_CH) + TC90412_IRQ_IRC)
#define TXX9_SIO_REG    (TC90412_SIO_REG(CONFIG_UAT_UCOM_CH))
#define TXX9_PIO_REG    (TC90412_PIO_REG)
#elif defined (CONFIG_TOSHIBA_TC90416)
#define UAT_UCOM_IRQ    (TC90416_IR_SIO(CONFIG_UAT_UCOM_CH) + TC90416_IRQ_IRC)
#define TXX9_SIO_REG    (TC90416_SIO_REG(CONFIG_UAT_UCOM_CH))
#define TXX9_PIO_REG    (TC90416_PIO_REG)
#else
#error Unknown Architecture
#endif

#define uat_in(off)      (*(volatile unsigned long*)(TXX9_SIO_REG+(off)))
#define uat_out(off,val) (*(volatile unsigned long*)(TXX9_SIO_REG+(off))=(val))
#define pio_in(off)      (*(volatile unsigned long*)(TXX9_PIO_REG+(off)))
#define pio_out(off,val) (*(volatile unsigned long*)(TXX9_PIO_REG+(off))=(val))

static void pio_set_bit(int bit, int value)
{
#if defined (CONFIG_TOSHIBA_TC90411)
	unsigned long flags;
	int reg_offset;
	unsigned int data;
	if(bit<32){
		reg_offset = XPIODO0;
	}else if(bit<64){
		reg_offset = XPIODO1;
		bit -=  32;
	}else if(bit<96){
		reg_offset = XPIODO2;
		bit -= 64;
	}else{
		return;
	}
	local_irq_save(flags);
	data = pio_in(reg_offset);
	if(value){
		data |= 1<<bit;
	}else{
		data &= ~(1<<bit);
	}
	pio_out(reg_offset,data);
	local_irq_restore(flags);
#else
	if(value){
		if(bit<32){
			pio_out(XPIODO0SET,1<<bit);
		}else if(bit<64){
			pio_out(XPIODO1SET,1<<(bit-32));
		}else if(bit<96){
			pio_out(XPIODO2SET,1<<(bit-64));
		}
	}else{
		if(bit<32){
			pio_out(XPIODO0RESET,1<<bit);
		}else if(bit<64){
			pio_out(XPIODO1RESET,1<<(bit-32));
		}else if(bit<96){
			pio_out(XPIODO2RESET,1<<(bit-64));
		}
	}
#endif
}

static void pio_dir_rmw_bit(int bit, int value)
{
	unsigned long flags;
	int reg_offset;
	unsigned int data;
	if(bit<32){
		reg_offset = XPIODIR0;
	}else if(bit<64){
		reg_offset = XPIODIR1;
		bit -=  32;
	}else if(bit<96){
		reg_offset = XPIODIR2;
		bit -= 64;
	}else{
		return;
	}
	local_irq_save(flags);
	data = pio_in(reg_offset);
	if(value){
		data |= 1<<bit;
	}else{
		data &= ~(1<<bit);
	}
	pio_out(reg_offset,data);
	local_irq_restore(flags);
}

#define TVMICON_HEADER_PIN	CONFIG_UAT_UCOM_HEAD_PIN
#define TVMICON_HEADER_PIN_2ND	CONFIG_UAT_UCOM_HEAD_PIN_2ND

static inline void uat_init_hdrpin(void) {
	pio_dir_rmw_bit(TVMICON_HEADER_PIN,1);
#ifdef CONFIG_UAT_UCOM_MULTI_HEAD
	pio_dir_rmw_bit(TVMICON_HEADER_PIN_2ND,1);
#endif
}

#ifdef CONFIG_UAT_UCOM_MULTI_HEAD
static inline void uat_set_hdrpin(int head) {
	if(head==0x3a){
		pio_set_bit(TVMICON_HEADER_PIN,1);
	}else if(head==0x3b){
		pio_set_bit(TVMICON_HEADER_PIN_2ND,1);
	}
}
static inline void uat_clr_hdrpin(void) {
	pio_set_bit(TVMICON_HEADER_PIN,0);
	pio_set_bit(TVMICON_HEADER_PIN_2ND,0);
}
#else
#define uat_set_hdrpin(head) pio_set_bit(TVMICON_HEADER_PIN,1)
#define uat_clr_hdrpin() pio_set_bit(TVMICON_HEADER_PIN,0)
#endif

#define DICR_TIR	0x2000
#define DICR_RIR	0x1000
#define DICR_SPIR	0x0800
#define DICR_STIR_OERS	0x0020
#define DICR_STIR_CTSS	0x0010
#define DICR_STIR_RBRKD	0x0008
#define DICR_STIR_TRDY	0x0004
#define DICR_STIR_TXALS	0x0002
#define DICR_STIR_UBRKD	0x0001
#define DICR_STIR_ERR	(DICR_STIR_OERS|DICR_STIR_RBRKD|DICR_STIR_UBRKD)
#define DISR_STIS	0x0040
#define DISR_RDIS	0x0080
#define DISR_TDIS	0x0100
#define DISR_TOUT	0x0200
#define DISR_ERI	0x0400
#define DISR_UOER	0x0800
#define DISR_UPER	0x1000
#define DISR_UFER	0x2000
#define DISR_UVALID	0x4000
#define DISR_UBRK	0x8000
#define DISR_RFDN_MASK	0x001F
#define SCISR_TXALS	0x0002
#define SCISR_TRDY	0x0004
#define FCR_TFRST	0x0004
#define FCR_RFRST	0x0002
#define FCR_FRSTE	0x0001
#define FCR_RDIL(a)	(((a)&0x3)<<7)
#define FCR_TDIL(a)	(((a)&0x3)<<3)

#define STS_IDLE	1
#define STS_HDRRDY	2
#define STS_HDREND	3
#define STS_DATARDY	4
#define STS_TXDONE	5

#define RBUF_SIZE	1024
#define TBUF_SIZE	1024


//#define Dprintk(fmt,arg...) printk(fmt, ## arg)
#define Dprintk(fmt,arg...)

static char tbuf[TBUF_SIZE];
static char rbuf[RBUF_SIZE];
static int tbuf_top, tbuf_tail, rbuf_top, rbuf_tail;
static int txstat;
static int writecnt;
static int readcnt;
static DECLARE_WAIT_QUEUE_HEAD(txwait);
static DECLARE_WAIT_QUEUE_HEAD(rxwait);
static int rxwait_flag;

static int uat_set_bgr(int bgr)
{
	bgr = bgr & 0xff;
	uat_out(UA0BGR, bgr);
	uat.baudrate = BAUD_BASECLK/2/16/bgr;
	Dprintk("bgr=0x%x,baud=%d\n", bgr, uat.baudrate);
	return 0;
}

static int uat_set_baudrate(int baudrate)
{
	unsigned int bclk, brd, bgr;
	unsigned int baseclk = BAUD_BASECLK / 2;
	bclk = 0;
	brd = ((baseclk / 16) + (baudrate / 2)) / baudrate;
	if (brd == 0) {
		return -1;
	}
	bgr = (bclk << 8) + brd;
	uat_out(UA0BGR, bgr);
	Dprintk("baud=%d, UA0BGR=0x%x\n", baudrate, bgr);
	uat.baudrate = baudrate;
	return 0;
}

static void uat_initialize(void)
{
	uat_init_hdrpin();
	uat_clr_hdrpin();

	uat_set_baudrate(uat.baudrate);
#ifdef SELECT_EXTCLK
	uat_out(UA0LCR, 0x4078);	/* ExtClk Even parity */
#else
	uat_out(UA0LCR, 0x4038);	/* InternalClk Even parity */
#endif
	uat_out(UA0DICR, 0x0000);
	uat_out(UA0FCR, FCR_TFRST | FCR_RFRST | FCR_FRSTE);
	uat_out(UA0FCR, FCR_RDIL(3));	// set FIFO threshold
	uat_out(UA0FLCR, 0x0002);
	tbuf_top = tbuf_tail = rbuf_top = rbuf_tail = 0;
	txstat = STS_IDLE;
	uat.lastrcverror = 0;
	uat.rcverror = 0;
	Dprintk("LCR=0x%x\n", uat_in(UA0LCR));
	Dprintk("DICR=0x%x\n", uat_in(UA0DICR));
	Dprintk("DISR=0x%x\n", uat_in(UA0DISR));
	Dprintk("SCISR=0x%x\n", uat_in(UA0SCISR));
	Dprintk("FCR=0x%x\n", uat_in(UA0FCR));
	Dprintk("FLCR=0x%x\n", uat_in(UA0FLCR));
	Dprintk("BGR=0x%x\n", uat_in(UA0BGR));
}

static irqreturn_t uat_interrupt(int irq, void *dev_id)
{
	unsigned long disr = uat_in(UA0DISR);
	unsigned long scisr = uat_in(UA0SCISR);

	Dprintk("uat_int: disr=0x%lx,scisr=0x%lx,dicr=0x%lx\n", disr, scisr, uat_in(UA0DICR));

	if (disr & DISR_STIS) {
		if (scisr & SCISR_TXALS) {
			if (txstat == STS_HDRRDY) {
				uat_set_hdrpin(tbuf[tbuf_tail]);
				uat_out(UA0TFIFO, tbuf[tbuf_tail++]);
				uat_out(UA0TFIFO, tbuf[tbuf_tail++]);

				txstat = STS_HDREND;
				uat_out(UA0DICR, uat_in(UA0DICR) | DICR_STIR_TXALS);
			} else if (txstat == STS_HDREND) {
				uat_clr_hdrpin();
				uat_out(UA0DICR, uat_in(UA0DICR) & ~DICR_STIR_TXALS);
				txstat = STS_DATARDY;
			}
		}
	}
	scisr = uat_in(UA0SCISR);
	if ((scisr & SCISR_TRDY) && (txstat == STS_DATARDY)) {
		Dprintk("uat_int: send data\n");
		while ((uat_in(UA0SCISR) & SCISR_TRDY) && tbuf_tail != tbuf_top) {
			uat_out(UA0TFIFO, tbuf[tbuf_tail++]);
		}
		if (tbuf_tail == tbuf_top) {
			Dprintk("uat_int: sent data\n");
			txstat = STS_TXDONE;
			wake_up_interruptible(&txwait);
			uat_out(UA0DICR, uat_in(UA0DICR) & ~DICR_TIR);
		} else {
			uat_out(UA0DISR, ~DISR_TDIS);
			uat_out(UA0DICR, uat_in(UA0DICR) | DICR_TIR);
		}
	}
	if (disr & (DISR_RDIS | DISR_TOUT)) {
		int maxretry = 32;
		while (!(disr & DISR_UVALID) && (maxretry-- > 0)) {
			if ((rbuf_top + 1) % RBUF_SIZE == rbuf_tail) {
				Dprintk("uat_int: rbuf full\n");
				uat_in(UA0RFIFO);
				break;
			}
			rbuf[rbuf_top++] = uat_in(UA0RFIFO);
			Dprintk("uat_int: rcv %02x\n", rbuf[rbuf_top - 1]);
			rbuf_top = rbuf_top % RBUF_SIZE;
			if (uat.rcverror == 0) {
				uat.rcverror = disr & (DISR_UBRK | DISR_UFER | DISR_UPER | DISR_UOER);
			}
			disr = uat_in(UA0DISR);
		}
		rxwait_flag = 1;
		wake_up_interruptible(&rxwait);
	}
	uat_out(UA0DISR, ~(DISR_ERI | DISR_TOUT | DISR_TDIS | DISR_RDIS | DISR_STIS));
#if 0
	if ((disr = uat_in(UA0DISR)) & DISR_RFDN_MASK != 0) {
		printk("*** uat_int: disr=%x\n", disr);
		printk("*** uat_int: dicr=%x\n", uat_in(UA0DICR));
		printk("*** uat_int: scisr=%x\n", uat_in(UA0SCISR));
	}
#endif

	return IRQ_HANDLED;
}

static ssize_t uat_write(struct file *file, const char __user *buf, size_t count, loff_t * ppos)
{
	int ret;
	unsigned long flags;
	int timeout = (uat.wr_timeout == 0) ? MAX_SCHEDULE_TIMEOUT : uat.wr_timeout;
	Dprintk("uat_write: buf[0,1]=0x%02x,0x%02x,count=%d\n", buf[0], buf[1], count);

	Dprintk("LCR=0x%x\n", uat_in(UA0LCR));
	Dprintk("DICR=0x%x\n", uat_in(UA0DICR));
	Dprintk("DISR=0x%x\n", uat_in(UA0DISR));
	Dprintk("SCISR=0x%x\n", uat_in(UA0SCISR));
	Dprintk("FCR=0x%x\n", uat_in(UA0FCR));
	Dprintk("FLCR=0x%x\n", uat_in(UA0FLCR));
	Dprintk("BGR=0x%x\n", uat_in(UA0BGR));

	/* lock */
	Dprintk("uat_write: lock\n");
	writecnt++;
	if (writecnt > 1)
		Dprintk("uat_write: cnt=%d\n", writecnt);

	if (copy_from_user(tbuf, buf, count))
		return -EFAULT;
	tbuf_top = count;
	tbuf_tail = 0;

	if (uat_in(UA0SCISR) & SCISR_TXALS) {
		Dprintk("uat_write: send header\n");
		uat_set_hdrpin(tbuf[tbuf_tail]);	/* header pin on */
		uat_out(UA0TFIFO, tbuf[tbuf_tail++]);
		uat_out(UA0TFIFO, tbuf[tbuf_tail++]);
		txstat = STS_HDREND;
	} else {
		Dprintk("uat_write: enable interrupt\n");
		txstat = STS_HDRRDY;
	}
	local_irq_save(flags);

	uat_out(UA0DISR, ~DISR_STIS);
	uat_out(UA0DICR, uat_in(UA0DICR) | DICR_STIR_TXALS);
	local_irq_restore(flags);

	ret = wait_event_interruptible_timeout(txwait, txstat == STS_TXDONE,
					       timeout);
	local_irq_save(flags);
	/* disable tx int */
	uat_out(UA0DICR, uat_in(UA0DICR) & ~(DICR_TIR | DICR_STIR_TXALS));
	local_irq_restore(flags);
	if (signal_pending(current)) {
		Dprintk("uat_write: received signal\n");
		return -ERESTARTSYS;
	}
	txstat = STS_IDLE;

	if (ret == 0) {
		Dprintk("uat_write: timeout\n");
		count = 0;
		return -ETIMEDOUT;
	}

	Dprintk("uat_write: tbuf[0,1]=0x%02x,0x%02x,0x%02x,0x%02x,count=%d\n",
		tbuf[0], tbuf[1], tbuf[2], tbuf[3], count);
	/* unlock */
	Dprintk("uat_write: unlcok. cnt=%d\n", writecnt);
	writecnt--;

	return count;
}

static ssize_t uat_read(struct file *file, char __user *buf, size_t count, loff_t * ppos)
{
	int rsize, size;
	int retrycnt = 0;
	int timeout = (uat.rd_timeout == 0) ? MAX_SCHEDULE_TIMEOUT : uat.rd_timeout;

	readcnt++;
	if (readcnt > 1)
		Dprintk("uat_read: cnt=%d\n", readcnt);
      retry:
	rsize = rbuf_top - rbuf_tail;
	Dprintk("uat_read: rsize=%d\n", rsize);
	if (rsize < 0)
		rsize = rsize + RBUF_SIZE;
	if (rsize >= count) {
		if ((rbuf_top > rbuf_tail) || ((RBUF_SIZE - rbuf_tail) >= count)) {
			if (copy_to_user(buf, &rbuf[rbuf_tail], count) < 0) {
				Dprintk("uat_read: copy_to_user is failed.\n");
				Dprintk("uat_read: tail=%d,count=%d\n", rbuf_tail, count);
				readcnt--;
				return -EFAULT;
			}
			rbuf_tail += count;
			rbuf_tail = rbuf_tail % RBUF_SIZE;
		} else {
			size = RBUF_SIZE - rbuf_tail;
			if (copy_to_user(buf, &rbuf[rbuf_tail], size) < 0) {
				Dprintk("uat_read: copy_to_user is failed\n");
				Dprintk("uat_read: tail=%d,size=%d\n", rbuf_tail, size);
				readcnt--;
				return -EFAULT;
			}
			if (copy_to_user(&buf[size], &rbuf[0], count - size) < 0) {
				Dprintk("uat_read: copy_to_user is failed\n");
				Dprintk("uat_read: tail=%d,count=%d,size=%d\n", rbuf_tail, count, size);
				readcnt--;
				return -EFAULT;
			}
			rbuf_tail = count - size;
		}
	} else {
		rsize = rbuf_top - rbuf_tail;
		if (rsize < 0)
			rsize = rsize + RBUF_SIZE;
		if (rsize < count) {
			timeout = wait_event_interruptible_timeout(rxwait,
								   rxwait_flag,
								   timeout);
			rxwait_flag = 0;
			if (signal_pending(current)) {
				readcnt--;
				Dprintk("uat_read: received signal\n");
				Dprintk("LCR=0x%x\n", uat_in(UA0LCR));
				Dprintk("DICR=0x%x\n", uat_in(UA0DICR));
				Dprintk("DISR=0x%x\n", uat_in(UA0DISR));
				Dprintk("SCISR=0x%x\n", uat_in(UA0SCISR));
				Dprintk("FCR=0x%x\n", uat_in(UA0FCR));
				Dprintk("FLCR=0x%x\n", uat_in(UA0FLCR));
				Dprintk("BGR=0x%x\n", uat_in(UA0BGR));
				return -ERESTARTSYS;
			}
		}
		retrycnt++;
		Dprintk("uat_read: retry=%d, count=%d\n", retrycnt, count);
#ifdef ENABLE_ERROR_RETURN
		if (uat.rcverror) {
			uat.lastrcverror = uat.rcverror;
			uat.rcverror = 0;
			printk("uat_read: error=0x%lx\n", uat.lastrcverror);
			return -EIO;
		}
#endif
		if (timeout == 0) {
			Dprintk("LCR=0x%x\n", uat_in(UA0LCR));
			Dprintk("DICR=0x%x\n", uat_in(UA0DICR));
			Dprintk("DISR=0x%x\n", uat_in(UA0DISR));
			Dprintk("SCISR=0x%x\n", uat_in(UA0SCISR));
			Dprintk("FCR=0x%x\n", uat_in(UA0FCR));
			Dprintk("FLCR=0x%x\n", uat_in(UA0FLCR));
			Dprintk("BGR=0x%x\n", uat_in(UA0BGR));
			return -ETIMEDOUT;
		}
		goto retry;
	}
	Dprintk("uat_read: count=%d. 0x%02x,0x%02x\n", count, buf[0], buf[1]);
	readcnt--;
#if 0
	{
		int i;
		printk("$");
		for (i = 0; i < count; i++) {
			printk("%02x ", (unsigned char) buf[i]);
		}
		printk("\n");
	}
#endif
	return count;
}

static int uat_ioctl(struct inode *inodep, struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case UAT_SET_BGR:
		uat_set_bgr(arg);
		break;

	case UAT_SET_BAUD:
		if (!arg || uat_set_baudrate(arg))
			return -EINVAL;
		break;
	case UAT_SET_WR_TIMEOUT:
		if (arg >= 0 && arg < 1000)
			uat.wr_timeout = arg * (HZ/100);
		else
			return -EINVAL;
		break;
	case UAT_SET_RD_TIMEOUT:
		if (arg >= 0 && arg < 1000)
			uat.rd_timeout = arg * (HZ/100);
		else
			return -EINVAL;
		break;
	case UAT_RESET:
		uat_initialize();
		break;
	case UAT_GET_LASTERROR:
		if (copy_to_user((void __user *) arg, &uat.lastrcverror, sizeof(int))) {
			return -EFAULT;
		}
		uat.lastrcverror = 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int uat_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, (void *) &uat.busy) != 0)
		return -EBUSY;
	uat_initialize();
	enable_irq(uat.irq);
	uat_out(UA0DICR, uat_in(UA0DICR) | DICR_RIR);	// enable rx int.
	return 0;
}

static int uat_release(struct inode *inode, struct file *file)
{
	disable_irq(uat.irq);
	uat_initialize();
	uat.busy = 0;
	return 0;
}

static struct file_operations uat_fops = {
      .ioctl = uat_ioctl,
      .read = uat_read,
      .write = uat_write,
      .open = uat_open,
      .release = uat_release
};

#ifdef CONFIG_PM
#ifdef CONFIG_PM_DEBUG
#include <asm/tx-boards/pmon.h>
#endif
unsigned char uat_write_cmd(int send_cksum, unsigned char *data, int size)
{
	int i;
	unsigned char cksum,ack_cksum,ack_data[5];
	int retrycnt = 0;
	int timeout = 1000; /* ms */
 retry:
	uat_initialize();
	uat_set_hdrpin(data[0]);	/* header pin on */
	uat_out(UA0TFIFO, data[0]);
	uat_out(UA0TFIFO, data[1]);
	while((uat_in(UA0SCISR) & SCISR_TXALS)==0)
		;
	uat_clr_hdrpin();	/* header pin on */
	cksum = data[0] + data[1];

	for(i=2;i<size;i++){
		while((uat_in(UA0SCISR) & SCISR_TRDY)==0)
			;
		uat_out(UA0TFIFO, data[i]);
		cksum += data[i];
	}
	if(send_cksum){
		int loopcnt = 0;
		uat_out(UA0TFIFO, cksum);
		while((uat_in(UA0DISR) & DISR_RFDN_MASK) < 5) {
			if (loopcnt++>timeout){
				goto to_retry;
			}
			udelay(1000);
		}
		if((uat_in(UA0DISR) & DISR_RFDN_MASK)<5){
			udelay(100000);
			goto to_retry;
		}
		ack_cksum = 0;
		for(i=0; i<4; i++){
			ack_data[i]=uat_in(UA0RFIFO);
			ack_cksum += ack_data[i];
		}
		ack_data[i]=uat_in(UA0RFIFO);
		if(ack_data[4] != ack_cksum){
			goto to_retry;
		}
		if(ack_data[3] == 0x34){ /* busy */
			udelay(100000);
			goto to_retry;
		}
		if(ack_data[3] == 0x32){ /* nack */
			goto to_retry;
		}
		if(ack_data[3] != 0x30){ /* !ack */
			goto to_retry;
		}
	}
	return cksum;
 to_retry:
	if(retrycnt++ >= 5){
		return -1;
	}
#ifdef CONFIG_PM_DEBUG
	prom_printf("uat_write_cmd: error");
	for(i=0;i<5;i++){
		prom_printf(" %02x",ack_data[i]);
	}
	prom_printf("\n");
#endif
	while(!(uat_in(UA0DISR) & DISR_UVALID))
		uat_in(UA0RFIFO);
	goto retry;
}

static int uat_suspend(struct platform_device *dev, pm_message_t state)
{
	/* nothing to do */
	return 0;
}

static int uat_resume(struct platform_device *dev)
{
	/* nothing to do */
	return 0;
}

static struct platform_device *uat_com_devs;

static struct platform_driver ucom_driver = {
	.suspend	= uat_suspend,
	.resume		= uat_resume,
	.driver         = {
		.name	= "uat_ucom",
		.owner  = THIS_MODULE,
	},
};
#endif

static struct miscdevice uat_dev = { UAT_MISC_MINOR, "uat_ucom", &uat_fops };

static int __init uat_init(void)
{
	int err;

	switch (mips_machgroup) {
	case (MACH_GROUP_TC90411):
		break;
	case (MACH_GROUP_TC90412):
		break;
	case (MACH_GROUP_TC90416):
		break;
	default:
		return -ENODEV;
		/* not reached */
	}
	uat.baudrate = UAT_DEFAULT_BAUDRATE;

	uat_initialize();
	uat.irq = UAT_UCOM_IRQ;
	uat.busy = 0;
	uat.wr_timeout = UAT_DEFAULT_WR_TIMEOUT;
	uat.rd_timeout = UAT_DEFAULT_RD_TIMEOUT;

	if (request_irq(uat.irq, uat_interrupt, IRQF_DISABLED, uat_dev.name, (void *) 0)) {
		printk(KERN_ERR "intdrv: cannot register IRQ %ld\n", uat.irq);
		return -EIO;
	}
	disable_irq(uat.irq);
	err = misc_register(&uat_dev);
	if (err) {
		printk(KERN_ERR "flash: unable to get misc minor\n");
		return err;
	}
	printk(KERN_INFO "UART for ucom Driver, v%s\n", THIS_DRV_VERSION);

#ifdef CONFIG_PM
	err = platform_driver_register(&ucom_driver);
	if (err < 0) {
		printk(KERN_ERR "ucom: unable to register pm functions\n");
		goto err_ret;
	}

	uat_com_devs = platform_device_register_simple("uat_ucom", -1, NULL, 0);
	if (IS_ERR(uat_com_devs)) {
		err = PTR_ERR(uat_com_devs);
		platform_driver_unregister(&ucom_driver);
		goto err_ret;
	}

#endif
	return 0;

#ifdef CONFIG_PM
err_ret:
#endif
	misc_deregister(&uat_dev);

	return err;
}

static void __exit uat_exit(void)
{
#ifdef CONFIG_PM
	platform_device_unregister(uat_com_devs);
	platform_driver_unregister(&ucom_driver);
#endif
	misc_deregister(&uat_dev);
}

module_init(uat_init);
module_exit(uat_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("UART for ucom driver");
