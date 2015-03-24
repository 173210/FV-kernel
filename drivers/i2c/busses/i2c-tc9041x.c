/*
 *  i2c-tc9041x.c
 *
 *  I2C adapter for the TC9041X I2C bus access.
 *
 *  (C) Copyright TOSHIBA CORPORATION 2005-2007
 *  All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  History:
 *    18 Feb 2005: Initial version. Support TC90411.
 *    31 Jul 2006: Support TC90412
 *    19 Sep 2007: Rename symbols.
 *
 *  Extended function:
 *   ioctl(fd,I2C_RDWR,arg):
 *    I2C_M_SUBADDR1:  buf[0] is handled as sub address.
 *    I2C_M_SUBADDR2:  buf[0],buf[1] are handled as sub address.
 *     Write:
 *      S,SLA,SUB,D[0-31],P
 *      S,SLA,SUB+32,D[32-n],P
 *     Read:
 *      S,SLA,SUB,S,SLA|R,D[0-31],P
 *      S,SLA,SUB+32,S,SLA|R,D[32-n],P
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>

#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/err.h>

#include <asm/system.h>
#include <asm/irq.h>

#include <asm/tx-boards/tsb-generic.h>
#include <asm/tx-boards/i2c-tc9041x.h>

#define THIS_DRV_VERSION "2.06"

/*
 * un-comment to use debug definition
 */
//#define DEBUG

//#define ENABLE_NACK_RETRY	/* Don't enable if older than TC90411 new#1 */
//#define SEQUENCILAL_TRANSFER /* TBD */
#define WORKAROUND_HIGHSPEED_MODE
#define WORKAROUND_DELAY_STOP

#ifdef CONFIG_TOSHIBA_TC90412
#define I2C_TC9041X_CH_MAX 2
#define TC9041X_I2C0_REG TC90412_I2C0_REG
#define TC9041X_I2C1_REG TC90412_I2C1_REG
#define I2C_TC9041X_IRQ(i) (TC90412_IR_I2C((i)) + TC90412_IRQ_IRC)
#define ENABLE_HW_RESET
#define CCFG_CLKRSTCTR_I2C(x) (1<<(24+(x)))
#define SET_BIT_CLKSTRCTR(bit) tc90412_ccfgptr->clkrstctr |= (bit)
#define CLR_BIT_CLKSTRCTR(bit) tc90412_ccfgptr->clkrstctr &= ~(bit)
#elif defined(CONFIG_TOSHIBA_TC90416)
#define I2C_TC9041X_CH_MAX 3
#define TC9041X_I2C0_REG TC90416_I2C0_REG
#define TC9041X_I2C1_REG TC90416_I2C1_REG
#define TC9041X_I2C2_REG TC90416_I2C2_REG
#define I2C_TC9041X_IRQ(i) (TC90416_IR_I2C((i)) + TC90416_IRQ_IRC)
#define ENABLE_HW_RESET
#define CCFG_CLKRSTCTR_I2C(x) (1<<(24+(x)))
#define SET_BIT_CLKSTRCTR(bit) tc90416_ccfgptr->clkrstctr |= (bit)
#define CLR_BIT_CLKSTRCTR(bit) tc90416_ccfgptr->clkrstctr &= ~(bit)
#else
#define I2C_TC9041X_CH_MAX 3
#define TC9041X_I2C0_REG TC90411_I2C0_REG
#define TC9041X_I2C1_REG TC90411_I2C1_REG
#define TC9041X_I2C2_REG TC90411_I2C2_REG
#define I2C_TC9041X_IRQ(i) (TC90411_IR_I2C((i)) + TC90411_IRQ_IRC)
#endif

#ifdef DEBUG
#define I2CDBG_ERR   0x01
#define I2CDBG_WRN   0x02
#define I2CDBG_FLOW  0x04
static unsigned int i2c_debug = I2CDBG_ERR|I2CDBG_WRN;
//static unsigned int i2c_debug = I2CDBG_ERR|I2CDBG_WRN|I2CDBG_FLOW;
#define I2CDBGPRINT(flag,format) if(i2c_debug & (flag)) printk format;
#define LOCAL
#else
#define i2c_debug	0
#define LOCAL  static
#define I2CDBGPRINT(flag,format)
#endif

#define i2c_outb(dev,off,val)  *(volatile unsigned char*)(((struct i2c_tc9041x_algo_data *)(dev))->baseaddr + (off))=val
#define i2c_outh(dev,off,val)  *(volatile unsigned short*)(((struct i2c_tc9041x_algo_data *)(dev))->baseaddr + (off))=val
#define i2c_outw(dev,off,val)  *(volatile unsigned int*)(((struct i2c_tc9041x_algo_data *)(dev))->baseaddr + (off))=val
#define i2c_inb(dev,off)  (*(volatile unsigned char*)(((struct i2c_tc9041x_algo_data *)(dev))->baseaddr + (off)))
#define i2c_inh(dev,off)  (*(volatile unsigned short*)(((struct i2c_tc9041x_algo_data *)(dev))->baseaddr + (off)))
#define i2c_inw(dev,off)  (*(volatile unsigned int*)(((struct i2c_tc9041x_algo_data *)(dev))->baseaddr + (off)))


LOCAL int i2c_tc9041x_wait_bus_not_busy(struct i2c_adapter *adp);
#if 0
LOCAL int i2c_tc9041x_wait_bus_busy(struct i2c_adapter *adp);
LOCAL int i2c_tc9041x_stop(struct i2c_adapter *adp);
LOCAL int i2c_tc9041x_abort(struct i2c_adapter *adp);
#endif
LOCAL int i2c_tc9041x_check_busy(struct i2c_adapter *adp);
LOCAL int i2c_tc9041x_set_clock(struct i2c_adapter *adp, int val);
LOCAL int i2c_tc9041x_get_clock(struct i2c_adapter *adp);
LOCAL int i2c_tc9041x_init(struct i2c_adapter *adp);
LOCAL int i2c_tc9041x_reset(struct i2c_adapter *adp);
LOCAL int i2c_tc9041x_client_register(struct i2c_client *client);
LOCAL int i2c_tc9041x_client_unregister(struct i2c_client *client);

LOCAL void i2c_tc9041x_subsettrxreg(struct i2c_tc9041x_algo_data *dev);
LOCAL void i2c_tc9041x_subrestorecfg(struct i2c_tc9041x_algo_data *dev);
LOCAL u32 i2c_tc9041x_setconfig(struct i2c_tc9041x_algo_data *dev, i2c_cfg_t *param);
LOCAL int i2c_tc9041x_hw_reset(struct i2c_adapter *adp);
#ifdef WORKAROUND_DELAY_STOP
LOCAL void i2c_tc9041x_delay_stop(struct i2c_tc9041x_algo_data *dev);
#endif

/* forward declaration for algorithm */
LOCAL int i2c_tc9041x_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[], int num);
LOCAL int i2c_tc9041x_ioctl(struct i2c_adapter *adapter, unsigned int cmd, unsigned long arg);
LOCAL u32 i2c_tc9041x_func(struct i2c_adapter *adapter);

LOCAL int oldclocktable[]={1250000,694000,368000,189000,96000,48000,24000};

/* algorithm */
LOCAL struct i2c_algorithm i2c_tc9041x_algorithm = {
      .master_xfer = i2c_tc9041x_xfer,
      .algo_control = i2c_tc9041x_ioctl,
      .functionality = i2c_tc9041x_func,
};

LOCAL int i2c_tc9041x_valid_messages(struct i2c_msg msgs[], int num)
{
	int i;
	if (num < 1 || num > MAX_MESSAGES) {
		I2CDBGPRINT(I2CDBG_ERR,("Invalid number of messages (max=%d, num=%d)\n", MAX_MESSAGES, num));
		return -EINVAL;
	}

	/* check consistency of our messages */
	for (i = 0; i < num; i++) {
		if (&msgs[i] == NULL) {
			I2CDBGPRINT(I2CDBG_ERR,("Msgs[%d] is NULL\n",i));
			return -EINVAL;
		}
		if (msgs[i].buf == NULL) {
			I2CDBGPRINT(I2CDBG_ERR,("msgs[%d].buf is NULL\n",i));
			return -EINVAL;
		}
#if 0
		if (msgs[0].len == 0) {	/* this is SMBUS_QUICK */
			I2CDBGPRINT(I2CDBG_ERR,("msgs[%d].len is zero\n",i));
			return 0;
		}
#endif
	}

	return 1;
}

LOCAL int i2c_tc9041x_read(struct i2c_adapter *adp,struct i2c_msg *pmsg,int lastmsg)
{
	struct i2c_tc9041x_algo_data *dev = (struct i2c_tc9041x_algo_data*)adp->algo_data;
	i2c_trx_buf_t	*trxbuf;
	u32	trxmode;
	u32	trxtimes;
	u32	datasize;
	u8	burstsize;
	u8	lastsize	= 0;
	u8	surplus		= 0;
	u8	subsize		= 0;
	u16	subaddr		= 0x0000;
	u8	slaveaddr;
	u32	rc		= _I2C_OK;
	int     timeout;

	/* Initialization check */
	if ( dev->init_flg == 0 ) {
		I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),_I2C_NG\n",__LINE__,__func__));
		return _I2C_NG;
	}

	/* Parameter check */
	if ( pmsg == NULL ) {
		I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),_I2C_NG_PARAM\n",__LINE__,__func__));
		return _I2C_NG_PARAM;
	}

	/* Check Slave-Address */
	if ( pmsg->addr == 0 ) {
		I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),_I2C_NG_PARAM\n",__LINE__,__func__));
		return _I2C_NG_PARAM;
	}
	slaveaddr = pmsg->addr << 1;

	/* Check data size */
	if ( pmsg->len == 0 ) {
		I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),_I2C_NG_PARAM\n",__LINE__,__func__));
		return _I2C_NG_PARAM;
	}
	if ( pmsg->buf == NULL ) {
		I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),_I2C_NG_PARAM\n",__LINE__,__func__));
		return _I2C_NG_PARAM;
	}
	trxmode	= _I2C_TRX_READ;

	/* Check Sub-Address size */
	if(pmsg->flags & I2C_M_SUBADDR_MASK){
		switch(pmsg->flags & I2C_M_SUBADDR_MASK){
		case I2C_M_SUBADDR1:
			subsize = 1;
			subaddr = pmsg->buf[0];
			break;
		case I2C_M_SUBADDR2:
			subsize = 2;
			subaddr = (pmsg->buf[0] << 8) | pmsg->buf[1];
			break;
		default:
			return _I2C_NG_PARAM;
		}
	}

	datasize = pmsg->len;
	if(datasize <= _I2C_CFG_BURST_DEF){
		burstsize = datasize;	/* Burst size compensation */
	} else {
		burstsize = _I2C_CFG_BURST_DEF;
		lastsize  = datasize % burstsize; /* Surplus calculation */
		if ( lastsize > 0 ) {
			surplus	= 1;
		}
	}
	/* The number of times of transmission */
	trxtimes = (datasize / burstsize);
	if ( trxtimes > _I2C_RW_TIMES_MAX ) {
		I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),_I2C_NG_PARAM\n",__LINE__,__func__));
		return _I2C_NG_PARAM;
	}

	if(!mutex_trylock(&dev->trx_mutex)){
		rc |= _I2C_NG_SEM;
		I2CDBGPRINT(I2CDBG_WRN,("%d,%s(),_I2C_NG_SEM\n",__LINE__,__func__));
	}
	if ( rc != _I2C_OK ) {
		I2CDBGPRINT(I2CDBG_WRN,("%d,%s(),!_I2C_OK,rc=0x%08x\n",__LINE__,__func__,rc));
		return rc;
	}

	dev->ist_data = _I2C_OK; /* Clear Interruption status */

	/* Set Read Parameter */
	dev->trxmode = trxmode;		/* TRXmode : READ */
	trxbuf	= &dev->trx_buf;
	trxbuf->trxtimes = trxtimes; /* The number of times of transmission */
	trxbuf->datasize = datasize;		/* Transmission data size */
	trxbuf->data   = pmsg->buf;	/* Transmission data */
	trxbuf->burstsize = burstsize;	/* Burst size */
	trxbuf->lastsize = lastsize;	/* Last transmission data size */
	trxbuf->surplus = surplus;	/* Existence of a surplus */
	trxbuf->slaveaddr = slaveaddr;	/* Slave-Address */
	trxbuf->subsize	= subsize;	/* Sub-Address size */
	trxbuf->subaddr	= subaddr;	/* Sub-Address */
	trxbuf->nostop	= !lastmsg; /* Stop */
	trxbuf->nostart	= 0; /* Start */
#ifdef WORKAROUND_DELAY_STOP
	if (pmsg->flags & I2C_M_DELAY_STOP){
		trxbuf->nostop = 1;
	}
#endif

	/* Transmission start preparation and a transmission start */
	i2c_tc9041x_subsettrxreg(dev);

	/* Waiting for the completion of transmitting */
	timeout = wait_event_timeout(dev->trx_wq,dev->trxmode == _I2C_TRX_STANDBY,dev->cfg_data.timeout*HZ);
	if(dev->trxmode!=_I2C_TRX_STANDBY && timeout==0){
		rc |= _I2C_NG_SEM;
		I2CDBGPRINT(I2CDBG_WRN,("%d,%s(),timeout,trxmode=0x%08x\n",__LINE__,__func__,dev->trxmode));
		I2CDBGPRINT(I2CDBG_WRN,("%d,%s(),reg_ist=0x%08x\n",__LINE__,__func__,i2c_inw(dev,_I2CIST)));
	}

#ifdef WORKAROUND_DELAY_STOP
	if ((pmsg->flags & I2C_M_DELAY_STOP) && lastmsg){
		i2c_tc9041x_delay_stop(dev);
	}
#endif
	/* Saving the result of transmission */
	rc |= dev->ist_data;		/* Get Interruption status */
	dev->ist_data = _I2C_OK;	/* Clear Interruption status */

	/* Restore I2CBUF register */
	i2c_tc9041x_subrestorecfg(dev);

	mutex_unlock(&dev->trx_mutex);

	/* Error check */
	if ( rc != _I2C_OK ) {
		i2c_cfg_t *cfg = &dev->cfg_data;
		I2CDBGPRINT(I2CDBG_WRN,("%d,%s(),!_I2C_OK,tc=0x%08x\n",__LINE__,__func__,rc));
		/* Reset I2C Controler */
		rc |= i2c_tc9041x_hw_reset(adp);
		rc |= i2c_tc9041x_setconfig(dev, cfg);
		rc |= i2c_tc9041x_reset(adp);
	}

	return rc;
}

LOCAL int i2c_tc9041x_write(struct i2c_adapter *adp,struct i2c_msg *pmsg,int lastmsg)
{
	struct i2c_tc9041x_algo_data *dev = (struct i2c_tc9041x_algo_data*)adp->algo_data;
	i2c_trx_buf_t	*trxbuf;
	unsigned long	trxmode;
	unsigned long	trxtimes	= 1;
	unsigned long	datasize	= 0;
	unsigned char	burstsize	= 1;
	unsigned char	lastsize	= 0;
	unsigned char	surplus		= 0;
	unsigned char	subsize		= 0;
	unsigned short	subaddr		= 0x0000;
	unsigned char	slaveaddr;
	unsigned long	rc		= _I2C_OK;
	int     timeout;

	/* Initialization check */
	if ( dev->init_flg == 0 ) {
		I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),_I2C_NG\n",__LINE__,__func__));
		return _I2C_NG;
	}

	/*******************/
	/* Parameter check */
	/*******************/
	if ( pmsg == NULL ) {
		I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),_I2C_NG_PARAM\n",__LINE__,__func__));
		return _I2C_NG_PARAM;
	}

	/* Check Slave-Address */
	if ( pmsg->addr == 0 ) {
		I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),_I2C_NG_PARAM\n",__LINE__,__func__));
		return _I2C_NG_PARAM;
	}
	slaveaddr = pmsg->addr << 1;

	/* Check data size */
	if ( pmsg->len == 0 ) { /* Only for a slave address transmission */
		trxmode = _I2C_TRX_SLAVE;
	} else {
		if ( pmsg->buf == NULL ) {
			I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),_I2C_NG_PARAM\n",__LINE__,__func__));
			return _I2C_NG_PARAM;
		}
		/* Check Sub-Address size */
		if(pmsg->flags & I2C_M_SUBADDR_MASK){
			switch(pmsg->flags & I2C_M_SUBADDR_MASK){
			case I2C_M_SUBADDR1:
				subsize = 1;
				subaddr = pmsg->buf[0];
				break;
			case I2C_M_SUBADDR2:
				subsize = 2;
				subaddr = (pmsg->buf[0] << 8) | pmsg->buf[1];
				break;
			default:
				return _I2C_NG_PARAM;
			}
		}
		trxmode	= _I2C_TRX_WRITE;
		datasize = pmsg->len - subsize;
		if(datasize == 0){
			return _I2C_NG_PARAM;
		}
		if(datasize <= _I2C_CFG_BURST_DEF){
			burstsize = datasize;
		}else{
			burstsize = _I2C_CFG_BURST_DEF;
			/* Surplus calculation */
			lastsize = datasize % burstsize;
			if ( lastsize > 0 ) {
				surplus	= 1;
			}
		}

		/* The number of times of transmission */
		trxtimes = (datasize / burstsize);
		if ( trxtimes > _I2C_RW_TIMES_MAX ) {
			I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),_I2C_NG_PARAM\n",__LINE__,__func__));
			return _I2C_NG_PARAM;
		}
	}

	if(!mutex_trylock(&dev->trx_mutex)){
		rc |= _I2C_NG_SEM;
		I2CDBGPRINT(I2CDBG_WRN,("%d,%s(),_I2C_NG_SEM\n",__LINE__,__func__));
	}
	if ( rc != _I2C_OK ) {
		return rc;
	}

	dev->ist_data = _I2C_OK; /* Clear Interruption status */

	/***********************/
	/* Set Write Parameter */
	/***********************/
	dev->trxmode = trxmode; /* TRXmode : SLAVE / WRITE */
	trxbuf = &dev->trx_buf;
	trxbuf->trxtimes = trxtimes; /* The number of times of transmission */
	trxbuf->datasize = datasize; /* Transmission data size */
	trxbuf->data = pmsg->buf+subsize; /* Transmission data */
	trxbuf->burstsize = burstsize; /* Burst size */
	trxbuf->lastsize = lastsize; /* Last transmission data size */
	trxbuf->surplus	= surplus; /* Existence of a surplus */
	trxbuf->slaveaddr = slaveaddr; /* Slave-Address */
	trxbuf->subsize	= subsize; /* Sub-Address size */
	trxbuf->subaddr	= subaddr; /* Sub-Address */
	trxbuf->nostop	= !lastmsg; /* Stop */
	trxbuf->nostart	= 0; /* Start */
#ifdef WORKAROUND_DELAY_STOP
	if (pmsg->flags & I2C_M_DELAY_STOP){
		trxbuf->nostop = 1;
	}
#endif

	/* Transmission start preparation and a transmission start */
	i2c_tc9041x_subsettrxreg(dev);

	/* Waiting for the completion of transmitting */
	timeout = wait_event_timeout(dev->trx_wq,dev->trxmode == _I2C_TRX_STANDBY,dev->cfg_data.timeout*HZ);
	if(dev->trxmode!=_I2C_TRX_STANDBY && timeout==0){
		rc |= _I2C_NG_SEM;
		I2CDBGPRINT(I2CDBG_WRN,("%d,%s(),timeout,trxmode=0x%08x\n",__LINE__,__func__,dev->trxmode));
	}

#ifdef WORKAROUND_DELAY_STOP
	if ((pmsg->flags & I2C_M_DELAY_STOP) && lastmsg){
		i2c_tc9041x_delay_stop(dev);
	}
#endif
	/* Saving the result of transmission */
	rc |= dev->ist_data;
	/* Clear Interruption status */
	dev->ist_data = _I2C_OK;

	/* Restore I2CBUF register */
	i2c_tc9041x_subrestorecfg(dev);

	mutex_unlock(&dev->trx_mutex);
	/* Error check */
	if ( rc != _I2C_OK ) {
		i2c_cfg_t *cfg = &dev->cfg_data;
		/* Reset I2C Controler */
		rc |= i2c_tc9041x_hw_reset(adp);
		rc |= i2c_tc9041x_setconfig(dev, cfg);
		rc |= i2c_tc9041x_reset(adp);
	}
	return rc;

}

LOCAL int i2c_tc9041x_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[], int num)
{
	struct i2c_msg *pmsg = NULL;
	int i,lastmsg;
	int ret;

	I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),msgnum=%d\n",__LINE__,__func__,num));
	ret = i2c_tc9041x_valid_messages(msgs, num);
	if (ret <= 0) {
		return ret;
	}

	ret = i2c_tc9041x_wait_bus_not_busy(i2c_adap);
	if (ret) {
		I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),bus busy timeout\n",__LINE__,__func__));
		return -ETIMEDOUT;
	}

	for (i = 0,lastmsg = 0; ret >= 0 && i < num; i++) {
		if(i==num-1)
			lastmsg = 1;
		pmsg = &msgs[i];

		if (pmsg->flags & I2C_M_RD){
			ret = i2c_tc9041x_read(i2c_adap,pmsg,lastmsg);
		}else{
			ret = i2c_tc9041x_write(i2c_adap,pmsg,lastmsg);
		}
	}
	if (ret != _I2C_OK) {
		if ((ret &_I2C_NG_NACK)==_I2C_NG_NACK) {
			I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),NACK\n",__LINE__,__func__));
			return -EREMOTEIO;
		} else if ((ret & _I2C_NG_SEM)==_I2C_NG_SEM) {
			I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),Timeout\n",__LINE__,__func__));
			return -ETIMEDOUT;
		} else {
			I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),ALOST\n",__LINE__,__func__));
			return -EIO;
		}
	}
	if (ret < 0) {
		I2CDBGPRINT(I2CDBG_WRN,("%d,%s(),ret=0x%08x\n",__LINE__,__func__,ret));
		return ret;
	} else {
		return i;
	}
}

LOCAL int i2c_tc9041x_ioctl(struct i2c_adapter *adapter, unsigned int cmd, unsigned long arg)
{
	int i, clock, status;
	switch (cmd) {
	case I2C_TC9041X_RESET:
		i2c_tc9041x_hw_reset(adapter);
		i2c_tc9041x_init(adapter);
		i2c_tc9041x_reset(adapter);
		break;

	case I2C_TC9041X_CHECK_BUSY:
		if (arg == 0)
			return -EINVAL;
		status = i2c_tc9041x_check_busy(adapter);
		if (copy_to_user((int __user *) arg, &status, sizeof(int)))
			return -EFAULT;
		break;

	case I2C_TC9041X_PORT_MODE:
		return -EINVAL;

	case I2C_TC9041X_SET_CLOCK:
		if(arg>=0 && arg<7){
			return i2c_tc9041x_set_clock(adapter,oldclocktable[arg]);
		}else{
			return -EINVAL;
		}
	case I2C_TC9041X_GET_CLOCK:
		/* bit0-7: value, bit8: 1=Fast,0=Low */
		if (arg == 0)
			return -EINVAL;
		clock = i2c_tc9041x_get_clock(adapter);
		for(i=7;i>=0;i--){
			if(clock<(oldclocktable[i]+1000))
				break;
		}
		clock = i;
		if (copy_to_user((int __user *) arg, &clock, sizeof(int)))
			return -EFAULT;
		break;

	case I2C_TC9041X_SET_CLOCK2:
		/* bit0-7: value, bit8: 1=Fast,0=Low */
		return i2c_tc9041x_set_clock(adapter, arg);

	case I2C_TC9041X_GET_CLOCK2:
		/* bit0-7: value, bit8: 1=Fast,0=Low */
		if (arg == 0)
			return -EINVAL;
		clock = i2c_tc9041x_get_clock(adapter);
		if (copy_to_user((int __user *) arg, &clock, sizeof(int)))
			return -EFAULT;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

/* functionality */
LOCAL u32 i2c_tc9041x_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C;
}

/* adapter */
LOCAL struct i2c_tc9041x_algo_data i2c_tc9041x_data[] = {
	{
		.channel = 0,
		.baseaddr = TC9041X_I2C0_REG,
	},
#if I2C_TC9041X_MAX_CH >= 2
	{
		.channel = 1,
		.baseaddr = TC9041X_I2C1_REG,
	},
#endif
#if I2C_TC9041X_MAX_CH >= 3
	{
		.channel = 2,
		.baseaddr = TC9041X_I2C2_REG,
	},
#endif
};

LOCAL u32 i2c_tc9041x_setconfig(struct i2c_tc9041x_algo_data *dev, i2c_cfg_t *cfg)
{
	u16		reg_buf;
	u8		reg_com;
	u8		reg_pclk;
	u32		rc = _I2C_OK;

	/* Initialization check */
	if ( dev->init_flg == 0 ) {
		return _I2C_NG;
	}

	/* Parameter check */
	if ( (cfg == NULL) ||
	     (cfg->burst < _I2C_CFG_BURST_MIN) ||
	     (cfg->burst > _I2C_CFG_BURST_MAX) ||
	     (cfg->nackretry > _I2C_CFG_RETRY_MAX) ) {
		return _I2C_NG_PARAM;
	}

	/* Read I2C register */
	reg_buf = i2c_inw(dev,_I2CBUF);
	reg_com = i2c_inw(dev,_I2CCOM);

	/* Burst(FIFO) size */
	reg_buf	&= _I2C_I2CBUF_BURST_MASK;
	reg_buf	|= ((cfg->burst - 1) << _I2C_I2CBUF_BURST_SFT);

	/* I2C system clock */
	reg_pclk = cfg->sysclk;

	/* I2C clock select */
	if ( cfg->clksel == _I2C_CLKSEL_FAST ) {
		reg_com |= _I2C_I2CCOM_CLKSEL;
	} else {
		reg_com	&= ~(_I2C_I2CCOM_CLKSEL);
	}

	/* Sub-address increment mode */
	switch ( cfg->sbamode ) {
	case _I2C_SBAINC_NOP:
		reg_buf	|= _I2C_I2CBUF_SADMOD;
		break;
	case _I2C_SBAINC_BST:
		reg_buf	&= ~(_I2C_I2CBUF_SADMOD);
		reg_buf	|= _I2C_I2CBUF_SADCMD;
		break;
	case _I2C_SBAINC_BYT:
		reg_buf	&= ~(_I2C_I2CBUF_SADMOD | _I2C_I2CBUF_SADCMD);
		break;
	default:
		break;
	}

	/* NACK detect */
	if ( cfg->nackdet == _I2C_NACKDET_DIS ) {
		reg_buf	|= _I2C_I2CBUF_NACKDET_DIS;
	} else {
		reg_buf	&= ~(_I2C_I2CBUF_NACKDET_DIS);
	}

	/* Write I2C register */
	i2c_outw(dev,_I2CBUF,reg_buf);
	i2c_outw(dev,_I2CCOM,reg_com);
	i2c_outw(dev,_I2CPCK,reg_pclk);

	return rc;
}

LOCAL int i2c_tc9041x_init(struct i2c_adapter *adp)
{
	struct i2c_tc9041x_algo_data *dev = (struct i2c_tc9041x_algo_data*)adp->algo_data;
	i2c_cfg_t *cfg = &dev->cfg_data;
	u32		rc;

	if ( dev->init_flg == 0 ) {
		dev->init_flg = 1;
	}

	/* Set the I2C device configuration */
	/* Setting API default parameters */
	cfg->burst	= _I2C_CFG_BURST_DEF;
	cfg->sysclk	= _I2C_CFG_SYSCLK_DEF;
	cfg->clksel	= _I2C_CFG_CLKSEL_DEF;
	cfg->sbamode	= _I2C_CFG_SBAINC_DEF;
	cfg->nackdet	= _I2C_CFG_NACKDET_DEF;
#ifdef ENABLE_NACK_RETRY
	cfg->nackretry	= _I2C_CFG_RETRY_DEF;
#else
	cfg->nackretry	= 0;
#endif
	cfg->timeout	= _I2C_CFG_TIME_DEF;

	rc = i2c_tc9041x_setconfig(dev, cfg);

	return rc;
}

/* ckeck busy status */
LOCAL int i2c_tc9041x_check_busy(struct i2c_adapter *adp)
{
	struct i2c_tc9041x_algo_data *dev = (struct i2c_tc9041x_algo_data*)adp->algo_data;
	return (i2c_inw(dev,_I2CCOM) & _I2C_I2CCOM_STATUS)? 1 : 0;
}

/*
  set bus clock frequency

  input:
    val ---   0:low mode(clock), 1:high mode(clock*4), other:clock

      sysclk = TXX9_IMCLK/(clock*32)-1
      clock = TXX9_IMCLK/(sysclk+1)/32
      high clock = clock*4
*/
LOCAL int i2c_tc9041x_set_clock(struct i2c_adapter *adp, int val)
{
	struct i2c_tc9041x_algo_data *dev = (struct i2c_tc9041x_algo_data*)adp->algo_data;
	i2c_cfg_t *cfg = &dev->cfg_data;
	int sysclk;

#ifndef WORKAROUND_HIGHSPEED_MODE
	if (val==0 || val==1){
		u8 reg_com;
		/* I2C clock select */
		if(val==0)
			cfg->clksel = _I2C_CLKSEL_LOW;
		else
			cfg->clksel = _I2C_CLKSEL_FAST;
		reg_com = i2c_inw(dev,_I2CCOM);
		if ( cfg->clksel == _I2C_CLKSEL_FAST ) {
			reg_com |= _I2C_I2CCOM_CLKSEL;
		} else {
			reg_com	&= ~(_I2C_I2CCOM_CLKSEL);
		}
		i2c_outw(dev,_I2CCOM,reg_com);
	}else
#endif
	if ((val < (TXX9_IMCLK/256/32/2)) || ((TXX9_IMCLK/32*2) < val) ){
		printk("i2c_tc9041x_set_clock: range must be %d<val<%d\n",TXX9_IMCLK/256/32/2,TXX9_IMCLK/32*2);
		return -EINVAL;
	}else{
		sysclk = (TXX9_IMCLK+(val*32/2))/(val*32)-1;
		cfg->sysclk = sysclk;
		i2c_outw(dev,_I2CPCK,cfg->sysclk);
	}
	return 0;
}

/* get bus clock frequency */
LOCAL int i2c_tc9041x_get_clock(struct i2c_adapter *adp)
{
	struct i2c_tc9041x_algo_data *dev = (struct i2c_tc9041x_algo_data*)adp->algo_data;
	int clock;
	clock = TXX9_IMCLK/(dev->cfg_data.sysclk+1)/32;
	if(dev->cfg_data.clksel==_I2C_CLKSEL_FAST)
		clock = clock*4;
	return clock;
}

LOCAL int i2c_tc9041x_wait_bus_not_busy(struct i2c_adapter *adp)
{
	struct i2c_tc9041x_algo_data *dev = (struct i2c_tc9041x_algo_data*)adp->algo_data;
	int timeout = DEF_TIMEOUT * 100;

	while (timeout-- && (i2c_inw(dev,_I2CCOM) & _I2C_I2CCOM_STATUS)) {
		udelay(1);	/* wait for 5 us */
	}

	return (timeout <= 0);
}

#if 0
LOCAL int i2c_tc9041x_stop(struct i2c_adapter *adp)
{
	/* XXX */
}

LOCAL int i2c_tc9041x_abort(struct i2c_adapter *adp)
{
	/* XXX */
}

LOCAL int i2c_tc9041x_wait_bus_busy(struct i2c_adapter *adp)
{
	struct i2c_tc9041x_algo_data *dev = (struct i2c_tc9041x_algo_data*)adp->algo_data;
	int timeout = DEF_TIMEOUT * 100;

	while (timeout-- && !(i2c_inw(dev,_I2CCOM) & _I2C_I2CCOM_STATUS)) {
		udelay(1);	/* wait for 5 us */
	}

	return (timeout <= 0);
}
#endif

LOCAL void i2c_tc9041x_subread(struct i2c_tc9041x_algo_data *dev)
{
	i2c_trx_buf_t	*trxbuf;
	int		i;

	trxbuf = &dev->trx_buf;

	/* Acquisition of 1 burst size Read-Data */
	for ( i = 0; i < trxbuf->burstsize; i++ ) {
		*trxbuf->data++ = i2c_inw(dev,_I2CDT) & 0xff;
	}
	trxbuf->trxtimes--;
}

LOCAL void i2c_tc9041x_subwrite(struct i2c_tc9041x_algo_data *dev)
{
	i2c_trx_buf_t	*trxbuf;
	int		i;
	trxbuf = &dev->trx_buf;

	/* Setup of 1 burst size Write-Data */
	for ( i = 0; i < trxbuf->burstsize; i++ ) {
		i2c_outw(dev,_I2CDT,*trxbuf->data++);	/* Set Write-Data */
	}
	trxbuf->trxtimes--;
}

LOCAL void i2c_tc9041x_subsettrxreg(struct i2c_tc9041x_algo_data *dev)
{
	u32		trxmode;
	i2c_trx_buf_t	*trxbuf;
	u32		reg_buf;
	u32		reg_sub;
	u32     	reg_sla;
	u32		reg_com;
	u32		reg_ien;
	u32		reg_ist;
	u32		reg_byl;
	u32		reg_byu;
	u32		reg_retry;
	int		nostop=0;
	int		nostart=0;

	/* Read I2C register */
	reg_buf	= i2c_inw(dev,_I2CBUF);
	reg_com	= i2c_inw(dev,_I2CCOM);
	reg_ien	= i2c_inw(dev,_I2CIEN);
	reg_ist	= i2c_inw(dev,_I2CIST);

	trxmode	= dev->trxmode;
	trxbuf	= &dev->trx_buf;

	/*******************/
	/* I2CBUF register */
	/*******************/
	/* Backup I2CBUF register */
	trxbuf->reg_buf_bak = (reg_buf & ~(_I2C_I2CBUF_I2CEN));

	/* Data transmission	: Disable */
	reg_buf	&= ~(_I2C_I2CBUF_I2CEN);

	/* ADVmode */
	if ( trxbuf->nostop != 0 || trxbuf->subsize == 0) {
		if ((trxmode != _I2C_TRX_RESET) &&
		    !((trxmode==_I2C_TRX_WRITE) && (trxbuf->burstsize==1))){
			reg_buf	|= _I2C_I2CBUF_ADVMD; /* adv mode */
		}
	}else{
		reg_buf	&= ~(_I2C_I2CBUF_ADVMD); /* normal */
	}

	/* Burst size	: (burstsize)-1 */
	reg_buf	&= ~_I2C_I2CBUF_BURST_MASK;
	reg_buf	|= ((trxbuf->burstsize - 1) << _I2C_I2CBUF_BURST_SFT);

	/* subsize				: 2 / 1or0 */
	if ( trxbuf->subsize == 2 ) {
		reg_buf	|= _I2C_I2CBUF_SAD2BYT;							/* subsize		: 2 */
	} else {
		reg_buf	&= ~(_I2C_I2CBUF_SAD2BYT);						/* subsize		: 1or0 */
	}

	/* HEADON			: Enable / Disable */
	if ( trxbuf->subsize != 0 ) {	/* subsize	: >0 */
		reg_buf	|= _I2C_I2CBUF_HEADON;	/* HEADON	: Enable */
	} else {
		reg_buf	&= ~(_I2C_I2CBUF_HEADON); /* HEADON	: Disable */
	}

	/* NACK detect			: Enable / Disable */
	if ( trxmode == _I2C_TRX_RESET ) {							/* TRXmode		: START_STOP(RESET) */
		reg_buf	|= _I2C_I2CBUF_NACKDET_DIS;						/* NACK detect	: Disable */
	}

	/* I2CSUB register */
	reg_sub		= trxbuf->subaddr;

	/* I2CSLA register */
	reg_sla		= (trxbuf->slaveaddr & _I2C_I2CSLA_MASK);
	if ( trxmode == _I2C_TRX_READ ) {	/* TRXmode : READ */
		reg_sla	|= _I2C_I2CSLA_READ;	/* Slave-Address LSB = '1' */
	}

	/* I2CBYT_U, I2CBYT_L register */
	/* Number-of-times setup of transmission */
#ifndef SEQUENCILAL_TRANSFER
	reg_byl		= 0;
	reg_byu		= 0;
#else
	reg_byl		= (trxbuf->trxtimes - 1) &  _I2C_I2CBYT2_BYTE_MASK;
	reg_byu		= (trxbuf->trxtimes - 1) >> _I2C_I2CBYT1_BYTE_SFT;
#endif

	/* check nostop */
	if ( trxbuf->nostop==1 ||
	     ((trxbuf->subsize==0) && (trxbuf->trxtimes+trxbuf->surplus)>1)){
		nostop = 1;
	}
	/* check nostart */
	if ( trxbuf->nostart==1 ){
		nostart = 1;
	}
	/* I2CCOM register */
	/* Note! Negative logic... 0:Enable, 1:Disable */
	reg_com		|= _I2C_I2CCOM_ALL_DIS;
	if (trxmode == _I2C_TRX_RESET) {
		reg_com	&= ~_I2C_I2CCOM_START;	/* Enable : [START] */
		reg_com	&= ~_I2C_I2CCOM_STOP;	/* Enable : [STOP] */
	} else if (trxbuf->subsize == 0){
		u32 bs,bm,be,sr;
		sr = _I2C_I2CCOM_ALL_DIS;
		if ( trxbuf->datasize != 0 ) {
			sr &= ~(_I2C_I2CCOM_DATA);
			reg_com	&= ~(_I2C_I2CCOM_DATA);
		}
		bs = bm = be = sr;
		if ( nostart == 0 ){
			bs &= ~(_I2C_I2CCOM_START|_I2C_I2CCOM_SLA);
			sr &= ~(_I2C_I2CCOM_START|_I2C_I2CCOM_SLA);
			reg_com	&= ~(_I2C_I2CCOM_START|_I2C_I2CCOM_SLA);
		}
		if ( nostop == 0 ){
			sr &= ~(_I2C_I2CCOM_STOP);
			be &= ~(_I2C_I2CCOM_STOP);
			reg_com &= ~(_I2C_I2CCOM_STOP);
		}
		if ( trxmode == _I2C_TRX_READ ) {	/* TRXmode : READ */
			i2c_outw(dev,_I2CCOMA_SR,sr);
			i2c_outw(dev,_I2CCOMA_BRS,bs);
			i2c_outw(dev,_I2CCOMA_BRM,bm);
			i2c_outw(dev,_I2CCOMA_BRE,be);
			I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(), sr=0x%x\n",__LINE__,__func__,sr));
			I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),brs=0x%x\n",__LINE__,__func__,bs));
			I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),brm=0x%x\n",__LINE__,__func__,bm));
			I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),bre=0x%x\n",__LINE__,__func__,be));
		}else{
			i2c_outw(dev,_I2CCOMA_BWS,bs);
			i2c_outw(dev,_I2CCOMA_BWM,bm);
			i2c_outw(dev,_I2CCOMA_BWE,be);
			I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),bws=0x%x\n",__LINE__,__func__,bs));
			I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),bwm=0x%x\n",__LINE__,__func__,bm));
			I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),bwe=0x%x\n",__LINE__,__func__,be));
		}
	}else{
		reg_com	&= ~(_I2C_I2CCOM_SAD);	/* Enable : [SUB] */
		reg_com	&= ~(_I2C_I2CCOM_SLA);	/* Enable : [SLAVE] */
		if ( trxbuf->datasize != 0 ) {
			reg_com	&= ~(_I2C_I2CCOM_DATA);	/* Enable : [DATA] */
		}
		if ( trxbuf->nostart == 0 ){
			reg_com	&= ~_I2C_I2CCOM_START;	/* Enable : [START] */
		}
		if ( nostop == 0 ){
			reg_com	&= ~_I2C_I2CCOM_STOP;	/* Enable : [STOP] */
		}
	}

	/* I2CIEN register */
	reg_ien	&= _I2C_I2CIST_ALL_DIS;
	reg_ien	|= (_I2C_I2CIST_ALERI | _I2C_I2CIST_NAERI |	/* Interrupt Enable	: ALerr, NACKerr,		*/
		    _I2C_I2CIST_UDRI  | _I2C_I2CIST_OVRI);	/* UNDERerr, OVERerr,	*/
	if ( nostop == 0 ) {
		reg_ien |= _I2C_I2CIST_TENDI;	/* STOPcondition */
	}else{
		reg_ien |= _I2C_I2CIST_BSTFI;	/* Burst Comp */
	}

	/* I2CIST register */
	reg_ist	|= _I2C_I2CIST_ALL_CLR;	/* ALL interrupt status clear */

	I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),reg_buf=0x%x\n",__LINE__,__func__,reg_buf));
	I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),reg_sla=0x%x\n",__LINE__,__func__,reg_sla));
	I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),reg_sub=0x%x\n",__LINE__,__func__,reg_sub));
	I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),reg_byl=0x%x\n",__LINE__,__func__,reg_byl));
	I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),reg_byu=0x%x\n",__LINE__,__func__,reg_byu));
	I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),reg_com=0x%x\n",__LINE__,__func__,reg_com));
	I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),reg_ien=0x%x\n",__LINE__,__func__,reg_ien));
	/* Nack retry */
	if((trxmode == _I2C_TRX_WRITE) &&
	   (reg_buf & _I2C_I2CBUF_HEADON) &&
	   !(reg_com & _I2C_I2CCOM_START))
		reg_retry = dev->cfg_data.nackretry;
	else
		reg_retry = 0;

#if 0
	printk("%s",(reg_buf & _I2C_I2CBUF_ADVMD)? "ADV:":"STD:");
	printk("%c",(trxmode == _I2C_TRX_READ)? 'R':'W');
	printk(",%s",(reg_buf & _I2C_I2CBUF_HEADON)? "Hon":"Hof");
	if(!(reg_com & _I2C_I2CCOM_START))	printk(",S");
	if(!(reg_com & _I2C_I2CCOM_SLA))	printk(",SLA");
	if(!(reg_com & _I2C_I2CCOM_SAD))	printk(",SUB");
	if(!(reg_com & _I2C_I2CCOM_DATA))	printk(",D[%d]",((reg_buf & _I2C_I2CBUF_BURST_MASK)>>_I2C_I2CBUF_BURST_SFT) +1);
	if(!(reg_com & _I2C_I2CCOM_STOP))	printk(",P");
	printk("\n");
#endif

	/* Set I2C registers */
	i2c_outw(dev,_I2CBUF,reg_buf);
	i2c_outw(dev,_I2CSLA,reg_sla);
	i2c_outw(dev,_I2CSUB,reg_sub);
	i2c_outw(dev,_I2CBYT1,reg_byl);
	i2c_outw(dev,_I2CBYT2,reg_byu);
	i2c_outw(dev,_I2CCOM,reg_com);
	i2c_outw(dev,_I2CIST,reg_ist);
	i2c_outw(dev,_I2CRETRY,reg_retry);
	I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),reg_istn=0x%x\n",__LINE__,__func__,i2c_inw(dev,_I2CIST)));

	reg_buf	|= _I2C_I2CBUF_I2CEN;
	i2c_outw(dev,_I2CBUF,reg_buf);	/* TRX Start */

	if ( trxmode == _I2C_TRX_WRITE ) {
		i2c_tc9041x_subwrite(dev);	/* Write-Data is set to FIFO */
	}
	I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),reg_istn=0x%x\n",__LINE__,__func__,i2c_inw(dev,_I2CIST)));

	i2c_outw(dev,_I2CIEN,reg_ien);
	/* Data transmission	: Enable */
}

#ifdef WORKAROUND_DELAY_STOP
LOCAL void i2c_tc9041x_delay_stop(struct i2c_tc9041x_algo_data *dev)
{
	i2c_trx_buf_t	*trxbuf = &dev->trx_buf;
	u32	reg_sla, reg_com, reg_buf, reg_ist;
	int loops;
	
	/* Set I2C registers */
	msleep(1);
	reg_sla = (trxbuf->slaveaddr & _I2C_I2CSLA_MASK); /* SlaveAddr|Write */
	i2c_outw(dev,_I2CSLA,reg_sla);
	reg_com	= i2c_inw(dev,_I2CCOM) | _I2C_I2CCOM_ALL_DIS;
	reg_com	&= ~_I2C_I2CCOM_STOP;	/* enable stop */
	i2c_outw(dev,_I2CCOM,reg_com);
	reg_buf = _I2C_I2CBUF_I2CEN; /* burst=0,advmd=0,i2cen=1 */
	i2c_outw(dev,_I2CBUF,reg_buf);
	loops=0;
	reg_ist=i2c_inw(dev,_I2CIST);
	while (!(reg_ist & _I2C_I2CIST_ENDI)){
		msleep(1);
		reg_ist=i2c_inw(dev,_I2CIST);
		if(loops++>1000){
			I2CDBGPRINT(I2CDBG_WRN,("%d,%s(),delay stop timeout.\n",__LINE__,__func__);)
			break;
		}
	}
	if(loops<1000){
		I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),delay stop success.\n",__LINE__,__func__);)
	}
	i2c_outw(dev,_I2CIST,reg_ist);
}
#endif

LOCAL void i2c_tc9041x_subrestorecfg(struct i2c_tc9041x_algo_data *dev)
{
	/* Restore I2CBUF register */
	i2c_outw(dev,_I2CBUF,dev->trx_buf.reg_buf_bak);
}

LOCAL int i2c_tc9041x_hw_reset(struct i2c_adapter *adp)
{
#ifdef ENABLE_HW_RESET
	struct i2c_tc9041x_algo_data *dev = (struct i2c_tc9041x_algo_data*)adp->algo_data;
	unsigned long reset_bit;
	unsigned long flags;
	int channel = dev->channel;

	if (channel < 0 || channel > (I2C_TC9041X_CH_MAX - 1))
 		return _I2C_NG;

	reset_bit = CCFG_CLKRSTCTR_I2C(dev->channel);

	local_irq_save(flags);
	SET_BIT_CLKSTRCTR(reset_bit);
	udelay(6);
	CLR_BIT_CLKSTRCTR(reset_bit);
	local_irq_restore(flags);

#endif
	return _I2C_OK;
}

LOCAL int i2c_tc9041x_reset(struct i2c_adapter *adp)
{
	u32		rc = _I2C_OK;
#ifndef ENABLE_HW_RESET
	struct i2c_tc9041x_algo_data *dev = (struct i2c_tc9041x_algo_data*)adp->algo_data;
	i2c_trx_buf_t	*trxbuf;
	u16		reg_buf;
	u8		reg_err;
	int timeout;

	I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),Reset I2C Controller Unit ch.%d\n",__LINE__,__func__,dev->channel));

	/* Initialization check */
	if ( dev->init_flg == 0 ) {
		I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),_I2C_NG\n",__LINE__,__func__));
		return _I2C_NG;
	}

	if(!mutex_trylock(&dev->trx_mutex)){
		rc |= _I2C_NG_SEM;
		I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),_I2C_NG_SEM\n",__LINE__,__func__));
	}
	if ( rc != _I2C_OK ) {
		return rc;
	}

	dev->ist_data = _I2C_OK;	/* Clear Interruption status */

	/* Set Reset Parameter */
	dev->trxmode = _I2C_TRX_RESET;	/* TRXmode : RESET */
	trxbuf = &dev->trx_buf;
	trxbuf->trxtimes = 1;	/* The number of times of transmission */
	trxbuf->datasize = 0;	/* Transmission data size */
	trxbuf->data = NULL;	/* Transmission data */
	trxbuf->burstsize = 1;	/* Burst size */
	trxbuf->lastsize = 0;	/* Last transmission data size */
	trxbuf->surplus	= 0;	/* Existence of a surplus */
	trxbuf->slaveaddr = 0x00; /* Slave-Address */
	trxbuf->subsize	= 0;	/* Sub-Address size */
	trxbuf->subaddr	= 0x0000; /* Sub-Address */
	trxbuf->nostop	= 0; /* Stop */
	trxbuf->nostart	= 0; /* Start */

	/* Transmission start preparation and a transmission start */
	i2c_tc9041x_subsettrxreg(dev);

	/* Waiting for the completion of transmitting */
	timeout = wait_event_timeout(dev->trx_wq,dev->trxmode == _I2C_TRX_STANDBY,dev->cfg_data.timeout*HZ);
	if(dev->trxmode!=_I2C_TRX_STANDBY && timeout==0){
		rc |= _I2C_NG_SEM;
		I2CDBGPRINT(I2CDBG_WRN,("%d,%s(),timeout,trxmode=0x%08x\n",__LINE__,__func__,dev->trxmode));
	}

	/* Saving the result of transmission */
	rc |= dev->ist_data;
	/* Clear Interruption status */
	dev->ist_data = _I2C_OK;

	/* Check a transmission end */
	reg_buf = i2c_inw(dev,_I2CBUF);
	if ( (reg_buf & _I2C_I2CBUF_I2CEN) != 0 ) {
		rc |= _I2C_NG;
	}

	/* FIFO reset */
	reg_err  = i2c_inw(dev,_I2CERR);
	reg_err |= _I2C_I2CERR_FFCL;
	i2c_outw(dev,_I2CERR,reg_err);

	/* Restore I2CBUF register */
	i2c_tc9041x_subrestorecfg(dev);

	I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),Reset I2C Controller Unit ch.%d done\n",__LINE__,__func__,dev->channel));

	mutex_unlock(&dev->trx_mutex);
#endif

	return rc;
}


LOCAL irqreturn_t i2c_tc9041x_handler(int this_irq, void *dev_id)
{
	struct i2c_tc9041x_algo_data *dev = (struct i2c_tc9041x_algo_data*)dev_id;
	u16		reg_buf;
	u16		reg_ien;
	u16		reg_ist;
	u32		trxmode;
	i2c_trx_buf_t	*trxbuf;
	int		endflag = 0;

	reg_buf	= i2c_inw(dev,_I2CBUF);
	reg_ien	= i2c_inw(dev,_I2CIEN);
	reg_ist	= i2c_inw(dev,_I2CIST);

	I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),reg_buf=0x%08x\n",__LINE__,__func__,reg_buf));
	I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),reg_ist=0x%08x\n",__LINE__,__func__,reg_ist));
	I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),reg_ien=0x%08x\n",__LINE__,__func__,reg_ien));

	trxmode	= dev->trxmode;
	trxbuf	= &dev->trx_buf;

	/* Error interruption status check */
	if ( reg_ist & _I2C_I2CIST_ALERI ) {
		/* Arbitration lost error */
		dev->ist_data |= _I2C_NG_ALOST;
		I2CDBGPRINT(I2CDBG_WRN,("%d,%s(),A-Lost\n",__LINE__,__func__));
	}
	if ( reg_ist & _I2C_I2CIST_NAERI ) {
		/* NACK error */
		dev->ist_data |= _I2C_NG_NACK;
		I2CDBGPRINT(I2CDBG_WRN,("%d,%s(),NACK\n",__LINE__,__func__));
	}
	if ( reg_ist & _I2C_I2CIST_UDRI ) {
		/* Read-buffer underrun error */
		dev->ist_data |= _I2C_NG_UNDER;
		I2CDBGPRINT(I2CDBG_WRN,("%d,%s(),Underrun\n",__LINE__,__func__));
	}
	if ( reg_ist & _I2C_I2CIST_OVRI ) {
		/* Write-buffer overrun error */
		dev->ist_data |= _I2C_NG_OVER;
		I2CDBGPRINT(I2CDBG_WRN,("%d,%s(),Overrun\n",__LINE__,__func__));
	}

	/* Interruption status check */
	if ( dev->ist_data == _I2C_OK ) {
		switch ( trxmode ) {
		/* TRXmode : START_STOP(RESET) */
		case _I2C_TRX_RESET:
			endflag = 1;
			break;

		/* TRXmode : SLAVE */
		case _I2C_TRX_SLAVE:
			endflag = 1;
			break;

		/* TRXmode : WRITE */
		case _I2C_TRX_WRITE:
			if(trxbuf->subsize == 0)
				trxbuf->nostart = 1;
			if ( trxbuf->trxtimes == 0 ) {
				if ( trxbuf->surplus == 1){
					/* Burst size compensation */
					trxbuf->trxtimes = 1;
					trxbuf->surplus = 0;
#ifndef SEQUENCILAL_TRANSFER
					trxbuf->subaddr = trxbuf->subaddr + trxbuf->burstsize;
#else
					trxbuf->subaddr = trxbuf->subaddr + trxbuf->datasize - trxbuf->lastsize;
#endif
					trxbuf->burstsize = trxbuf->lastsize;
					i2c_tc9041x_subsettrxreg(dev);
				}else{
					endflag = 1;
				}
			} else {
#ifndef SEQUENCILAL_TRANSFER
				trxbuf->subaddr = trxbuf->subaddr + trxbuf->burstsize;
				i2c_tc9041x_subsettrxreg(dev);
#else
				/* set Write-Data into FIFO */
				i2c_tc9041x_subwrite(dev);
				reg_ist = _I2C_I2CIST_BSTFI;
				/* Clear Burst transfer complete status */
				i2c_outw(dev,_I2CIST,reg_ist);
#endif
			}
			break;

		/* TRXmode : READ */
		case _I2C_TRX_READ:
			/* acquire Read-Data from FIFO */
			i2c_tc9041x_subread(dev);
			if(trxbuf->subsize == 0)
				trxbuf->nostart = 1;
			if ( trxbuf->trxtimes == 0 ) {
				/* In the case of surplus data */
				if ( trxbuf->surplus == 1){
					/* Burst size compensation */
					trxbuf->trxtimes = 1;
					trxbuf->surplus = 0;
#ifndef SEQUENCILAL_TRANSFER
					trxbuf->subaddr = trxbuf->subaddr + trxbuf->burstsize;
#else
					trxbuf->subaddr = trxbuf->datasize - trxbuf->lastsize;
#endif
					trxbuf->burstsize = trxbuf->lastsize;
					i2c_tc9041x_subsettrxreg(dev);
				}else{
					endflag = 1;
				}
			}
#ifndef SEQUENCILAL_TRANSFER
			else{
				trxbuf->subaddr = trxbuf->subaddr + trxbuf->burstsize;
				i2c_tc9041x_subsettrxreg(dev);
			}
#else
			/* Clear Burst transfer complete status */
			i2c_outw(dev,_I2CIST,_I2C_I2CIST_BSTFI);
#endif
			break;

		/* TRXmode : STANDBY or Unknown */
		default:
			dev->ist_data |= _I2C_NG;
			endflag = 1;
		}
	}else{
		endflag = 1;
	}

	/* Transmission end */
	if ( endflag == 1 ) {
		/* TRXmode : STANDBY */
		//dev->trxmode = _I2C_TRX_STANDBY;
		/* Data transmission Disable */
		reg_buf &= ~(_I2C_I2CBUF_I2CEN);
		/* ALL interrupt status clear */
		reg_ist |= _I2C_I2CIST_ALL_CLR;
		reg_ien &= _I2C_I2CIST_ALL_DIS;							/* ALL interrupt disable */
		i2c_outw(dev,_I2CBUF,reg_buf);
		i2c_outw(dev,_I2CIST,reg_ist);
		i2c_outw(dev,_I2CIEN,reg_ien);
		while(i2c_inw(dev,_I2CIST)&0x4){
			i2c_inw(dev,_I2CDT);
			i2c_outw(dev,_I2CIST,_I2C_I2CIST_ALL_CLR);
		}

		I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),reg_buf=%08x\n",__LINE__,__func__,reg_buf));
		I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),reg_ist=%08x\n",__LINE__,__func__,reg_ist));
		I2CDBGPRINT(I2CDBG_FLOW,("%d,%s(),reg_ien=%08x\n",__LINE__,__func__,reg_ien));

		/* TRXmode : STANDBY */
		dev->trxmode = _I2C_TRX_STANDBY;
		wake_up(&dev->trx_wq);
	}

	return IRQ_HANDLED;
}

LOCAL int i2c_tc9041x_resource_init(struct i2c_adapter *adp)
{
	struct i2c_tc9041x_algo_data *dev = adp->algo_data;

	init_waitqueue_head(&dev->trx_wq);
	mutex_init(&dev->trx_mutex);
	i2c_outw(dev,_I2CIEN,0);

	if (request_irq(dev->irq, &i2c_tc9041x_handler, IRQF_DISABLED, adp->name, dev) < 0) {
		dev->irq = 0;
		I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),Failed to register I2C irq i2c-%i\n",__LINE__,__func__,dev->channel));
		return -ENODEV;
	}
	printk(KERN_INFO "I2C: register IRQ %d\n",dev->irq);
	return 0;
}

LOCAL void i2c_tc9041x_resource_release(struct i2c_adapter *adp)
{
	struct i2c_tc9041x_algo_data *dev = adp->algo_data;

	if (dev->irq > 0) {
		disable_irq(dev->irq);
		free_irq(dev->irq, dev);
		dev->irq = 0;
	}
}

LOCAL int i2c_tc9041x_client_register(struct i2c_client *client)
{
	return 0;
}

LOCAL int i2c_tc9041x_client_unregister(struct i2c_client *client)
{
	return 0;
}

#ifdef CONFIG_PM
LOCAL int i2c_tc9041x_suspend(struct platform_device *plat_dev, pm_message_t state)
{
	struct i2c_adapter *adp =  platform_get_drvdata(plat_dev);
	struct i2c_tc9041x_algo_data *dev = (struct i2c_tc9041x_algo_data*)adp->algo_data;
	disable_irq(dev->irq);
	return 0;
}
LOCAL int i2c_tc9041x_resume(struct platform_device *plat_dev)
{
	struct i2c_adapter *adp = platform_get_drvdata(plat_dev);
	struct i2c_tc9041x_algo_data *dev = (struct i2c_tc9041x_algo_data*)adp->algo_data;
	i2c_tc9041x_hw_reset(adp);
	enable_irq(dev->irq);
	i2c_outw(dev,_I2CIEN,0);
	i2c_tc9041x_init(adp);
	i2c_tc9041x_reset(adp);
	return 0;
}
#endif

LOCAL int i2c_tc9041x_add_bus(struct i2c_adapter *i2c_adap)
{
	int ret;
	printk(KERN_DEBUG "I2C: Adding %s.\n", i2c_adap->name);

	/* register new adapter to i2c module... */
	ret = i2c_add_adapter(i2c_adap);
	if (ret)
		return ret;

	i2c_tc9041x_hw_reset(i2c_adap);
	i2c_tc9041x_init(i2c_adap);
	i2c_tc9041x_reset(i2c_adap);
	return 0;
}

LOCAL int i2c_tc9041x_del_bus(struct i2c_adapter *i2c_adap)
{
	int res;
	if ((res = i2c_del_adapter(i2c_adap)) < 0)
		return res;

	printk(KERN_INFO "I2C: Removing %s.\n", i2c_adap->name);

	return 0;
}

static int __init i2c_tc9041x_probe(struct platform_device *plat_dev)
{
	int ret;
	int ch = plat_dev->id;
	struct i2c_adapter *adp = kzalloc(sizeof(*adp), GFP_KERNEL);
	if (!adp)
		return -ENOMEM;
	sprintf(adp->name, "I2C-CH%d", ch);
	adp->algo = &i2c_tc9041x_algorithm;
	adp->algo_data = &i2c_tc9041x_data[ch];
	adp->client_register = i2c_tc9041x_client_register;
	adp->client_unregister = i2c_tc9041x_client_unregister;
	adp->retries = 2;
	i2c_tc9041x_data[ch].irq = I2C_TC9041X_IRQ(ch);
	ret = i2c_tc9041x_resource_init(adp);
	if (ret) {
		kfree(adp);
		return ret;
	}
	adp->owner = THIS_MODULE;
	adp->dev.parent = &plat_dev->dev;
	ret = i2c_tc9041x_add_bus(adp);
	if (ret) {
		i2c_tc9041x_resource_release(adp);
		I2CDBGPRINT(I2CDBG_ERR,("%d,%s(),Failed to add bus ch=%d\n",
					__LINE__, __func__, ch));
		printk(KERN_INFO "I2C: Failed to add bus ch=%d\n", ch);
		kfree(adp);
		return ret;
	}
	platform_set_drvdata(plat_dev, adp);
	printk(KERN_INFO "I2C: added bus ch=%d\n", ch);
	return 0;
}

static int i2c_tc9041x_remove(struct platform_device *plat_dev)
{
	struct i2c_adapter *adp = platform_get_drvdata(plat_dev);
	i2c_tc9041x_del_bus(adp);
	i2c_tc9041x_resource_release(adp);
	kfree(adp);
	return 0;
}

static struct platform_driver i2c_tc9041x_driver = {
	.probe = i2c_tc9041x_probe,
	.remove = i2c_tc9041x_remove,
#ifdef CONFIG_PM
	.suspend = i2c_tc9041x_suspend,
	.resume = i2c_tc9041x_resume,
#endif
	.driver = {
		.name = "i2c-tc9041x",
		.owner = THIS_MODULE,
	}
};

LOCAL struct platform_device *i2c_tc9041x_devs[I2C_TC9041X_MAX_CH];

LOCAL int __init i2c_adap_tc9041x_init(void)
{
	int i, ret;

	if (IS_OPBD493X)
		return -ENODEV;

	printk("I2C TC9041X Driver, v%s\n",THIS_DRV_VERSION);

	for (i = 0; i < I2C_TC9041X_MAX_CH; i++) {
		struct platform_device *pdev;
		pdev = platform_device_register_simple("i2c-tc9041x", i,
						       NULL, 0);
		if (IS_ERR(pdev)) {
			while (--i >= 0) {
				platform_device_unregister(i2c_tc9041x_devs[i]);
				i2c_tc9041x_devs[i] = NULL;
			}
			return PTR_ERR(pdev);
		}
		i2c_tc9041x_devs[i] = pdev;
	}
	ret = platform_driver_register(&i2c_tc9041x_driver);
	if (ret) {
		for (i = 0; i < I2C_TC9041X_MAX_CH; i++) {
			platform_device_unregister(i2c_tc9041x_devs[i]);
			i2c_tc9041x_devs[i] = NULL;
		}
	}
	return ret;
}

LOCAL void i2c_adap_tc9041x_exit(void)
{
	int i;

	platform_driver_unregister(&i2c_tc9041x_driver);
	for (i = 0; i < I2C_TC9041X_MAX_CH; i++)
		platform_device_unregister(i2c_tc9041x_devs[i]);
	printk(KERN_INFO "I2C: Successfully removed bus\n");
}

module_init(i2c_adap_tc9041x_init);
module_exit(i2c_adap_tc9041x_exit);
MODULE_LICENSE("GPL");
