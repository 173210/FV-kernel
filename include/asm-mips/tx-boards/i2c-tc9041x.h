/*
 *  i2c-tc9041x.h
 *
 *  (C) Copyright TOSHIBA CORPORATION 2005-2007
 *  All Rights Reserved.
 */
#ifndef _I2C_TC9041X_H_
#define _I2C_TC9041X_H_

#ifdef CONFIG_TOSHIBA_TC90412
#define I2C_TC9041X_MAX_CH 2
#else
#define I2C_TC9041X_MAX_CH 3
#endif

#define I2C_BASE TC9041X_I2C0_REG
#define _I2CDT		0x00
#define _I2CBUF		0x04
#define _I2CSUB		0x08
#define _I2CSLA		0x0C
#define _I2CCOM		0x10
#define _I2CIEN		0x14
#define _I2CIST		0x18
#define _I2CERR		0x1C
#define _I2CBYT1	0x20
#define _I2CBYT2	0x24
#define _I2CPCK		0x2C
#define _I2CCOMA_BWS	0x34
#define _I2CCOMA_BWM	0x38
#define _I2CCOMA_BWE	0x3C
#define _I2CCOMA_HD	0x40
#define _I2CCOMA_SR	0x44
#define _I2CCOMA_BRS	0x48
#define _I2CCOMA_BRM	0x4C
#define _I2CCOMA_BRE	0x50
#define _I2CRETRY	0x60

/* I2C transmission data */
typedef struct tag_i2c_trx_buf_t {
	u32 trxtimes;	/* The number of times of transmission */
	u32 datasize;	/* Transmission data size */
	u8 *data;	/* Transmission data */
	u8 burstsize;/* Burst size */
	u8 lastsize;	/* Last transmission data size */
	u8 surplus;	/* Existence of a surplus */
	u8 slaveaddr;/* Slave-Address */
	u8 subsize;	/* Sub-Address size */
	u16 subaddr;	/* Sub-Address */
	u16 reg_buf_bak;	/* Backup & Restore I2CBUF register */
	u8 nostop;  /* not send stop if 1 */
	u8 nostart;  /* not send start if 1 */
} i2c_trx_buf_t;

typedef struct tag_i2c_cfg_t {
	u8	burst;		/* Burst(FIFO) size */
	u8	sysclk;		/* I2C system clock */
	u8	clksel;		/* I2C clock select */
	u8	sbamode;	/* Sub-address increment mode */
	u8	nackdet;	/* NACK detect */
	u16	nackretry;	/* NACK auto retry times */
	u32	timeout;	/* Semaphore timeout interval */
} i2c_cfg_t;

struct i2c_tc9041x_algo_data {
	int channel;
	unsigned long baseaddr;
	int irq;
	i2c_trx_buf_t trx_buf;
	i2c_cfg_t cfg_data;
	int init_flg;
	int ist_data;
	int trxmode;
	int nackretry;
	struct mutex trx_mutex;
	wait_queue_head_t trx_wq;
};

#define DEF_TIMEOUT             4
#define BUS_ERROR               (-EREMOTEIO)
#define ACK_DELAY               0       /* time to delay before checking bus error */
#define MAX_MESSAGES            65536   /* maximum number of messages to send */

#define I2C_SLEEP_TIMEOUT       (5*(HZ/100)) /* time to sleep for on i2c transactions */
#define I2C_RETRY               (-2000) /* an error has occurred retry transmit */
#define I2C_ACKERR              (-2001) /* ack error */
#define I2C_ALERR               (-2002) /* AL error */
#define I2C_OTHERERR            (-2003) /* Other error */

#define I2C_TC9041X_RESET		730
#define I2C_TC9041X_CHECK_BUSY	731
#define I2C_TC9041X_PORT_MODE	732
#define I2C_TC9041X_SET_CLOCK	733
#define I2C_TC9041X_GET_CLOCK	734
#define I2C_TC9041X_SET_CLOCK2	735
#define I2C_TC9041X_GET_CLOCK2	736

#define	I2C_TC9041X_CLK_1250K = 0x00,            /* I2C Clock = 1250   KHz */
#define	I2C_TC9041X_CLK_694K  = 0x01,            /* I2C Clock =  694   KHz */
#define	I2C_TC9041X_CLK_368K  = 0x02,            /* I2C Clock =  368   KHz */
#define	I2C_TC9041X_CLK_189K  = 0x03,            /* I2C Clock =  189   KHz */
#define	I2C_TC9041X_CLK_96K   = 0x04,            /* I2C Clock =   96.2 KHz */
#define	I2C_TC9041X_CLK_48K   = 0x05,            /* I2C Clock =   48.4 KHz */
#define	I2C_TC9041X_CLK_24K   = 0x06,            /* I2C Clock =   24.3 KHz */

#define I2C_TRANSMIT		1
#define I2C_RECEIVE		0
#define I2C_TC9041X_SLAVE_ADDR      0x1    /* slave TC9041X unit address */
#define I2C_ICR_INIT            (ICR_BEIE | ICR_IRFIE | ICR_ITEIE | ICR_GCD | ICR_SCLE) /* ICR initialization value */
/* ICR initialize bit values
*
*  15. FM       0 (100 Khz operation)
*  14. UR       0 (No unit reset)
*  13. SADIE    0 (Disables the unit from interrupting on slave addresses
*                                       matching its slave address)
*  12. ALDIE    0 (Disables the unit from interrupt when it loses arbitration
*                                       in master mode)
*  11. SSDIE    0 (Disables interrupts from a slave stop detected, in slave mode)
*  10. BEIE     1 (Enable interrupts from detected bus errors, no ACK sent)
*  9.  IRFIE    1 (Enable interrupts from full buffer received)
*  8.  ITEIE    1 (Enables the I2C unit to interrupt when transmit buffer empty)
*  7.  GCD      1 (Disables i2c unit response to general call messages as a slave)
*  6.  IUE      0 (Disable unit until we change settings)
*  5.  SCLE     1 (Enables the i2c clock output for master mode (drives SCL)
*  4.  MA       0 (Only send stop with the ICR stop bit)
*  3.  TB       0 (We are not transmitting a byte initially)
*  2.  ACKNAK   0 (Send an ACK after the unit receives a byte)
*  1.  STOP     0 (Do not send a STOP)
*  0.  START    0 (Do not send a START)
*
*/

#define I2C_ISR_INIT            0x7FF  /* status register init */
/* I2C status register init values
 *
 * 10. BED      1 (Clear bus error detected)
 * 9.  SAD      1 (Clear slave address detected)
 * 7.  IRF      1 (Clear IDBR Receive Full)
 * 6.  ITE      1 (Clear IDBR Transmit Empty)
 * 5.  ALD      1 (Clear Arbitration Loss Detected)
 * 4.  SSD      1 (Clear Slave Stop Detected)
 */

/* Register Bit Definitions */
	/* I2C Buffer Control Register */
#define _I2C_I2CBUF_NACKDET_DIS         0x8000
#define _I2C_I2CBUF_BURST_MASK       0x7C00
#define	_I2C_I2CBUF_BURST_SFT		10	/* Burst(FIFO) size */
#define _I2C_I2CBUF_ADVMD            0x0200
#define _I2C_I2CBUF_SADCMD           0x0100
#define _I2C_I2CBUF_I2CEN            0x0080
#define _I2C_I2CBUF_HEADON           0x0040
#define _I2C_I2CBUF_DMAON            0x0020
#define _I2C_I2CBUF_SADMOD           0x0010
#define _I2C_I2CBUF_SAD2BYT          0x0008
	/* I2C Sub Address Register */
	/* I2C Slave Address Register */
#define _I2C_I2CSLA_WRITE            0x0000
#define _I2C_I2CSLA_READ             0x0001
#define _I2C_I2CSLA_MASK             0x00FE
	/* I2C Command Register */
#define _I2C_I2CCOM_STATUS           0x0080
#define _I2C_I2CCOM_CLKSEL           0x0040
#define _I2C_I2CCOM_START            0x0020
#define _I2C_I2CCOM_SLA              0x0010
#define _I2C_I2CCOM_SAD              0x0008
#define _I2C_I2CCOM_DATA             0x0004
#define _I2C_I2CCOM_STOP             0x0002
#define _I2C_I2CCOM_ENDFLAG          0x0001
	/* I2C Interrupt Enable Register */
#define _I2C_I2CIEN_BSTFIEN          0x8000
#define _I2C_I2CIEN_ALEREN           0x0400
#define _I2C_I2CIEN_NAEREN           0x0200
#define _I2C_I2CIEN_ENDIEN           0x0080
#define _I2C_I2CIEN_TENDIEN          0x0040
#define _I2C_I2CIEN_SADRIEN          0x0020
#define _I2C_I2CIEN_ADRIEN           0x0010
#define _I2C_I2CIEN_UDRIEN           0x0008
#define _I2C_I2CIEN_RDBFIEN          0x0004
#define _I2C_I2CIEN_OVRIEN           0x0002
#define _I2C_I2CIEN_WRBFIEN          0x0001
	/* I2C Interrupt Status Register */
#define _I2C_I2CIST_BSTFI            0x8000
#define _I2C_I2CIST_ALERI            0x0400
#define _I2C_I2CIST_NAERI            0x0200
#define _I2C_I2CIST_STATEIDLE        0x0100
#define _I2C_I2CIST_ENDI             0x0080
#define _I2C_I2CIST_TENDI            0x0040
#define _I2C_I2CIST_SADRI            0x0020
#define _I2C_I2CIST_ADRI             0x0010
#define _I2C_I2CIST_UDRI             0x0008
#define _I2C_I2CIST_RDBFI            0x0004
#define _I2C_I2CIST_OVRI             0x0002
#define _I2C_I2CIST_WRBFI            0x0001
#define	_I2C_I2CIST_ALL_DIS	     0x7900    /* ALL interrupt disable */
#define	_I2C_I2CIST_ALL_CLR	     0x86FF    /* ALL interrupt status clear */

	/* I2C Error Status Register */
#define _I2C_I2CERR_FFCL             0x0080
#define _I2C_I2CERR_ALERR            0x0002
#define _I2C_I2CERR_NAERR            0x0001
	/* I2C Transfer Byte 1 Register */
#define _I2C_I2CBYT_UFIN             0x0080
#define _I2C_I2CBYT1_BYTE_MASK       0x0003
#define	_I2C_I2CBYT1_BYTE_SFT		  8
	/* I2C Transfer Byte 2 Register */
#define _I2C_I2CBYT2_BYTE_MASK       0x00FF
	/* I2C Clock Register */
#define _I2C_I2CPCK_I2CPCK_MASK      0x00FF
#define _I2C_CLK_94K_378K   0x15       /* I2C Clock LO=94KHz,HI=378KHz */
#define	_I2C_I2CPCK_VAL		     _I2C_CLK_94K_378K	/* System clock */

	/* I2C_COMM */
#define	_I2C_I2CCOM_ALL_DIS	       0x3E	/* ALL Command Disable */
#define	_I2C_I2CCOM_ALL_ENA	       0xC1	/* ALL Command Enable */
	/* I2C BWS Register */
#define _I2C_I2CCOMA_BWS_START       0x0020
#define _I2C_I2CCOMA_BWS_SLA         0x0010
#define _I2C_I2CCOMA_BWS_SAD         0x0008
#define _I2C_I2CCOMA_BWS_DATA        0x0004
#define _I2C_I2CCOMA_BWS_STOP        0x0002
	/* I2C BWM Register */
#define _I2C_I2CCOMA_BWM_START       0x0020
#define _I2C_I2CCOMA_BWM_SLA         0x0010
#define _I2C_I2CCOMA_BWM_SAD         0x0008
#define _I2C_I2CCOMA_BWM_DATA        0x0004
#define _I2C_I2CCOMA_BWM_STOP        0x0002
	/* I2C BWE Register */
#define _I2C_I2CCOMA_BWE_START       0x0020
#define _I2C_I2CCOMA_BWE_SLA         0x0010
#define _I2C_I2CCOMA_BWE_SAD         0x0008
#define _I2C_I2CCOMA_BWE_DATA        0x0004
#define _I2C_I2CCOMA_BWE_STOP        0x0002
	/* I2C SR Register */
#define _I2C_I2CCOMA_SR_START        0x0020
#define _I2C_I2CCOMA_SR_SLA          0x0010
#define _I2C_I2CCOMA_SR_SAD          0x0008
#define _I2C_I2CCOMA_SR_DATA         0x0004
#define _I2C_I2CCOMA_SR_STOP         0x0002
	/* I2C BRS Register */
#define _I2C_I2CCOMA_BRS_START       0x0020
#define _I2C_I2CCOMA_BRS_SLA         0x0010
#define _I2C_I2CCOMA_BRS_SAD         0x0008
#define _I2C_I2CCOMA_BRS_DATA        0x0004
#define _I2C_I2CCOMA_BRS_STOP        0x0002
	/* I2C BRM Register */
#define _I2C_I2CCOMA_BRM_START       0x0020
#define _I2C_I2CCOMA_BRM_SLA         0x0010
#define _I2C_I2CCOMA_BRM_SAD         0x0008
#define _I2C_I2CCOMA_BRM_DATA        0x0004
#define	_I2C_I2CCOMA_BRM_STOP        0x0002
	/* I2C BRE Register */
#define _I2C_I2CCOMA_BRE_START       0x0020
#define _I2C_I2CCOMA_BRE_SLA         0x0010
#define _I2C_I2CCOMA_BRE_SAD         0x0008
#define _I2C_I2CCOMA_BRE_DATA        0x0004
#define _I2C_I2CCOMA_BRE_STOP        0x0002
	/* I2C Retry Register */
#define _I2C_I2CRETRY_RETRY_MASK     0x0FFF

/* I2C transmission mode  */
#define	_I2C_TRX_STANDBY			0x00
#define	_I2C_TRX_RESET				0x01
#define	_I2C_TRX_SLAVE				0x02
#define	_I2C_TRX_WRITE				0x04
#define	_I2C_TRX_READ				0x08
#define	_I2C_TRX_ADV_W				0x10
#define	_I2C_TRX_ADV_WR				0x20
#define	_I2C_TRX_ADV_R				0x40
#define	_I2C_TRX_ADV_SR				0x80

/* I2C All function Return Values */
#define	_I2C_OK		0x00000000
#define	_I2C_NG		0x80000001
#define	_I2C_NG_PARAM	0x80000002
#define	_I2C_NG_BUSY	0x80000004
#define	_I2C_NG_SEM	0x80000008
#define	_I2C_NG_ALOST	0x80010000
#define	_I2C_NG_NACK	0x80020000
#define	_I2C_NG_UNDER	0x80040000
#define	_I2C_NG_OVER	0x80080000

/* for I2C_SetConfig(), I2C_Init() */
#define	_I2C_CFG_BURST_MIN		1
#define	_I2C_CFG_BURST_MAX		32
#define	_I2C_CFG_BURST_DEF		_I2C_CFG_BURST_MAX
#define	_I2C_CFG_SYSCLK_DEF		_I2C_I2CPCK_VAL
#define	_I2C_CFG_CLKSEL_DEF		_I2C_CLKSEL_LOW
#define	_I2C_CFG_SBAINC_DEF		_I2C_SBAINC_BYT
#define	_I2C_CFG_NACKDET_DEF		_I2C_NACKDET_ENA
#define	_I2C_CFG_RETRY_MAX		_I2C_I2CRETRY_MAX
#define	_I2C_CFG_RETRY_DEF		_I2C_CFG_RETRY_MAX
#define	_I2C_CFG_TIME_DEF		1

/* for I2C_Read(), I2C_Write() */
#define	_I2C_RW_TIMES_MAX		1024

/* for I2C_AdvRead(), I2C_AdvWrite() */
#define	_I2C_ADV_SBA_MAX		32

/* Clock Defines */
#define _I2C_NO_ACK_CLK 0x00 << 4      /* I2C Clock Off */
#define _I2C_W_ACK_CLK  0x01 << 4      /* I2C Clock On  */

#define _I2C_8_BITS     0x00 << 5      /* I2C Data Size 8 data bits */
#define _I2C_1_BITS     0x01 << 5      /* I2C Data Size 1 data bits */
#define _I2C_2_BITS     0x02 << 5      /* I2C Data Size 2 data bits */
#define _I2C_3_BITS     0x03 << 5      /* I2C Data Size 3 data bits */
#define _I2C_4_BITS     0x04 << 5      /* I2C Data Size 4 data bits */
#define _I2C_5_BITS     0x05 << 5      /* I2C Data Size 5 data bits */
#define _I2C_6_BITS     0x06 << 5      /* I2C Data Size 6 data bits */
#define _I2C_7_BITS     0x07 << 5       /* I2C Data Size 7 data bits */

/* default value */
#define I2C_DEF_CTL_0_VAL       (_I2C_8_BITS | _I2C_W_ACK_CLK )
#define I2C_DEF_CTL_0_NO_ACK    (_I2C_8_BITS | _I2C_NO_ACK_CLK)

/* I2CRETRY register */
#define	_I2C_I2CRETRY_MAX	4095	/* Max NACK retry times */
#define	_I2C_I2CRETRY_MASK	0xF000

/* clock select */
#define _I2C_CLKSEL_LOW   0
#define _I2C_CLKSEL_FAST  1

/* Sub-address increment mode */
#define _I2C_SBAINC_NOP   0
#define _I2C_SBAINC_BST   1
#define _I2C_SBAINC_BYT   2

/* NACK detect */
#define _I2C_NACKDET_ENA  0
#define _I2C_NACKDET_DIS  1

/*
 * extened flags of struct i2c_msg in i2c.h
 *  used flag is 0x7810.
 */
#define I2C_M_SUBADDR0		0x0000
#define I2C_M_SUBADDR1		0x0100
#define I2C_M_SUBADDR2		0x0200
#define I2C_M_SUBADDR3		0x0300
#define I2C_M_SUBADDR_MASK	0x0300

#define I2C_M_DELAY_STOP	0x0080

#endif /*_I2C_TC9041X_H_ */
