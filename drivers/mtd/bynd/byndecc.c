/*
 * (C) Copyright TOSHIBA CORPORATION 2009-2010
 * All Rights Reserved.
 */

#include "bynd.h"

/******************************************************************************
ecctable.c
Making CP0-CP5 code table of ECC
March 1996
TOSHIBA Corp.
******************************************************************************/
/*CP0-CP5 code table */
static unsigned char ecctable[256] =
    { 0x00, 0x55, 0x56, 0x03, 0x59, 0x0C, 0x0F, 0x5A, 0x5A, 0x0F, 0x0C,
	0x59, 0x03, 0x56, 0x55, 0x00, 0x65, 0x30, 0x33, 0x66, 0x3C, 0x69,
	0x6A, 0x3F, 0x3F, 0x6A, 0x69, 0x3C, 0x66, 0x33, 0x30, 0x65, 0x66,
	0x33, 0x30, 0x65, 0x3F, 0x6A, 0x69, 0x3C, 0x3C, 0x69, 0x6A, 0x3F,
	0x65, 0x30, 0x33, 0x66, 0x03, 0x56, 0x55, 0x00, 0x5A, 0x0F, 0x0C,
	0x59, 0x59, 0x0C, 0x0F, 0x5A, 0x00, 0x55, 0x56, 0x03, 0x69, 0x3C,
	0x3F, 0x6A, 0x30, 0x65, 0x66, 0x33, 0x33, 0x66, 0x65, 0x30, 0x6A,
	0x3F, 0x3C, 0x69, 0x0C, 0x59, 0x5A, 0x0F, 0x55, 0x00, 0x03, 0x56,
	0x56, 0x03, 0x00, 0x55, 0x0F, 0x5A, 0x59, 0x0C, 0x0F, 0x5A, 0x59,
	0x0C, 0x56, 0x03, 0x00, 0x55, 0x55, 0x00, 0x03, 0x56, 0x0C, 0x59,
	0x5A, 0x0F, 0x6A, 0x3F, 0x3C, 0x69, 0x33, 0x66, 0x65, 0x30, 0x30,
	0x65, 0x66, 0x33, 0x69, 0x3C, 0x3F, 0x6A, 0x6A, 0x3F, 0x3C, 0x69,
	0x33, 0x66, 0x65, 0x30, 0x30, 0x65, 0x66, 0x33, 0x69, 0x3C, 0x3F,
	0x6A, 0x0F, 0x5A, 0x59, 0x0C, 0x56, 0x03, 0x00, 0x55, 0x55, 0x00,
	0x03, 0x56, 0x0C, 0x59, 0x5A, 0x0F, 0x0C, 0x59, 0x5A, 0x0F, 0x55,
	0x00, 0x03, 0x56, 0x56, 0x03, 0x00, 0x55, 0x0F, 0x5A, 0x59, 0x0C,
	0x69, 0x3C, 0x3F, 0x6A, 0x30, 0x65, 0x66, 0x33, 0x33, 0x66, 0x65,
	0x30, 0x6A, 0x3F, 0x3C, 0x69, 0x03, 0x56, 0x55, 0x00, 0x5A, 0x0F,
	0x0C, 0x59, 0x59, 0x0C, 0x0F, 0x5A, 0x00, 0x55, 0x56, 0x03, 0x66,
	0x33, 0x30, 0x65, 0x3F, 0x6A, 0x69, 0x3C, 0x3C, 0x69, 0x6A, 0x3F,
	0x65, 0x30, 0x33, 0x66, 0x65, 0x30, 0x33, 0x66, 0x3C, 0x69, 0x6A,
	0x3F, 0x3F, 0x6A, 0x69, 0x3C, 0x66, 0x33, 0x30, 0x65, 0x00, 0x55,
	0x56, 0x03, 0x59, 0x0C, 0x0F, 0x5A, 0x5A, 0x0F, 0x0C, 0x59, 0x03,
	0x56, 0x55, 0x00
};

/******************************************************************************
ecccor.c
Correcting by ECC code
March 1996
TOSHIBA Corp.
******************************************************************************/

#define BIT7 0x80
#define BIT6 0x40
#define BIT5 0x20
#define BIT4 0x10
#define BIT3 0x08
#define BIT2 0x04
#define BIT1 0x02
#define BIT0 0x01
#define BIT1BIT0 0x03
#define BIT15 0x00008000L
#define BIT23 0x00800000L
#define MASK_CPS 0x3f

struct ecc_for_256_t {
	unsigned char lp_low;
	unsigned char lp_high;
	unsigned char cp;
};

/*
Transfer result
LP14,12,10,...&LP15,13,11,...->LP15,14,13,...&LP7,6,5,..
*/

static void __trans_result(unsigned char reg2,	/*LP14,LP12,LP10,... */
			   unsigned char reg3,	/*LP15,LP13,LP11,... */
			   unsigned char *ecc1,	/*LP15,LP14,LP13,... */
			   unsigned char *ecc2	/*LP07,LP06,LP05,... */
   )
{
	unsigned char a;	/*Working for reg2,reg3 */
	unsigned char b;	/*Working for ecc1,ecc2 */
	unsigned char i;	/*For counting */

	/* if */
	a = BIT7;
	b = BIT7;		/*80h=10000000b */
	*ecc1 = *ecc2 = 0;	/*Clear ecc1,ecc2 */
	for (i = 0; i < 4; ++i) {
		if ((reg3 & a) != 0)
			*ecc1 |= b;	/*LP15,13,11,9 ->ecc1 */
		b = b >> 1;	/*Right shift */
		if ((reg2 & a) != 0)
			*ecc1 |= b;	/*LP14,12,10,8 ->ecc1 */
		b = b >> 1;	/*Right shift */
		a = a >> 1;	/*Right shift */
	}

	b = BIT7;		/*80h=10000000b */
	for (i = 0; i < 4; ++i) {
		if ((reg3 & a) != 0)
			*ecc2 |= b;	/*LP7,5,3,1 ->ecc2 */
		b = b >> 1;	/*Right shift */
		if ((reg2 & a) != 0)
			*ecc2 |= b;	/*LP6,4,2,0 ->ecc2 */
		b = b >> 1;	/*Right shift */
		a = a >> 1;	/*Right shift */
	}
}

/*
Calculating ECC
data[ 0-255] ->ecc1,ecc2,ecc3 using CP0-CP5 code table[ 0-255]
*/

static void __calculate_ecc(const unsigned char *table,
			    const unsigned char *const data,
			    int data_size, unsigned char *ecc1,
			    unsigned char *ecc2,
			    unsigned char *ecc3
    )
{
	/* !!! if data size == 8 , don't care !!!! */

	unsigned int i;		/*For counting */
	unsigned char a;	/*Working for table */
	unsigned char reg1;	/*D-all,CP5,CP4,CP3,... */
	unsigned char reg2;	/*LP14,LP12,L10,... */
	unsigned char reg3;	/*LP15,LP13,L11,... */
	reg1 = reg2 = reg3 = 0;	/*Clear parameter */
	for (i = 0; i < data_size; ++i) {
		a = table[data[i]];	/*Get CP0-CP5 code from table */
		reg1 ^= (a & MASK_CPS);	/*XOR with a */
		if ((a & BIT6) != 0) {	/*If D_all(all bit XOR)=1 */
			/*XOR with counter */
			reg3 ^= (unsigned char)i;
			/* XOR with inv.of counter */
			reg2 ^= ~((unsigned char)i);
		}
	}

/*Trans LP14,12,10,...&LP15,13,11,...->LP15,14,13,...&LP7,6,5,..*/
	__trans_result(reg2, reg3, ecc1, ecc2);

	*ecc1 = ~(*ecc1);
	*ecc2 = ~(*ecc2);	/*Inv.ecc2 &ecc3 */
	*ecc3 = ((~reg1) << 2) | BIT1BIT0;	/*Make TEL format */
}

static int calc_ecc(const unsigned char *const data,
		    const int data_size, struct ecc_for_256_t *calc_ecc)
{
	/* !!! if data size == 8 , don't care !!! */
	__calculate_ecc(ecctable, data, data_size,
			&calc_ecc->lp_high, &calc_ecc->lp_low, &calc_ecc->cp);
	return 0;
}

static int correct_data(unsigned char *data,
			int data_size,
			struct ecc_for_256_t *read_ecc,
			const struct ecc_for_256_t *calc_ecc)
{

	/* !!! if data size == 8 , don't care !!!! */

	unsigned long l;	/*Working to check d */
	unsigned long d;	/*Result of comparison */
	unsigned int i;		/*For counting */
	unsigned char d1 = 0, d2, d3;	/*Result of comparison */
	unsigned char a;	/*Working for add */
	unsigned char add;	/*Byte address of cor.DATA */
	unsigned char b;	/*Working for bit */
	unsigned char bit;	/*Bit address of cor.DATA */

	unsigned long correctable = 0;
	unsigned long ecc_mask = 0;
	if (data_size == 256) {
		ecc_mask = 0x00ffffff;
		correctable = 0x00555554;
		/* d1 = ecc1 ^ eccdata[1]; */
		d1 = read_ecc->lp_high ^ calc_ecc->lp_high;
	} else if (data_size == 8) {
		ecc_mask = 0x0000ffff;
		correctable = 0x00005554;
		d1 = 0;		/* don't care lp_high; */
	}

	d2 = read_ecc->lp_low ^ calc_ecc->lp_low;	/* Compare LP's */
	d3 = read_ecc->cp ^ calc_ecc->cp;	/* Comapre CP's */

	d = ((unsigned long)d1 << 16)	/*Result of comparison */
	    +((unsigned long)d2 << 8) + (unsigned long)d3;

	if (d == 0)
		return 0;	/* If No error,return */
	if (((d ^ (d >> 1)) & correctable) == correctable) {
		/* If correctablei */
		l = BIT23;
		add = 0;	/* Clear parameter */
		a = BIT7;
		for (i = 0; i < 8; ++i) {	/* Checking 8 bit */
			if ((d & l) != 0)
				/* Make byte address from LP's */
				add |= a;
			l >>= 2;
			a >>= 1;	/* Right Shift */
		}
		bit = 0;	/* Clear parameter */
		b = BIT2;
		for (i = 0; i < 3; ++i) {	/* Checking 3 bit */
			if ((d & l) != 0)
				bit |= b;	/* Make bit address from CP's */
			l >>= 2;
			b >>= 1;	/* Right shift */
		}
		b = BIT0;
		if (add >= data_size) {
			printk(KERN_INFO " err address >= data_size\n");
			return 3;
		}
		data[add] ^= (b << bit);	/* Put corrected data */
		return 1;
	}
	i = 0;			/* Clear count */
	d &= ecc_mask;		/* Masking */
	while (d) {		/* If d=0 finish counting */
		if (d & BIT0)
			++i;	/* Count number of 1 bit */
		d >>= 1;	/* Right shift */
	}
	if (i == 1) {		/* If ECC error */
		/* Put right ECC code */

		*read_ecc = *calc_ecc;
		return 2;
	}
	return 3;		/* Uncorrectable error */
}

/* for oob ecc */
#define OOB_ECC_TARGET_SIZE 8

static inline int check_and_copy_oob(unsigned char *oob_buf,
				      unsigned char *copy_target,
				      struct bynd_oob_format format_e)
{
	int i;

	for (i = 0; i < format_e.size; i++)
		copy_target[i] = oob_buf[format_e.pos[i]];
	return 0;
}

static inline int oob_to_linear_str(unsigned char *oob_buf,
				     unsigned char *ecc_target,
				     const struct bynd_layout *format)
{
	unsigned char *p_ecc = ecc_target;

	check_and_copy_oob(oob_buf, p_ecc, format->logaddr);
	p_ecc += format->logaddr.size;

	check_and_copy_oob(oob_buf, p_ecc, format->blockversion);
	p_ecc += format->blockversion.size;

	check_and_copy_oob(oob_buf, p_ecc, format->datastatus);
	p_ecc += format->datastatus.size;

	check_and_copy_oob(oob_buf, p_ecc, format->blockstatus);
	p_ecc += format->blockstatus.size;

	check_and_copy_oob(oob_buf, p_ecc, format->start);
	p_ecc += format->start.size;

	check_and_copy_oob(oob_buf, p_ecc, format->end);
	p_ecc += format->end.size;

	for (; p_ecc < (ecc_target + OOB_ECC_TARGET_SIZE); p_ecc++)
		*p_ecc = 0xff;

	return 0;
}

static inline int check_and_copy_linear_str(unsigned char *oob_buf,
				    unsigned char *copy_target,
				    const struct bynd_oob_format format_e)
{
	int i;
	for (i = 0; i < format_e.size; i++)
		oob_buf[format_e.pos[i]] = copy_target[i];
	return 0;
}

static inline int linear_str_to_oob(unsigned char *oob_buf,
				     unsigned char *ecc_target,
				     const struct bynd_layout *format)

{
	unsigned char *p_ecc = ecc_target;

	check_and_copy_linear_str(oob_buf, p_ecc, format->logaddr);
	p_ecc += format->logaddr.size;

	check_and_copy_linear_str(oob_buf, p_ecc, format->blockversion);
	p_ecc += format->blockversion.size;

	check_and_copy_linear_str(oob_buf, p_ecc, format->datastatus);
	p_ecc += format->datastatus.size;

	check_and_copy_linear_str(oob_buf, p_ecc, format->blockstatus);
	p_ecc += format->blockstatus.size;

	check_and_copy_linear_str(oob_buf, p_ecc, format->start);
	p_ecc += format->start.size;

	check_and_copy_linear_str(oob_buf, p_ecc, format->end);
	p_ecc += format->end.size;

	return 0;
}

int calc_ecc_for_oob(struct bynd_record *bynd, unsigned char *oob_buf)
{

	unsigned char ecc_target[OOB_ECC_TARGET_SIZE];
	struct ecc_for_256_t work_ecc;

	oob_to_linear_str(oob_buf, ecc_target, bynd->layout);

	calc_ecc(ecc_target, OOB_ECC_TARGET_SIZE, &work_ecc);

	oob_buf[bynd->layout->eccr.pos[0]] = work_ecc.lp_low;
	oob_buf[bynd->layout->eccr.pos[1]] = work_ecc.cp;

	return 0;
}

int correct_oob_data(struct bynd_record *bynd, unsigned char *oob_buf)
{
	int ret;
	unsigned char ecc_target[OOB_ECC_TARGET_SIZE];
	struct ecc_for_256_t work_ecc_org;
	struct ecc_for_256_t work_ecc_calc;

	work_ecc_org.lp_low = oob_buf[bynd->layout->eccr.pos[0]];
	work_ecc_org.cp = oob_buf[bynd->layout->eccr.pos[1]];

	oob_to_linear_str(oob_buf, ecc_target, bynd->layout);

	calc_ecc(ecc_target, OOB_ECC_TARGET_SIZE, &work_ecc_calc);

	work_ecc_org.lp_low |= 0x80;
	work_ecc_calc.lp_low |= 0x80;

	ret = correct_data(ecc_target, OOB_ECC_TARGET_SIZE,
			   &work_ecc_org, &work_ecc_calc);
	if (ret == 1)
		linear_str_to_oob(oob_buf, ecc_target, bynd->layout);
	return ret;
}

