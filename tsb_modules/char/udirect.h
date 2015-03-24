/*
 * (C) Copyright TOSHIBA CORPORATION 2002-2010
 * All Rights Reserved.
 */

#ifndef _UDIRECT_H_
#define _UDIRECT_H_

#define UDIRECT_MISC_MINOR          240

#define IOCTL_INTDRV_INTSET         0x01
#define IOCTL_INTDRV_INTUNSET       0x02
#define IOCTL_INTDRV_INTWAIT        0x03
#define IOCTL_INTDRV_INTENABLE      0x04
#define IOCTL_INTDRV_INTDISABLE     0x05
#define IOCTL_INTDRV_CANCEL         0x06

#define IOCTL_PIO_BITWRITE	    0x20
#define IOCTL_GET_PMON_VERSION	    0x21
#define IOCTL_OPBD_ALOADDONEBIT     0x22
#define IOCTL_OPBD_KLOADDONEBIT     0x23
#define IOCTL_OPBD_WAITPBOOT        0x24
#define IOCTL_OPBD_WAITKBOOT        0x25

#define IOCTL_REGPHYS               0x30

#define INTDRV_NONEEDTOACK          0xffffffff

#define MAX_UDIRECT_MMAP            10

struct udirect_mmap {
	char          name[8];  /* name */
	unsigned long physaddr; /* 36 .. 4 bit */
	unsigned long size;     /* size */
	unsigned long cache;    /* cache */
	unsigned long offset;   /* offset */
};

struct req_bitwrite_data {
        unsigned long offset;
        unsigned char bit_no;
        unsigned char write_data;
};

struct reqirq_data{
	unsigned long irc;
	char name[20];
};

#ifdef __KERNEL__
#define UDIRET_CAPABILITY_CHECK      !capable(CAP_SYS_RAWIO)
#endif /* __KERNEL__ */

#endif /* _UDIRECT_H_ */

