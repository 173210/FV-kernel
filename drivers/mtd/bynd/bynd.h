/*
 * (C) Copyright TOSHIBA CORPORATION 2009-2010
 * All Rights Reserved.
 */

#ifndef BYND_OOB_FORMAT_H
#define BYND_OOB_FORMAT_H

#include <linux/list.h>
#include <linux/mtd/blktrans.h>

struct bynd_oob_format {
	uint8_t size;
	uint8_t pos[64];
};

struct bynd_layout {
	struct bynd_oob_format logaddr;
	struct bynd_oob_format blockversion;
	struct bynd_oob_format datastatus;
	struct bynd_oob_format blockstatus;
	struct bynd_oob_format start;
	struct bynd_oob_format end;
	struct bynd_oob_format ecc;
	struct bynd_oob_format eccr;
};

struct bynd_list_entry {
	struct list_head list;
	unsigned int physnum;
};

struct bynd_record {
	struct mtd_blktrans_dev mbd;
	int usecount;
	unsigned char heads;
	unsigned char sectors;
	unsigned short cylinders;
	int *log_block_map;
	unsigned int badblocks;

	uint8_t *log_block_version;
	char *page_buf;
	struct list_head erase;
	struct list_head onebiterror;
	struct list_head oldversion;

	int map_len;			/* n. phys_blocks */
	int map_size;			/* n. log_blocks  */

	struct bynd_layout *layout;

#ifdef CONFIG_BYND_WRITE_CACHE
	unsigned long cache_logaddr;
	unsigned int cache_size;
	enum { STATE_EMPTY, STATE_DIRTY } cache_state;
#endif
};

/* byndecc.c */
int calc_ecc_for_oob(struct bynd_record *bynd, unsigned char *oob_buf);
int correct_oob_data(struct bynd_record *bynd, unsigned char *oob_buf);

#endif	/* BYND_OOB_FORMAT_H */

