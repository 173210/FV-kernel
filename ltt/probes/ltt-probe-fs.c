/*
 * ltt-probe-fs.c
 *
 * FS probe
 *
 * Part of LTTng
 *
 * Mathieu Desnoyers, March 2007
 *
 * Licensed under the GPLv2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/crc32.h>
#include <linux/marker.h>
#include <linux/ltt-facilities.h>
#include <linux/ltt-tracer.h>

#include <asm/uaccess.h>

/* Expects va args : (int elem_num, const char __user *s)
 * Element size is implicit (sizeof(char)). */
static char *ltt_serialize_fs_data(char *buffer, char *str,
	struct ltt_serialize_closure *closure,
	int align, const char *fmt, va_list *args)
{
	int elem_size;
	int elem_num;
	const char __user  *s;
	unsigned long noncopy;

	elem_num = va_arg(*args, int);
	s = va_arg(*args, const char __user *);
	elem_size = sizeof(*s);

	if (align)
		str += ltt_align((long)str, sizeof(int));
	if (buffer)
		*(int*)str = elem_num;
	str += sizeof(int);

	if (elem_num > 0) {
		/* No alignment required for char */
		if (buffer) {
			noncopy = __copy_from_user_inatomic(str, s,
					elem_num*elem_size);
			memset(str+(elem_num*elem_size)-noncopy, 0, noncopy);
		}
		str += (elem_num*elem_size);
	}
	/* Following alignment for genevent
	 * compatibility */
	if (align)
		str += ltt_align((long)str, sizeof(void*));
	return str;
}

#define FACILITY_NAME "fs"

static struct ltt_probe_data probe_array[] =
{
	{ "fs_buffer_wait_start", "%p", GET_CHANNEL_INDEX(cpu),
		.callbacks[0] = ltt_serialize_data },
	{ "fs_buffer_wait_end", "%p", GET_CHANNEL_INDEX(cpu),
		.callbacks[0] = ltt_serialize_data },
	{ "fs_exec", "%s", GET_CHANNEL_INDEX(processes),
		.callbacks[0] = ltt_serialize_data },
	{ "fs_open", "%d %s", GET_CHANNEL_INDEX(cpu),
		.callbacks[0] = ltt_serialize_data },
	{ "fs_close", "%u", GET_CHANNEL_INDEX(cpu),
		.callbacks[0] = ltt_serialize_data },
	{ "fs_read", "%u %zu", GET_CHANNEL_INDEX(cpu),
		.callbacks[0] = ltt_serialize_data },
	{ "fs_pread64", "%u %zu %8b", GET_CHANNEL_INDEX(cpu),
		.callbacks[0] = ltt_serialize_data },
	{ "fs_readv", "%lu %lu", GET_CHANNEL_INDEX(cpu),
		.callbacks[0] = ltt_serialize_data },
	{ "fs_write", "%u %zu", GET_CHANNEL_INDEX(cpu),
		.callbacks[0] = ltt_serialize_data },
	{ "fs_pwrite64", "%u %zu %8b", GET_CHANNEL_INDEX(cpu),
		.callbacks[0] = ltt_serialize_data },
	{ "fs_writev", "%lu %lu", GET_CHANNEL_INDEX(cpu),
		.callbacks[0] = ltt_serialize_data },
	{ "fs_lseek", "%u %4b %u", GET_CHANNEL_INDEX(cpu),
		.callbacks[0] = ltt_serialize_data },
	{ "fs_llseek", "%u %8b %u", GET_CHANNEL_INDEX(cpu),
		.callbacks[0] = ltt_serialize_data },
	{ "fs_ioctl", "%u %u %lu", GET_CHANNEL_INDEX(cpu),
		.callbacks[0] = ltt_serialize_data },
	{ "fs_select", "%d %8b", GET_CHANNEL_INDEX(cpu),
		.callbacks[0] = ltt_serialize_data },
	{ "fs_pollfd", "%d", GET_CHANNEL_INDEX(cpu),
		.callbacks[0] = ltt_serialize_data },
	{ "fs_read_data", "%u %zd %k", GET_CHANNEL_INDEX(cpu),
		.callbacks[0] = ltt_serialize_data,
		.callbacks[1] = ltt_serialize_fs_data},
	{ "fs_write_data", "%u %zd %k", GET_CHANNEL_INDEX(cpu),
		.callbacks[0] = ltt_serialize_data,
		.callbacks[1] = ltt_serialize_fs_data },
};

static const char *event_array_for_appdev[] =
{
	"fs_buffer_wait_start",
	"fs_buffer_wait_end",
	"fs_exec",
	"fs_open",
	"fs_close", 
	"fs_read",
	"fs_pread64",
	"fs_readv",
	"fs_write",
	"fs_pwrite64",
	"fs_writev",
	"fs_lseek",
	"fs_llseek",
	"fs_ioctl",
	"fs_select",
	"fs_pollfd",
	"fs_read_data",
	"fs_write_data",
	NULL
};

static const char *event_array_for_ctxsw[] = { NULL };
static const char *event_array_for_null[] = { NULL };

#define NUM_PROBES (sizeof(probe_array) / sizeof(struct ltt_probe_data))

static struct ltt_facility facility = {
	.name = FACILITY_NAME,
	.num_events = NUM_PROBES,
	.checksum = 0,
	.id = 0xFF,
	.alignment = 1,	/* 1: true, 0: false */
};

static char *eventset = "all";
module_param( eventset, charp, 0444);

static int is_in_event_set(const char *event_name, const char **event_array)
{
	int i;	
	
	for (i = 0; event_array[i] != 0 ; i++) {
		if (strcmp(event_name,event_array[i]) == 0)
			return 1;
	}
	return 0;
}

static int __init probe_init(void)
{
	int result;
	uint8_t eID;
	int ret;
	const char **event_array = NULL;

	/* FIXME : LTTV is unable to compute this CRC (for now) */
	for (eID = 0; eID < NUM_PROBES; eID++) {
		facility.checksum =
			crc32(facility.checksum, probe_array[eID].name,
				strlen(probe_array[eID].name));
		facility.checksum =
			crc32(facility.checksum, probe_array[eID].format,
				strlen(probe_array[eID].format));

	}
	ret = ltt_facility_kernel_register(&facility);
	if (ret < 0) {
		printk(KERN_WARNING "LTT : Error in registering facility %s\n",
			facility.name);
		return ret;
	}
	facility.id = (uint8_t)ret;

	printk("LTT : Facility %s registered with id %hu\n", facility.name,
		facility.id);

	if (strcmp(eventset, "appdev") == 0) {
		event_array = event_array_for_appdev;
	} else if (strcmp(eventset, "ctxsw") == 0) {
		event_array = event_array_for_ctxsw;
	} else if (strcmp(eventset, "null") == 0) {
		event_array = event_array_for_null;
	} else if (strcmp(eventset, "all") != 0) {
		printk(KERN_WARNING "LTT : unknown eventset '%s'!\n", eventset);
		return -EINVAL;
	}

	for (eID = 0; eID < NUM_PROBES; eID++) {
		if (event_array && !is_in_event_set(probe_array[eID].name,
							event_array)) 
			continue;
		probe_array[eID].fID = facility.id;
		probe_array[eID].eID = eID;
		probe_array[eID].align = facility.alignment;
		//probe_array[eID].callbacks[0] = ltt_serialize_data;
		result = marker_set_probe(probe_array[eID].name,
				probe_array[eID].format,
				ltt_trace, &probe_array[eID]);
		if (!result)
			printk(KERN_INFO "LTT unable to register probe %s\n",
				probe_array[eID].name);
	}
	return 0;
}

static void __exit probe_fini(void)
{
	uint8_t eID;
	int err;

	for (eID = 0; eID < NUM_PROBES; eID++) {
		marker_remove_probe(probe_array[eID].name);
	}
	synchronize_sched();	/* Wait for probes to finish */
	err = ltt_facility_unregister(facility.id);
	if (err)
		printk(KERN_WARNING
			"LTT : Error in unregistering facility %s\n",
			facility.name);
}

module_init(probe_init);
module_exit(probe_fini);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION(FACILITY_NAME " probe");
