/*
 * ltt-probe-kernel_arch_mips.c
 *
 * kernel_arch probe
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


#define FACILITY_NAME "kernel_arch"

static struct ltt_probe_data probe_array[] =
{
	{ "kernel_arch_trap_entry", "%d %ld", GET_CHANNEL_INDEX(cpu) },
	{ "kernel_arch_trap_exit", MARK_NOARGS, GET_CHANNEL_INDEX(cpu) },
	{ "kernel_arch_syscall_entry", "%d %ld", GET_CHANNEL_INDEX(cpu) },
	{ "kernel_arch_syscall_exit", MARK_NOARGS, GET_CHANNEL_INDEX(cpu) },
	{ "kernel_arch_ipc_call", "%u %d", GET_CHANNEL_INDEX(cpu) },
	{ "kernel_arch_kthread_create", "%ld %p",
		GET_CHANNEL_INDEX(processes) },
};

static const char *event_array_for_appdev[] =
{
	"kernel_arch_trap_entry", 
	"kernel_arch_trap_exit",
	"kernel_arch_syscall_entry",
	"kernel_arch_syscall_exit",
	"kernel_arch_kthread_create",
	NULL
};

static const char *event_array_for_ctxsw[] =
{
	"kernel_arch_kthread_create",
	NULL
};

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
		probe_array[eID].callbacks[0] = ltt_serialize_data;
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
