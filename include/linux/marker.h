#ifndef _LINUX_MARKER_H
#define _LINUX_MARKER_H

/*
 * marker.h
 *
 * Code markup for dynamic and static tracing.
 *
 * See Documentation/marker.txt.
 *
 * (C) Copyright 2006 Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 *
 * This file is released under the GPLv2.
 * See the file COPYING for more details.
 */

#ifdef __KERNEL__

struct __mark_marker_data;

typedef void marker_probe_func(const struct __mark_marker_data *mdata,
	const char *fmt, ...);

struct __mark_marker_data {
	const char *name;
	const char *format;
	int flags;
	marker_probe_func *call;
	void *pdata;
} __attribute__((packed));

struct __mark_marker {
	struct __mark_marker_data *mdata;
	void *enable;
} __attribute__((packed));

#ifdef CONFIG_MARKERS

/* Marker flags : selects the mechanism used to connect the probes to the
 * markers and what can be executed within the probes. This is primarily
 * used at reentrancy-unfriendly sites. */
#define MF_OPTIMIZED	0	/* Use optimized markers */
#define MF_LOCKDEP	1	/* Can call lockdep */
#define MF_PRINTK	2	/* vprintk can be called in the probe */
#define MF_NR		3	/* Number of marker flags */

#define _MF_OPTIMIZED	(1 << MF_OPTIMIZED)
#define _MF_LOCKDEP	(1 << MF_LOCKDEP)
#define _MF_PRINTK	(1 << MF_PRINTK)

#define DECLARE_MARKER_DATA(flags, name, format) \

/* Generic marker flavor always available */
#define MARK_GENERIC(flags, name, format, args...) \
	do { \
		static const char __mstrtab_name_##name[] \
		__attribute__((section("__markers_strings"))) \
		= #name; \
		static const char __mstrtab_format_##name[] \
		__attribute__((section("__markers_strings"))) \
		= format; \
		static struct __mark_marker_data __mark_data_##name \
		__attribute__((section("__markers_data"))) = \
		{ __mstrtab_name_##name,  __mstrtab_format_##name, \
		(flags) & ~_MF_OPTIMIZED, __mark_empty_function, NULL }; \
		static char __marker_enable_##name = 0; \
		static const struct __mark_marker __mark_##name \
			__attribute__((section("__markers"))) = \
			{ &__mark_data_##name, &__marker_enable_##name } ; \
		asm volatile ( "" : : "i" (&__mark_##name)); \
		__mark_check_format(format, ## args); \
		if (unlikely(__marker_enable_##name)) { \
			preempt_disable(); \
			(*__mark_data_##name.call)(&__mark_data_##name, \
						format, ## args); \
			preempt_enable(); \
		} \
	} while (0)

#define MARK_GENERIC_ENABLE_IMMEDIATE_OFFSET 0
#define MARK_GENERIC_ENABLE_TYPE char
/* Dereference enable as lvalue from a pointer to its instruction */
#define MARK_GENERIC_ENABLE(a) \
	*(MARK_GENERIC_ENABLE_TYPE*) \
		((char*)a+MARK_GENERIC_ENABLE_IMMEDIATE_OFFSET)

static inline int marker_generic_set_enable(void *address, char enable)
{
	MARK_GENERIC_ENABLE(address) = enable;
	return 0;
}

#else /* !CONFIG_MARKERS */
#define MARK_GENERIC(flags, name, format, args...) \
		__mark_check_format(format, ## args)
#endif /* CONFIG_MARKERS */

#ifdef CONFIG_MARKERS_ENABLE_OPTIMIZATION
#include <asm/marker.h>			/* optimized marker flavor */
#else
#include <asm-generic/marker.h>		/* fallback on generic markers */
#endif

#define MARK_MAX_FORMAT_LEN	1024
/* Pass this as a format string for a marker with no argument */
#define MARK_NOARGS " "

/* To be used for string format validity checking with sparse */
static inline
void __mark_check_format(const char *fmt, ...)
{ }

extern marker_probe_func __mark_empty_function;

extern int _marker_set_probe(int flags, const char *name, const char *format,
				marker_probe_func *probe, void *pdata);

#define marker_set_probe(name, format, probe, pdata) \
	_marker_set_probe(_MF_DEFAULT, name, format, probe, pdata)

extern int marker_remove_probe(const char *name);
extern int marker_list_probe(marker_probe_func *probe);

#endif /* __KERNEL__ */
#endif
