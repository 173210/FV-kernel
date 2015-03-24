/*
 * ltt-serialize.c
 *
 * LTTng serializing code.
 *
 * Copyright Mathieu Desnoyers, March 2007.
 *
 * Licensed under the GPLv2.
 */

#include <stdarg.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/ltt-tracer.h>

static int skip_atoi(const char **s)
{
	int i=0;

	while (isdigit(**s))
		i = i*10 + *((*s)++) - '0';
	return i;
}

/* Inspired from vsnprintf */
/* The serialization format string supports the basic printf format strings.
 * In addition, it also defines new formats that can be used to serialize
 * more complex/non portable data structures.
 *
 * Serialization specific formats :
 *
 * Fixed length struct, union or array.
 * %*r     expects sizeof(*ptr), ptr
 * %*.*r   expects sizeof(*ptr), __alignof__(*ptr), ptr
 *
 * Variable length sequence
 * %*.*:*v expects sizeof(*ptr), __alignof__(*ptr), elem_num, ptr
 *         where elem_num is the number of elements in the sequence
 *
 * Callback
 * %k      callback (taken from the probe data)
 *
 * Fixed size integers
 * %1b     expects uint8_t
 * %2b     expects uint16_t
 * %4b     expects uint32_t
 * %8b     expects uint64_t
 * %*b     expects sizeof(data), data
 *         where sizeof(data) is 1, 2, 4 or 8
 */
__attribute__((no_instrument_function))
char *ltt_serialize_data(char *buffer, char *str,
			struct ltt_serialize_closure *closure,
			int align,
			const char *fmt, va_list *args)
{
	const char *s;
	int elem_size;		/* Size of the integer for 'b' */
				/* Size of the data contained by 'r' */
	int elem_alignment;	/* Element alignment for 'r' */
	int elem_num;		/* Number of elements in 'v' */
	int qualifier;		/* 'h', 'l', or 'L' for integer fields */
				/* 'z' support added 23/7/1999 S.H.    */
				/* 'z' changed to 'Z' --davidm 1/25/99 */
				/* 't' added for ptrdiff_t */

	for (; *fmt ; ++fmt) {
		if (*fmt != '%') {
			/* Skip text */
			continue;
		}

		/* process flags : ignore standard print formats for now. */
		repeat:
			++fmt;		/* this also skips first '%' */
			switch (*fmt) {
				case '-':
				case '+':
				case ' ':
				case '#':
				case '0': goto repeat;
			}

		/* get element size */
		elem_size = -1;
		if (isdigit(*fmt))
			elem_size = skip_atoi(&fmt);
		else if (*fmt == '*') {
			++fmt;
			/* it's the next argument */
			elem_size = va_arg(*args, int);
		}

		/* get the alignment */
		elem_alignment = -1;
		if (*fmt == '.') {
			++fmt;	
			if (isdigit(*fmt))
				elem_alignment = skip_atoi(&fmt);
			else if (*fmt == '*') {
				++fmt;
				/* it's the next argument */
				elem_alignment = va_arg(*args, int);
			}
		}

		/* get the number of elements */
		elem_num = -1;
		if (*fmt == ':') {
			++fmt;	
			if (isdigit(*fmt))
				elem_num = skip_atoi(&fmt);
			else if (*fmt == '*') {
				++fmt;
				/* it's the next argument */
				elem_num = va_arg(*args, int);
			}
		}

		/* get the conversion qualifier */
		qualifier = -1;
		if (*fmt == 'h' || *fmt == 'l' || *fmt == 'L' ||
		    *fmt =='Z' || *fmt == 'z' || *fmt == 't' ||
		    *fmt == 'S') {
			qualifier = *fmt;
			++fmt;
			if (qualifier == 'l' && *fmt == 'l') {
				qualifier = 'L';
				++fmt;
			}
		}

		switch (*fmt) {
			case 'c':
				if (buffer)
					*str = (char)va_arg(*args, int);
				else
					(void)va_arg(*args, int);
				str += sizeof(char);
				continue;

			case 's':
				s = va_arg(*args, const char *);
				if ((unsigned long)s < PAGE_SIZE)
					s = "<NULL>";
				elem_size = strlen(s)+1;
				if (buffer)
					memcpy(str, s, elem_size);
				str += elem_size;
				/* Following alignment for genevent
				 * compatibility */
				if (align)
					str += ltt_align((long)str,
							sizeof(void*));
				continue;

			case 'p':
				if (align)
					str += ltt_align((long)str,
							sizeof(void*));
				if (buffer)
					*(void**)str = va_arg(*args, void *);
				else
					(void)va_arg(*args, void *);
				str += sizeof(void*);
				continue;

			case 'r':
				/* For array, struct, union */
				if (elem_alignment > 0 && align)
					str += ltt_align((long)str,
							elem_alignment);
				s = va_arg(*args, const char *);
				if (elem_size > 0) {
					if (buffer)
						memcpy(str, s, elem_size);
					str += elem_size;
				}
				continue;

			case 'v':
				/* For sequence */
				if (align)
					str += ltt_align((long)str,
						max((int)sizeof(int),
						elem_alignment));
				if (buffer)
					*(int*)str = elem_num;
				str += sizeof(int);
				if (elem_alignment > 0 && align)
					str += ltt_align((long)str,
						elem_alignment);
				s = va_arg(*args, const char *);
				if (elem_num > 0) {
					if (buffer)
						memcpy(str, s,
							elem_num*elem_size);
					str += elem_num*elem_size;
				}
				/* Following alignment for genevent
				 * compatibility */
				if (align)
					str += ltt_align((long)str,
							sizeof(void*));
				continue;

			case 'k':
				/* For callback */
				 /* The callback will take as many arguments
				  * as it needs from args. They won't be
				  * type verified. */
				if (closure->cb_idx < LTT_NR_CALLBACKS-1) {
					ltt_serialize_cb cb;
					closure->cb_idx++;
					cb = closure->callbacks[closure->cb_idx];
					str = cb(buffer, str, closure,
							align,
							fmt, args);
				}
				continue;

			case 'n':
				/* FIXME:
				* What does C99 say about the overflow case
				* here? */
				if (qualifier == 'l') {
					long * ip = va_arg(*args, long *);
					*ip = (str - buffer);
				} else if (qualifier == 'Z'
					|| qualifier == 'z') {
					size_t * ip = va_arg(*args, size_t *);
					*ip = (str - buffer);
				} else {
					int * ip = va_arg(*args, int *);
					*ip = (str - buffer);
				}
				continue;

			case '%':
				continue;

			case 'b':
				if (elem_size < 0)
					elem_size = 0;
				if (elem_size > 0 && align)
					str += ltt_align((long)str, elem_size);
				switch (elem_size) {
				case 1:
					if (buffer)
						*(int8_t*)str =
						(int8_t)va_arg(*args, int);
					else
						(void)va_arg(*args, int);
					break;
				case 2:
					if (buffer)
						*(int16_t*)str =
						(int16_t)va_arg(*args, int);
					else
						(void)va_arg(*args, int);
					break;
				case 4:
					if (buffer)
						*(int32_t*)str =
						va_arg(*args, int32_t);
					else
						(void)va_arg(*args, int32_t);
					break;
				case 8:
					if (buffer)
						*(int64_t*)str =
						va_arg(*args, int64_t);
					else
						(void)va_arg(*args, int64_t);
					break;
				}
				str += elem_size;
				continue;

			case 'o':
			case 'X':
			case 'x':
			case 'd':
			case 'i':
			case 'u':
				break;

			default:
				if (!*fmt)
					--fmt;
				continue;
		}
		switch (qualifier) {
		case 'L':
			if (align)
				str += ltt_align((long)str, sizeof(long long));
			if (buffer)
				*(long long*)str = va_arg(*args, long long);
			else
				(void)va_arg(*args, long long);
			str += sizeof(long long);
			break;
		case 'l':
			if (align)
				str += ltt_align((long)str, sizeof(long));
			if (buffer)
				*(long*)str = va_arg(*args, long);
			else
				(void)va_arg(*args, long);
			str += sizeof(long);
			break;
		case 'Z':
		case 'z':
			if (align)
				str += ltt_align((long)str, sizeof(size_t));
			if (buffer)
				*(size_t*)str = va_arg(*args, size_t);
			else
				(void)va_arg(*args, size_t);
			str += sizeof(size_t);
			break;
		case 't':
			if (align)
				str += ltt_align((long)str, sizeof(ptrdiff_t));
			if (buffer)
				*(ptrdiff_t*)str = va_arg(*args, ptrdiff_t);
			else
				(void)va_arg(*args, ptrdiff_t);
			str += sizeof(ptrdiff_t);
			break;
		case 'h':
			if (align)
				str += ltt_align((long)str, sizeof(short));
			if (buffer)
				*(short*)str = (short) va_arg(*args, int);
			else
				(void)va_arg(*args, int);
			str += sizeof(short);
			break;
		default:
			if (align)
				str += ltt_align((long)str, sizeof(int));
			if (buffer)
				*(int*)str = va_arg(*args, int);
			else
				(void)va_arg(*args, int);
			str += sizeof(int);
		}
	}
	return str;
}
EXPORT_SYMBOL_GPL(ltt_serialize_data);

/* Calculate data size */
/* Assume that the padding for alignment starts at a
 * sizeof(void *) address. */
static __attribute__((no_instrument_function))
size_t ltt_get_data_size(struct ltt_serialize_closure *closure,
				int align,
				const char *fmt, va_list *args)
{
	ltt_serialize_cb cb = closure->callbacks[0];
	closure->cb_idx = 0;
	return (size_t)cb(NULL, NULL, closure, align, fmt, args);
}

static __attribute__((no_instrument_function))
void ltt_write_event_data(char *buffer,
				struct ltt_serialize_closure *closure,
				int align,
				const char *fmt, va_list *args)
{
	ltt_serialize_cb cb = closure->callbacks[0];
	closure->cb_idx = 0;
	cb(buffer, buffer, closure, align, fmt, args);
}


__attribute__((no_instrument_function))
void ltt_vtrace(const struct __mark_marker_data *mdata,
		const char *fmt, va_list args)
{
	int flags;
	int align;
	struct ltt_probe_data *pdata;
	uint8_t fID, eID;
	size_t data_size, slot_size;
	int channel_index;
	struct ltt_channel_struct *channel;
	struct ltt_trace_struct *trace, *dest_trace = NULL;
	void *transport_data;
	uint64_t tsc;
	char *buffer;
	va_list args_copy;
	struct ltt_serialize_closure closure;

	flags = mdata->flags;
	pdata = (struct ltt_probe_data *)mdata->pdata;
	fID = pdata->fID;
	eID = pdata->eID;
	align = pdata->align;
	closure.callbacks = pdata->callbacks;

	/* This test is useful for quickly exiting static tracing when no
	 * trace is active. */
	if (likely(ltt_traces.num_active_traces == 0
		&& !(flags & ltt_flag_mask(LTT_FLAG_FORCE))))
		return;

	preempt_disable();
	ltt_nesting[smp_processor_id()]++;
	
	if (unlikely(flags & ltt_flag_mask(LTT_FLAG_TRACE)))
		dest_trace = va_arg(args, struct ltt_trace_struct *);
	if (unlikely(flags & ltt_flag_mask(LTT_FLAG_CHANNEL)))
		channel_index = va_arg(args, int);
	else
		channel_index = pdata->channel_index;
	/* Force write in the compact channel if compact flag is used */
	if (unlikely(flags & ltt_flag_mask(LTT_FLAG_COMPACT)))
		channel_index = GET_CHANNEL_INDEX(compact);

	va_copy(args_copy, args);
	/* Skip the compact data for size calculation */
	if (likely(flags & ltt_flag_mask(LTT_FLAG_COMPACT_DATA)))
		(void)va_arg(args_copy, u32);
	data_size = ltt_get_data_size(&closure, align, fmt, &args_copy);
	va_end(args_copy);

	/* Iterate on each traces */
	list_for_each_entry_rcu(trace, &ltt_traces.head, list) {
		if (unlikely(!trace->active
			&& !(flags & ltt_flag_mask(LTT_FLAG_FORCE))))
			continue;
		if (unlikely(flags & ltt_flag_mask(LTT_FLAG_TRACE)
			&& trace != dest_trace))
			continue;
		if (!ltt_run_filter(trace, fID, eID))
			continue;
		channel = ltt_get_channel_from_index(trace, channel_index);
		/* reserve space : header and data */
		buffer = ltt_reserve_slot(trace, channel, &transport_data,
						data_size, &slot_size, &tsc);
		if (unlikely(!buffer))
			continue; /* buffer full */

		va_copy(args_copy, args);
		/* Out-of-order write : header and data */
		if (likely(!(flags & ltt_flag_mask(LTT_FLAG_COMPACT))))
			buffer = ltt_write_event_header(trace, channel, buffer,
						fID, eID, data_size, tsc);
		else {
			u32 compact_data = 0;
			if (likely(flags &
					ltt_flag_mask(LTT_FLAG_COMPACT_DATA)))
				compact_data = va_arg(args_copy, u32);
			buffer = ltt_write_compact_header(trace, channel,
						buffer, fID, eID, data_size,
						tsc, compact_data);
		}
		ltt_write_event_data(buffer, &closure, align, fmt, &args_copy);
		va_end(args_copy);
		/* Out-of-order commit */
		ltt_commit_slot(channel, &transport_data, buffer, slot_size);
	}
	ltt_nesting[smp_processor_id()]--;
	preempt_enable();
}
EXPORT_SYMBOL_GPL(ltt_vtrace);

__attribute__((no_instrument_function))
void ltt_trace(const struct __mark_marker_data *mdata, const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	ltt_vtrace(mdata, fmt, args);
	va_end(args);
}
EXPORT_SYMBOL_GPL(ltt_trace);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("Linux Trace Toolkit Next Generation Serializer");
