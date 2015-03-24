#ifndef _ASM_GENERIC_MARKER_H
#define _ASM_GENERIC_MARKER_H

/*
 * marker.h
 *
 * Code markup for dynamic and static tracing. Generic header.
 *
 * This file is released under the GPLv2.
 * See the file COPYING for more details.
 *
 * Note : the empty asm volatile with read constraint is used here instead of a
 * "used" attribute to fix a gcc 4.1.x bug.
 */

/* Default flags, used by MARK() */
#define _MF_DEFAULT			(_MF_LOCKDEP | _MF_PRINTK)

/* Fallback on the generic markers, since no optimized version is available */
#define MARK_OPTIMIZED			MARK_GENERIC
#define _MARK				MARK_GENERIC

/* Marker with default behavior */
#define MARK(format, args...)		_MARK(_MF_DEFAULT, format, ## args)

/* Architecture dependant marker information, used internally for marker
 * activation. */

#define MARK_OPTIMIZED_ENABLE_IMMEDIATE_OFFSET \
		MARK_GENERIC_ENABLE_IMMEDIATE_OFFSET
#define MARK_OPTIMIZED_ENABLE_TYPE	MARK_GENERIC_ENABLE_TYPE
/* Dereference enable as lvalue from a pointer to its instruction */
#define MARK_OPTIMIZED_ENABLE		MARK_GENERIC_ENABLE

#define marker_optimized_set_enable marker_generic_set_enable

#endif /* _ASM_GENERIC_MARKER_H */
