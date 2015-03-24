/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#ifndef __ASM_TX_BOARDS_YAMON_H
#define __ASM_TX_BOARDS_YAMON_H

struct yamon_env {
	__s32 name;
	__s32 var;
};

extern char *yamon_getenv(const char *name) __init;

#endif /* __ASM_TX_BOARDS_YAMON_H */
