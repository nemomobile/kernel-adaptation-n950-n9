/**********************************************************************
 *
 * Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful but, except
 * as otherwise stated in writing, without any warranty; without even the
 * implied warranty of merchantability or fitness for a particular purpose.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK
 *
 ******************************************************************************/

#ifndef __SERVICES_PROC_H__
#define __SERVICES_PROC_H__

#include <asm/system.h>
#include <linux/proc_fs.h>

#define END_OF_FILE ((off_t) -1)

off_t printAppend(char *buffer, size_t size, off_t off,
		  const char *format, ...)
		  __attribute__ ((format(printf, 4, 5)));

int CreateProcEntries(void);
int CreateProcReadEntry(const char *name,
			off_t (handler)(char *, size_t, off_t));
int CreateProcEntry(const char *name, read_proc_t rhandler,
		    write_proc_t whandler, void *data);

int CreatePerProcessProcEntry(u32 pid, const char *name, read_proc_t rhandler,
			      void *data);

void RemoveProcEntry(const char *name);

void RemovePerProcessProcEntry(u32 pid, const char *name);

void RemoveProcEntries(void);

#endif
