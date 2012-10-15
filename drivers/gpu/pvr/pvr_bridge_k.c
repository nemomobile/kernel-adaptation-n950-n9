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

#include <linux/sched.h>

#include "img_defs.h"
#include "services.h"
#include "pvr_bridge.h"
#include "pvr_bridge_km.h"
#include "perproc.h"
#include "syscommon.h"
#include "pvr_debug.h"
#include "proc.h"
#include "private_data.h"

#include "sgx_bridge.h"

#include "bridged_pvr_bridge.h"

/* Global driver lock protecting all HW and SW state tracking objects. */
DEFINE_MUTEX(gPVRSRVLock);
static int pvr_dev_locked;
static DECLARE_WAIT_QUEUE_HEAD(pvr_dev_wq);
int pvr_disabled;

/*
 * The pvr_dvfs_* interface is needed to suppress a lockdep warning in
 * the following situation during a clock rate change:
 * On the CLK_PRE_RATE_CHANGE:
 * 1. lock A           <- __blocking_notifier_call_chain:nh->rwsem
 * 2. lock B           <- vdd2_pre_post_func:gPVRSRVLock
 * 3. unlock A
 *
 * On the CLK_POST_RATE_CHANGE/CLK_ABORT_RATE_CHANGE:
 * 4. lock A
 * 5. unlock B
 * 6. unlock A
 *
 * The above has an ABA lock pattern which triggers the warning. This can't
 * lead to a dead lock though since at 3. we always release A, before it's
 * again acquired at 4. To avoid the warning use a wait queue based approach
 * so that we can unlock B before 3.
 */
void pvr_dev_lock(void)
{
	while (cmpxchg(&pvr_dev_locked, 0, 1)) {
		DEFINE_WAIT(pvr_dev_wait);
		prepare_to_wait(&pvr_dev_wq, &pvr_dev_wait,
				TASK_UNINTERRUPTIBLE);
		if (pvr_dev_locked)
			schedule();
		finish_wait(&pvr_dev_wq, &pvr_dev_wait);
	}
}

void pvr_dev_unlock(void)
{
	BUG_ON(!pvr_dev_locked);
	pvr_dev_locked = 0;
	wake_up(&pvr_dev_wq);
}

#if defined(DEBUG_BRIDGE_KM)
static off_t printLinuxBridgeStats(char *buffer, size_t size, off_t off);
#endif

enum PVRSRV_ERROR LinuxBridgeInit(void)
{
#if defined(DEBUG_BRIDGE_KM)
	{
		int iStatus;
		iStatus =
		    CreateProcReadEntry("bridge_stats", printLinuxBridgeStats);
		if (iStatus != 0)
			return PVRSRV_ERROR_OUT_OF_MEMORY;
	}
#endif
	return PVRSRV_OK;
}

void LinuxBridgeDeInit(void)
{
#if defined(DEBUG_BRIDGE_KM)
	RemoveProcEntry("bridge_stats");
#endif
}

#if defined(DEBUG_BRIDGE_KM)
static off_t printLinuxBridgeStats(char *buffer, size_t count, off_t off)
{
	struct PVRSRV_BRIDGE_DISPATCH_TABLE_ENTRY *psEntry;
	off_t Ret;

	pvr_lock();

	if (!off) {
		if (count < 500) {
			Ret = 0;
			goto unlock_and_return;
		}
		Ret = printAppend(buffer, count, 0,
			"Total ioctl call count = %u\n"
			"Total number of bytes copied via copy_from_user = %u\n"
			"Total number of bytes copied via copy_to_user = %u\n"
			"Total number of bytes copied via copy_*_user = %u\n\n"
			"%-2s | %-40s | %10s | %20s | %10s\n",
		  g_BridgeGlobalStats.ui32IOCTLCount,
		  g_BridgeGlobalStats.ui32TotalCopyFromUserBytes,
		  g_BridgeGlobalStats.ui32TotalCopyToUserBytes,
		  g_BridgeGlobalStats.ui32TotalCopyFromUserBytes +
			  g_BridgeGlobalStats.ui32TotalCopyToUserBytes,
		  "ID", "Wrapper Function",
		  "Call Count", "copy_from_user Bytes",
		  "copy_to_user Bytes");

		goto unlock_and_return;
	}

	if (off > BRIDGE_DISPATCH_TABLE_ENTRY_COUNT) {
		Ret = END_OF_FILE;
		goto unlock_and_return;
	}

	if (count < 300) {
		Ret = 0;
		goto unlock_and_return;
	}

	psEntry = &g_BridgeDispatchTable[off - 1];
	Ret = printAppend(buffer, count, 0,
			  "%02lX   %-40s   %-10u   %-20u   %-10u\n",
			  off - 1, psEntry->pszFunctionName,
			  psEntry->ui32CallCount,
			  psEntry->ui32CopyFromUserTotalBytes,
			  psEntry->ui32CopyToUserTotalBytes);

unlock_and_return:
	pvr_unlock();

	return Ret;
}
#endif

long PVRSRV_BridgeDispatchKM(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	u32 ui32BridgeID = PVRSRV_GET_BRIDGE_ID(cmd);
	struct PVRSRV_BRIDGE_PACKAGE __user *psBridgePackageUM =
	    (struct PVRSRV_BRIDGE_PACKAGE __user *)arg;
	struct PVRSRV_BRIDGE_PACKAGE sBridgePackageKM;
	u32 ui32PID = OSGetCurrentProcessIDKM();
	struct PVRSRV_FILE_PRIVATE_DATA *priv;
	struct PVRSRV_PER_PROCESS_DATA *psPerProc;
	int err = -EFAULT;

	pvr_lock();

	if (pvr_is_disabled()) {
		pvr_unlock();
		return -ENODEV;
	}

	if (!OSAccessOK(PVR_VERIFY_WRITE, psBridgePackageUM,
			sizeof(struct PVRSRV_BRIDGE_PACKAGE))) {
		PVR_DPF(PVR_DBG_ERROR,
			 "%s: Received invalid pointer to function arguments",
			 __func__);

		goto unlock_and_return;
	}

	if (OSCopyFromUser(NULL, &sBridgePackageKM, psBridgePackageUM,
			   sizeof(struct PVRSRV_BRIDGE_PACKAGE)) != PVRSRV_OK)
		goto unlock_and_return;

	priv = filp->private_data;
	psPerProc = priv->proc;
	BUG_ON(!psPerProc);

	if (ui32BridgeID !=
	    PVRSRV_GET_BRIDGE_ID(PVRSRV_BRIDGE_CONNECT_SERVICES)) {
		if (psPerProc->ui32PID != ui32PID) {
			PVR_DPF(PVR_DBG_ERROR,
				 "%s: Process %d tried to access data "
				 "belonging to process %d", __func__,
				 ui32PID, psPerProc->ui32PID);
			goto unlock_and_return;
		}
	}

	sBridgePackageKM.ui32BridgeID = PVRSRV_GET_BRIDGE_ID(
						sBridgePackageKM.ui32BridgeID);

	err = BridgedDispatchKM(filp, psPerProc, &sBridgePackageKM);
	if (err != PVRSRV_OK)
		goto unlock_and_return;

unlock_and_return:
	pvr_unlock();

	return err;
}
