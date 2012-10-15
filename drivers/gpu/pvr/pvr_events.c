
#include "pvr_events.h"
#include "servicesint.h"
#include "sgx_bridge.h"
#include "perproc.h"
#include "pvr_bridge_km.h"
#include "pvr_trace_cmd.h"

#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/notifier.h>
#include <linux/slab.h>

#include <plat/display.h>

static spinlock_t event_lock;
static struct list_head sync_wait_list;
static struct list_head flip_wait_list;
static struct notifier_block dss_nb;

static inline bool is_render_complete(const struct PVRSRV_SYNC_DATA *sync,
				      u32 write_ops_pending)
{
	return (int)sync->ui32WriteOpsComplete - (int)write_ops_pending >= 0;
}

static void pvr_signal_sync_event(struct pvr_pending_sync_event *e,
					const struct timeval *now)
{
	e->event.tv_sec = now->tv_sec;
	e->event.tv_usec = now->tv_usec;

	list_move_tail(&e->base.link, &e->base.file_priv->event_list);

	wake_up_interruptible(&e->base.file_priv->event_wait);
}

int pvr_sync_event_req(struct PVRSRV_FILE_PRIVATE_DATA *priv,
			const struct PVRSRV_KERNEL_SYNC_INFO *sync_info,
			u64 user_data)
{
	struct pvr_pending_sync_event *e;
	struct timeval now;
	unsigned long flags;

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (e == NULL)
		return -ENOMEM;

	e->event.base.type = PVR_EVENT_SYNC;
	e->event.base.length = sizeof(e->event);
	e->event.sync_info = sync_info;
	e->event.user_data = user_data;
	e->base.event = &e->event.base;
	e->base.file_priv = priv;
	e->base.destroy = (void (*)(struct pvr_pending_event *))kfree;
	e->base.write_ops_pending = sync_info->psSyncData->ui32WriteOpsPending;

	do_gettimeofday(&now);
	spin_lock_irqsave(&event_lock, flags);

	if (priv->event_space < sizeof(e->event)) {
		spin_unlock_irqrestore(&event_lock, flags);
		kfree(e);
		return -ENOMEM;
	}

	priv->event_space -= sizeof(e->event);

	list_add_tail(&e->base.link, &sync_wait_list);
	if (is_render_complete(sync_info->psSyncData,
			       e->base.write_ops_pending))
		pvr_signal_sync_event(e, &now);

	spin_unlock_irqrestore(&event_lock, flags);

	return 0;
}

static void pvr_signal_flip_event(struct pvr_pending_flip_event *e,
					const struct timeval *now)
{
	e->event.tv_sec = now->tv_sec;
	e->event.tv_usec = now->tv_usec;

	list_move_tail(&e->base.link, &e->base.file_priv->event_list);

	wake_up_interruptible(&e->base.file_priv->event_wait);
}

int pvr_flip_event_req(struct PVRSRV_FILE_PRIVATE_DATA *priv,
			 unsigned int overlay,
			 enum pvr_sync_wait_seq_type type, u64 user_data)
{
	struct pvr_pending_flip_event *e;
	struct timeval now;
	unsigned long flags;

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (e == NULL)
		return -ENOMEM;

	switch (type) {
	case _PVR_SYNC_WAIT_FLIP:
		e->event.base.type = PVR_EVENT_FLIP;
		e->dss_event = OMAP_DSS_NOTIFY_GO_OVL;
		break;
	case _PVR_SYNC_WAIT_UPDATE:
		e->event.base.type = PVR_EVENT_UPDATE;
		e->dss_event = OMAP_DSS_NOTIFY_UPDATE_OVL;
		break;
	default:
		BUG();
	}
	e->event.base.length = sizeof(e->event);
	e->base.event = &e->event.base;
	e->event.overlay = overlay;
	e->event.user_data = user_data;
	e->base.file_priv = priv;
	e->base.destroy = (void (*)(struct pvr_pending_event *))kfree;

	do_gettimeofday(&now);
	spin_lock_irqsave(&event_lock, flags);

	if (priv->event_space < sizeof(e->event)) {
		spin_unlock_irqrestore(&event_lock, flags);
		kfree(e);
		return -ENOMEM;
	}

	priv->event_space -= sizeof(e->event);

	list_add_tail(&e->base.link, &flip_wait_list);

	spin_unlock_irqrestore(&event_lock, flags);

	return omap_dss_request_notify(e->dss_event, overlay);
}

static bool pvr_dequeue_event(struct PVRSRV_FILE_PRIVATE_DATA *priv,
		size_t total, size_t max, struct pvr_pending_event **out)
{
	struct pvr_pending_event *e;
	unsigned long flags;
	bool ret = false;

	spin_lock_irqsave(&event_lock, flags);

	*out = NULL;
	if (list_empty(&priv->event_list))
		goto err;

	e = list_first_entry(&priv->event_list, struct pvr_pending_event, link);
	if (e->event->length + total > max)
		goto err;

	priv->event_space += e->event->length;
	list_del(&e->link);
	*out = e;
	ret = true;

err:
	spin_unlock_irqrestore(&event_lock, flags);

	return ret;
}

static void trace_pvr_event(struct pvr_pending_event *e)
{
	struct PVRSRV_PER_PROCESS_DATA *proc;

	proc = e->file_priv->proc;
	pvr_trcmd_lock();

	switch (e->event->type) {
	case PVR_EVENT_SYNC:
	{
		struct pvr_event_sync *evsync;
		struct pvr_trcmd_syn *ts;

		evsync = (struct pvr_event_sync *)e->event;
		ts = pvr_trcmd_alloc(PVR_TRCMD_SGX_QBLT_SYNCOMP,
				     proc->ui32PID, proc->name, sizeof(*ts));
		pvr_trcmd_set_syn(ts, evsync->sync_info);
		break;
	}
	case PVR_EVENT_FLIP:
		pvr_trcmd_alloc(PVR_TRCMD_SGX_QBLT_FLPCOMP,
				proc->ui32PID, proc->name, 0);
		break;
	case PVR_EVENT_UPDATE:
		pvr_trcmd_alloc(PVR_TRCMD_SGX_QBLT_UPDCOMP,
				proc->ui32PID, proc->name, 0);
		break;
	default:
		__WARN();
	}

	pvr_trcmd_unlock();
}

ssize_t pvr_read(struct file *filp, char __user *buf, size_t count, loff_t *off)
{
	struct PVRSRV_FILE_PRIVATE_DATA *priv = filp->private_data;
	struct pvr_pending_event *e;
	size_t total;
	ssize_t ret;

	ret = wait_event_interruptible(priv->event_wait,
					!list_empty(&priv->event_list));

	if (ret < 0)
		return ret;

	total = 0;
	while (pvr_dequeue_event(priv, total, count, &e)) {
		if (copy_to_user(buf + total, e->event, e->event->length)) {
			total = -EFAULT;
			break;
		}

		trace_pvr_event(e);

		total += e->event->length;
		e->destroy(e);
	}

	return total;
}

unsigned int pvr_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct PVRSRV_FILE_PRIVATE_DATA *priv = filp->private_data;
	unsigned int mask = 0;

	poll_wait(filp, &priv->event_wait, wait);

	if (!list_empty(&priv->event_list))
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

void pvr_handle_sync_events(void)
{
	struct pvr_pending_sync_event *e;
	struct pvr_pending_sync_event *t;
	struct timeval now;
	unsigned long flags;

	do_gettimeofday(&now);

	spin_lock_irqsave(&event_lock, flags);

	list_for_each_entry_safe(e, t, &sync_wait_list, base.link) {
		if (!is_render_complete(e->event.sync_info->psSyncData,
					e->base.write_ops_pending))
			continue;

		pvr_signal_sync_event(e, &now);
	}

	spin_unlock_irqrestore(&event_lock, flags);
}

static int dss_notifier_callback(struct notifier_block *nb,
				 unsigned long event, void *data)
{
	struct pvr_pending_flip_event *e;
	struct pvr_pending_flip_event *t;
	struct timeval now;
	unsigned long flags;
	long overlay = (long) data;

	do_gettimeofday(&now);

	spin_lock_irqsave(&event_lock, flags);

	list_for_each_entry_safe(e, t, &flip_wait_list, base.link) {
		if (!(e->dss_event & event))
			continue;
		if (e->event.overlay != overlay)
			continue;

		pvr_signal_flip_event(e, &now);
	}

	spin_unlock_irqrestore(&event_lock, flags);

	return 0;
}

void pvr_release_events(struct PVRSRV_FILE_PRIVATE_DATA *priv)
{
	struct pvr_pending_event *w;
	struct pvr_pending_event *z;
	struct pvr_pending_sync_event *e;
	struct pvr_pending_sync_event *t;
	struct pvr_pending_flip_event *e2;
	struct pvr_pending_flip_event *t2;
	unsigned long flags;

	spin_lock_irqsave(&event_lock, flags);

	/* Remove pending waits */
	list_for_each_entry_safe(e, t, &sync_wait_list, base.link)
		if (e->base.file_priv == priv) {
			list_del(&e->base.link);
			e->base.destroy(&e->base);
		}

	list_for_each_entry_safe(e2, t2, &flip_wait_list, base.link)
		if (e2->base.file_priv == priv) {
			list_del(&e2->base.link);
			e2->base.destroy(&e2->base);
		}

	/* Remove unconsumed events */
	list_for_each_entry_safe(w, z, &priv->event_list, link)
		w->destroy(w);

	spin_unlock_irqrestore(&event_lock, flags);
}

void pvr_init_events(void)
{
	spin_lock_init(&event_lock);
	INIT_LIST_HEAD(&sync_wait_list);
	INIT_LIST_HEAD(&flip_wait_list);

	dss_nb.notifier_call = dss_notifier_callback;
	omap_dss_register_notifier(&dss_nb);
}

void pvr_exit_events(void)
{
	omap_dss_unregister_notifier(&dss_nb);
}
