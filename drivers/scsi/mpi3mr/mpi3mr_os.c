// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Broadcom MPI3 Storage Controllers
 *
 * Copyright (C) 2017-2023 Broadcom Inc.
 *  (mailto: mpi3mr-linuxdrv.pdl@broadcom.com)
 *
 */

#include "mpi3mr.h"
#include <linux/idr.h>

/* global driver scop variables */
LIST_HEAD(mrioc_list);
DEFINE_SPINLOCK(mrioc_list_lock);
static DEFINE_IDA(mrioc_ida);
static int warn_non_secure_ctlr;
atomic64_t event_counter;

MODULE_AUTHOR(MPI3MR_DRIVER_AUTHOR);
MODULE_DESCRIPTION(MPI3MR_DRIVER_DESC);
MODULE_LICENSE(MPI3MR_DRIVER_LICENSE);
MODULE_VERSION(MPI3MR_DRIVER_VERSION);

/* Module parameters*/
int prot_mask = -1;
module_param(prot_mask, int, 0);
MODULE_PARM_DESC(prot_mask, "Host protection capabilities mask, def=0x07");

static int prot_guard_mask = 3;
module_param(prot_guard_mask, int, 0);
MODULE_PARM_DESC(prot_guard_mask, " Host protection guard mask, def=3");
static int logging_level;
module_param(logging_level, int, 0);
MODULE_PARM_DESC(logging_level,
	" bits for enabling additional logging info (default=0)");
static int max_sgl_entries = MPI3MR_DEFAULT_SGL_ENTRIES;
module_param(max_sgl_entries, int, 0444);
MODULE_PARM_DESC(max_sgl_entries,
	"Preferred max number of SG entries to be used for a single I/O\n"
	"The actual value will be determined by the driver\n"
	"(Minimum=256, Maximum=2048, default=256)");

/* Forward declarations*/
static void mpi3mr_send_event_ack(struct mpi3mr_ioc *mrioc, u8 event,
	struct mpi3mr_drv_cmd *cmdparam, u32 event_ctx);

#define MPI3MR_DRIVER_EVENT_TG_QD_REDUCTION	(0xFFFF)

#define MPI3_EVENT_WAIT_FOR_DEVICES_TO_REFRESH	(0xFFFE)

/*
 * SAS Log info code for a NCQ collateral abort after an NCQ error:
 * IOC_LOGINFO_PREFIX_PL | PL_LOGINFO_CODE_SATA_NCQ_FAIL_ALL_CMDS_AFTR_ERR
 * See: drivers/message/fusion/lsi/mpi_log_sas.h
 */
#define IOC_LOGINFO_SATA_NCQ_FAIL_AFTER_ERR	0x31080000

/**
 * mpi3mr_host_tag_for_scmd - Get host tag for a scmd
 * @mrioc: Adapter instance reference
 * @scmd: SCSI command reference
 *
 * Calculate the host tag based on block tag for a given scmd.
 *
 * Return: Valid host tag or MPI3MR_HOSTTAG_INVALID.
 */
static u16 mpi3mr_host_tag_for_scmd(struct mpi3mr_ioc *mrioc,
	struct scsi_cmnd *scmd)
{
	struct scmd_priv *priv = NULL;
	u32 unique_tag;
	u16 host_tag, hw_queue;

	unique_tag = blk_mq_unique_tag(scsi_cmd_to_rq(scmd));

	hw_queue = blk_mq_unique_tag_to_hwq(unique_tag);
	if (hw_queue >= mrioc->num_op_reply_q)
		return MPI3MR_HOSTTAG_INVALID;
	host_tag = blk_mq_unique_tag_to_tag(unique_tag);

	if (WARN_ON(host_tag >= mrioc->max_host_ios))
		return MPI3MR_HOSTTAG_INVALID;

	priv = scsi_cmd_priv(scmd);
	/*host_tag 0 is invalid hence incrementing by 1*/
	priv->host_tag = host_tag + 1;
	priv->scmd = scmd;
	priv->in_lld_scope = 1;
	priv->req_q_idx = hw_queue;
	priv->meta_chain_idx = -1;
	priv->chain_idx = -1;
	priv->meta_sg_valid = 0;
	return priv->host_tag;
}

/**
 * mpi3mr_scmd_from_host_tag - Get SCSI command from host tag
 * @mrioc: Adapter instance reference
 * @host_tag: Host tag
 * @qidx: Operational queue index
 *
 * Identify the block tag from the host tag and queue index and
 * retrieve associated scsi command using scsi_host_find_tag().
 *
 * Return: SCSI command reference or NULL.
 */
static struct scsi_cmnd *mpi3mr_scmd_from_host_tag(
	struct mpi3mr_ioc *mrioc, u16 host_tag, u16 qidx)
{
	struct scsi_cmnd *scmd = NULL;
	struct scmd_priv *priv = NULL;
	u32 unique_tag = host_tag - 1;

	if (WARN_ON(host_tag > mrioc->max_host_ios))
		goto out;

	unique_tag |= (qidx << BLK_MQ_UNIQUE_TAG_BITS);

	scmd = scsi_host_find_tag(mrioc->shost, unique_tag);
	if (scmd) {
		priv = scsi_cmd_priv(scmd);
		if (!priv->in_lld_scope)
			scmd = NULL;
	}
out:
	return scmd;
}

/**
 * mpi3mr_clear_scmd_priv - Cleanup SCSI command private date
 * @mrioc: Adapter instance reference
 * @scmd: SCSI command reference
 *
 * Invalidate the SCSI command private data to mark the command
 * is not in LLD scope anymore.
 *
 * Return: Nothing.
 */
static void mpi3mr_clear_scmd_priv(struct mpi3mr_ioc *mrioc,
	struct scsi_cmnd *scmd)
{
	struct scmd_priv *priv = NULL;

	priv = scsi_cmd_priv(scmd);

	if (WARN_ON(priv->in_lld_scope == 0))
		return;
	priv->host_tag = MPI3MR_HOSTTAG_INVALID;
	priv->req_q_idx = 0xFFFF;
	priv->scmd = NULL;
	priv->in_lld_scope = 0;
	priv->meta_sg_valid = 0;
	if (priv->chain_idx >= 0) {
		clear_bit(priv->chain_idx, mrioc->chain_bitmap);
		priv->chain_idx = -1;
	}
	if (priv->meta_chain_idx >= 0) {
		clear_bit(priv->meta_chain_idx, mrioc->chain_bitmap);
		priv->meta_chain_idx = -1;
	}
}

static void mpi3mr_dev_rmhs_send_tm(struct mpi3mr_ioc *mrioc, u16 handle,
	struct mpi3mr_drv_cmd *cmdparam, u8 iou_rc);
static void mpi3mr_fwevt_worker(struct work_struct *work);

/**
 * mpi3mr_fwevt_free - firmware event memory dealloctor
 * @r: k reference pointer of the firmware event
 *
 * Free firmware event memory when no reference.
 */
static void mpi3mr_fwevt_free(struct kref *r)
{
	kfree(container_of(r, struct mpi3mr_fwevt, ref_count));
}

/**
 * mpi3mr_fwevt_get - k reference incrementor
 * @fwevt: Firmware event reference
 *
 * Increment firmware event reference count.
 */
static void mpi3mr_fwevt_get(struct mpi3mr_fwevt *fwevt)
{
	kref_get(&fwevt->ref_count);
}

/**
 * mpi3mr_fwevt_put - k reference decrementor
 * @fwevt: Firmware event reference
 *
 * decrement firmware event reference count.
 */
static void mpi3mr_fwevt_put(struct mpi3mr_fwevt *fwevt)
{
	kref_put(&fwevt->ref_count, mpi3mr_fwevt_free);
}

/**
 * mpi3mr_alloc_fwevt - Allocate firmware event
 * @len: length of firmware event data to allocate
 *
 * Allocate firmware event with required length and initialize
 * the reference counter.
 *
 * Return: firmware event reference.
 */
static struct mpi3mr_fwevt *mpi3mr_alloc_fwevt(int len)
{
	struct mpi3mr_fwevt *fwevt;

	fwevt = kzalloc(sizeof(*fwevt) + len, GFP_ATOMIC);
	if (!fwevt)
		return NULL;

	kref_init(&fwevt->ref_count);
	return fwevt;
}

/**
 * mpi3mr_fwevt_add_to_list - Add firmware event to the list
 * @mrioc: Adapter instance reference
 * @fwevt: Firmware event reference
 *
 * Add the given firmware event to the firmware event list.
 *
 * Return: Nothing.
 */
static void mpi3mr_fwevt_add_to_list(struct mpi3mr_ioc *mrioc,
	struct mpi3mr_fwevt *fwevt)
{
	unsigned long flags;

	if (!mrioc->fwevt_worker_thread)
		return;

	spin_lock_irqsave(&mrioc->fwevt_lock, flags);
	/* get fwevt reference count while adding it to fwevt_list */
	mpi3mr_fwevt_get(fwevt);
	INIT_LIST_HEAD(&fwevt->list);
	list_add_tail(&fwevt->list, &mrioc->fwevt_list);
	INIT_WORK(&fwevt->work, mpi3mr_fwevt_worker);
	/* get fwevt reference count while enqueueing it to worker queue */
	mpi3mr_fwevt_get(fwevt);
	queue_work(mrioc->fwevt_worker_thread, &fwevt->work);
	spin_unlock_irqrestore(&mrioc->fwevt_lock, flags);
}

/**
 * mpi3mr_hdb_trigger_data_event - Add hdb trigger data event to
 * the list
 * @mrioc: Adapter instance reference
 * @event_data: Event data
 *
 * Add the given hdb trigger data event to the firmware event
 * list.
 *
 * Return: Nothing.
 */
void mpi3mr_hdb_trigger_data_event(struct mpi3mr_ioc *mrioc,
	struct trigger_event_data *event_data)
{
	struct mpi3mr_fwevt *fwevt;
	u16 sz = sizeof(*event_data);

	fwevt = mpi3mr_alloc_fwevt(sz);
	if (!fwevt) {
		ioc_warn(mrioc, "failed to queue hdb trigger data event\n");
		return;
	}

	fwevt->mrioc = mrioc;
	fwevt->event_id = MPI3MR_DRIVER_EVENT_PROCESS_TRIGGER;
	fwevt->send_ack = 0;
	fwevt->process_evt = 1;
	fwevt->evt_ctx = 0;
	fwevt->event_data_size = sz;
	memcpy(fwevt->event_data, event_data, sz);

	mpi3mr_fwevt_add_to_list(mrioc, fwevt);
}

/**
 * mpi3mr_fwevt_del_from_list - Delete firmware event from list
 * @mrioc: Adapter instance reference
 * @fwevt: Firmware event reference
 *
 * Delete the given firmware event from the firmware event list.
 *
 * Return: Nothing.
 */
static void mpi3mr_fwevt_del_from_list(struct mpi3mr_ioc *mrioc,
	struct mpi3mr_fwevt *fwevt)
{
	unsigned long flags;

	spin_lock_irqsave(&mrioc->fwevt_lock, flags);
	if (!list_empty(&fwevt->list)) {
		list_del_init(&fwevt->list);
		/*
		 * Put fwevt reference count after
		 * removing it from fwevt_list
		 */
		mpi3mr_fwevt_put(fwevt);
	}
	spin_unlock_irqrestore(&mrioc->fwevt_lock, flags);
}

/**
 * mpi3mr_dequeue_fwevt - Dequeue firmware event from the list
 * @mrioc: Adapter instance reference
 *
 * Dequeue a firmware event from the firmware event list.
 *
 * Return: firmware event.
 */
static struct mpi3mr_fwevt *mpi3mr_dequeue_fwevt(
	struct mpi3mr_ioc *mrioc)
{
	unsigned long flags;
	struct mpi3mr_fwevt *fwevt = NULL;

	spin_lock_irqsave(&mrioc->fwevt_lock, flags);
	if (!list_empty(&mrioc->fwevt_list)) {
		fwevt = list_first_entry(&mrioc->fwevt_list,
		    struct mpi3mr_fwevt, list);
		list_del_init(&fwevt->list);
		/*
		 * Put fwevt reference count after
		 * removing it from fwevt_list
		 */
		mpi3mr_fwevt_put(fwevt);
	}
	spin_unlock_irqrestore(&mrioc->fwevt_lock, flags);

	return fwevt;
}

/**
 * mpi3mr_cancel_work - cancel firmware event
 * @fwevt: fwevt object which needs to be canceled
 *
 * Return: Nothing.
 */
static void mpi3mr_cancel_work(struct mpi3mr_fwevt *fwevt)
{
	/*
	 * Wait on the fwevt to complete. If this returns 1, then
	 * the event was never executed.
	 *
	 * If it did execute, we wait for it to finish, and the put will
	 * happen from mpi3mr_process_fwevt()
	 */
	if (cancel_work_sync(&fwevt->work)) {
		/*
		 * Put fwevt reference count after
		 * dequeuing it from worker queue
		 */
		mpi3mr_fwevt_put(fwevt);
		/*
		 * Put fwevt reference count to neutralize
		 * kref_init increment
		 */
		mpi3mr_fwevt_put(fwevt);
	}
}

/**
 * mpi3mr_cleanup_fwevt_list - Cleanup firmware event list
 * @mrioc: Adapter instance reference
 *
 * Flush all pending firmware events from the firmware event
 * list.
 *
 * Return: Nothing.
 */
void mpi3mr_cleanup_fwevt_list(struct mpi3mr_ioc *mrioc)
{
	struct mpi3mr_fwevt *fwevt = NULL;

	if ((list_empty(&mrioc->fwevt_list) && !mrioc->current_event) ||
	    !mrioc->fwevt_worker_thread)
		return;

	while ((fwevt = mpi3mr_dequeue_fwevt(mrioc)))
		mpi3mr_cancel_work(fwevt);

	if (mrioc->current_event) {
		fwevt = mrioc->current_event;
		/*
		 * Don't call cancel_work_sync() API for the
		 * fwevt work if the controller reset is
		 * get called as part of processing the
		 * same fwevt work (or) when worker thread is
		 * waiting for device add/remove APIs to complete.
		 * Otherwise we will see deadlock.
		 */
		if (current_work() == &fwevt->work || fwevt->pending_at_sml) {
			fwevt->discard = 1;
			return;
		}

		mpi3mr_cancel_work(fwevt);
	}
}

/**
 * mpi3mr_queue_qd_reduction_event - Queue TG QD reduction event
 * @mrioc: Adapter instance reference
 * @tg: Throttle group information pointer
 *
 * Accessor to queue on synthetically generated driver event to
 * the event worker thread, the driver event will be used to
 * reduce the QD of all VDs in the TG from the worker thread.
 *
 * Return: None.
 */
static void mpi3mr_queue_qd_reduction_event(struct mpi3mr_ioc *mrioc,
	struct mpi3mr_throttle_group_info *tg)
{
	struct mpi3mr_fwevt *fwevt;
	u16 sz = sizeof(struct mpi3mr_throttle_group_info *);

	/*
	 * If the QD reduction event is already queued due to throttle and if
	 * the QD is not restored through device info change event
	 * then dont queue further reduction events
	 */
	if (tg->fw_qd != tg->modified_qd)
		return;

	fwevt = mpi3mr_alloc_fwevt(sz);
	if (!fwevt) {
		ioc_warn(mrioc, "failed to queue TG QD reduction event\n");
		return;
	}
	*(struct mpi3mr_throttle_group_info **)fwevt->event_data = tg;
	fwevt->mrioc = mrioc;
	fwevt->event_id = MPI3MR_DRIVER_EVENT_TG_QD_REDUCTION;
	fwevt->send_ack = 0;
	fwevt->process_evt = 1;
	fwevt->evt_ctx = 0;
	fwevt->event_data_size = sz;
	tg->modified_qd = max_t(u16, (tg->fw_qd * tg->qd_reduction) / 10, 8);

	dprint_event_bh(mrioc, "qd reduction event queued for tg_id(%d)\n",
	    tg->id);
	mpi3mr_fwevt_add_to_list(mrioc, fwevt);
}

/**
 * mpi3mr_invalidate_devhandles -Invalidate device handles
 * @mrioc: Adapter instance reference
 *
 * Invalidate the device handles in the target device structures
 * . Called post reset prior to reinitializing the controller.
 *
 * Return: Nothing.
 */
void mpi3mr_invalidate_devhandles(struct mpi3mr_ioc *mrioc)
{
	struct mpi3mr_tgt_dev *tgtdev;
	struct mpi3mr_stgt_priv_data *tgt_priv;

	list_for_each_entry(tgtdev, &mrioc->tgtdev_list, list) {
		tgtdev->dev_handle = MPI3MR_INVALID_DEV_HANDLE;
		if (tgtdev->starget && tgtdev->starget->hostdata) {
			tgt_priv = tgtdev->starget->hostdata;
			tgt_priv->dev_handle = MPI3MR_INVALID_DEV_HANDLE;
			tgt_priv->io_throttle_enabled = 0;
			tgt_priv->io_divert = 0;
			tgt_priv->throttle_group = NULL;
			tgt_priv->wslen = 0;
			if (tgtdev->host_exposed)
				atomic_set(&tgt_priv->block_io, 1);
		}
	}
}

/**
 * mpi3mr_print_scmd - print individual SCSI command
 * @rq: Block request
 * @data: Adapter instance reference
 *
 * Print the SCSI command details if it is in LLD scope.
 *
 * Return: true always.
 */
static bool mpi3mr_print_scmd(struct request *rq, void *data)
{
	struct mpi3mr_ioc *mrioc = (struct mpi3mr_ioc *)data;
	struct scsi_cmnd *scmd = blk_mq_rq_to_pdu(rq);
	struct scmd_priv *priv = NULL;

	if (scmd) {
		priv = scsi_cmd_priv(scmd);
		if (!priv->in_lld_scope)
			goto out;

		ioc_info(mrioc, "%s :Host Tag = %d, qid = %d\n",
		    __func__, priv->host_tag, priv->req_q_idx + 1);
		scsi_print_command(scmd);
	}

out:
	return(true);
}

/**
 * mpi3mr_flush_scmd - Flush individual SCSI command
 * @rq: Block request
 * @data: Adapter instance reference
 *
 * Return the SCSI command to the upper layers if it is in LLD
 * scope.
 *
 * Return: true always.
 */

static bool mpi3mr_flush_scmd(struct request *rq, void *data)
{
	struct mpi3mr_ioc *mrioc = (struct mpi3mr_ioc *)data;
	struct scsi_cmnd *scmd = blk_mq_rq_to_pdu(rq);
	struct scmd_priv *priv = NULL;

	if (scmd) {
		priv = scsi_cmd_priv(scmd);
		if (!priv->in_lld_scope)
			goto out;

		if (priv->meta_sg_valid)
			dma_unmap_sg(&mrioc->pdev->dev, scsi_prot_sglist(scmd),
			    scsi_prot_sg_count(scmd), scmd->sc_data_direction);
		mpi3mr_clear_scmd_priv(mrioc, scmd);
		scsi_dma_unmap(scmd);
		scmd->result = DID_RESET << 16;
		scsi_print_command(scmd);
		scsi_done(scmd);
		mrioc->flush_io_count++;
	}

out:
	return(true);
}

/**
 * mpi3mr_count_dev_pending - Count commands pending for a lun
 * @rq: Block request
 * @data: SCSI device reference
 *
 * This is an iterator function called for each SCSI command in
 * a host and if the command is pending in the LLD for the
 * specific device(lun) then device specific pending I/O counter
 * is updated in the device structure.
 *
 * Return: true always.
 */

static bool mpi3mr_count_dev_pending(struct request *rq, void *data)
{
	struct scsi_device *sdev = (struct scsi_device *)data;
	struct mpi3mr_sdev_priv_data *sdev_priv_data = sdev->hostdata;
	struct scsi_cmnd *scmd = blk_mq_rq_to_pdu(rq);
	struct scmd_priv *priv;

	if (scmd) {
		priv = scsi_cmd_priv(scmd);
		if (!priv->in_lld_scope)
			goto out;
		if (scmd->device == sdev)
			sdev_priv_data->pend_count++;
	}

out:
	return true;
}

/**
 * mpi3mr_count_tgt_pending - Count commands pending for target
 * @rq: Block request
 * @data: SCSI target reference
 *
 * This is an iterator function called for each SCSI command in
 * a host and if the command is pending in the LLD for the
 * specific target then target specific pending I/O counter is
 * updated in the target structure.
 *
 * Return: true always.
 */

static bool mpi3mr_count_tgt_pending(struct request *rq, void *data)
{
	struct scsi_target *starget = (struct scsi_target *)data;
	struct mpi3mr_stgt_priv_data *stgt_priv_data = starget->hostdata;
	struct scsi_cmnd *scmd = blk_mq_rq_to_pdu(rq);
	struct scmd_priv *priv;

	if (scmd) {
		priv = scsi_cmd_priv(scmd);
		if (!priv->in_lld_scope)
			goto out;
		if (scmd->device && (scsi_target(scmd->device) == starget))
			stgt_priv_data->pend_count++;
	}

out:
	return true;
}

/**
 * mpi3mr_flush_host_io -  Flush host I/Os
 * @mrioc: Adapter instance reference
 *
 * Flush all of the pending I/Os by calling
 * blk_mq_tagset_busy_iter() for each possible tag. This is
 * executed post controller reset
 *
 * Return: Nothing.
 */
void mpi3mr_flush_host_io(struct mpi3mr_ioc *mrioc)
{
	struct Scsi_Host *shost = mrioc->shost;

	mrioc->flush_io_count = 0;
	ioc_info(mrioc, "%s :Flushing Host I/O cmds post reset\n", __func__);
	blk_mq_tagset_busy_iter(&shost->tag_set,
	    mpi3mr_flush_scmd, (void *)mrioc);
	ioc_info(mrioc, "%s :Flushed %d Host I/O cmds\n", __func__,
	    mrioc->flush_io_count);
}

/**
 * mpi3mr_flush_cmds_for_unrecovered_controller - Flush all pending cmds
 * @mrioc: Adapter instance reference
 *
 * This function waits for currently running IO poll threads to
 * exit and then flushes all host I/Os and any internal pending
 * cmds. This is executed after controller is marked as
 * unrecoverable.
 *
 * Return: Nothing.
 */
void mpi3mr_flush_cmds_for_unrecovered_controller(struct mpi3mr_ioc *mrioc)
{
	struct Scsi_Host *shost = mrioc->shost;
	int i;

	if (!mrioc->unrecoverable)
		return;

	if (mrioc->op_reply_qinfo) {
		for (i = 0; i < mrioc->num_queues; i++) {
			while (atomic_read(&mrioc->op_reply_qinfo[i].in_use))
				udelay(500);
			atomic_set(&mrioc->op_reply_qinfo[i].pend_ios, 0);
		}
	}
	mrioc->flush_io_count = 0;
	blk_mq_tagset_busy_iter(&shost->tag_set,
	    mpi3mr_flush_scmd, (void *)mrioc);
	mpi3mr_flush_delayed_cmd_lists(mrioc);
	mpi3mr_flush_drv_cmds(mrioc);
}

/**
 * mpi3mr_alloc_tgtdev - target device allocator
 *
 * Allocate target device instance and initialize the reference
 * count
 *
 * Return: target device instance.
 */
static struct mpi3mr_tgt_dev *mpi3mr_alloc_tgtdev(void)
{
	struct mpi3mr_tgt_dev *tgtdev;

	tgtdev = kzalloc(sizeof(*tgtdev), GFP_ATOMIC);
	if (!tgtdev)
		return NULL;
	kref_init(&tgtdev->ref_count);
	return tgtdev;
}

/**
 * mpi3mr_tgtdev_add_to_list -Add tgtdevice to the list
 * @mrioc: Adapter instance reference
 * @tgtdev: Target device
 *
 * Add the target device to the target device list
 *
 * Return: Nothing.
 */
static void mpi3mr_tgtdev_add_to_list(struct mpi3mr_ioc *mrioc,
	struct mpi3mr_tgt_dev *tgtdev)
{
	unsigned long flags;

	spin_lock_irqsave(&mrioc->tgtdev_lock, flags);
	mpi3mr_tgtdev_get(tgtdev);
	INIT_LIST_HEAD(&tgtdev->list);
	list_add_tail(&tgtdev->list, &mrioc->tgtdev_list);
	tgtdev->state = MPI3MR_DEV_CREATED;
	spin_unlock_irqrestore(&mrioc->tgtdev_lock, flags);
}

/**
 * mpi3mr_tgtdev_del_from_list -Delete tgtdevice from the list
 * @mrioc: Adapter instance reference
 * @tgtdev: Target device
 * @must_delete: Must delete the target device from the list irrespective
 * of the device state.
 *
 * Remove the target device from the target device list
 *
 * Return: Nothing.
 */
static void mpi3mr_tgtdev_del_from_list(struct mpi3mr_ioc *mrioc,
	struct mpi3mr_tgt_dev *tgtdev, bool must_delete)
{
	unsigned long flags;

	spin_lock_irqsave(&mrioc->tgtdev_lock, flags);
	if ((tgtdev->state == MPI3MR_DEV_REMOVE_HS_STARTED) || (must_delete == true)) {
		if (!list_empty(&tgtdev->list)) {
			list_del_init(&tgtdev->list);
			tgtdev->state = MPI3MR_DEV_DELETED;
			mpi3mr_tgtdev_put(tgtdev);
		}
	}
	spin_unlock_irqrestore(&mrioc->tgtdev_lock, flags);
}

/**
 * __mpi3mr_get_tgtdev_by_handle -Get tgtdev from device handle
 * @mrioc: Adapter instance reference
 * @handle: Device handle
 *
 * Accessor to retrieve target device from the device handle.
 * Non Lock version
 *
 * Return: Target device reference.
 */
static struct mpi3mr_tgt_dev  *__mpi3mr_get_tgtdev_by_handle(
	struct mpi3mr_ioc *mrioc, u16 handle)
{
	struct mpi3mr_tgt_dev *tgtdev;

	assert_spin_locked(&mrioc->tgtdev_lock);
	list_for_each_entry(tgtdev, &mrioc->tgtdev_list, list)
		if (tgtdev->dev_handle == handle)
			goto found_tgtdev;
	return NULL;

found_tgtdev:
	mpi3mr_tgtdev_get(tgtdev);
	return tgtdev;
}

/**
 * mpi3mr_get_tgtdev_by_handle -Get tgtdev from device handle
 * @mrioc: Adapter instance reference
 * @handle: Device handle
 *
 * Accessor to retrieve target device from the device handle.
 * Lock version
 *
 * Return: Target device reference.
 */
struct mpi3mr_tgt_dev *mpi3mr_get_tgtdev_by_handle(
	struct mpi3mr_ioc *mrioc, u16 handle)
{
	struct mpi3mr_tgt_dev *tgtdev;
	unsigned long flags;

	spin_lock_irqsave(&mrioc->tgtdev_lock, flags);
	tgtdev = __mpi3mr_get_tgtdev_by_handle(mrioc, handle);
	spin_unlock_irqrestore(&mrioc->tgtdev_lock, flags);
	return tgtdev;
}

/**
 * __mpi3mr_get_tgtdev_by_perst_id -Get tgtdev from persist ID
 * @mrioc: Adapter instance reference
 * @persist_id: Persistent ID
 *
 * Accessor to retrieve target device from the Persistent ID.
 * Non Lock version
 *
 * Return: Target device reference.
 */
static struct mpi3mr_tgt_dev  *__mpi3mr_get_tgtdev_by_perst_id(
	struct mpi3mr_ioc *mrioc, u16 persist_id)
{
	struct mpi3mr_tgt_dev *tgtdev;

	assert_spin_locked(&mrioc->tgtdev_lock);
	list_for_each_entry(tgtdev, &mrioc->tgtdev_list, list)
		if (tgtdev->perst_id == persist_id)
			goto found_tgtdev;
	return NULL;

found_tgtdev:
	mpi3mr_tgtdev_get(tgtdev);
	return tgtdev;
}

/**
 * mpi3mr_get_tgtdev_by_perst_id -Get tgtdev from persistent ID
 * @mrioc: Adapter instance reference
 * @persist_id: Persistent ID
 *
 * Accessor to retrieve target device from the Persistent ID.
 * Lock version
 *
 * Return: Target device reference.
 */
static struct mpi3mr_tgt_dev *mpi3mr_get_tgtdev_by_perst_id(
	struct mpi3mr_ioc *mrioc, u16 persist_id)
{
	struct mpi3mr_tgt_dev *tgtdev;
	unsigned long flags;

	spin_lock_irqsave(&mrioc->tgtdev_lock, flags);
	tgtdev = __mpi3mr_get_tgtdev_by_perst_id(mrioc, persist_id);
	spin_unlock_irqrestore(&mrioc->tgtdev_lock, flags);
	return tgtdev;
}

/**
 * __mpi3mr_get_tgtdev_from_tgtpriv -Get tgtdev from tgt private
 * @mrioc: Adapter instance reference
 * @tgt_priv: Target private data
 *
 * Accessor to return target device from the target private
 * data. Non Lock version
 *
 * Return: Target device reference.
 */
static struct mpi3mr_tgt_dev  *__mpi3mr_get_tgtdev_from_tgtpriv(
	struct mpi3mr_ioc *mrioc, struct mpi3mr_stgt_priv_data *tgt_priv)
{
	struct mpi3mr_tgt_dev *tgtdev;

	assert_spin_locked(&mrioc->tgtdev_lock);
	tgtdev = tgt_priv->tgt_dev;
	if (tgtdev)
		mpi3mr_tgtdev_get(tgtdev);
	return tgtdev;
}

/**
 * mpi3mr_set_io_divert_for_all_vd_in_tg -set divert for TG VDs
 * @mrioc: Adapter instance reference
 * @tg: Throttle group information pointer
 * @divert_value: 1 or 0
 *
 * Accessor to set io_divert flag for each device associated
 * with the given throttle group with the given value.
 *
 * Return: None.
 */
static void mpi3mr_set_io_divert_for_all_vd_in_tg(struct mpi3mr_ioc *mrioc,
	struct mpi3mr_throttle_group_info *tg, u8 divert_value)
{
	unsigned long flags;
	struct mpi3mr_tgt_dev *tgtdev;
	struct mpi3mr_stgt_priv_data *tgt_priv;

	spin_lock_irqsave(&mrioc->tgtdev_lock, flags);
	list_for_each_entry(tgtdev, &mrioc->tgtdev_list, list) {
		if (tgtdev->starget && tgtdev->starget->hostdata) {
			tgt_priv = tgtdev->starget->hostdata;
			if (tgt_priv->throttle_group == tg)
				tgt_priv->io_divert = divert_value;
		}
	}
	spin_unlock_irqrestore(&mrioc->tgtdev_lock, flags);
}

/**
 * mpi3mr_print_device_event_notice - print notice related to post processing of
 *					device event after controller reset.
 *
 * @mrioc: Adapter instance reference
 * @device_add: true for device add event and false for device removal event
 *
 * Return: None.
 */
void mpi3mr_print_device_event_notice(struct mpi3mr_ioc *mrioc,
	bool device_add)
{
	ioc_notice(mrioc, "Device %s was in progress before the reset and\n",
	    (device_add ? "addition" : "removal"));
	ioc_notice(mrioc, "completed after reset, verify whether the exposed devices\n");
	ioc_notice(mrioc, "are matched with attached devices for correctness\n");
}

/**
 * mpi3mr_remove_tgtdev_from_host - Remove dev from upper layers
 * @mrioc: Adapter instance reference
 * @tgtdev: Target device structure
 *
 * Checks whether the device is exposed to upper layers and if it
 * is then remove the device from upper layers by calling
 * scsi_remove_target().
 *
 * Return: 0 on success, non zero on failure.
 */
void mpi3mr_remove_tgtdev_from_host(struct mpi3mr_ioc *mrioc,
	struct mpi3mr_tgt_dev *tgtdev)
{
	struct mpi3mr_stgt_priv_data *tgt_priv;

	ioc_info(mrioc, "%s :Removing handle(0x%04x), wwid(0x%016llx)\n",
	    __func__, tgtdev->dev_handle, (unsigned long long)tgtdev->wwid);
	if (tgtdev->starget && tgtdev->starget->hostdata) {
		tgt_priv = tgtdev->starget->hostdata;
		atomic_set(&tgt_priv->block_io, 0);
		tgt_priv->dev_handle = MPI3MR_INVALID_DEV_HANDLE;
	}

	if (!mrioc->sas_transport_enabled || (tgtdev->dev_type !=
	    MPI3_DEVICE_DEVFORM_SAS_SATA) || tgtdev->non_stl) {
		if (tgtdev->starget) {
			if (mrioc->current_event)
				mrioc->current_event->pending_at_sml = 1;
			scsi_remove_target(&tgtdev->starget->dev);
			tgtdev->host_exposed = 0;
			if (mrioc->current_event) {
				mrioc->current_event->pending_at_sml = 0;
				if (mrioc->current_event->discard) {
					mpi3mr_print_device_event_notice(mrioc,
					    false);
					return;
				}
			}
		}
	} else
		mpi3mr_remove_tgtdev_from_sas_transport(mrioc, tgtdev);
	mpi3mr_global_trigger(mrioc,
	    MPI3_DRIVER2_GLOBALTRIGGER_DEVICE_REMOVAL_ENABLED);

	ioc_info(mrioc, "%s :Removed handle(0x%04x), wwid(0x%016llx)\n",
	    __func__, tgtdev->dev_handle, (unsigned long long)tgtdev->wwid);
}

/**
 * mpi3mr_report_tgtdev_to_host - Expose device to upper layers
 * @mrioc: Adapter instance reference
 * @perst_id: Persistent ID of the device
 *
 * Checks whether the device can be exposed to upper layers and
 * if it is not then expose the device to upper layers by
 * calling scsi_scan_target().
 *
 * Return: 0 on success, non zero on failure.
 */
static int mpi3mr_report_tgtdev_to_host(struct mpi3mr_ioc *mrioc,
	u16 perst_id)
{
	int retval = 0;
	struct mpi3mr_tgt_dev *tgtdev;

	if (mrioc->reset_in_progress || mrioc->pci_err_recovery)
		return -1;

	tgtdev = mpi3mr_get_tgtdev_by_perst_id(mrioc, perst_id);
	if (!tgtdev) {
		retval = -1;
		goto out;
	}
	if (tgtdev->is_hidden || tgtdev->host_exposed) {
		retval = -1;
		goto out;
	}
	if (!mrioc->sas_transport_enabled || (tgtdev->dev_type !=
	    MPI3_DEVICE_DEVFORM_SAS_SATA) || tgtdev->non_stl){
		tgtdev->host_exposed = 1;
		if (mrioc->current_event)
			mrioc->current_event->pending_at_sml = 1;
		scsi_scan_target(&mrioc->shost->shost_gendev,
		    mrioc->scsi_device_channel, tgtdev->perst_id,
		    SCAN_WILD_CARD, SCSI_SCAN_INITIAL);
		if (!tgtdev->starget)
			tgtdev->host_exposed = 0;
		if (mrioc->current_event) {
			mrioc->current_event->pending_at_sml = 0;
			if (mrioc->current_event->discard) {
				mpi3mr_print_device_event_notice(mrioc, true);
				goto out;
			}
		}
		dprint_event_bh(mrioc,
		    "exposed target device with handle(0x%04x), perst_id(%d)\n",
		    tgtdev->dev_handle, perst_id);
		goto out;
	} else
		mpi3mr_report_tgtdev_to_sas_transport(mrioc, tgtdev);
out:
	if (tgtdev)
		mpi3mr_tgtdev_put(tgtdev);

	return retval;
}

/**
 * mpi3mr_change_queue_depth- Change QD callback handler
 * @sdev: SCSI device reference
 * @q_depth: Queue depth
 *
 * Validate and limit QD and call scsi_change_queue_depth.
 *
 * Return: return value of scsi_change_queue_depth
 */
static int mpi3mr_change_queue_depth(struct scsi_device *sdev,
	int q_depth)
{
	struct scsi_target *starget = scsi_target(sdev);
	struct Scsi_Host *shost = dev_to_shost(&starget->dev);
	int retval = 0;

	if (!sdev->tagged_supported)
		q_depth = 1;
	if (q_depth > shost->can_queue)
		q_depth = shost->can_queue;
	else if (!q_depth)
		q_depth = MPI3MR_DEFAULT_SDEV_QD;
	retval = scsi_change_queue_depth(sdev, q_depth);
	sdev->max_queue_depth = sdev->queue_depth;

	return retval;
}

static void mpi3mr_configure_nvme_dev(struct mpi3mr_tgt_dev *tgt_dev,
		struct queue_limits *lim)
{
	u8 pgsz = tgt_dev->dev_spec.pcie_inf.pgsz ? : MPI3MR_DEFAULT_PGSZEXP;

	lim->max_hw_sectors = tgt_dev->dev_spec.pcie_inf.mdts / 512;
	lim->virt_boundary_mask = (1 << pgsz) - 1;
}

static void mpi3mr_configure_tgt_dev(struct mpi3mr_tgt_dev *tgt_dev,
		struct queue_limits *lim)
{
	if (tgt_dev->dev_type == MPI3_DEVICE_DEVFORM_PCIE &&
	    (tgt_dev->dev_spec.pcie_inf.dev_info &
	     MPI3_DEVICE0_PCIE_DEVICE_INFO_TYPE_MASK) ==
			MPI3_DEVICE0_PCIE_DEVICE_INFO_TYPE_NVME_DEVICE)
		mpi3mr_configure_nvme_dev(tgt_dev, lim);
}

/**
 * mpi3mr_update_sdev - Update SCSI device information
 * @sdev: SCSI device reference
 * @data: target device reference
 *
 * This is an iterator function called for each SCSI device in a
 * target to update the target specific information into each
 * SCSI device.
 *
 * Return: Nothing.
 */
static void
mpi3mr_update_sdev(struct scsi_device *sdev, void *data)
{
	struct mpi3mr_tgt_dev *tgtdev;
	struct queue_limits lim;

	tgtdev = (struct mpi3mr_tgt_dev *)data;
	if (!tgtdev)
		return;

	mpi3mr_change_queue_depth(sdev, tgtdev->q_depth);

	lim = queue_limits_start_update(sdev->request_queue);
	mpi3mr_configure_tgt_dev(tgtdev, &lim);
	WARN_ON_ONCE(queue_limits_commit_update(sdev->request_queue, &lim));
}

/**
 * mpi3mr_refresh_tgtdevs - Refresh target device exposure
 * @mrioc: Adapter instance reference
 *
 * This is executed post controller reset to identify any
 * missing devices during reset and remove from the upper layers
 * or expose any newly detected device to the upper layers.
 *
 * Return: Nothing.
 */
static void mpi3mr_refresh_tgtdevs(struct mpi3mr_ioc *mrioc)
{
	struct mpi3mr_tgt_dev *tgtdev, *tgtdev_next;
	struct mpi3mr_stgt_priv_data *tgt_priv;

	dprint_reset(mrioc, "refresh target devices: check for removals\n");
	list_for_each_entry_safe(tgtdev, tgtdev_next, &mrioc->tgtdev_list,
	    list) {
		if (((tgtdev->dev_handle == MPI3MR_INVALID_DEV_HANDLE) ||
		     tgtdev->is_hidden) &&
		     tgtdev->host_exposed && tgtdev->starget &&
		     tgtdev->starget->hostdata) {
			tgt_priv = tgtdev->starget->hostdata;
			tgt_priv->dev_removed = 1;
			atomic_set(&tgt_priv->block_io, 0);
		}
	}

	list_for_each_entry_safe(tgtdev, tgtdev_next, &mrioc->tgtdev_list,
	    list) {
		if (tgtdev->dev_handle == MPI3MR_INVALID_DEV_HANDLE) {
			dprint_reset(mrioc, "removing target device with perst_id(%d)\n",
			    tgtdev->perst_id);
			if (tgtdev->host_exposed)
				mpi3mr_remove_tgtdev_from_host(mrioc, tgtdev);
			mpi3mr_tgtdev_del_from_list(mrioc, tgtdev, true);
			mpi3mr_tgtdev_put(tgtdev);
		} else if (tgtdev->is_hidden & tgtdev->host_exposed) {
			dprint_reset(mrioc, "hiding target device with perst_id(%d)\n",
				     tgtdev->perst_id);
			mpi3mr_remove_tgtdev_from_host(mrioc, tgtdev);
		}
	}

	tgtdev = NULL;
	list_for_each_entry(tgtdev, &mrioc->tgtdev_list, list) {
		if ((tgtdev->dev_handle == MPI3MR_IN                                                                                                                                                                                                                                                                                                   debug_dump_devpg0 - Dump device page0                                                           page 0.
 *
 * Prints pertinent details of the device page 0                                                debug_dump_devpg0(struct mpi3mr_ioc *mrioc,                                  )
{                        device_pg0:                             , wwid(0x%016llx), encl_handle(0x%04x), slot(%d)\n",
	                                                                              ,
	                              ,                                                                   slot)                    "device_pg0: access_status(0x%02x), flags(0x%04x), device_form(0x%02x), queue_depth(%d)\n",
	    dev_pg0->access_status, le16_                flags),
	    dev_pg0->device_form,                                                      "device_pg0: parent_handle(0x%04x), iounit_port(%d)\n",
	                                           ,                      );

	switch (dev_pg0->device_form                                                                                                                                                                      device_pg0: sas_sata: sas_address(0x%016llx),flags(0x%04x),\n"
		    "device_info(0x%04x), phy_num(%d), attached_phy_id(%d),negotiated_link_rate(0x%02x)\n"                                        ,                   sasinf->flags),                                       , sasinf->phy_num,                                      , sasinf->negotiated_link_rate                                             :
	{
                                                                                                                    device_pg0: pcie: port_num(%d), device_info(0x%04x), mdts(%d), page_sz(0x%02x)\n",
		    pcieinf->port_num, le16_to_cpu(pcieinf              ,                                                       ,
		    pcieinf->page_size                            device_pg0: pcie: abort_timeout(%d), reset_timeout(%d) capabilities                    pcieinf->nvme_abort_to                                                                                                                                                                                                                                       device_pg0: vd: state(0x%02x), raid_level(%d), flags(0x%04x),\n"
		    "device_info(0x%04x) abort_timeout(%d), reset_timeout(%d)\n",
		    vdinf->vd_state, vdinf->raid_level,                          flags),                          device_info),
		    vdinf->vd_abort_to, vdinf->vd_reset_to                            device_pg0: vd: tg_id(%d), high(%dMiB), low(%dMiB), qd_reduction_factor(%d)\n",
		                            ,                                           _high),                                           _low),
		    ((                   flags) &
		       MPI3_DEVICE0_VD_FLAGS_IO_THROTTLE_GROUP_QD_MASK) >> 12));
		break;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        if (mrioc->logging_level &
	    (MPI3_DEBUG_EVENT |                                mpi3mr_debug_dump_devpg0(mrioc, dev_pg0)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       	if (!mrioc->sas_transport_enabled)
		tgtdev->non_stl = 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               negotiated_link_rate =
			sasinf->negotiated_link_rate                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         reset_to =
			max_t(u8, vdinf->vd                                                                         abort_to =
			max_t(u8, vdinf->vd_abort_to,
			                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          _th                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        _th(mrioc,
			(char                          , sz)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            ) {
		if (tgtdev->dev_                                 
			timeout = cmd_priv ?                                   
					   :                                      	else if                                                 timeout = cmd_priv ?                         abort_to
					   :                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  unused: gendisk                                                                                                                                                                                                                                  gendisk *unused                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              && tgt_dev->non_stl) {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          enum scsi_qc_status                                       			                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  trscpy(mrioc->driver_name,                   ,
	    sizeof(mrioc->driver_name));
	scnprintf(mrioc->name, sizeof(mrioc->name),
	    "%s%u                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          