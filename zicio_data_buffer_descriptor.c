#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/irq_work.h>
#include <linux/jiffies.h>
#include <linux/percpu.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/syscalls.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/printk.h>

#include <linux/zicio_notify.h>

#include "zicio_cmd.h"
#include "zicio_device.h"
#include "zicio_files.h"
#include "zicio_atomic.h"
#include "zicio_firehose_ctrl.h"

#include "zicio_nvme_cmd_timer_wheel.h"
#include "zicio_flow_ctrl.h"

static void zicio_try_trigger_io_flow(struct zicio_notify_descriptor *zicio_notify_desc);

#define ZICIO_NO_SPACE_FOR_NEW_IO	(0)
#define ZICIO_ALL_BATCHES_CONSUMED	(1)
#define ZICIO_NEW_IO_PREPARED		(2)

/*
 * Concurrency control for the @zicio_notify_desc->round[] is already have been done
 * in the caller function's zicio_prepare_next_local_huge_page_id().
 *
 * Return false if there is no batches to consume anymore.
 */
static int
zicio_get_huge_page_nvme_cmd_infos(struct zicio_notify_descriptor *zicio_notify_desc,
	int local_huge_page_idx, u32 *start_nvme_cmd_info_offset,
	u16 *nr_nvme_cmd_info, u32 *nr_nvme_pages, ktime_t *expires)
{
	int round = atomic_read(zicio_notify_desc->buf_desc.round + local_huge_page_idx);
	int start_offset_idx
		= round * ZICIO_MAX_NUM_CHUNK + local_huge_page_idx;
	unsigned int last_nvme_cmd_info_offset;
	int i, chunk_idx, elem_idx;

#if (CONFIG_ZICIO_DEBUG_LEVEL >= 2)
	printk("[cpu %d] zicio_get_huge_page_nvme_cmd_infos() huge_page_idx: %d, round: %d, start_offset_idx: %d, nr_nvme_cmd_info_start_offsets: %d\n",
			zicio_notify_desc->zicio_desc.cpu_id,
			local_huge_page_idx,
			round,
			start_offset_idx,
			zicio_notify_desc->nr_nvme_cmd_info_start_offsets);
#endif

	if (start_offset_idx >= zicio_notify_desc->nr_nvme_cmd_info_start_offsets)
		return ZICIO_ALL_BATCHES_CONSUMED;

	BUG_ON(round < 0);

	chunk_idx = start_offset_idx / ZICIO_NVME_CMD_INFO_START_OFFSET_PER_CHUNK;
	elem_idx = start_offset_idx % ZICIO_NVME_CMD_INFO_START_OFFSET_PER_CHUNK;

	*start_nvme_cmd_info_offset
		= zicio_notify_desc->nvme_cmd_info_start_offsets[chunk_idx][elem_idx];

	if (start_offset_idx != (zicio_notify_desc->nr_nvme_cmd_info_start_offsets - 1)) {
		if (++elem_idx == ZICIO_NVME_CMD_INFO_START_OFFSET_PER_CHUNK) {
			++chunk_idx;
			elem_idx = 0;
		}

		/* Just before the start offset of the next huge page */
		last_nvme_cmd_info_offset
			= zicio_notify_desc->nvme_cmd_info_start_offsets[chunk_idx][elem_idx] - 1;

		/* If it's not the last huge page, it'w always 2MB (512 4KB pages) */
		*nr_nvme_pages = ZICIO_CHUNK_SIZE >> ZICIO_PAGE_SHIFT;
	} else {
		/* If there is no next huge page */
		last_nvme_cmd_info_offset
			= zicio_notify_desc->nr_nvme_cmd_infos - 1;

		BUG_ON(*nr_nvme_pages != 0);

		/* We have to calculate the number of pages */
		chunk_idx = *start_nvme_cmd_info_offset / ZICIO_NVME_CMD_INFO_PER_CHUNK;
		elem_idx = *start_nvme_cmd_info_offset % ZICIO_NVME_CMD_INFO_PER_CHUNK;
		for (i = *start_nvme_cmd_info_offset;
				i <= last_nvme_cmd_info_offset; i++) {
			zicio_nvme_cmd_info info
				= zicio_notify_desc->nvme_cmd_infos[chunk_idx][elem_idx];

			*nr_nvme_pages
				+= ZICIO_NVME_CMD_INFO_GET_LENGTH(info) + 1;

			if (++elem_idx == ZICIO_NVME_CMD_INFO_PER_CHUNK) {
				++chunk_idx;
				elem_idx = 0;
			}
		}
	}

	atomic_set(zicio_notify_desc->buf_desc.round + local_huge_page_idx, round + 1);

	BUG_ON(last_nvme_cmd_info_offset < *start_nvme_cmd_info_offset);

	*nr_nvme_cmd_info
		= last_nvme_cmd_info_offset - *start_nvme_cmd_info_offset + 1;

	*expires
		= zicio_calc_expiration_time(zicio_notify_desc, local_huge_page_idx);

#if (CONFIG_ZICIO_DEBUG_LEVEL >= 2)
	printk("[cpu %d] zicio_get_huge_page_nvme_cmd_infos() huge page idx: %d, start_nvme_cmd_info_offset: %d, nr_nvme_cmd_info: %d, nr_nvme_pages: %d, expires: %lld\n", zicio_notify_desc->zicio_desc.cpu_id, local_huge_page_idx,
			*start_nvme_cmd_info_offset, *nr_nvme_cmd_info, *nr_nvme_pages,
			*expires);
#endif

	return ZICIO_NEW_IO_PREPARED;
}

static int
zicio_prepare_next_huge_page_cmds(struct zicio_notify_descriptor *zicio_notify_desc)
{
	int local_huge_page_idx = zicio_prepare_next_local_huge_page_id(
		(zicio_descriptor *)zicio_notify_desc);
	u32 start_nvme_cmd_info_offset, nr_nvme_pages = 0;
	u16 nr_nvme_cmd_info = 0;
	ktime_t expires = 0;
	int next_huge_page_state;

	if (local_huge_page_idx == ZICIO_INVALID_LOCAL_HUGE_PAGE_IDX)
		return ZICIO_NO_SPACE_FOR_NEW_IO;

	next_huge_page_state
		= zicio_get_huge_page_nvme_cmd_infos(zicio_notify_desc, local_huge_page_idx,
			&start_nvme_cmd_info_offset, &nr_nvme_cmd_info,
			&nr_nvme_pages, &expires);

	if (next_huge_page_state == ZICIO_ALL_BATCHES_CONSUMED) {
		/*
		 * If this flag is not set to 0, ZICIO_NO_SPACE_FOR_NEW_IO might be
		 * returned instead of ZICIO_ALL_BATCHES_CONSUMED, causing the trigger
		 * to persist indefinitely.
		 */
		atomic_set(zicio_notify_desc->zicio_desc.firehose_ctrl.requested_flag_per_local_huge_page
			+ local_huge_page_idx, 0);
		return ZICIO_ALL_BATCHES_CONSUMED;
	}

	BUG_ON(nr_nvme_pages == 0);
	zicio_set_needed_nvme_pages((zicio_descriptor *)zicio_notify_desc,
		local_huge_page_idx, nr_nvme_pages);

	BUG_ON(nr_nvme_cmd_info == 0);
	BUG_ON(expires == 0);
	zicio_insert_nvme_cmd_timer(zicio_notify_desc, start_nvme_cmd_info_offset,
		nr_nvme_cmd_info, local_huge_page_idx, 0, expires);

#if (CONFIG_ZICIO_DEBUG_LEVEL >= 2)
	printk("[cpu %d] zicio_prepare_next_huge_page_cmds() next page: %d\n",
			zicio_notify_desc->zicio_desc.cpu_id,
			local_huge_page_idx);
#endif

	return ZICIO_NEW_IO_PREPARED;
}

/*
 * zicio_try_trigger_huge_page_prepare()'s callback function.
 */
static void
zicio_trigger_io_flow_callback(struct timer_list *timer)
{
	struct zicio_notify_descriptor *zicio_notify_desc
		= from_timer(zicio_notify_desc, timer, trigger);
	struct zicio_descriptor *zicio_desc
		= (zicio_descriptor *)zicio_notify_desc;
	int next_huge_page_state;

	atomic_set(&zicio_notify_desc->trigger_running, 0);

#if (CONFIG_ZICIO_DEBUG_LEVEL >= 2)
	printk("[cpu %d] zicio_trigger_io_flow_callback()\n",
			zicio_notify_desc->zicio_desc.cpu_id);
#endif

	next_huge_page_state
		= zicio_prepare_next_huge_page_cmds(zicio_notify_desc);

	if (next_huge_page_state == ZICIO_NEW_IO_PREPARED)
		zicio_notify_trigger_softtimer_timer_softirq(zicio_desc->cpu_id, 0);
	else if (next_huge_page_state == ZICIO_NO_SPACE_FOR_NEW_IO)
		zicio_try_trigger_io_flow(zicio_notify_desc); /* retry in the future */
	else
		/* do nothing */;
}

/*
 * If preparing the next huge page cmds fails and there are no timers in the
 * timer wheel, resubmission cycle will be stoped. Preprare for this scenario.
 */
static void
zicio_try_trigger_io_flow(struct zicio_notify_descriptor *zicio_notify_desc)
{
	zicio_descriptor *zicio_desc = (zicio_descriptor *)zicio_notify_desc;

	if (atomic_cmpxchg(&zicio_notify_desc->trigger_running, 0, 1) == 0) {
		ktime_t io_timing
			= zicio_get_data_buffer_io_timing(zicio_notify_desc);
		unsigned long io_timing_jiffies
			= nsecs_to_jiffies(io_timing);

		/* consider scheduling time */
		if (io_timing_jiffies >= 1)
			io_timing_jiffies -= 1;

#if (CONFIG_ZICIO_DEBUG_LEVEL >= 2)
		printk("[cpu %d] zicio_try_trigger_io_flow(), interval: %ld\n",
			zicio_notify_desc->zicio_desc.cpu_id, io_timing_jiffies);
#endif

		BUG_ON(timer_pending(&zicio_notify_desc->trigger));
		timer_setup(&zicio_notify_desc->trigger,
			zicio_trigger_io_flow_callback, TIMER_PINNED);
		zicio_notify_desc->trigger.expires
			= get_jiffies_64() + io_timing_jiffies;
		add_timer_on(&zicio_notify_desc->trigger, zicio_desc->cpu_id);
	}
}

/**
 * zicio_notify_complete_command - functino to handle zicIO nvme command
 * @req_ptr: handling request pointer
 * @queue_depth: nvme queue depth
 *
 * This function operates similarly to zicio_complete_command(), but its
 * detailed behavior has been implemented differently to match the revised
 * design.
 *
 * In order to utilize legacy code (zicio) as much as possible, we use
 * @req->zicio_cmd. The difference is that in the original
 * zicio_complete_command(), a new zicio_nvme_cmd_list object was
 * fetched from the timer wheel and the pointer was changed to the new one, but
 * zicIO does not do this.
 *
 * Instead, we keeps the same zicio_nvme_cmd_list object and only changes
 * the values of the member variables for resubmitting a new NVMe command.
 * Therefore, zicio_nvme_cmd_list object is only freed when there is no
 * resubmission (original version always free it). 
 */
void zicio_notify_complete_command(void *req_ptr, u32 queue_depth)
{
	int need_next;
	struct request *req = (struct request *)req_ptr;
	struct zicio_notify_descriptor *zicio_notify_desc
		= (zicio_notify_descriptor *)req->bio->zicio_desc;
	int next_huge_page_state;

#if (CONFIG_ZICIO_DEBUG_LEVEL >= 2)
	printk("[cpu %d] zicio_complete_nvme_command(), req->zicio_cmd: %p, zicio_desc: %p\n",
			zicio_notify_desc->zicio_desc.cpu_id,
			req->zicio_cmd, zicio_desc);
#endif

	BUG_ON(req->zicio_cmd == NULL);
	BUG_ON(zicio_notify_desc == NULL);

	zicio_notify_update_flow_ctrl(req, queue_depth);

	need_next = zicio_complete_firehose_command(
		(zicio_descriptor *) zicio_notify_desc, req->zicio_cmd); // legacy code

	/*
	 * Put the new NVMe command infos into the timer wheel.
	 */
	if (need_next == ZICIO_NEXT_CHUNK_ENABLED) {
		next_huge_page_state = zicio_prepare_next_huge_page_cmds(zicio_notify_desc);

#if (CONFIG_ZICIO_DEBUG_LEVEL >= 2)
		printk("[cpu %d] zicio_complete_nvme_command() next_huge_pge_idx: %d\n",
				zicio_notify_desc->zicio_desc.cpu_id,
				next_local_huge_page_idx);
#endif

		/* Retry in the future */
		if (next_huge_page_state == ZICIO_NO_SPACE_FOR_NEW_IO)
			zicio_try_trigger_io_flow(zicio_notify_desc);
	}

	/*
	 * The replacement of information needed for resubmission occurs here.
	 */
	zicio_notify_do_softtimer_irq_cycle(req_ptr);
}
EXPORT_SYMBOL(zicio_notify_complete_command);
