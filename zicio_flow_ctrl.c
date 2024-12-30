#include <asm/timer.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpuset.h>
#include <linux/irq_work.h>
#include <linux/percpu.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/irqflags.h>

#include <linux/zicio_notify.h>

#include "zicio_cmd.h"
#include "zicio_device.h"
#include "zicio_req_timer.h"
#include "zicio_firehose_ctrl.h"
#include "zicio_md_flow_ctrl.h"
#include "zicio_shared_pool.h"
#include "zicio_req_submit.h"
#include "zicio_ghost.h"

#include "zicio_flow_ctrl.h"
#include "zicio_nvme_cmd_timer_wheel.h"

#define ZICIO_NUM_IO_SIZE_TYPE	(10)
#define ZICIO_2MB_IDX			(9)
#define ZICIO_1MB_IDX			(8)
#define ZICIO_512KB_IDX			(7)
#define ZICIO_256KB_IDX			(6)
#define ZICIO_128KB_IDX			(5)
#define ZICIO_64KB_IDX			(4)
#define ZICIO_32KB_IDX			(3)
#define ZICIO_16KB_IDX			(2)
#define ZICIO_8KB_IDX			(1)
#define ZICIO_4KB_IDX			(0)

#define ZICIO_MAX_REQ_COUNT(nr_req)	(nr_req - ((nr_req) >> 2))

#define ZICIO_RECOUNT_PERIOD_TICK (64)
#define ZICIO_RECOUNT_PERIOD_MASK (ZICIO_RECOUNT_PERIOD_TICK - 1)

#define ZICIO_PBR_PERIOD_MSEC (100)
#define ZICIO_PBR_TOLERANCE (5)
#define ZICIO_DEVICE_SATURATED_POINT(nr_cpu) (nr_cpu >> 1)
#define ZICIO_IO_TIME_SATURATED_POINT(io_time) (io_time)

struct zicio_flow_ctrl {
	raw_spinlock_t		lock;
	u64					tick;
	ktime_t				io_time_ema[ZICIO_NUM_IO_SIZE_TYPE];
	ktime_t				io_time_max[ZICIO_NUM_IO_SIZE_TYPE];
	ktime_t				io_time_total_avg[ZICIO_NUM_IO_SIZE_TYPE];
	u64					io_time_measurment_count[ZICIO_NUM_IO_SIZE_TYPE];
	ktime_t				last_pbr_check_time;
	int					pbr_tolerance_count;
	int					pbr_delay_on;
	int					pbr_prefire_on;
	int					pbr_div;
	int					pbr_mult;
	int					recounting_enabled;
	int					aggressive_recounting_enabled;
	int					cpu;
	atomic_t			getting_req;
	int					cur_req_count;
	int					min_req_count;
	int					zicio_channel_count;
	s64					user_consumption_throughput;
	s64					device_bandwidth;
	ktime_t (*calc_io_time_ema[ZICIO_NUM_IO_SIZE_TYPE])(ktime_t, ktime_t);
	int					last_io_size_idx;
} ____cacheline_aligned;

static DEFINE_PER_CPU(struct zicio_flow_ctrl, flow_ctrl);

#define NR_NVME_PAGE_128KB	(32) /* the number of 4KB pages per 128KB */
#define NR_NVME_PAGE_64KB	(16) /* the number of 4KB pages per 64KB */
#define NR_NVME_PAGE_32KB	(8)  /* the number of 4KB pages per 32KB */
#define NR_NVME_PAGE_16KB	(4)  /* the number of 4KB pages per 16KB */
#define NR_NVME_PAGE_8KB	(2)  /* the number of 4KB pages per 8KB */
#define NR_NVME_PAGE_4KB	(1)  /* the number of 4KB pages per 4KB */

#define NR_IO_PER_HUGE_PAGE_SHIFT(io_size_idx) (ZICIO_2MB_IDX - (io_size_idx))

/*
 * Depending on the structure of ext4_extent, the number of pages used for I/O
 * may not always be a power of two. Therefore, to approximate the number of
 * pages, it's more appropriate to use an if-else statement with inequalities
 * rahter than a switch-case statement.
 */
static int zicio_get_io_size_idx(int nr_nvme_pages)
{
	/*
	 * We don't consider larger than 256KB. Because in our ssd, the largest I/O
	 * size is 256KB.
	 */

	if (nr_nvme_pages > NR_NVME_PAGE_128KB) {
		return ZICIO_256KB_IDX;
	} else if (nr_nvme_pages > NR_NVME_PAGE_64KB) {
		return ZICIO_128KB_IDX;
	} else if (nr_nvme_pages > NR_NVME_PAGE_32KB) {
		return ZICIO_64KB_IDX;
	} else if (nr_nvme_pages > NR_NVME_PAGE_16KB) {
		return ZICIO_32KB_IDX;
	} else if (nr_nvme_pages > NR_NVME_PAGE_8KB) {
		return ZICIO_16KB_IDX;
	} else if (nr_nvme_pages > NR_NVME_PAGE_4KB) {
		return ZICIO_8KB_IDX;
	} else {
		return ZICIO_4KB_IDX;
	}
}

ktime_t
zicio_get_data_buffer_io_timing(struct zicio_notify_descriptor *zicio_notify_desc)
{
	u64 huge_page_consumption_tsc
		= zicio_notify_desc->consumption_stat.huge_page_consumption_time;

	if (huge_page_consumption_tsc == 0) {
		printk("[cpu %d] zicio_get_data_buffer_consumption_time(), tsc is 0\n",
				zicio_notify_desc->zicio_desc.cpu_id);
		return 0;
	}

	/* When the user consumed the half of data buffer */
	return zicio_tsc_to_ktime(huge_page_consumption_tsc *
		ZICIO_MAX_NUM_CHUNK / 2);
}
EXPORT_SYMBOL(zicio_get_data_buffer_io_timing);

/*
 * Calculate how many huge pages @c2 is from @c1 in the data buffer the user is
 * loocking at.
 */
static int zicio_calc_huge_page_distance(int c1, int c2)
{
	if (c1 < c2) {
		return c2 - c1 - 1;
	} else {
		return ZICIO_MAX_NUM_CHUNK - c1 - 1 + c2;
	}
}

/*
 * The kernel and libzicio use fetch_add() on the counter, and the value
 * determines the huge page location.
 *
 * Therefore, knowing the counter value, we can get the local huge page id
 * using modular operation.
 */
static inline int
zicio_calc_local_huge_page_idx(unsigned long counter)
{
	return counter & (ZICIO_MAX_NUM_CHUNK - 1);
}

/*
 * Get the time it takes to reach the new local huge page, based on how far 
 * the new local huge page differs from the huge page the user is in.
 */
static inline ktime_t
zicio_calc_user_arrival_interval(
	struct zicio_switch_board *sb, int new_huge_page_id)
{
	int cur_huge_page_id, huge_page_id_delta;
	ktime_t avg_ktime_delta = zicio_tsc_to_ktime(sb->avg_tsc_delta);

	if (sb->consumed == 0)
		return 0;

	cur_huge_page_id = zicio_calc_local_huge_page_idx(sb->consumed - 1);
	huge_page_id_delta = zicio_calc_huge_page_distance(
		cur_huge_page_id, new_huge_page_id);

	return huge_page_id_delta * avg_ktime_delta;
}

/*
 * Calculate the approximate time taken to perform 2MB I/O using 256KB commands.
 */
static inline ktime_t zicio_get_approximated_huge_page_io_time(
	struct zicio_flow_ctrl *ctrl)
{
	int idx = ctrl->last_io_size_idx;
	return ctrl->io_time_ema[idx] << NR_IO_PER_HUGE_PAGE_SHIFT(idx);
}

/*
 * Calculate when the nvme commands must be submitted to the device.
 * Note that this function is only called for chunk data nvme commands.
 */
ktime_t zicio_calc_expiration_time(struct zicio_notify_descriptor *zicio_notify_desc,
	int huge_page_id)
{
	struct zicio_descriptor *zicio_desc = (zicio_descriptor *)zicio_notify_desc;
	struct zicio_switch_board *sb = zicio_desc->switch_board;
	struct zicio_flow_ctrl *ctrl = per_cpu_ptr(&flow_ctrl, zicio_desc->cpu_id);
	ktime_t interval = 0;

	interval += zicio_calc_user_arrival_interval(sb, huge_page_id);
	interval -= zicio_get_approximated_huge_page_io_time(ctrl);
	
	if (interval < 0)
		interval = 0;

	if (ctrl->pbr_delay_on) {
		/* 
		 * ->pbr_div is a variable used to divide the number of requests, but
		 * here it is used to reduce bandwidth usage by increasing the release
		 * time.
		 */
		interval *= ctrl->pbr_div;
	} else if (ctrl->pbr_prefire_on) {
		/*
		 * ->pbr_mult is a variable used to multiple the number of requests, but
		 * here it is used to increase bandwidth usage by reducing the relase
		 * time.
		 */
		interval /= ctrl->pbr_mult;
	}

	return ktime_add(ktime_get(), interval);
}
EXPORT_SYMBOL(zicio_calc_expiration_time);

/*
 * Return how many chunks the user can consume during the fixed amount of
 * time (KTIME_MAX).
 */
static inline s64 zicio_calc_throughput(ktime_t t)
{
	return (KTIME_MAX / t);
}

static inline void zicio_reset_user_consumption_stat(
	struct zicio_huge_page_consumption_stat *consumption_stat,
	unsigned long avg_tsc_delta)
{
	ktime_t chunk_consumption_time = zicio_tsc_to_ktime(avg_tsc_delta);
	if (chunk_consumption_time) {
		consumption_stat->huge_page_consumption_throughput
			= zicio_calc_throughput(chunk_consumption_time);
		consumption_stat->huge_page_consumption_time = avg_tsc_delta;
	}
}

/**
 * __zicio_calc_io_time_ema_4kb - calculate new average for 4k size
 * @ema: old ema
 * @new: new value
 *
 * Use EMA to find the average elapsed time.
 *
 * Here we use the "e" as 1/8192, it can be changed in later.
 * EMA(t) = val(t) * (1/8192) + (1 - 1/8192) * EMA(t-1)
 */
static inline ktime_t
__zicio_calc_io_time_ema_4kb(ktime_t ema, ktime_t new)
{
	return (new + 8191 * ema) >> 13;
}

/**
 * __zicio_calc_io_time_ema_8kb - calculate new average for 8k size
 * @ema: old ema
 * @new: new value
 *
 * Use EMA to find the average elapsed time.
 *
 * Here we use the "e" as 1/4096, it can be changed in later.
 * EMA(t) = val(t) * (1/4096) + (1 - 1/4096) * EMA(t-1)
 */
static inline ktime_t
__zicio_calc_io_time_ema_8kb(ktime_t ema, ktime_t new)
{
	return (new + 4095 * ema) >> 12;
}

/**
 * __zicio_calc_io_time_ema_16kb - calculate new average for 16k size
 * @ema: old ema
 * @new: new value
 *
 * Use EMA to find the average elapsed time.
 *
 * Here we use the "e" as 1/2048, it can be changed in later.
 * EMA(t) = val(t) * (1/2048) + (1 - 1/2048) * EMA(t-1)
 */
static inline ktime_t
__zicio_calc_io_time_ema_16kb(ktime_t ema, ktime_t new)
{
	return (new + 2047 * ema) >> 11;
}

/**
 * __zicio_calc_io_time_ema_32kb - calculate new average for 32k size
 * @ema: old ema
 * @new: new value
 *
 * Use EMA to find the average elapsed time.
 *
 * Here we use the "e" as 1/1024, it can be changed in later.
 * EMA(t) = val(t) * (1/1024) + (1 - 1/1024) * EMA(t-1)
 */
static inline ktime_t
__zicio_calc_io_time_ema_32kb(ktime_t ema, ktime_t new)
{
	return (new + 1023 * ema) >> 10;
}

/**
 * __zicio_calc_io_time_ema_64kb - calculate new average for 64k size
 * @ema: old ema
 * @new: new value
 *
 * Use EMA to find the average elapsed time.
 *
 * Here we use the "e" as 1/512, it can be changed in later.
 * EMA(t) = val(t) * (1/512) + (1 - 1/512) * EMA(t-1)
 */
static inline ktime_t
__zicio_calc_io_time_ema_64kb(ktime_t ema, ktime_t new)
{
	return (new + 511 * ema) >> 9;
}

/**
 * __zicio_calc_io_time_ema_128kb - calculate new average for 128k size
 * @ema: old ema
 * @new: new value
 *
 * Use EMA to find the average elapsed time.
 *
 * Here we use the "e" as 1/256, it can be changed in later.
 * EMA(t) = val(t) * (1/256) + (1 - 1/256) * EMA(t-1)
 */
static inline ktime_t
__zicio_calc_io_time_ema_128kb(ktime_t ema, ktime_t new)
{
	return (new + 255 * ema) >> 8;
}

/**
 * __zicio_calc_io_time_ema_256kb - calculate new average for 256k size
 * @ema: old ema
 * @new: new value
 *
 * Use EMA to find the average elapsed time.
 *
 * Here we use the "e" as 1/128, it can be changed in later.
 * EMA(t) = val(t) * (1/128) + (1 - 1/128) * EMA(t-1)
 */
static inline ktime_t
__zicio_calc_io_time_ema_256kb(ktime_t ema, ktime_t new)
{
	return (new + 127 * ema) >> 7;
}

/*
 * The user's consumption rate is based on the average consumption of each huge
 * page for given 16 huge pages.
 *
 * Therefore, when calculating the average of the I/O consumption time,
 * different criteria must be applied according to the I/O size.
 */
static inline ktime_t zicio_calc_io_time_ema(
	struct zicio_flow_ctrl *ctrl, int idx, ktime_t new_io_time)
{
	return ctrl->calc_io_time_ema[idx](ctrl->io_time_ema[idx], new_io_time);
}

/*
 * The overall average of I/O time is used as a criterion for starting PBR.
 */
static inline ktime_t zicio_calc_io_time_total_avg(
	struct zicio_flow_ctrl *ctrl, int idx, ktime_t new_io_time)
{
	ktime_t total_io_time_sum = 
		ctrl->io_time_total_avg[idx] * ctrl->io_time_measurment_count[idx]++;
	return ((total_io_time_sum + new_io_time) / 
		ctrl->io_time_measurment_count[idx]);
}

/*
 * Check whether a new requests are needed. If so, indicate that we will get
 * new requests by incrementing the reuqest counter.
 *
 * If the attempt to get the new request fails, we have to call
 * zicio_dec_request_count() to rollback the state.
 *
 * Return how many request is needed.
 */
static int zicio_test_and_inc_request_count(
	struct zicio_flow_ctrl *ctrl)
{
	unsigned long flags;
	int ret = 0;
	raw_spin_lock_irqsave(&ctrl->lock, flags);
	if (ctrl->cur_req_count < ctrl->min_req_count) {
		ret = ctrl->min_req_count - ctrl->cur_req_count;
		ctrl->cur_req_count = ctrl->min_req_count;
	}
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
	return ret;
}

/*
 * If we failed to get new requests, we have to call this function
 * to rollback the request allocation state.
 */
static void zicio_dec_request_count(
	struct zicio_flow_ctrl *ctrl, int count)
{
	unsigned long flags;
	raw_spin_lock_irqsave(&ctrl->lock, flags);
	ctrl->cur_req_count -= count;
	BUG_ON(ctrl->cur_req_count < 0);
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
}

/*
 * Check whetehr the request need to be droped. If so, decrease request count
 * and return 1. Otherwise return 0.
 */
static int zicio_test_and_dec_request_count(
	struct zicio_flow_ctrl *ctrl)
{
	unsigned long flags;
	int ret = 0;
	raw_spin_lock_irqsave(&ctrl->lock, flags);
	if (ctrl->cur_req_count > ctrl->min_req_count) {
		ctrl->cur_req_count--;
		BUG_ON(ctrl->cur_req_count < 0);
		ret = 1;
	}
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
	return ret;
}

/*
 * Increase the tick to manage contol period. If a control period is reached,
 * turn on the flag.
 */
static inline void
zicio_flow_ctrl_tick(struct zicio_flow_ctrl *ctrl)
{
	unsigned long flags;
	raw_spin_lock_irqsave(&ctrl->lock, flags);
	if ((ctrl->tick++ & ZICIO_RECOUNT_PERIOD_MASK) == 0)
		ctrl->recounting_enabled = 1;
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
}

/*
 * Return 0 if retry is required. Otherwise return 1.
 */
static int __zicio_acquire_new_requests(int cpu, int type)
{
	struct zicio_nvme_cmd_timer_data fetched_data;
	struct zicio_flow_ctrl *ctrl;
	int i, count, chunk_idx, cmd_idx, retry = 0, ret = 1;
	bool enable_fetch_non_expired = false;

	ctrl = per_cpu_ptr(&flow_ctrl, cpu);
	zicio_flow_ctrl_tick(ctrl);

	/* Only allow one execution flow to avoid unexpected bugs */
	BUG_ON(raw_smp_processor_id() != cpu);
	if (atomic_cmpxchg(&ctrl->getting_req, 0, 1) != 0)
		return 1;

	/* How many new requests should we get? */
	count = zicio_test_and_inc_request_count(ctrl);
	if (!count)
		goto out_getting_req;

	if (ctrl->aggressive_recounting_enabled)
		enable_fetch_non_expired = true;

	/* We need to get new requests as much as @count */
	for (i = 0; i < count; i++) {
		bool fetched = zicio_fetch_nvme_cmd_data_from_wheel(&fetched_data, cpu,
				enable_fetch_non_expired);

		/* If there is a fecthed nvme command, try submit it */
		if (fetched) {
			zicio_descriptor *zicio_desc
				= (zicio_descriptor *)fetched_data.zicio_notify_desc;
			zicio_nvme_cmd_info nvme_cmd_info;
			chunk_idx = fetched_data.nvme_cmd_info_offset /
						ZICIO_NVME_CMD_INFO_PER_CHUNK;
			cmd_idx = fetched_data.nvme_cmd_info_offset %
					  ZICIO_NVME_CMD_INFO_PER_CHUNK;
			nvme_cmd_info =
				fetched_data.zicio_notify_desc->nvme_cmd_infos[chunk_idx][cmd_idx];

			unsigned long start_mem
				= (unsigned long)zicio_desc->buffers.data_buffer[fetched_data.local_huge_page_idx]
					+ ((unsigned long)fetched_data.page_offset << ZICIO_PAGE_SHIFT);

			zicio_nvme_cmd_list **zicio_cmd_lists
				= zicio_notify_create_command(
					(zicio_descriptor *)fetched_data.zicio_notify_desc,
					fetched_data.local_huge_page_idx, nvme_cmd_info, start_mem);
			BUG_ON(zicio_cmd_lists[0] == NULL);

			if (zicio_trigger_read_from_softirq(
					zicio_desc, zicio_cmd_lists[0], cpu) < 0) {
				/* XXX different from original function */
				zicio_free_nvme_cmd_list(zicio_cmd_lists[0]);

				/* Request is not acquired, we have to retry */
				zicio_insert_nvme_cmd_timer(fetched_data.zicio_notify_desc,
					fetched_data.nvme_cmd_info_offset, 1,
					fetched_data.local_huge_page_idx, fetched_data.page_offset,
					ktime_get());
				retry++;
			}

			zicio_free_cmd_lists_set_with_desc(zicio_desc, 0, zicio_cmd_lists);
		} else {
			/*
			 * There is no command to submit.
			 * So we cannot get a new request and cannot retry.
			 * Just rollback the request count.
			 */
			zicio_dec_request_count(ctrl, 1);
		}
	}

	if (retry) {
		zicio_dec_request_count(ctrl, retry);
		ret = 0;
	}

out_getting_req:
	BUG_ON(atomic_read(&ctrl->getting_req) != 1);
	atomic_set(&ctrl->getting_req, 0);
	return ret;
}

/*
 * Timer shoftirq is one of the softtimers used by zicIO. Try to get new
 * requests for this cpu's flow controller.
 */
int zicio_notify_do_softtimer_timer_softirq(int cpu)
{
	return  __zicio_acquire_new_requests(cpu,
		ZICIO_SOFTTIMER_TIMER_SOFTIRQ);
}
EXPORT_SYMBOL(zicio_notify_do_softtimer_timer_softirq);

#if 0
int zicio_do_softtimer_idle_loop(int cpu)
{
	int ret;
	preempt_disable();
	ret = __zicio_acquire_new_requests(cpu, ZICIO_SOFTTIMER_IDLE_LOOP);
	preempt_enable();
	return ret;
}
EXPORT_SYMBOL(zicio_do_softtimer_idle_loop);
#endif /* deprecated  */

/*
 * The interactions with the timer wheel and the way zicio_nvme_cmd is
 * handled differ from the original function, zicio_prepare_resubmit().
 *
 * According to the new design, instead of obtaining zicio_nvme_cmd from the
 * timer wheel, we retrieve zicio_nvme_cmd_info which contains encoded
 * information.
 *
 * The @req->zicio_cmd is not replaced or freed but updated with the new
 * information of timer object.
 */
static void zicio_prepare_resubmit(struct zicio_flow_ctrl *ctrl,
	struct request *req)
{
	bool enable_fetch_non_expired = false, fetched = false;
	struct zicio_nvme_cmd_timer_data fetched_data = { 0, };
	struct zicio_notify_descriptor *zicio_notify_desc;
	struct zicio_descriptor *zicio_desc;
	zicio_nvme_cmd_info nvme_cmd_info;
	int chunk_idx, cmd_idx;

	BUG_ON(req->zicio_cmd == NULL);

	if (ctrl->aggressive_recounting_enabled)
		enable_fetch_non_expired = true;

	fetched = zicio_fetch_nvme_cmd_data_from_wheel(&fetched_data, ctrl->cpu,
		enable_fetch_non_expired);

	/* If there is no nvme command to submit, drop the request */
	if (!fetched) {
		zicio_dec_request_count(ctrl, 1);

		/* XXX different from original function, zicio_prepare_resubmit() */
#if (CONFIG_ZICIO_DEBUG_LEVEL >= 2)
		printk("[cpu %d] zicio_prepare_resubmit() free zicio_cmd\n",
				ctrl->cpu);
#endif
		zicio_free_nvme_cmd_list(req->zicio_cmd);

		req->zicio_cmd = NULL;
		req->zicio_io_start_time = 0;
		return;
	}

	zicio_notify_desc = fetched_data.zicio_notify_desc;
	zicio_desc = (zicio_descriptor *)zicio_notify_desc;

	BUG_ON(zicio_desc == NULL);
	BUG_ON(req->zicio_cmd->is_metadata);

	chunk_idx = fetched_data.nvme_cmd_info_offset /
				ZICIO_NVME_CMD_INFO_PER_CHUNK;
	cmd_idx = fetched_data.nvme_cmd_info_offset %
			  ZICIO_NVME_CMD_INFO_PER_CHUNK;

	nvme_cmd_info = zicio_notify_desc->nvme_cmd_infos[chunk_idx][cmd_idx];

	/*
	 * Replace the required information for nvme submit
	 */
	req->zicio_cmd->cmd.rw.opcode = nvme_cmd_read;
	req->zicio_cmd->cmd.rw.slba
		= cpu_to_le64(
			(ZICIO_NVME_CMD_INFO_GET_FSBLK(nvme_cmd_info)
				<< ZICIO_PAGE_TO_SECTOR_SHIFT)
					+ zicio_notify_desc->bd_start_sector);
	req->zicio_cmd->cmd.rw.length
		= cpu_to_le16(
			((ZICIO_NVME_CMD_INFO_GET_LENGTH(nvme_cmd_info) + 1)
				<< ZICIO_PAGE_TO_SECTOR_SHIFT)
					- 1);

#if (CONFIG_ZICIO_DEBUG_LEVEL >= 2)
	printk("[cpu %d] zicio_prepare_resubmit() huge_page_idx: %d, fsblk: %lld, length: %lld, page_offset: %d\n",
			ctrl->cpu, fetched_data.local_huge_page_idx,
			ZICIO_NVME_CMD_INFO_GET_FSBLK(nvme_cmd_info),
			ZICIO_NVME_CMD_INFO_GET_LENGTH(nvme_cmd_info),
			fetched_data.page_offset);
#endif

	/* 
	 * Replace the required information for zicio_set_dma_mapping_for_nvme()
 	 */
	req->zicio_cmd->local_huge_page_idx = fetched_data.local_huge_page_idx;
	req->zicio_cmd->start_mem
		= (unsigned long)zicio_desc->buffers.data_buffer[fetched_data.local_huge_page_idx]
			+ ((unsigned long)fetched_data.page_offset << ZICIO_PAGE_SHIFT);

	req->bio->zicio_desc = zicio_desc;
	req->zicio_io_start_time = ktime_get();
}

/*
 * This function is called every ticks of NVMe IRQ softtimer. It decides whether
 * or not to resubmit next NVMe command.
 *
 * If so, try to prepare next NVMe command.
 */
void zicio_notify_do_softtimer_irq_cycle(void *req_ptr)
{
	struct request *req = (struct request *)req_ptr;
	struct zicio_descriptor *zicio_desc;
	struct zicio_flow_ctrl *ctrl;

	BUG_ON(req->bio->zicio_desc == NULL);
	zicio_desc = req->bio->zicio_desc;
	ctrl = per_cpu_ptr(&flow_ctrl, zicio_desc->cpu_id);
	zicio_flow_ctrl_tick(ctrl);

	/* XXX we do not free the zicio_nvme_cmd in here !!! */

	/* If there are too much requests, drop the current request */
	if (zicio_test_and_dec_request_count(ctrl)) {
		/*
		 * XXX different from original function,
		 * zicio_do_softtimer_irq_cycle()
		 */
#if (CONFIG_ZICIO_DEBUG_LEVEL >= 2)
		printk("[cpu %d] zicio_do_softtimer_irq_cycle() free zicio_cmd\n",
				ctrl->cpu);
#endif
		zicio_free_nvme_cmd_list(req->zicio_cmd);

		req->zicio_cmd = NULL;
		req->zicio_io_start_time = 0;
		return;
	}

	mb();
	zicio_prepare_resubmit(ctrl, req);
}
EXPORT_SYMBOL(zicio_notify_do_softtimer_irq_cycle);

/*
 * Update average time based on how long an I/O of a given size took.
 */
static void zicio_update_avg_io_time(struct zicio_flow_ctrl *ctrl, int idx,
	ktime_t new_io_time)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&ctrl->lock, flags);
	ctrl->io_time_ema[idx] = 
		zicio_calc_io_time_ema(ctrl, idx, new_io_time);
	ctrl->io_time_max[idx] =
		new_io_time > ctrl->io_time_max[idx] ?
			new_io_time : ctrl->io_time_max[idx];
	ctrl->io_time_total_avg[idx] =
		zicio_calc_io_time_total_avg(ctrl, idx, new_io_time);
	ctrl->last_io_size_idx = idx;
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
}

/*
 * Note that we use 256KB I/O as a default and use EMA to calculate bandwidth.
 */
static inline void zicio_update_device_bandwidth(struct zicio_flow_ctrl *ctrl)
{
	s64 single_request_bandwidth;
	int idx = ctrl->last_io_size_idx;
	ktime_t io_time_ema = ctrl->io_time_ema[idx];

	if (io_time_ema) {
		/*
		 * Calculate the throughput in terms of 2MB units based on the size of
		 * the last performed I/O operation.
 		 *
		 * This value indicates how many 2MB the device can deliver within a
		 * given time (KTIME_MAX).
		 */
		single_request_bandwidth
			= zicio_calc_throughput(io_time_ema)
				>> NR_IO_PER_HUGE_PAGE_SHIFT(idx);
		ctrl->device_bandwidth = ctrl->cur_req_count * single_request_bandwidth;
	} else {
		ctrl->device_bandwidth = 0;
	}
}

static inline void zicio_update_user_consumption_throughput(
	struct zicio_flow_ctrl *ctrl, unsigned long avg_tsc_delta,
	struct zicio_huge_page_consumption_stat *consumption_stat)
{
	/* Quick exit */
	if (consumption_stat->huge_page_consumption_time == avg_tsc_delta) {
		return;
	}

	ctrl->user_consumption_throughput
		-= consumption_stat->huge_page_consumption_throughput;

	zicio_reset_user_consumption_stat(consumption_stat, avg_tsc_delta);

	ctrl->user_consumption_throughput
		+= consumption_stat->huge_page_consumption_throughput;
}

/*
 * Update user's consumption stat and device bandwidth to the flow controller.
 *
 * The original function is zicio_update_bandwidth().
 */
static inline void zicio_update_flow_ctrl_stat(struct zicio_flow_ctrl *ctrl,
	struct zicio_notify_descriptor *zicio_notify_desc)
{
	struct zicio_descriptor *zicio_desc = (zicio_descriptor *)zicio_notify_desc;
	unsigned long flags;

	raw_spin_lock_irqsave(&ctrl->lock, flags);
	zicio_update_user_consumption_throughput(ctrl,
		zicio_desc->switch_board->avg_tsc_delta, &zicio_notify_desc->consumption_stat);
	zicio_update_device_bandwidth(ctrl);
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
}

/*
 * Gather the device bandwidth all cpus are getting and the user bandwidth
 * required. Based on that, calculate how much the current cpu needs to
 * reshuffle requests.
 */
static int zicio_do_pbr(struct zicio_flow_ctrl *ctrl)
{
	s64 user_throughput_inverse_ratio, device_bandwidth_inverse_ratio;
	s64 user_throughput_sum = 0, device_bandwidth_sum = 0;
	int cpu, nr_valid_cpu = 0, nr_saturated = 0;
	struct zicio_flow_ctrl *tmp_ctrl;
	int idx = ctrl->last_io_size_idx;

	/* Initial states, quick exit */
	if (ctrl->user_consumption_throughput == 0 || ctrl->device_bandwidth == 0)
		return 0;

	for_each_possible_cpu(cpu) {
		tmp_ctrl = per_cpu_ptr(&flow_ctrl, cpu);

		if (tmp_ctrl->user_consumption_throughput == 0)
			continue;

		user_throughput_sum += tmp_ctrl->user_consumption_throughput;
		device_bandwidth_sum += tmp_ctrl->device_bandwidth;

		/* We use default I/O size as large as possible (256KB) */
		if (tmp_ctrl->io_time_ema[idx] >
			ZICIO_IO_TIME_SATURATED_POINT(tmp_ctrl->io_time_total_avg[idx]))
			nr_saturated++;
		nr_valid_cpu++;
	}

	/*
	 * To avoid floating point calculation, summation is placed in the numerator.
	 * The higher the ratio, the smaller the proportion of the total.
	 */
	user_throughput_inverse_ratio =
		user_throughput_sum / ctrl->user_consumption_throughput;
	device_bandwidth_inverse_ratio =
		device_bandwidth_sum / ctrl->device_bandwidth;
	BUG_ON(user_throughput_inverse_ratio == 0 ||
		device_bandwidth_inverse_ratio == 0);

	if (user_throughput_inverse_ratio >= device_bandwidth_inverse_ratio) {
		/* We have too much device bandwidth, it should be reduced */
		ctrl->pbr_mult = 1;
		ctrl->pbr_div = user_throughput_inverse_ratio /
			device_bandwidth_inverse_ratio;
	} else {
		/* We have insufficient device bandwidth, it should be increased */ 
		ctrl->pbr_mult = device_bandwidth_inverse_ratio /
			user_throughput_inverse_ratio;
		ctrl->pbr_div = 1;
	}

	if (nr_saturated >= ZICIO_DEVICE_SATURATED_POINT(nr_valid_cpu) &&
		nr_valid_cpu > 1)
		return 1;
	return 0;
}

/*
 * Recount requests based on values obtained from PBR.
 * Note that request count has limitaion. If we cannot recount the request,
 * use alternatives like delayed firing or prefiring.
 *
 * Return 1 if new request is needed, otherwise return 0.
 */
static int zicio_do_reshuffling(struct zicio_flow_ctrl *ctrl, int max_nr_req)
{
	int new_req_count, cur_req_count, ret = 0;

	cur_req_count = ctrl->cur_req_count ? ctrl->cur_req_count : 1;
	if (ctrl->pbr_mult != 1) {
		BUG_ON(ctrl->pbr_div != 1);
		new_req_count = cur_req_count * ctrl->pbr_mult;
		if (new_req_count < max_nr_req) {
			ctrl->min_req_count = new_req_count;
			ctrl->pbr_prefire_on = 0;
			ctrl->pbr_delay_on = 0;
			ret = 1;
		} else {
			/*
			 * Since we cannot get more request,
			 * try prefetching the nvme commands as an alternative.
			 */
			ctrl->min_req_count = max_nr_req;
			ctrl->pbr_prefire_on = 1;
			ctrl->pbr_delay_on = 0;
		}		
	} else if (ctrl->pbr_div != 1) {
		BUG_ON(ctrl->pbr_mult != 1);
		new_req_count = cur_req_count / ctrl->pbr_div;
		if (new_req_count > 0) {
			ctrl->min_req_count = new_req_count;
			ctrl->pbr_prefire_on = 0;
			ctrl->pbr_delay_on = 0;
		} else {
			/*
			 * Since we cannot drop more request,
			 * try delayed nvme command fetching  as an alternative.
			 */
			ctrl->min_req_count = 1;
			ctrl->pbr_prefire_on = 0;
			ctrl->pbr_delay_on = 1;
		}
	} else {
		ctrl->pbr_prefire_on = 0;
		ctrl->pbr_delay_on = 0;
	}

	return ret;
}

/*
 * Proportional Bandwidth Reshuffling (PBR) is an operation that compares the
 * bandwidth of all cores and resuffles the request count. Since this operation
 * is quite expensive, we do it in long period, typically 100 milliseconds.
 *
 * Return 1 if new request is needed.
 */
static int zicio_test_and_do_pbr(struct zicio_flow_ctrl *ctrl,
	int max_nr_req, ktime_t know)
{
	ktime_t interval = ktime_sub(know, ctrl->last_pbr_check_time);

	if (ktime_to_ms(interval) >= ZICIO_PBR_PERIOD_MSEC) {
		ctrl->last_pbr_check_time = know;
		if (zicio_do_pbr(ctrl)) {
			ctrl->aggressive_recounting_enabled = 0;
			ctrl->pbr_tolerance_count = 0;
		} else if (++ctrl->pbr_tolerance_count == ZICIO_PBR_TOLERANCE) {
			/* We reached stable state. No need to reshuffle */
			ctrl->pbr_mult = 1;
			ctrl->pbr_div = 1;
			ctrl->aggressive_recounting_enabled = 1;
		}
	}
	return zicio_do_reshuffling(ctrl, max_nr_req);
}

/*
 * Adjust the ->min_req_count(@ctrl) by comparing the bandwidth of the user and
 * the device.
 *
 * Return 1 if we have to get new requests, otherwise return 0.
 */
static int zicio_aggressive_request_recounting(struct zicio_flow_ctrl *ctrl,
	int max_nr_req)
{
	int new_count, ret = 0;

	BUG_ON(max_nr_req == 0);

	/* Initial states, quick exit */
	if (ctrl->user_consumption_throughput == 0 || ctrl->device_bandwidth == 0)
		return 0;

	if (!ctrl->aggressive_recounting_enabled)
		return 0;

	if (ctrl->user_consumption_throughput >= ctrl->device_bandwidth) {
		new_count = ctrl->cur_req_count + 1;
		if (new_count >= max_nr_req)
			new_count = max_nr_req;
	} else {
		new_count = ctrl->cur_req_count - 1;
		if (new_count < 1)
			new_count = 1;
	}

	if (new_count > ctrl->cur_req_count)
		ret = 1;
	ctrl->min_req_count = new_count;
	return ret;
}

/*
 * The change in bandwidth of users and device is slow, but the frequency of
 * function calls is very high.
 *
 * To avoid meaningless computational overhead, we adjust the number of
 * requests only when the contol period is reached.
 */
static inline void zicio_request_recounting(struct zicio_flow_ctrl *ctrl,
	int max_nr_req, ktime_t know)
{
	unsigned long flags;
	int newreq = 0;

	raw_spin_lock_irqsave(&ctrl->lock, flags);

	if (ctrl->recounting_enabled) {	
		/* Eagerly acquire new requests */
		newreq = zicio_aggressive_request_recounting(ctrl, max_nr_req);

		/* Proportional Bandwidth Reshuffling */
		//newreq |= zicio_test_and_do_pbr(ctrl, max_nr_req, know);

		/* If new request is needed, create timer softirq */
		if (newreq)
			zicio_notify_trigger_softtimer_timer_softirq(ctrl->cpu, 0);

		ctrl->recounting_enabled = 0;
	}

	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
}

/*
 * Update flow controller using the given request.
 *
 * Note that we limit the maximum number of requests to prevent zicio from
 * monopolizing hardware resources.
 */
void zicio_notify_update_flow_ctrl(struct request *req, u32 queue_depth)
{
	struct zicio_flow_ctrl *ctrl;
	struct zicio_nvme_cmd_list *zicio_cmd;
	struct zicio_descriptor *zicio_desc;
	ktime_t io_time, know = ktime_get();
	int idx, nr_nvme_pages, max_nr_req;

	/* Get the information about I/O size */
	BUG_ON(req->bio->zicio_desc == NULL || req->zicio_cmd == NULL);
	zicio_desc = req->bio->zicio_desc;
	zicio_cmd = req->zicio_cmd;
	nr_nvme_pages = zicio_get_nr_nvme_pages(zicio_cmd->cmd.rw.length);
	idx = zicio_get_io_size_idx(nr_nvme_pages);
	BUG_ON(zicio_desc->switch_board == NULL);

	ctrl = per_cpu_ptr(&flow_ctrl, zicio_desc->cpu_id);

	/* Update I/O elapsed time and bandwidth */
	BUG_ON(req->zicio_io_start_time == 0);
	io_time = know - req->zicio_io_start_time;
	zicio_update_avg_io_time(ctrl, idx, io_time);
	zicio_update_flow_ctrl_stat(ctrl, (zicio_notify_descriptor *)zicio_desc);

	/* Recount request */
	max_nr_req = ZICIO_MAX_REQ_COUNT(queue_depth);
	zicio_request_recounting(ctrl, max_nr_req, know);
}
EXPORT_SYMBOL(zicio_notify_update_flow_ctrl);

/*
 * This function is called when a new zicio channel is created.
 *
 * Note that currently zicio must avoid cpu migration. So remember which cpu
 * id is used for this zicio.
 *
 * Return 0 if success, otherwise return -1.
 */
int zicio_register_channel_into_flow_ctrl(
	struct zicio_notify_descriptor *zicio_notify_desc)
{
	struct zicio_flow_ctrl *ctrl;
	struct zicio_descriptor *zicio_desc = (zicio_descriptor *)zicio_notify_desc;
	unsigned long flags;
	int cpu = zicio_desc->cpu_id;

	BUG_ON(cpu < 0);
	if (sched_setaffinity(0, cpumask_of(cpu)))
		return -1;

	ctrl = per_cpu_ptr(&flow_ctrl, cpu);
	BUG_ON(zicio_notify_desc->consumption_stat.huge_page_consumption_time != 0);
	BUG_ON(zicio_notify_desc->consumption_stat.huge_page_consumption_throughput != 0);

	raw_spin_lock_irqsave(&ctrl->lock, flags);
	ctrl->min_req_count++;
	ctrl->zicio_channel_count++;
	BUG_ON(ctrl->zicio_channel_count <= 0);
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);

	return 0;
}
EXPORT_SYMBOL(zicio_register_channel_into_flow_ctrl);

/*
 * This function is called when a zicio channel is closed.
 *
 * Adjust user bandwitdh.
 *
 * Note that this function allows migration again. Therefore, it should be
 * called after all I/O is completed.
 */
void zicio_remove_channel_from_flow_ctrl(struct zicio_notify_descriptor *zicio_notify_desc)
{
	struct zicio_descriptor *zicio_desc = (zicio_descriptor *)zicio_notify_desc;
	struct zicio_flow_ctrl *ctrl = per_cpu_ptr(&flow_ctrl, zicio_desc->cpu_id);
	struct zicio_huge_page_consumption_stat *consumption_stat
		= &zicio_notify_desc->consumption_stat;
	unsigned long flags;

	raw_spin_lock_irqsave(&ctrl->lock, flags);
	ctrl->user_consumption_throughput
		-= consumption_stat->huge_page_consumption_throughput;
	ctrl->zicio_channel_count--;

	BUG_ON(ctrl->zicio_channel_count < 0);
	if (ctrl->zicio_channel_count == 0)
		ctrl->min_req_count = 0;
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);

	/* Enable migration */
	sched_setaffinity(0, cpu_possible_mask);
}
EXPORT_SYMBOL(zicio_remove_channel_from_flow_ctrl);

/*
 * This function is called for zicio_init_read_trigger() to manage request
 * count.
 */
void zicio_init_trigger_flow_control(struct zicio_notify_descriptor *zicio_notify_desc)
{
	struct zicio_descriptor *zicio_desc = (zicio_descriptor *)zicio_notify_desc;
	struct zicio_flow_ctrl *ctrl = per_cpu_ptr(&flow_ctrl, zicio_desc->cpu_id);
	unsigned long flags;

	raw_spin_lock_irqsave(&ctrl->lock, flags);
	ctrl->cur_req_count++;
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
}
EXPORT_SYMBOL(zicio_init_trigger_flow_control);

/*
 * This function is called for zicio_init_read_triger() to manage when we
 * failed to get the first request.
 */
void zicio_init_trigger_rollback_flow_control(struct zicio_notify_descriptor *zicio_notify_desc)
{
	struct zicio_descriptor *zicio_desc = (zicio_descriptor *)zicio_notify_desc;
	struct zicio_flow_ctrl *ctrl = per_cpu_ptr(&flow_ctrl, zicio_desc->cpu_id);
	unsigned long flags;

	raw_spin_lock_irqsave(&ctrl->lock, flags);
	ctrl->cur_req_count--;
	BUG_ON(ctrl->cur_req_count < 0);
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
}
EXPORT_SYMBOL(zicio_init_trigger_rollback_flow_control);

/*
 * Initialize flow controller for the given cpu
 */
static void __init zicio_init_flow_controller_cpu(int cpu)
{
	struct zicio_flow_ctrl *ctrl = per_cpu_ptr(&flow_ctrl, cpu);
	int i;

	ctrl->tick = 0;
	ctrl->cpu = cpu;
	ctrl->pbr_div = 1;
	ctrl->pbr_mult = 1;
	ctrl->pbr_delay_on = 0;
	ctrl->pbr_prefire_on = 0;
	ctrl->pbr_tolerance_count = 0;
	ctrl->cur_req_count = 0;
	ctrl->min_req_count = 0;
	ctrl->user_consumption_throughput = 0;
	ctrl->device_bandwidth = 0;
	ctrl->recounting_enabled = 0;
	ctrl->aggressive_recounting_enabled = 1;
	raw_spin_lock_init(&ctrl->lock);

	atomic_set(&ctrl->getting_req, 0);

	for (i = 0; i < ZICIO_NUM_IO_SIZE_TYPE; i++) {
		ctrl->io_time_ema[i] = 0;
		ctrl->io_time_max[i] = 0;
		ctrl->io_time_total_avg[i] = 0;
		ctrl->io_time_measurment_count[i] = 0;
	}

	ctrl->calc_io_time_ema[ZICIO_256KB_IDX] =  __zicio_calc_io_time_ema_256kb;

	ctrl->calc_io_time_ema[ZICIO_128KB_IDX] =  __zicio_calc_io_time_ema_128kb;

	ctrl->calc_io_time_ema[ZICIO_64KB_IDX] =  __zicio_calc_io_time_ema_64kb;

	ctrl->calc_io_time_ema[ZICIO_32KB_IDX] =  __zicio_calc_io_time_ema_32kb;

	ctrl->calc_io_time_ema[ZICIO_16KB_IDX] =  __zicio_calc_io_time_ema_16kb;

	ctrl->calc_io_time_ema[ZICIO_8KB_IDX] =  __zicio_calc_io_time_ema_8kb;

	ctrl->calc_io_time_ema[ZICIO_4KB_IDX] =  __zicio_calc_io_time_ema_4kb;

	ctrl->zicio_channel_count = 0;
}

/*
 * Initialize flow controller for each cpu
 */
void __init zicio_init_flow_controller(void)
{
	int cpu;
	for_each_possible_cpu(cpu)
		zicio_init_flow_controller_cpu(cpu);
}
EXPORT_SYMBOL(zicio_init_flow_controller);

void zicio_dump_flow_ctrl(int cpu)
{
	struct zicio_flow_ctrl *ctrl = per_cpu_ptr(&flow_ctrl, cpu);
	unsigned long flags;

	raw_spin_lock_irqsave(&ctrl->lock, flags);

	printk(KERN_WARNING "[ZICIO_FLOW] cpu: %d, 256K, ema: %lld, max: %lld, total_avg: %lld\n",
		ctrl->cpu,
		ctrl->io_time_ema[ZICIO_256KB_IDX],
		ctrl->io_time_max[ZICIO_256KB_IDX],
		ctrl->io_time_total_avg[ZICIO_256KB_IDX]);
	printk(KERN_WARNING "[ZICIO_FLOW] cpu: %d, 128K, ema: %lld, max: %lld, total_avg: %lld\n",
		ctrl->cpu,
		ctrl->io_time_ema[ZICIO_128KB_IDX],
		ctrl->io_time_max[ZICIO_128KB_IDX],
		ctrl->io_time_total_avg[ZICIO_128KB_IDX]);
	printk(KERN_WARNING "[ZICIO_FLOW] cpu: %d, 64K, ema: %lld, max: %lld, total_avg: %lld\n",
		ctrl->cpu,
		ctrl->io_time_ema[ZICIO_64KB_IDX],
		ctrl->io_time_max[ZICIO_64KB_IDX],
		ctrl->io_time_total_avg[ZICIO_64KB_IDX]);
	printk(KERN_WARNING "[ZICIO_FLOW] cpu: %d, 32K, ema: %lld, max: %lld, total_avg: %lld\n",
		ctrl->cpu,
		ctrl->io_time_ema[ZICIO_32KB_IDX],
		ctrl->io_time_max[ZICIO_32KB_IDX],
		ctrl->io_time_total_avg[ZICIO_32KB_IDX]);
	printk(KERN_WARNING "[ZICIO_FLOW] cpu: %d, 16K, ema: %lld, max: %lld, total_avg: %lld\n",
		ctrl->cpu,
		ctrl->io_time_ema[ZICIO_16KB_IDX],
		ctrl->io_time_max[ZICIO_16KB_IDX],
		ctrl->io_time_total_avg[ZICIO_16KB_IDX]);
	printk(KERN_WARNING "[ZICIO_FLOW] cpu: %d, 8K, ema: %lld, max: %lld, total_avg: %lld\n",
		ctrl->cpu,
		ctrl->io_time_ema[ZICIO_8KB_IDX],
		ctrl->io_time_max[ZICIO_8KB_IDX],
		ctrl->io_time_total_avg[ZICIO_8KB_IDX]);
	printk(KERN_WARNING "[ZICIO_FLOW] cpu: %d, 4K, ema: %lld, max: %lld, total_avg: %lld\n",
		ctrl->cpu,
		ctrl->io_time_ema[ZICIO_4KB_IDX],
		ctrl->io_time_max[ZICIO_4KB_IDX],
		ctrl->io_time_total_avg[ZICIO_4KB_IDX]);
	printk(KERN_WARNING "[ZICIO_FLOW] cpu: %d, cur_req_count: %d, min_req_count:%d\n",
		ctrl->cpu, ctrl->cur_req_count, ctrl->min_req_count);
	printk(KERN_WARNING "[ZICIO_FLOW] cpu: %d, user consumption_throughput: %lld, device bandwidth: %lld\n",
		ctrl->cpu, ctrl->user_consumption_throughput, ctrl->device_bandwidth);

	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
}
EXPORT_SYMBOL(zicio_dump_flow_ctrl);
