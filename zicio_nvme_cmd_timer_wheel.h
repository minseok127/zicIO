#ifndef _LINUX_ZICIO_NVME_CMD_TIMER_WHEEL_H
#define _LINUX_ZICIO_NVME_CMD_TIMER_WHEEL_H

#include <linux/blkdev.h>
#include <linux/list.h>
#include <linux/ktime.h>
#include <linux/stddef.h>
#include <linux/debugobjects.h>
#include <linux/stringify.h>

#include <linux/zicio_notify.h>

/**
 * zicio_nvme_cmd_timer - unit of zicIO's timer wheel.
 *
 * @entry: list_head for linking timers in a timer wheel
 * @sibling: list_head for linking timers in a same zicio channel
 * @expires: time to expire
 * @flags: timer's features related to a timer wheel
 * @start_nvme_cmd_info_offset: starting offset of cmds managed by this timer
 * @nr_nvme_cmd_info: the number of cmds managed by this timer
 * @next_nvme_cmd_info_index: index of next cmd to be fetched
 * @local_huge_page_idx: huge page index where the cmds will be requested 
 * @next_cmd_page_offset: next cmd's 4KB page offset in the huge page
 * @zicio_desc: zicio descriptor to which the timer belongs
 *
 * A timer is allocated for a single huge page's I/O control. Typically,
 * because multiple NVMe commands are generated for a single huge page, each
 * timer covers multiple NVMe command infos.
 *
 * NVMe command infos are stored in an array called @nvme_cmd_infos within the
 * @timer->zicio_desc. The @timer->start_nvme_cmd_info_offset represents the
 * first info in this array that the timer manages, and @timer->nr_cmd_info
 * indicates the number of infos covered by the timer. When fetching NVMe
 * command info from the timer wheel, this information is used to access the
 * corresponding element (nvme_cmd_info) in the array.
 */
struct zicio_nvme_cmd_timer {
	struct list_head			entry;
	struct list_head			sibling;
	ktime_t						expires;
	u32							flags;
	u32							start_nvme_cmd_info_offset;
	u16							nr_cmd_info;
	u16							next_cmd_info_index;
	u16							local_huge_page_idx;
	u16							next_cmd_page_offset;
	struct zicio_notify_descriptor		*zicio_notify_desc;
} ____cacheline_aligned;

static inline void
ZICIO_INIT_NVME_CMD_TIMER(
		struct zicio_nvme_cmd_timer *timer,
		struct zicio_notify_descriptor *zicio_notify_desc,
		u32 start_nvme_cmd_info_offset,
		u16 nr_nvme_cmd_info,
		u16 local_huge_page_idx,
		u16 next_cmd_page_offset)
{
	INIT_LIST_HEAD(&timer->entry);
	INIT_LIST_HEAD(&timer->sibling);
	timer->expires = 0;
	timer->zicio_notify_desc = zicio_notify_desc;
	timer->start_nvme_cmd_info_offset = start_nvme_cmd_info_offset;
	timer->nr_cmd_info = nr_nvme_cmd_info;
	timer->next_cmd_info_index = 0;
	timer->local_huge_page_idx = local_huge_page_idx;
	timer->next_cmd_page_offset = next_cmd_page_offset;
	timer->flags = 0;
}

extern struct zicio_nvme_cmd_timer *zicio_allocate_nvme_cmd_timer(void);

extern void zicio_free_nvme_cmd_timer(void *timer);

extern void zicio_insert_nvme_cmd_timer(
	struct zicio_notify_descriptor *zicio_notify_desc, u32 start_nvme_cmd_info_offset,
	u16 nr_nvme_cmd_info, u16 local_huge_page_idx, u16 start_page_offset,
	ktime_t expires);

extern void zicio_init_trigger_insert_nvme_cmd_timer(
	struct zicio_notify_descriptor *zicio_notify_desc, u16 nr_nvme_cmd_info,
	u16 next_cmd_page_offset);

extern void zicio_delete_nvme_cmd_timer(struct zicio_nvme_cmd_timer *timer);

/*
 * Information to be returned by zicio_fetch_nvme_cmd_data().
 */
struct zicio_nvme_cmd_timer_data {
	struct zicio_notify_descriptor		*zicio_notify_desc;
	u16									local_huge_page_idx;
	u16									page_offset;
	u32									nvme_cmd_info_offset;
};

extern bool zicio_fetch_nvme_cmd_data_from_wheel(
	struct zicio_nvme_cmd_timer_data *fetched_data,
	int cpu, bool enable_fetch_non_expired);

extern void zicio_notify_trigger_softtimer_timer_softirq(
	int cpu, unsigned long interval);

extern void __init zicio_init_nvme_cmd_timer_wheel(void);

extern void __init zicio_init_nvme_cmd_timer_slab_cache(void);

extern void zicio_dump_nvme_cmd_timer_wheel(int cpu);

#endif /* _LINUX_ZICIO_NVME_CMD_TIMER_WHEEL_H */
