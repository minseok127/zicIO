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

#include "zicio_nvme_cmd_timer_wheel.h"

/*
 * Why does ZicIO use a timer wheel concept for request control, not a
 * hrtimer's red-black tree?
 *
 * The timer wheel implementation has changed over time. In the ealry days,
 * "cascade" operation was used to manage timers precisely. However, as noted in
 * this document(https://lwn.net/Articles/152436/), the cascade process can be
 * too expensive if the timer interrupt frequency is raised.
 *
 * So, the data structure for the high-resolution timer(hrtimer) does not use
 * the timer wheel. hrtimer manages timers in a red-black tree based on
 * expriation time. This may increase the time complexity of insertion and
 * deletetion compared to the timer wheel, but it does not request cascade
 * operation.
 *
 * After all, the hrtimer, rather than the "cascading" timer wheel, was used for
 * accurate timer management. Accordingly, the cascade operation of the timer
 * wheel is no longer meaningful.
 *
 * Today's timer wheel implementation do not use cascade operation as shown in
 * this document(https://lwn.net/Articles/646950/). Of curse, the timer wheel
 * now loses the accuracy it had before for timers above level 1. However, as a
 * benefit of this, it still has fast insertion, deletion, and search time
 * complexity.
 *
 * This trade-off led us to the timer wheel concept to manage I/O requests.
 * (1) ZicIO resubmits I/O requests from the I/O interrupt handler.
 * Therefore fast insertion time complextity is requried. (2) ZicIO needs
 * the cycle of I/O resubmits. It does not require nanosecond accuracy like
 * hrtimer does. It just needs to defer the request until the appropriate timer
 * slack.
 *
 * Unfortunately, kernel/time/timer.c cannot be used as-is. Because it is an
 * implementation that is very closely related to millsecond jiffies. So we
 * created this new file to implement the new timer wheel since we want
 * granularity in microseconds.
 *
 * This file is almost same with kernel/time/timer.c except the two things.
 * (1) This timer wheel has microsecond granularity. (2) The level is much lower
 * than the original timer wheel because this is good enough.
 */

/*
 * Unlike the original timer wheel, zicIO uses ktime_t instead of jiffies.
 *
 * ktime_t is a nanosecond. Since this value is too small, zicio does not use
 * it as a minimum unit (level 0).
 *
 * ZICIO_KTIME_DEFAULT_CLK_SHIT is used to transform the ktime_t to the 
 * unit of the level 0. If ZICIO_DEFULAT_CLK_SHIT is 10, which means 
 * that the unit of level 0 is 1 microsecond. (We will process the raw
 * ktime_t like this, ktime_t >> ZICIO_KTIME_DEFAULT_CLK_SHIT)
 *
 * So, changed granulartity and ragne levels are:
 *
 * ZICIO_KTIME_DEFAULT_CLK_SHIFT 10
 * Level Offset  Granularity              Range
 *  0      0         1 us                  0 us -          63 us
 *  1     64         8 us                 64 us -         511 us
 *  2    128        64 us                512 us -        4095 us (512us - ~4ms)
 *  3    192       512 us               4096 us -       32767 us (~4ms - ~32ms)
 *  4    256      4096 us (~4ms)       32768 us -      262143 us (~32ms - ~256ms)
 *  5    320     32768 us (~32ms)     262144 us -     2097151 us (~256ms - ~2s)
 *  6    384    262144 us (~256ms)   2097152 us -    16777215 us (~2s - ~16s)
 *  7    448   2097152 us (~2s)     16777216 us -   134217727 us (~16s - ~2m)
 *  8    512  16777216 us (~16s)   134217728 us -  1073741822 us (~2m - ~17m)
 *
 * ZICIO_KTIME_DEFAULT_CLK_SHIFT 8
 * Level Offset  Granularity              Range
 *  0	   0         4 us                  0 us -         255 us
 *  1	  64        32 us                256 us -        2047 us (256us - ~2ms)
 *  2	 128       256 us               2048 us -       16383 us (~2ms - ~16ms)
 *  3	 192      2048 us (~2ms)       16384 us -      131071 us (~16ms - ~128ms)
 *  4	 256     16384 us (~16ms)     131072 us -     1048575 us (~128m - ~1s)
 *  5	 320    131072 us (~128ms)   1048576 us -     8388607 us (~1s - ~8s)
 *  6	 384   1048576 us (~1s)      8388608 us -    67108863 us (~8s - ~64s)
 *  7	 448   8388608 us (~8s)     67108864 us -   536870911 us (~64s - ~8m)
 *  8    512  67108864 us (~64s)   536870912 us -  4294967288 us (~8m - ~70m)
 */
#define ZICIO_KTIME_DEFAULT_CLK_SHIFT	(6)

/* Clock divisor for the next level */
#define ZICIO_LVL_CLK_SHIFT	(3)
#define ZICIO_LVL_CLK_DIV	(1UL << ZICIO_LVL_CLK_SHIFT)
#define ZICIO_LVL_CLK_MASK	(ZICIO_LVL_CLK_DIV - 1)
#define ZICIO_LVL_SHIFT(n)	((n) * ZICIO_LVL_CLK_SHIFT)
#define ZICIO_LVL_GRAN(n)	(1UL << ZICIO_LVL_SHIFT(n))

/*
 * The time start value for each level to select the bucket at enqueue time. We
 * start from the last possible delta of the previous level so that we can later
 * add an extra ZICIO_LVL_GRAN(n) to n (see zicio_calc_index()).
 */
#define ZICIO_LVL_START(n) \
	((ZICIO_LVL_SIZE - 1) << (((n) - 1) * ZICIO_LVL_CLK_SHIFT))

/* Size of each clock level */
#define ZICIO_LVL_BITS		(6)
#define ZICIO_LVL_SIZE		(1UL << ZICIO_LVL_BITS)
#define ZICIO_LVL_MASK		(ZICIO_LVL_SIZE - 1)
#define ZICIO_LVL_OFFS(n)	((n) * ZICIO_LVL_SIZE)

/* Level depth.*/
#define ZICIO_LVL_DEPTH		(9)

/* The cutoff (max. capacity of the wheel) */
#define ZICIO_WHEEL_TIMEOUT_CUTOFF	(ZICIO_LVL_START(ZICIO_LVL_DEPTH))
#define ZICIO_WHEEL_TIMEOUT_MAX	\
	(ZICIO_WHEEL_TIMEOUT_CUTOFF - ZICIO_LVL_GRAN(ZICIO_LVL_DEPTH - 1))

/* The resulting wheel size */
#define ZICIO_WHEEL_SIZE	(ZICIO_LVL_SIZE * ZICIO_LVL_DEPTH)

/**
 * zicio_request_timer_base - timer wheel for zicio's request control.
 * @lock: spinlock for used for raw_spin_(lock/unlock)_irq(save/restore).
 * @clk: base time for adding a new timer
 * @next_expiry: next expiring time
 * @running_timers: fetched request timers
 * @expired_queue: expired requests are moved to this queue
 * @cpu: timer base's cpu
 * @trigger: request trigger timer
 * @trigger_running: is trigger running?
 * @pending_map: bitmap representing the index of the pending timer
 * @vectors: array representing the timer wheel for each level
 *
 * This data structure mimics timer_base in kernel/time/timer.c to represent
 * timer wheel. ZicIO use this data structure to submit I/O requests at the
 * right time.
 *
 * Unlike the original timer_base, there are changes to some members:
 *
 * - Removed
 *
 *   (1) is_idle, timers_pending, next_expiry_recalc
 *     This member is related to timer interrupt. We don't need it because
 *     zicio doesn't expire the request timers in the timer interrupt. 
 * 
 *   (2) expiry_lock, timer_waiters
 *     See the comment (1) of __zicio_fetch_next_request_timers() for these
 *     members.
 *
 * - Added
 * 
 *   (1) expired_queue
 *     Expired request timers come into this queue to reserve timer wheel's
 *     space.
 *
 *   (2) trigger
 *     If the next request timer is too far away, zicIO can return the
 *     assigned struct request. In this case, the request must be reassigned at
 *     an appropriate time, using a timer softirq.
 *
 * - Changed
 *
 *   (1) data type of clk and next_expiry
 *     The type of these members are changed from unsigned long to the ktime_t
 *     to represent microsecond time scale.
 *
 *   (2) data type of vectors
 *     As mentioned above, expired request timers are moved to expired_queue.
 *     To do this efficiently, zicio uses one more pointer for each index
 *     of the timer wheel. 
 */
struct zicio_nvme_cmd_timer_base {
	raw_spinlock_t		lock;
	ktime_t				clk;
	ktime_t				next_expiry;
	struct list_head	running_timers;
	struct list_head 	expired_queue;
	struct timer_list	trigger;
	atomic_t			trigger_running;
	unsigned int		cpu;
	DECLARE_BITMAP(pending_map, ZICIO_WHEEL_SIZE);
	struct list_head 	vectors[ZICIO_WHEEL_SIZE];
} ____cacheline_aligned;

static DEFINE_PER_CPU(struct zicio_nvme_cmd_timer_base, timer_base);

/* Slab allocator for zicio_nvme_cmd_timer */
static struct kmem_cache *zicio_nvme_cmd_timer_cache;

#define ZICIO_DEFAULT_TIMER_SOFTIRQ_INTERVAL (25)
static void zicio_softtimer_timer_softirq_callback(struct timer_list *timer);

/**
 * zicio_ktime_apply_default_clk_shift - apply default clk shift to the ktime_t
 * @know: raw ktime_t to apply ZICIO_KTIME_DEFAULT_CLK_SHIFT
 *
 * In most cases, level 0 of the zicio timer wheel does not use ktime_t as
 * it is. Therefore, before using raw ktime_t, process it in unit of level 0
 * through this function.
 */
static inline ktime_t 
zicio_ktime_apply_default_clk_shift(ktime_t know)
{
	return (know >> ZICIO_KTIME_DEFAULT_CLK_SHIFT);
}

/**
 * zicio_ktime_get_back_from_default_clk_unit - get back from default clk into
 * raw ktime_t
 * @t: time to transform
 *
 * ZicIO uses its own time resolution. Transform it to the raw ktime_t.
 */
static inline ktime_t
zicio_ktime_get_back_from_default_clk_unit(ktime_t t)
{
	return (t << ZICIO_KTIME_DEFAULT_CLK_SHIFT);
}

/**
 * These macros are part of the macros present in include/linux/timer.h
 * Only the necessary parts for zicIO were excerpted. So, the remaining
 * flags may be added later as needed.
 */
#define ZICIO_NVME_CMD_TIMER_CPUMASK		0x0003FFFF
#define ZICIO_NVME_CMD_TIMER_BASEMASK		(ZICIO_NVME_CMD_TIMER_CPUMASK)
#define ZICIO_NVME_CMD_TIMER_ARRAYSHIFT		22
#define ZICIO_NVME_CMD_TIMER_ARRAYMASK		0xFFC00000

static inline struct zicio_nvme_cmd_timer_base *
zicio_get_nvme_cmd_timer_cpu_base(u32 cpu)
{
	struct zicio_nvme_cmd_timer_base *base
		= per_cpu_ptr(&timer_base, cpu);
	return base;
}

static inline struct zicio_nvme_cmd_timer_base *
zicio_get_nvme_cmd_timer_base(u32 tflags)
{
	return zicio_get_nvme_cmd_timer_cpu_base(
		tflags & ZICIO_NVME_CMD_TIMER_CPUMASK);
}

static inline unsigned int
zicio_nvme_cmd_timer_get_idx(struct zicio_nvme_cmd_timer *timer)
{
	return (timer->flags & ZICIO_NVME_CMD_TIMER_ARRAYMASK)
		>> ZICIO_NVME_CMD_TIMER_ARRAYSHIFT;
}

static inline void
zicio_nvme_cmd_timer_set_idx(struct zicio_nvme_cmd_timer *timer,
	unsigned int idx)
{
	timer->flags = (timer->flags & ~ZICIO_NVME_CMD_TIMER_ARRAYMASK)
		| (idx << ZICIO_NVME_CMD_TIMER_ARRAYSHIFT);
}

/*
 * return 1 if the timer is pending, 0 if not.
 */
static inline int
zicio_nvme_cmd_timer_pending(const struct zicio_nvme_cmd_timer *timer)
{
	return !list_empty(&timer->entry)
		&& !(timer->entry.next == LIST_POISON1);
}

static inline void
zicio_detach_nvme_cmd_timer(struct zicio_nvme_cmd_timer *timer)
{
	struct list_head *entry = &timer->entry;
	list_del_init(entry);
}

static void
zicio_detach_nvme_cmd_timer_if_pending(
	struct zicio_nvme_cmd_timer *timer,
	struct zicio_nvme_cmd_timer_base *base)
{
	unsigned idx = zicio_nvme_cmd_timer_get_idx(timer);
	struct list_head *vector;

	if (!zicio_nvme_cmd_timer_pending(timer))
		return;

	if (idx < ZICIO_WHEEL_SIZE)
		vector = base->vectors + idx;
	else
		vector = &base->expired_queue;

	/*
	 * Note that even if idx is less than ZICIO_WHEEL_SIZE, this timer can
	 * exist in @base->expired_queue. So check once more with list_is_first().
	 */
	if (list_is_singular(vector) && list_is_first(&timer->entry, vector))
		__clear_bit(idx, base->pending_map);

	zicio_detach_nvme_cmd_timer(timer);
}

/*
 * Note that the @expires is the value after ZICIO_KTIME_DEFAULT_CLK_SHIFT has
 * been applied to raw ktime_t.
 */
static inline unsigned
zicio_calc_index(ktime_t expires, unsigned lvl, ktime_t *bucket_expiry)
{
	/*
	 * Unlike the original function, calc_index(), zicIO have to round down the
	 * expire time. This is because, in the original timer wheel, it must be
	 * prevented to expire the timer before the expiration time. But in zicIO,
	 * it must be prevented from expiring later than the expiration time.
	 */
	expires = expires >> ZICIO_LVL_SHIFT(lvl);
	*bucket_expiry = expires << ZICIO_LVL_SHIFT(lvl);
	return ZICIO_LVL_OFFS(lvl) + (expires & ZICIO_LVL_MASK);
}

/*
 * Calculate the index at which the new timer will go. The level to enter
 * is determined by how far the timer's expiration is relative to base clk.
 *
 * If @expires - @clk is negative, it means that the expiration time of the
 * timer is earlier than @base->clk. This indicates that the given timer should
 * be dequeued from the timer wheel as soon as possible.
 *
 * Note that the @expires is the value after ZICIO_KTIME_DEFAULT_CLK_SHIFT has
 * been applied to raw ktime_t.
 */
static int
zicio_calc_wheel_index(ktime_t expires, ktime_t clk, ktime_t *bucket_expiry)
{
	ktime_t delta = expires - clk;
	unsigned int idx;

	if (delta < 0) {
		idx = -1;
		*bucket_expiry = clk;
	} else if (delta < ZICIO_LVL_START(1)) {
		idx = zicio_calc_index(expires, 0, bucket_expiry);
	} else if (delta < ZICIO_LVL_START(2)) {
		idx = zicio_calc_index(expires, 1, bucket_expiry);
	} else if (delta < ZICIO_LVL_START(3)) {
		idx = zicio_calc_index(expires, 2, bucket_expiry);
	} else if (delta < ZICIO_LVL_START(4)) {
		idx = zicio_calc_index(expires, 3, bucket_expiry);
	} else if (delta < ZICIO_LVL_START(5)) {
		idx = zicio_calc_index(expires, 4, bucket_expiry);
	} else if (delta < ZICIO_LVL_START(6)) {
		idx = zicio_calc_index(expires, 5, bucket_expiry);
	} else if (delta < ZICIO_LVL_START(7)) {
		idx = zicio_calc_index(expires, 6, bucket_expiry);
	} else if (delta < ZICIO_LVL_START(8)) {
		idx = zicio_calc_index(expires, 7, bucket_expiry);
	} else {
		/*
		 * Large timeouts to expire at the capacity limit of the wheel.
		 */
		if (delta >= ZICIO_WHEEL_TIMEOUT_CUTOFF)
			expires = clk + ZICIO_WHEEL_TIMEOUT_MAX;

		idx = zicio_calc_index(expires, ZICIO_LVL_DEPTH - 1, bucket_expiry);
	}

	return idx;
}

/*
 * Enqueue the timer into the bucket, mark it pending in the bitmap, store the
 * index in the timer flags.
 *
 * Unlike the original function, enqueue_timer(), we don't have to call the
 * trigger_dyntick_cpu(). This is because enqueued timers have nothing to do
 * with timer interrupts.
 *
 * Note that the @expires is the value after ZICIO_KTIME_DEFAULT_CLK_SHIFT has
 * been applied to raw ktime_t.
 *
 * Note that the ktime_t values used here are already applied
 * ZICIO_KTIME_DEFAULT_CLK_SHIFT.
 */
static void
zicio_enqueue_nvme_cmd_timer(struct zicio_nvme_cmd_timer_base *base,
	struct zicio_nvme_cmd_timer *timer, int idx, ktime_t bucket_expiry)
{
	if (idx >= 0) {
		list_add_tail(&timer->entry, base->vectors + idx);
		__set_bit(idx, base->pending_map);
		zicio_nvme_cmd_timer_set_idx(timer, idx);
	} else {
		BUG_ON(idx != -1);
		list_add_tail(&timer->entry, &base->expired_queue);
		zicio_nvme_cmd_timer_set_idx(timer, ZICIO_WHEEL_SIZE);
	}

	if (ktime_before(bucket_expiry, base->next_expiry))
		base->next_expiry = bucket_expiry;
}

/*
 * Note that the ktime_t values used here are already applied
 * ZICIO_KTIME_DEFAULT_CLK_SHIFT.
 */
static void
zicio_internal_add_nvme_cmd_timer(struct zicio_nvme_cmd_timer_base *base,
	struct zicio_nvme_cmd_timer *timer)
{
	ktime_t bucket_expiry;
	int idx;

	idx = zicio_calc_wheel_index(timer->expires, base->clk, &bucket_expiry);
	zicio_enqueue_nvme_cmd_timer(base, timer, idx, bucket_expiry);
}

/*
 * Note that the ktime_t values used here are already applied
 * ZICIO_KTIME_DEFAULT_CLK_SHIFT.
 */
static inline void
zicio_forward_nvme_cmd_timer_base(struct zicio_nvme_cmd_timer_base *base)
{
	/* raw ktime_t => default clk unit of zicIO */
	ktime_t know = zicio_ktime_apply_default_clk_shift(ktime_get());

	if (know - base->clk < 1)
		return;

	if (ktime_after(base->next_expiry, know)) {
		base->clk = know;
	} else {
		if (WARN_ON_ONCE(ktime_before(base->next_expiry, base->clk)))
			return;
		base->clk = base->next_expiry;
	}
}

/*
 * Note that we do not consider migrating the timer to another base. So unlike
 * the original function, lock_timer_base(), we do not loop to check whether the
 * timer is migrating or not.
 */
static struct zicio_nvme_cmd_timer_base *
zicio_lock_nvme_cmd_timer_base(struct zicio_nvme_cmd_timer *timer,
	unsigned long *flags)
{
	struct zicio_nvme_cmd_timer_base *base;
	base = zicio_get_nvme_cmd_timer_base(timer->flags);
	raw_spin_lock_irqsave(&base->lock, *flags);
	return base;
}

/*
 * Note that @expires is a raw ktime_t. So we have to apply
 * ZICIO_KTIME_DEFAULT_CLK_SHIFT to it before adding it into the timer wheel.
 */
static void
zicio_add_nvme_cmd_timer_on(struct zicio_nvme_cmd_timer *timer,
	ktime_t expires, int cpu)
{
	struct zicio_nvme_cmd_timer_base *base;
	unsigned long flags;

	if (WARN_ON_ONCE(timer == NULL))
		return;
	if (WARN_ON_ONCE(zicio_nvme_cmd_timer_pending(timer)))
		return;

	expires = zicio_ktime_apply_default_clk_shift(expires);
	timer->expires = expires;

	timer->flags = (timer->flags & ~ZICIO_NVME_CMD_TIMER_CPUMASK)
		| (cpu & ZICIO_NVME_CMD_TIMER_CPUMASK);

	base = zicio_lock_nvme_cmd_timer_base(timer, &flags);
	zicio_forward_nvme_cmd_timer_base(base);
	zicio_internal_add_nvme_cmd_timer(base, timer);
	raw_spin_unlock_irqrestore(&base->lock, flags);
}

/*
 * Allocate new timer and insert it to the channel's linke list and wheel's
 * linked list.
 */
void zicio_insert_nvme_cmd_timer(struct zicio_notify_descriptor *zicio_notify_desc,
	u32 start_nvme_cmd_info_offset, u16 nr_nvme_cmd_info,
	u16 local_huge_page_idx, u16 start_page_offset, ktime_t expires)
{
	struct zicio_nvme_cmd_timer *timer = zicio_allocate_nvme_cmd_timer();
	struct zicio_descriptor *zicio_desc = (zicio_descriptor *)zicio_notify_desc;
	unsigned long flags;

#if (CONFIG_ZICIO_DEBUG_LEVEL >= 2)
	printk("[cpu %d] zicio_insert_nvme_cmd_timer() start_nvme_cmd_info_offset: %d, nr_nvme_cmd_info: %d, local_huge_page_idx: %d\n", zicio_notify_desc->zicio_desc.cpu_id, start_nvme_cmd_info_offset, nr_nvme_cmd_info, local_huge_page_idx);
#endif

	BUG_ON(timer == NULL);

	ZICIO_INIT_NVME_CMD_TIMER(timer, zicio_notify_desc, start_nvme_cmd_info_offset,
		nr_nvme_cmd_info, local_huge_page_idx, start_page_offset);

	/* Add it to the channel */
	spin_lock_irqsave(&zicio_notify_desc->lock, flags);
	list_add_tail(&timer->sibling, &zicio_notify_desc->active_nvme_cmd_timers);
	spin_unlock_irqrestore(&zicio_notify_desc->lock, flags);

	/* Add it to the wheel */
	zicio_add_nvme_cmd_timer_on(timer, expires, zicio_desc->cpu_id);
}
EXPORT_SYMBOL(zicio_insert_nvme_cmd_timer);

/*
 * This function is called for init trigger called when opening the channel,
 * responsible for collecting commands related to the first huge page request.
 */
void zicio_init_trigger_insert_nvme_cmd_timer(
	struct zicio_notify_descriptor *zicio_notify_desc, u16 nr_nvme_cmd_info,
	u16 next_cmd_page_offset)
{
	struct zicio_nvme_cmd_timer *timer = zicio_allocate_nvme_cmd_timer();
	struct zicio_descriptor *zicio_desc = (zicio_descriptor *)zicio_notify_desc;
	unsigned long flags;

#if (CONFIG_ZICIO_DEBUG_LEVEL >= 2)
	printk("[cpu %d] zicio_init_trigger_insert_nvme_cmd_timer() nr_nvme_cmd_info: %d, next_cmd_page_offset: %d\n", zicio_notify_desc->zicio_desc.cpu_id, nr_nvme_cmd_info, next_cmd_page_offset);
#endif

	BUG_ON(timer == NULL);

	/*
	 * The first nvme_cmd_info is used directly in the init trigger function.
	 * So the start_nvme_cmd_info_offset is 1 and huge page index is 0.
	 */
	ZICIO_INIT_NVME_CMD_TIMER(timer, zicio_notify_desc, 1,
		nr_nvme_cmd_info, 0, next_cmd_page_offset);

	/* Add it to the channel */
	spin_lock_irqsave(&zicio_notify_desc->lock, flags);
	list_add_tail(&timer->sibling, &zicio_notify_desc->active_nvme_cmd_timers);
	spin_unlock_irqrestore(&zicio_notify_desc->lock, flags);

	/* Add it to the wheel */
	zicio_add_nvme_cmd_timer_on(timer, ktime_get(), zicio_desc->cpu_id);
}
EXPORT_SYMBOL(zicio_init_trigger_insert_nvme_cmd_timer);

/*
 * Remove it from the channel's linked list and wheel's linked list. Then free
 * it.
 */
void zicio_delete_nvme_cmd_timer(struct zicio_nvme_cmd_timer *timer)
{
	struct zicio_nvme_cmd_timer_base *base;
	struct zicio_notify_descriptor *zicio_notify_desc;
	unsigned long flags;
	
	BUG_ON(timer == NULL);

	zicio_notify_desc = timer->zicio_notify_desc;

	/* Remove it from the wheel */
	base = zicio_lock_nvme_cmd_timer_base(timer, &flags);
	if (zicio_nvme_cmd_timer_pending(timer))
		zicio_detach_nvme_cmd_timer_if_pending(timer, base);
	raw_spin_unlock_irqrestore(&base->lock, flags);

	/* Remove it from the channel */
	spin_lock_irqsave(&zicio_notify_desc->lock, flags);
	list_del_init(&timer->sibling);
	spin_unlock_irqrestore(&zicio_notify_desc->lock, flags);

	zicio_free_nvme_cmd_timer(timer);
}
EXPORT_SYMBOL(zicio_delete_nvme_cmd_timer);

/**
 * zicio_next_pending_bucket - find next bit in the pending map.
 * @base: timer wheel base
 * @offset: first offset of the target level
 * @clk: offset within the same level of the target clock
 *
 * Find the next pending bucket of a level. Search from level start (@offset) +
 * @clk upwards and if nothing there, search from start of the level (@offset)
 * up to @offset + clk.
 *
 * Note that the ktime_t values used here are already applied
 * ZICIO_KTIME_DEFAULT_CLK_SHIFT.
 */
static int zicio_next_pending_bucket(
	struct zicio_nvme_cmd_timer_base *base, unsigned offset, ktime_t clk)
{
	unsigned pos, start = offset + clk;
	unsigned end = offset + ZICIO_LVL_SIZE;

	pos = find_next_bit(base->pending_map, end, start);
	if (pos < end)
		return pos - start;

	pos = find_next_bit(base->pending_map, start, offset);
	return pos < start ? pos + ZICIO_LVL_SIZE - start : -1;
}

/*
 * Caller must hold @base->lock.
 *
 * Note that the ktime_t values used here are already applied
 * ZICIO_KTIME_DEFAULT_CLK_SHIFT.
 */
static ktime_t zicio_next_expire_time(struct zicio_nvme_cmd_timer_base *base)
{
	ktime_t clk, next;
	unsigned lvl, offset, adj = 0;

	next = KTIME_MAX;
	clk = base->clk;
	for (lvl = 0, offset = 0; lvl < ZICIO_LVL_DEPTH;
			lvl++, offset += ZICIO_LVL_SIZE) {
		int pos = zicio_next_pending_bucket(base, offset, clk & ZICIO_LVL_MASK);
		ktime_t lvl_clk = clk & ZICIO_LVL_CLK_MASK;

		if (pos >= 0) {
			ktime_t tmp = clk + (ktime_t) pos;

			tmp <<= ZICIO_LVL_SHIFT(lvl);
			if (ktime_before(tmp, next))
				next = tmp;

			/*
			 * If the next expiration happens before we reach the next level, no
			 * need to check further.
			 */
			if (pos <= ((ZICIO_LVL_CLK_DIV - lvl_clk)
						& ZICIO_LVL_CLK_MASK))
				break;
		}

		/*
		 * If the next expiration is outside the granularity of the curent
		 * level, the next level must be checked.
		 */
		adj = lvl_clk ? 1 : 0;
		clk >>= ZICIO_LVL_CLK_SHIFT;
		clk += adj;
	}

	return next;
}

/*
 * Collect expired timers. 
 *
 * It is guaranteed that the current time has passed @base->next_expiry. So set
 * the @base->clk to the @base->next_expiry and move the expired request timers
 * to the @base->expired_queue.
 */
static void
zicio_collect_expired_nvme_cmd_timers(struct zicio_nvme_cmd_timer_base *base)
{
	ktime_t clk = base->clk = base->next_expiry;
	struct list_head *vec;
	unsigned int idx;
	int i;

	for (i = 0; i < ZICIO_LVL_DEPTH; i++) {
		idx = (clk & ZICIO_LVL_MASK) + i * ZICIO_LVL_SIZE;

		if (__test_and_clear_bit(idx, base->pending_map)) {
			vec = base->vectors + idx;
			list_bulk_move_tail(&base->expired_queue, vec->next, vec->prev);
		}

		/* Is it time to look at the next level? */
		if (clk & ZICIO_LVL_CLK_MASK)
			break;
		/* Shift clock for the next level granularity */
		clk >>= ZICIO_LVL_CLK_SHIFT;
	}
}

/*
 * Move only the expired timers to the running timer queue.
 *
 * In certain situations, timers that have not yet expired can also be fetched.
 * In other words, timers that were not in the @base->expired_queue can also be
 * fetched.
 *
 * Therefore, @base->running_timers comprehensively manages timers that can be
 * fetched regardless of their expiration time.
 * 
 * This function moves the timers that are in the @base->expired_queue to
 * @base->running_timers.
 */
static inline void
zicio_make_expired_timers_running(struct zicio_nvme_cmd_timer_base *base)
{
	if (list_empty(&base->expired_queue))
		return;

	list_bulk_move_tail(&base->running_timers, base->expired_queue.next,
		base->expired_queue.prev);
}

/*
 * Move a non-expired timer to the running timer queue.
 */
static inline void 
zicio_make_non_expired_timer_running(struct zicio_nvme_cmd_timer_base *base)
{
	unsigned int idx;
	struct list_head *vec;

	for (idx = 0; idx < ZICIO_WHEEL_SIZE; idx++) {
		if (__test_and_clear_bit(idx, base->pending_map)) {
			vec = base->vectors + idx;
			BUG_ON(list_empty(vec));
			list_bulk_move_tail(&base->running_timers, vec->next, vec->prev);
			break;
		}
	}
}

/*
 * This function corresponds to the __run_timers(). But has some differences.
 *
 * (1) It does not use expiry_lock.
 *
 *   The original timer wheel has expiry_lock. This is used to increase the
 *   responsiveness of real-time systems. The original timer has a callback
 *   function and expiry_lock is acquired/released every callback function's
 *   invoke/end. This gave an opportunity to execute the callback function in
 *   other execution flows that want to run on the same timer base.
 *
 *   However, zicIO's timer is not about callback function. We only changes the
 *   link of the timers on the timer base, not invokes callback function.
 *
 * (2) A new member, @base->expired_queue.
 *
 *   The original timer wheel executes callback functions of *all* timers
 *   corresponding to the expire time. However, here, only the timers
 *   corresponding to the exprie time need to be returned to the caller (even
 *   those not corresponding to the expire time can be returned depending on the
 *   given boolean value, @fetch_non_expired_timers).
 *
 *   The problem is that the caller does not require multiple timers (maybe).
 *   Therefore, the remaining timers may be taking up space on the timer wheel.
 *   This causes new timers added in the future to go to a higher level. Then it
 *   will be a factor that unnecessarily lowers the accuracy of the timer wheel.
 *
 *   Therefore, in this function, timers whose expire time has passed are
 *   detatched from the timer wheel and moved to the @base->expired_queue. 
 *
 * Note that @base->lock must be acquired by caller.
 */
static void
zicio_make_nvme_cmd_timers_running(
	struct zicio_nvme_cmd_timer_base *base, bool fetch_non_expired_timer)
{
	ktime_t know = zicio_ktime_apply_default_clk_shift(ktime_get());

	while (know >= base->clk && know >= base->next_expiry) {
		zicio_collect_expired_nvme_cmd_timers(base);
		base->clk++;
		base->next_expiry = zicio_next_expire_time(base);
	}

	zicio_make_expired_timers_running(base);

	if (list_empty(&base->running_timers) && fetch_non_expired_timer)
		zicio_make_non_expired_timer_running(base);
}

/*
 * If the timer wheel stops, a softirq is issued later to restart the timer
 * wheel.
 *
 * Note that add_timer_on() should not be called multiple times for the same
 * timer_list. So we use the @base->trigger_running.
 */
static void
zicio_try_trigger_softtimer_timer_softirq(
	struct zicio_nvme_cmd_timer_base *base, unsigned long interval)
{
	if (atomic_cmpxchg(&base->trigger_running, 0, 1) == 0) {
		BUG_ON(timer_pending(&base->trigger));
		timer_setup(&base->trigger,
			zicio_softtimer_timer_softirq_callback, TIMER_PINNED);
		base->trigger.expires = get_jiffies_64() + interval;
		add_timer_on(&base->trigger, base->cpu);
	}
}

/*
 * zicio_try_trigger_softtimer_timer_softirq() requires timer wheel structure
 * which is not accessible outside this file. To export this functionaliy, we
 * defined this function.
 */
void zicio_notify_trigger_softtimer_timer_softirq(int cpu, unsigned long interval)
{
	struct zicio_nvme_cmd_timer_base *base;
	base = per_cpu_ptr(&timer_base, cpu);
	zicio_try_trigger_softtimer_timer_softirq(base, interval);
}
EXPORT_SYMBOL(zicio_notify_trigger_softtimer_timer_softirq);

static int zicio_wakeup_softirqd(int cpu)
{
	struct task_struct *tsk = per_cpu(ksoftirqd, cpu);
	unsigned long flags;
	int wakeup = 0;

	BUG_ON(tsk == NULL);

	local_irq_save(flags);
	if (!task_is_running(tsk) || __kthread_should_park(tsk))
		wakeup = wake_up_process(tsk);
	local_irq_restore(flags);

	return wakeup;
}

/*
 * Note that this callback function does not directly restart the NVMe command
 * timer wheel.
 *
 * Softirqs can be executed not only by 'kosftirqd' process but also after
 * handling interrupts. Restarting the timer wheel requires I/O, and performing
 * I/O in the interrupt context can lead to unpredictable situations.
 *
 * Therefore, we only wake up 'ksoftirqd' here and then stop the function.
 * In the main function of 'kosftirqd', it will attempt to restart our timer
 * wheel in the process context. 
 */
static void zicio_softtimer_timer_softirq_callback(struct timer_list *timer)
{
	struct zicio_nvme_cmd_timer_base *base = from_timer(base,timer,trigger);
	int wakeup = 0;

	atomic_set(&base->trigger_running, 0);

	wakeup = zicio_wakeup_softirqd(base->cpu);
	if (!wakeup)
		zicio_try_trigger_softtimer_timer_softirq(base, 0);
}

/*
 * Fetch the most urgent nvme command info from the timer wheel.
 *
 * Return true when the info is fetched.
 */
bool zicio_fetch_nvme_cmd_data_from_wheel(
	struct zicio_nvme_cmd_timer_data *fetched_data, int cpu,
	bool enable_fetch_non_expired)
{
	struct zicio_nvme_cmd_timer_base *base;
	struct zicio_nvme_cmd_timer *timer;
	bool fetched = false, free_timer = false;
	unsigned long flags;
	int chunk_idx, cmd_idx;
	
	BUG_ON(fetched_data == NULL);

	base = per_cpu_ptr(&timer_base, cpu);

	raw_spin_lock_irqsave(&base->lock, flags);

	if (list_empty(&base->running_timers))
		zicio_make_nvme_cmd_timers_running(base, enable_fetch_non_expired);

	if (!list_empty(&base->running_timers)) {
		struct zicio_notify_descriptor *zicio_notify_desc;
		zicio_nvme_cmd_info nvme_cmd_info;
		
		timer = (struct zicio_nvme_cmd_timer *) base->running_timers.next;

#if (CONFIG_ZICIO_DEBUG_LEVEL >= 2)
		printk("[cpu %d] __zicio_fetch_nvme_cmd_data() huge_page: %d, next_cmd_info_index: %d, nr_cmd_info: %d, start_nvme_cmd_info_offset: %d\n",
				timer->zicio_notify_desc->zicio_desc.cpu_id,
				timer->local_huge_page_idx,
				timer->next_cmd_info_index,
				timer->nr_cmd_info,
				timer->start_nvme_cmd_info_offset);
#endif

		BUG_ON(timer->zicio_notify_desc == NULL ||
			timer->next_cmd_info_index >= timer->nr_cmd_info);
		
		zicio_notify_desc = timer->zicio_notify_desc;

		fetched_data->zicio_notify_desc = zicio_notify_desc;
		fetched_data->local_huge_page_idx = timer->local_huge_page_idx;
		fetched_data->page_offset = timer->next_cmd_page_offset;
		fetched_data->nvme_cmd_info_offset
			= timer->start_nvme_cmd_info_offset + timer->next_cmd_info_index;
		fetched = true;

		/* Update timer for the next command */
		chunk_idx = fetched_data->nvme_cmd_info_offset /
					ZICIO_NVME_CMD_INFO_PER_CHUNK;
		cmd_idx = fetched_data->nvme_cmd_info_offset %
				  ZICIO_NVME_CMD_INFO_PER_CHUNK;
		nvme_cmd_info
			= zicio_notify_desc->nvme_cmd_infos[chunk_idx][cmd_idx];
		timer->next_cmd_info_index++;
		timer->next_cmd_page_offset
			+= ZICIO_NVME_CMD_INFO_GET_LENGTH(nvme_cmd_info) + 1;

		/*
		 * If this timer has no nvme command information anymore, detach it from
		 * the wheel. Note that we free it after unlock the @base->lock to avoid
		 * deadlock.
		 */
		if (timer->next_cmd_info_index == timer->nr_cmd_info) {
			list_del(&timer->entry);
			free_timer = true;
		}
	}

	raw_spin_unlock_irqrestore(&base->lock, flags);

	if (free_timer)
		zicio_delete_nvme_cmd_timer(timer);

	return fetched;
}
EXPORT_SYMBOL(zicio_fetch_nvme_cmd_data_from_wheel);

/*
 * Timer wheel initialization function.
 * It runs during the kernel boot.
 */
void __init zicio_init_nvme_cmd_timer_wheel(void)
{
	struct zicio_nvme_cmd_timer_base *base;
	int cpu, i;

	for_each_possible_cpu(cpu) {
		base = per_cpu_ptr(&timer_base, cpu);
		base->cpu = cpu;
		raw_spin_lock_init(&base->lock);
		base->clk = zicio_ktime_apply_default_clk_shift(ktime_get());
		base->next_expiry = KTIME_MAX;

		for (i = 0; i < ZICIO_WHEEL_SIZE; i++) {
			INIT_LIST_HEAD(base->vectors + i);
		}
		INIT_LIST_HEAD(&base->running_timers);
		INIT_LIST_HEAD(&base->expired_queue);
		atomic_set(&base->trigger_running, 0);
	}
}
EXPORT_SYMBOL(zicio_init_nvme_cmd_timer_wheel);

/*
 * Slab allocator initialization function for zicio_nvme_cmd_timer.
 * It runs during the kernel boot.
 */
void __init zicio_init_nvme_cmd_timer_slab_cache(void)
{
	zicio_nvme_cmd_timer_cache
		= kmem_cache_create("zicio_nvme_cmd_timer",
			sizeof(struct zicio_nvme_cmd_timer),
			sizeof(struct zicio_nvme_cmd_timer),
			SLAB_PANIC | SLAB_HWCACHE_ALIGN, NULL);

	if (zicio_nvme_cmd_timer_cache == NULL) {
		printk("[ZICIO INIT] NVMe command timer slab allocation is failed\n");
		BUG_ON(true);
	}
}
EXPORT_SYMBOL(zicio_init_nvme_cmd_timer_slab_cache);

struct zicio_nvme_cmd_timer *zicio_allocate_nvme_cmd_timer(void)
{
	return kmem_cache_alloc(zicio_nvme_cmd_timer_cache, GFP_KERNEL);
}
EXPORT_SYMBOL(zicio_allocate_nvme_cmd_timer);

void zicio_free_nvme_cmd_timer(void *timer)
{
	kmem_cache_free(zicio_nvme_cmd_timer_cache, timer);
}
EXPORT_SYMBOL(zicio_free_nvme_cmd_timer);

/*
 * Dump function for debugging
 */
void zicio_dump_nvme_cmd_timer_wheel(int cpu)
{
	struct zicio_nvme_cmd_timer_base *base;
	struct list_head *pos;
	unsigned long flags;
	int i, count = 0;

	base = per_cpu_ptr(&timer_base, cpu);

	raw_spin_lock_irqsave(&base->lock, flags);

	list_for_each(pos, &base->running_timers) {
		count++;
	}
	printk(KERN_WARNING "[ZIC_REQ] cpu: %d, running: %d\n", cpu, count);

	count = 0;
	list_for_each(pos, &base->expired_queue) {
		count++;
	}
	printk(KERN_WARNING "[ZIC_REQ] cpu: %d, expired: %d\n", cpu, count);

	for (i = 0; i < ZICIO_WHEEL_SIZE; i++) {
		count = 0;
		list_for_each(pos, base->vectors + i) {
			count++;
		}
		printk(KERN_WARNING "[ZIC_REQ] cpu: %d, vec%d: %d\n", cpu, i, count);
	}

	printk(KERN_WARNING "[ZIC_REQ] cpu: %d, trigger_running: %d\n",
		cpu, atomic_read(&base->trigger_running));

	printk(KERN_WARNING "[ZIC_REQ] cpu: %d, trigger expires: %ld, current jiffies: %lld\n",
		cpu, base->trigger.expires, get_jiffies_64());

	raw_spin_unlock_irqrestore(&base->lock, flags);
}
EXPORT_SYMBOL(zicio_dump_nvme_cmd_timer_wheel);
