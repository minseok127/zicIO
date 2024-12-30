#ifndef _LINUX_ZICIO_H
#define _LINUX_ZICIO_H
#ifdef CONFIG_ZICIO /* it must be changed to CONFIG_ZICIO */

#include <linux/zicio.h>

/*
 * ZicIO minimizes filesystem retrieval during channel opening and does not
 * perform any retrieval during runtime. This approach is employed to reduce
 * read amplification related to extent blocks (ext4).
 * 
 * We do not access the filesystem every time we perform I/O. Once we have
 * performed an search for extent block during channel opening, we do not
 * re-search it again.
 *
 * This type is generated when we match the batches we need to read with
 * ext4_extent. Each information is 8byte and contains compressed information
 * necessary for one NVMe I/O operation (one batch can generate multiple I/O
 * operations).
 *
 * It contains three information:
 *
 * 1. FSBLK (filesystem-wide block address): 48bits
 *   The target filesystem of ZicIO is ext4. It's mapping information,
 *   ext4_extent, stores the physical address using the two variables,
 *   @ee_start_hi and @ee_start_lo. These are 48bits.
 *
 * 2. Length (the number of 4KB pages): 6bits
 *   The maximum size of I/O is 256KB (in our testing environment). When
 *   converted into 4KB units, this corresponds to values ranging from 0 to 63.
 *   So 6bits are needed to represent this range. (0 => 4KB, 63 => 256KB).
 *
 * 3. File index: 7bits
 *   The zicIO channel can read multiple files, achieved by allocating 7 bits,
 *   thereby covering up to 128 files.
 */
typedef __le64 zicio_nvme_cmd_info;

#define ZICIO_NVME_CMD_INFO_FSBLK_MASK		(0x0000FFFFFFFFFFFF) /* 48 bits */
#define ZICIO_NVME_CMD_INFO_LENGTH_MASK		(0x003F000000000000) /* 6 bits */
#define ZICIO_NVME_CMD_INFO_FILE_MASK		(0x1FC0000000000000) /* 7 bits */

#define ZICIO_NVME_CMD_INFO_LENGTH_SHIFT	(48) /* start of length field */
#define ZICIO_NVME_CMD_INFO_FILE_SHIFT		(54) /* start of file field */ 

#define ZICIO_NVME_CMD_INFO_MAX_LENGTH		(64)
#define ZICIO_NVME_CMD_INFO_MAX_NUMBER		(ZICIO_DATABUFFER_CHUNK_SIZE / \
											 ZICIO_NVME_PAGE_SIZE)

#define ZICIO_NVME_CMD_INFO_ARR_LENGTH		(64) /* CMD_INFOS, CMD_INFO_START_OFFSET */

#define ZICIO_NVME_CMD_INFO_CHUNK_SIZE		((unsigned long)1UL << 21)	/* 2 MiB */
#define ZICIO_NVME_CMD_INFO_PER_CHUNK		(ZICIO_NVME_CMD_INFO_CHUNK_SIZE / \
											 sizeof(zicio_nvme_cmd_info))
#define ZICIO_NVME_CMD_INFOS_CAPACITY		(ZICIO_NVME_CMD_INFO_PER_CHUNK * \
											 ZICIO_NVME_CMD_INFO_ARR_LENGTH)

#define ZICIO_NVME_CMD_INFO_START_OFFSET_CHUNK_SIZE	((unsigned long)1UL << 20) /* 1 MiB */
#define ZICIO_NVME_CMD_INFO_START_OFFSET_PER_CHUNK \
	(ZICIO_NVME_CMD_INFO_START_OFFSET_CHUNK_SIZE / \
	 sizeof(unsigned int))
#define ZICIO_NVME_CMD_INFO_START_OFFSETS_CAPACITY \
	(ZICIO_NVME_CMD_INFO_START_OFFSET_PER_CHUNK * \
	 ZICIO_NVME_CMD_INFO_ARR_LENGTH)

#define ZICIO_NVME_CMD_INFO_GET_FSBLK(nvme_cmd_info) \
	(nvme_cmd_info & ZICIO_NVME_CMD_INFO_FSBLK_MASK)
#define ZICIO_NVME_CMD_INFO_GET_LENGTH(nvme_cmd_info) \
	((nvme_cmd_info & ZICIO_NVME_CMD_INFO_LENGTH_MASK) >> \
	 ZICIO_NVME_CMD_INFO_LENGTH_SHIFT)
#define ZICIO_NVME_CMD_INFO_GET_FILE(nvme_cmd_info) \
	((nvme_cmd_info & ZICIO_NVME_CMD_INFO_FILE_MASK) >> \
	 ZICIO_NVME_CMD_INFO_FILE_SHIFT)

/*
 * The minimum required informatino for resubmitting NVMe command is encoded in
 * 8 bytes (zicio_nvme_cmd_info).
 */
static inline zicio_nvme_cmd_info
zicio_get_nvme_cmd_info(u64 fsblk, u64 len, u64 file_idx)
{
	zicio_nvme_cmd_info ret = 0;

#if (CONFIG_ZICIO_DEBUG_LEVEL >= 2)
	printk(KERN_WARNING "[ZICIO] fsblk: %llu, len: %llu, file: %lld\n",
		   fsblk, len, file_idx);
	BUG_ON((fsblk & ~ZICIO_NVME_CMD_INFO_FSBLK_MASK) != 0);
	BUG_ON(len > 63);
	BUG_ON(file_idx > 127);
#endif /* CONFIG_ZICIO_DEBUG_LEVEL >= 1 */

	ret |= fsblk;
	ret |= (len << ZICIO_NVME_CMD_INFO_LENGTH_SHIFT);
	ret |= (file_idx << ZICIO_NVME_CMD_INFO_FILE_SHIFT);

	return ret;
}

/*
 * Sector is 512 byte and NVMe page is 4KB.
 *
 * Change the number of sectors into the number of nvme pages.
 *
 * Note that if the @nr_sector is a value 7, it menas eight 512B (4KB).
 * In other words, @nr_sector value starts from 0, which is a single sector.
 */
static inline int
zicio_get_nr_nvme_pages(int nr_sector)
{
	BUG_ON((nr_sector & 7) != 7 || nr_sector == 0);
	return (nr_sector >> 3) + 1;
}

/**
 * zicio_data_buffer_descriptor
 *   - manages data buffer belonging to the zicIO channel
 *
 * TODO: this data structure is created to replace zicio_firehose_ctrl.
 * However, currently, not all functionalities of zicio_firehose_ctrl have
 * been replaced for legacy code compatibility; only new design-specific
 * variables have been added. During future refactoring, zicio_firehose_ctrl
 * should be removed entirely.
 */
typedef struct zicio_data_buffer_descriptor {
	atomic_t round[ZICIO_DATABUFFER_CHUNK_NUM];
} zicio_data_buffer_descriptor;

/**
 * zicio_huge_page_consumption_stat - manages flow control of the zicIO channel
 *
 * This data structure manages speed-related information for the channel.
 * These are used to communicate with zicio_flow_ctrl, which manages device
 * resources across all channels.
 *
 * Typically, the interval at which zicIO's softtimer operates is shorter than
 * the interval at which users update the switch board. So updating
 * zicio_flow_ctrl with every softtimer tick may be inefficient.
 *
 * This status is used to determine whether a situation has truly occurred where
 * the user has changed the speed, so needs to communicate with zicio_flow_ctrl.
 */
typedef struct zicio_huge_page_consumption_stat {
	u64 huge_page_consumption_time;
	s64 huge_page_consumption_throughput;
} zicio_huge_page_consumption_stat;

/**
 * zicio_descriptor - manages the operations of the zicIO channel
 *
 * To utilize the legacy code of zicio, a zicio_descriptor is placed at
 * the beginning, allowing it to be typecast to a zicio_descriptor.
 *
 * Newly defined variables have been added to support the new design, which is
 * separate from the existing zicio.
 */
typedef struct zicio_notify_descriptor {
	zicio_descriptor zicio_desc;
	zicio_data_buffer_descriptor buf_desc;
	zicio_huge_page_consumption_stat consumption_stat;
	int nr_fd_of_batches;
	int nr_nvme_cmd_infos;
	int nr_nvme_cmd_info_start_offsets;
	unsigned int **batches_array;
	unsigned int *nr_batches;
	int *vma_flags; // indicates if "batches" is a vma
	unsigned int *nvme_cmd_info_start_offsets[ZICIO_NVME_CMD_INFO_ARR_LENGTH];
	zicio_nvme_cmd_info *nvme_cmd_infos[ZICIO_NVME_CMD_INFO_ARR_LENGTH];
	sector_t bd_start_sector; /* TODO we assumed only one device */
	struct timer_list trigger;
	atomic_t trigger_running;
	spinlock_t lock;
	struct list_head active_nvme_cmd_timers;
} zicio_notify_descriptor;

extern bool zicio_is_notify(void *req_ptr);

extern void zicio_notify_complete_command(void *req_ptr, u32 queue_depth);

extern void zicio_notify_do_softtimer_irq_cycle(void *req_ptr);

extern int zicio_notify_do_softtimer_timer_softirq(int cpu);

extern void __init zicio_notify_kernel_init(void);

extern void zicio_channel_init(struct zicio_notify_descriptor *zicio_desc);

extern void zicio_release_timer_resources(struct zicio_notify_descriptor *zicio_desc);

#if 0
extern int zicio_do_softtimer_idle_loop(int cpu);
#endif /* deprecated */

extern zicio_nvme_cmd_list **
zicio_notify_create_command(zicio_descriptor *desc, int chunk_idx,
					 zicio_nvme_cmd_info nvme_cmd_info, unsigned long start_mem);

#endif /* CONFIG_ZICIO */
#endif /* _LINUX_ZICIO_H */
