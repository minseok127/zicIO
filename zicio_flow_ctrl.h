#ifndef	_LINUX_ZICIO_FLOW_CTRL_H
#define	_LINUX_ZICIO_FLOW_CTRL_H

#include <linux/blkdev.h>
#include <linux/ktime.h>

#include <linux/zicio_notify.h>

extern ktime_t zicio_calc_expiration_time(struct zicio_notify_descriptor *zicio_notify_desc,
	int huge_page_id);

extern ktime_t zicio_get_data_buffer_io_timing(
	struct zicio_notify_descriptor *zicio_notify_desc);

extern void zicio_notify_update_flow_ctrl(struct request *req, u32 queue_depth);

extern int zicio_register_channel_into_flow_ctrl(
	struct zicio_notify_descriptor *zicio_notify_desc);

extern void zicio_remove_channel_from_flow_ctrl(
	struct zicio_notify_descriptor *zicio_notify_desc);

extern void zicio_init_trigger_flow_control(
	struct zicio_notify_descriptor *zicio_notify_desc);

extern void zicio_init_trigger_rollback_flow_control(
	struct zicio_notify_descriptor *zicio_notify_desc);

extern void __init zicio_init_flow_controller(void);

extern void zicio_dump_flow_ctrl(int cpu);

#endif /* _LINUX_ZICIO_FLOW_CTRL_H */
