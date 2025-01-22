This repository archives the code I wrote, implementation details, and my thoughts about it. These are only partial codes, which cannot independently run the full system. The main entry point is nvme_handle_cqe() in pci.c, which is called by the NVMe interrupt handler nvme_irq(). The pci.c file is located in linux/drivers/nvme/host.

```
nvme_handle_cqe()
|
-- if req->bio->zicio_enabled
|    |
|    -- zicio_notify_nvme_complete_command()
|    |    |
|    |    -- update I/O latency statistics
|    |    |
|    |    -- adjust nvme usage (number of request count)
|    |    |
|    |    -- inform libzicio of the I/O results
|    |    |
|    |    -- push the new I/O information to the I/O scheduler
|    |    |
|    |    -- if there are too many requests
|    |    |    |
|    |    |    -- return
|    |    |
|    |    -- pop the next I/O information from the I/O scheduler
|    |    |
|    |    -- if the next I/O information is fetched
|    |        |
|    |        -- replace nvme command into the next
|    |
|    -- if the nvme command has been replaced
|        |
|        -- setup dma information 
|        |
|        -- nvme_submit_cmd() /* resubmission */
|        |
|        -- return
|
-- nvme_complete_req()
```

## Generating NVMe commands

### Files: zicio_notify.h, zicio_data_buffer_descriptor.c

Originally, zicIO was designed targeting PostgreSQL's sequential scan. Then it was later extended to support various columnar analytical systems, leading to some changes. PostgreSQL's sequential scan allows reading the entire file in any order, and zicIO was initially designed based on this assumption. But columnar analytical systems often read only specific parts of a file and order of reads is important. So new call paths and APIs were introduced, which are prefixed with 
*zicio_notify*. This naming reflects the concept that the user notifies zicIO of the regions and order to read.

The main change in *zicio_notify* was the process of generating NVMe commands. Creating an NVMe command requires a physical block address, which means an *ext4_extent* is needed when using the ext4 file system. If the read order is not important, we can simply select any leaf extent block and perform the corresponding I/O into any buffer location. But when the read order becomes important, it turns into a more complex problem. There may be no consistent reading pattern because it varies by file format and differs for each query. Then how can we search for the specific ext4_extent?

The ext4 file system tries to minimize I/O when it looks for an ext4_extent. Previously accessed extents are cached in memory using a red-black tree. Before performing an I/O, ext4 first searches for the required extent in this cache. If the extent is not found in the cache, ext4 accesses the root node of the extent tree through the inode and uses the OS page cache to traverse the child nodes. If the nodes are also not found in the page cache, it performs I/O.

In the original zicIO, the parent nodes of the leaf were stored in zicIO's own cache to prevent traversing the page cache or the ext4's red-black tree. Leaf ext4_extent blocks were fetched using a logic similar to file prefetching when they were needed. The assumption that the read order doesn't matter allows extents to be retrieved sequentially, sorted by the file's logical block addresses. When sorted, finding specific extent may not incur overhead. But what should be done if the read order is irregular or non-sequential?

zicIO needed to address this question within the interrupt handler while minimizing time complexity. So I started developing a mapping table that would allow the interrupt handler to access I/O information with minimal time complexity. The original zicIO used a 4MB buffer to cache ext4_extent, and my focus was on maintaining this amount of memory, avoiding extra resource overhead over the previous implementation.

The mapping table consists of two arrays. One is an array of data structures containing the minimal information needed to create a single NVMe command, each element corresponds to a single read I/O. The other specifies which range of the first array is required for each 2MB huge page that makes up the buffer. When creating I/O commands for a 2MB huge page, the range of offsets (start offset + end offset, total 8 bytes per 2MB) listed in the second array is used to access the elements of the first array. Since zicIO's buffer is a circular buffer composed of 2MB huge pages, knowing the buffer index and round determines the index to access in the second array, which in turn provides the offset range for the first array.

To calculate the size of read I/O that the mapping table can cover, we should know the size of the compressed information. The minimum I/O unit of an SSD is called a *sector*, which is 512 bytes. ext4 manages an NVMe page in units of 4KB, composed of 8 sectors. The device we used had a maximum I/O size of 256KB. Thus, the size of an I/O command can range from a minimum of 4KB to a maximum of 256KB, with intervals of 4KB. This range can be represented by values from 0 to 63, requiring 6 bits. The ext4_extent stores the physical block address across @ee_start_hi and @ee_start_lo, which together total 48 bits. zicIO can read up to 128 files on a single channel, requiring 7 bits. So a total of 61 bits is required for a single read I/O. This information is declared as 8 bytes to ensure alignment, with the remaining 3 bits reserved.

In summary, 8 bytes of compressed information can be used to create a single read I/O command ranging from 4KB to 256KB. Naively, in the worst case 4MB can cover 2GB of read I/O, and in the best case it can cover 128GB of read I/O. The worst case occurs when the data is fully fragmented into 4KB segments. In the experiments conducted for the paper, datasets ranging from 80GB to 100GB were covered using 4MB to 8MB of memory. 

This design still has constraints. The process of populating the mapping table occurs during channel opening. So if the file system changes logical-to-physical mappings after channel opening, affected information will need to be updated. Memory usage also depends on how well the file system handles fragmentation, making the design's overheads and constraints closely tied to file system behavior. While it might be possible to periodically refill the mapping table with information for new I/O operations, this has not yet been implemented.

## Managing NVMe usage

### Files: zicio_flow_ctrl.h, zicio_flow_ctrl.c

These files are related to the logic that controls how many I/O requests are issued in parallel. I think this has a stronger effect on preparing data in a timely manner, compared to setting the release timing of I/O (especially in highly optimized, high-performance DBMS, where I/O requests need to be issued continuously to keep up with data consumption).

In the NVMe interrupt handler, zicIO updates the exponential moving average (EMA) for the I/O latency (note: EMA is used because it requires only the most recent latency value and the current EMA, without storing previous latencies). The updated EMA for the device's I/O latency, combined with the user's current consumption rate, is then used to adjust the number of I/O requests. Here, *requests* refer to the data structure utilized by the linux *blk_mq* layer for handling I/O operations. More specifically, during server boot-up, the kernel communicates with the NVMe device through its admin queue to retrieve the number and size of hardware queues (hw queues). Then the kernel creates a set of *request* data structures corresponding to the queue's capacity. If a process wants to enqueue an I/O command into the hw queue, the corresponding *request* must be acquired through the blk_mq layer.

When the interrupt handler determines that additional *requests* are needed for zicIO, it registers a callback function to wake up the softirq daemon process using timer softirq functions. Then the softirq daemon process wakes up, its main function starts getting new *requests* for zicIO and triggers new I/O, creating additional resubmission cycle. Note that new *requests* are not obtained within the softirq callback function. Simply registering a callback function to acquire *requests* with the softirq can lead to issues because a softirq callback is not exclusively executed by the softirq daemon process. Instead, it may be invoked in various unexpected contexts, such as immediately after interrupt handling. If the new *requests* are not obtained in the process context, unexpected kernel panics may occur.

If the I/O throughput exceeds the user's consumption throughput, *requests* need to be returned to the blk_mq layer. In such cases, no additional processing is required in the zicIO logic, and the flow proceeds to the default NVMe interrupt handler logic (bypassing zicIO's resubmission logic naturally returns the request to the blk_mq layer).

As an additional note about the implementation, using softirq in this manner is inappropriate. By the time the implementation had progressed significantly, I became aware of various mechanisms in linux for defering tasks to run in the process context, such as work_queue. Since zicIO was already implemented using softirq, it wasn’t changed.

There’s some code here that’s implemented but not actually used right now. I think the responsibility for deciding block device usage is still left to users in typical I/O techniques. Users, however, are unaware of how much others are using the device due to kernel virtualization. If the kernel could regulate device usage per user, it could enable fairer resource allocation. This idea was implemented as 'PBR' in the code but is no longer used as it is not closely related to the paper's rationale.

## Scheduling NVMe commands

### Files: zicio_nvme_cmd_timer_wheel.h, zicio_nvme_cmd_timer_wheel.c

One might think that release timing could be unnecessary if it doesn't impact preparing data in a timely manner, particularly when continuous I/O requests are required. However, I think I/O timing still has an importance for designing an I/O scheduling mechanism.

How should I/O requests from multiple users be scheduled? Is an I/O request less urgent when it's farther from the user's current reading point? But what if the user is very fast? Considering both the user's location and speed essentially means estimating when they will reach the data. In zicIO, this estimated time is used as the priority for scheduling I/O commands (longer times are assigned lower priority, while shorter times are assigned higher priority).

The problem is that the interrupt handler is responsible for scheduling I/O commands in zicIO, requiring minimal time complexity. So simply using a multi-level queue was not suitable because lower-priority I/O commands must eventually be elevated to higher priorities over time by the interrupt handler to prevent starvation.

My observation was that timing determines the scheduling priority, and over time, the priority of all I/O commands changes equally. Then we don't need to move lower-priority I/O commands to a higher-priority queue. Instead, we can simply adjust the priority of the queue itself over time. The data structure that fit this purpose was the timer wheel. Therefore, I modified the existing Linux timer wheel and used it as the scheduling structure for I/O commands.
