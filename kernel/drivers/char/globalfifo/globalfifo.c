/**
 ** This file is part of the LinuxTrainningHome project.
 ** Copyright(C) duanzhonghuan Co., Ltd.
 ** All Rights Reserved.
 ** Unauthorized copying of this file, via any medium is strictly prohibited
 ** Proprietary and confidential
 **
 ** Written by ZhongHuan Duan <15818411038@163.com>, 2019-05-19
 **/


#include "linux/init.h"
#include "linux/module.h"
#include "linux/kdev_t.h"
#include "linux/cdev.h"
#include "linux/fs.h"
#include "linux/slab.h"
#include "linux/uaccess.h"
#include "linux/mutex.h"
#include "linux/wait.h"
#include "linux/sched.h"

#define  GLOBALFIFO_SIZE  (0X1000)
#define  GLOBALFIFO_MAJOR  (230)
#define  DEVICE_NUM  (1)
#define  FIFO_CLEAR  (0x01)
static int globalfifo_major = GLOBALFIFO_MAJOR;
#define globalfifo_debug

/**
 * @brief The globalfifo_dev struct - the description of the global memory
 */
struct globalfifo_dev
{
    struct cdev chrdev;
    unsigned int current_len;  // available read length
    unsigned char mem[GLOBALFIFO_SIZE];
    struct mutex mutex;
    wait_queue_head_t r_wait;
    wait_queue_head_t w_wait;
};
static struct globalfifo_dev *globalfifo_devp = NULL;

/**
 * @brief globalfifo_llseek - reposition read/write file offset
 * @param filep: the file pointer
 * @param offset: the offset
 * @param whence: SEEK_SET,SEEK_CUR,SEEK_END
 * @return: the actual offset, return -1 when failed
 */
static loff_t globalfifo_llseek (struct file *filep, loff_t offset, int whence)
{
    loff_t ret = 0;
    switch (whence)
    {
        case SEEK_SET:
            if (offset < 0 || offset > GLOBALFIFO_SIZE)
            {
                ret = -EINVAL;
                break;
            }
            filep->f_pos = offset;
            ret = offset;
        break;
        case SEEK_CUR:
            if (offset < 0 || filep->f_pos + offset > GLOBALFIFO_SIZE)
            {
                ret = -EINVAL;
                break;
            }
            filep->f_pos += offset;
            ret = filep->f_pos;
        break;
        case SEEK_END:
            filep->f_pos = GLOBALFIFO_SIZE;
            ret = filep->f_pos;
        break;
    default:
            ret = -EINVAL;
        break;
    }
    return ret;
}

/**
 * @brief globalfifo_read
 * @param filep
 * @param buf
 * @param count
 * @param ppos
 * @return
 */
static ssize_t globalfifo_read (struct file *filep, char __user *buf, size_t count, loff_t *ppos)
{
    ssize_t ret = 0;
    struct globalfifo_dev *dev;
#ifdef globalfifo_debug
    printk("globalfifo_read\n");
#endif
    dev = filep->private_data;
    if (!dev)
    {
        ret = -ENOMEM;
        return ret;
    }

    // declare a wait queue @read_global_fifo
    // set @current task to the @read_global_fifo
    DECLARE_WAITQUEUE(read_global_fifo, current);

    // add mutex
    // 首先我们考虑一下，为什么要在这个位置加锁呢？
	// 因为dev->mem这个对象是公共资源，当执行下面的
	// copy_to_user时，在copy_to_user中有可能发生调度事件，造成自己堵塞
	// 使得另一个任务开始执行，恰巧另一个任务也要对这个dev->mem进行拷贝操作
	// 那么，如果不加锁限制，就很可能造成dev->mem拷贝混乱。这就解释了为什么
	// 在这里加锁的原因，同时我们也可以总结到，只要涉及到公共资源被访问时，然后
	// 这些资源又是唯一排他性质的，这是我们就要加锁。
    mutex_lock_interruptible(&dev->mutex);
	// 我们设想一下为什么add_wait_queue()要放在加锁之后呢？
	// 主要还是因为dev->r_wait这个对象是公共资源，在add_wait_queue添加的时候
	// 虽然有其内部自旋锁的控制，让添加的时候，不会造成添加冲突，
	// 但是read_global_fifo这个变量在多任务下存在多个对象，如果不在互斥锁内
	// 添加这个变量的话，可能导致dev->r_wait添加多个read_global_fifo
    add_wait_queue(&dev->r_wait, &read_global_fifo);

    while (dev->current_len == 0)
    {
		// 如果是非堵塞函数的话，直接退出，等到任务下次在调用读接口
        if (filep->f_flags & O_NONBLOCK)
        {
            ret = -EAGAIN;
            goto out;
        }
		// 设置当前任务状态位为停止状态，就是挂起的意思
		// 但是可以通过信号唤醒。
		// 为什么不直接用包装好的set_current_state?
        __set_current_state(TASK_INTERRUPTIBLE);
		// 在这里分析一下为啥解锁了？
		// 因为后面主动调用调度函数，这样肯定不能加锁的，
		// 否则很容易造成死锁情况发生。一般加锁的区间只涉及到
		// 对共享资源区间进行加锁访问，私有的资源不再加锁，还有
		// 加锁区间不能有导致调度的情况发生，不然容易造成死锁
        mutex_unlock(&dev->mutex);

        // schedule it right now
        schedule();
        if (signal_pending(current))
        {
            ret = -ERESTARTSYS;
            goto out2;
        }

        mutex_lock(&dev->mutex);
    }

    if (count > dev->current_len)
    {
        count = dev->current_len;
    }

    if (copy_to_user(buf, dev->mem, count))
    {
        ret = -EFAULT;
        goto out;
    }
    else
    {
        memcpy(dev->mem, dev->mem + count, dev->current_len - count);
        dev->current_len -= count;
        wake_up_interruptible(&dev->w_wait);
        ret = count;
    }

out:
    mutex_unlock(&dev->mutex);
out2:
    remove_wait_queue(&dev->r_wait, &read_global_fifo);
    set_current_state(TASK_RUNNING);
    return ret;
}

static ssize_t globalfifo_write (struct file *filep, const char __user *buf, size_t count, loff_t *ppos)
{
    ssize_t ret = 0;
    struct globalfifo_dev *dev;
#ifdef globalfifo_debug
    printk("globalfifo_read\n");
#endif
    dev = filep->private_data;
    if (!dev)
    {
        ret = -ENOMEM;
        return ret;
    }

    // declare a wait queue @write_global_fifo
    // set @current task to the @write_global_fifo
    DECLARE_WAITQUEUE(write_global_fifo, current);

    // add mutex
    // 首先我们考虑一下，为什么要在这个位置加锁呢？
    // 因为dev->mem这个对象是公共资源，当执行下面的
    // copy_to_user时，在copy_to_user中有可能发生调度事件，造成自己堵塞
    // 使得另一个任务开始执行，恰巧另一个任务也要对这个dev->mem进行拷贝操作
    // 那么，如果不加锁限制，就很可能造成dev->mem拷贝混乱。这就解释了为什么
    // 在这里加锁的原因，同时我们也可以总结到，只要涉及到公共资源被访问时，然后
    // 这些资源又是唯一排他性质的，这是我们就要加锁。
    mutex_lock_interruptible(&dev->mutex);

    // 我们设想一下为什么add_wait_queue()要放在加锁之后呢？
    // 主要还是因为dev->r_wait这个对象是公共资源，在add_wait_queue添加的时候
    // 虽然有其内部自旋锁的控制，让添加的时候，不会造成添加冲突，
    // 但是read_global_fifo这个变量在多任务下存在多个对象，如果不在互斥锁内
    // 添加这个变量的话，可能导致dev->w_wait添加多个read_global_fifo
    add_wait_queue(&dev->w_wait, &write_global_fifo);

    while (dev->current_len == 0)
    {
        // 如果是非堵塞函数的话，直接退出，等到任务下次在调用读接口
        if (filep->f_flags & O_NONBLOCK)
        {
            ret = -EAGAIN;
            goto out;
        }
        // 设置当前任务状态位为停止状态，就是挂起的意思
        // 但是可以通过信号唤醒。
        // 为什么不直接用包装好的set_current_state?
        __set_current_state(TASK_INTERRUPTIBLE);
        // 在这里分析一下为啥解锁了？
        // 因为后面主动调用调度函数，这样肯定不能加锁的，
        // 否则很容易造成死锁情况发生。一般加锁的区间只涉及到
        // 对共享资源区间进行加锁访问，私有的资源不再加锁，还有
        // 加锁区间不能有导致调度的情况发生，不然容易造成死锁
        mutex_unlock(&dev->mutex);

        // schedule it right now
        schedule();
        if (signal_pending(current))
        {
            ret = -ERESTARTSYS;
            goto out2;
        }

        mutex_lock(&dev->mutex);
    }

    if (count > dev->current_len)
    {
        count = dev->current_len;
    }

    if (copy_to_user(buf, dev->mem, count))
    {
        ret = -EFAULT;
        goto out;
    }
    else
    {
        memcpy(dev->mem, dev->mem + count, dev->current_len - count);
        dev->current_len -= count;
        wake_up_interruptible(&dev->r_wait);
        ret = count;
    }

out:
    mutex_unlock(&dev->mutex);
out2:
    remove_wait_queue(&dev->w_wait, &write_global_fifo);
    set_current_state(TASK_RUNNING);
    return ret;
}

/**
 * @brief globalfifo_unlocked_ioctl
 * @param filep
 * @param cmd
 * @param arg
 * @return
 */
static long globalfifo_unlocked_ioctl (struct file *filep, unsigned int cmd, unsigned long arg)
{
    struct globalfifo_dev *dev = filep->private_data;
    switch (cmd)
    {
        case FIFO_CLEAR:
        memset(dev->mem, 0, GLOBALFIFO_SIZE);
        printk("mem clear success\n");
        break;
    default:
        break;
    }
    return 0;
}

static int globalfifo_open (struct inode *inode, struct file *filep)
{
    struct globalfifo_dev *dev = container_of(inode->i_cdev, struct globalfifo_dev, chrdev);
    filep->private_data = dev;
#ifdef globalfifo_debug
    printk("globalfifo_open\n");
#endif
    return 0;
}

static int globalfifo_release (struct inode *inode, struct file *filep)
{
    return 0;
}

static  struct file_operations chrdev_file_operations =
{
    .owner = THIS_MODULE,
    .llseek = globalfifo_llseek,
    .read = globalfifo_read,
    .write = globalfifo_write,
    .unlocked_ioctl = globalfifo_unlocked_ioctl,
    .open = globalfifo_open,
    .release = globalfifo_release,
};

/**
 * @brief globalfifo_init_dev - initialize the global memory decice
 * @param chrdev: out parameter for a char device
 * @param index: the minor device id
 */
static void globalfifo_init_dev(struct globalfifo_dev *chrdev, int index)
{
    int ret = 0;
    dev_t devno = MKDEV(globalfifo_major, index);
    cdev_init(&chrdev->chrdev, &chrdev_file_operations);
    // add the globalfifo device structure to the kobj_map
    ret = cdev_add(&chrdev->chrdev, devno, 1);
    if (ret)
    {
        printk("Error code = %d when adding globalfifo %d\n", ret, index);
    }
}

/**
 * @brief globalfifo_init - the function of initializing the global memory
 * @return: the status of initializing the global memory
 */
static int __init globalfifo_init(void)
{
    int i = 0;
    int ret = 0;
    // 1. get the device id
    dev_t devno = MKDEV(globalfifo_major, 0);
#ifdef globalfifo_debug
    printk(KERN_NOTICE "globalfifo_init\n");
#endif
    // 2. register the DEVICE_NUM of the char device
    if (globalfifo_major)
    {
        ret = register_chrdev_region(devno, DEVICE_NUM, "globalfifo");
    }
    else
    {
        // automatic register char device
        ret = alloc_chrdev_region(&devno, 0, DEVICE_NUM, "globalfifo");
    }
    // check the error code
    if (ret < 0)
    {
        return ret;
    }

    // 3. construct the globalfifo devices structure in the heap
    globalfifo_devp = kzalloc(sizeof(struct globalfifo_dev) * DEVICE_NUM, GFP_KERNEL);
    if (!globalfifo_devp)
    {
        ret = -ENOMEM;
        goto fail_malloc;
    }

    // initialize the mutex
    mutex_init(&globalfifo_devp->mutex);

    // initialize the write and read wait queue head
    init_waitqueue_head(&globalfifo_devp->w_wait);
    init_waitqueue_head(&globalfifo_devp->w_wait);

    // 4. add the globalfifo decices structure pointer to the kobjct map
    for (i = 0; i < DEVICE_NUM; i++)
    {
        globalfifo_init_dev(globalfifo_devp + i, i);
    }
    printk("globalfifo_init success\n");
    return 0;

 fail_malloc:
    unregister_chrdev_region(devno, DEVICE_NUM);
    return ret;
}

/**
 * @brief globalfifo_exit - exit the glboalmem device
 */
static void __exit globalfifo_exit(void)
{
    int i = 0;
#ifdef globalfifo_debug
    printk(KERN_NOTICE "globalfifo_exit\n");
#endif
    // 1. remove the globalfifo structure from teh kobject map
    for (i = 0; i < DEVICE_NUM; i++)
    {
        cdev_del(&(globalfifo_devp + i)->chrdev);
    }

    // 2. free the glboalmem structure in the heap
    kfree(globalfifo_devp);

    // 3. unregister the device id
    unregister_chrdev_region(MKDEV(globalfifo_major, 0), DEVICE_NUM);

    // 4. remove the device id
    printk("globalfifo_exit success\n");
}

module_init(globalfifo_init)
module_exit(globalfifo_exit)
// the declaration	of the author
MODULE_AUTHOR("ZhongHuan Duan <15818411038@163.com>");
// the declaration of the licence
MODULE_LICENSE("GPL v2");
