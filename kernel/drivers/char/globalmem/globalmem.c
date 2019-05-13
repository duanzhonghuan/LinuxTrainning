/**
 ** This file is part of the LinuxTrainningHome project.
 ** Copyright(C) duanzhonghuan Co., Ltd.
 ** All Rights Reserved.
 ** Unauthorized copying of this file, via any medium is strictly prohibited
 ** Proprietary and confidential
 **
 ** Written by ZhongHuan Duan <15818411038@163.com>, 2019-05-12
 **/


#include "linux/init.h"
#include "linux/module.h"
#include "linux/kdev_t.h"
#include "linux/cdev.h"
#include "linux/fs.h"
#include "linux/slab.h"
#include "linux/uaccess.h"

#define  GLOBALMEM_SIZE  (0X1000)
#define  GLOBALMEM_MAJOR  (230)
#define  DEVICE_NUM  (1)
#define  MEM_CLEAR  (0x01)
static int globalmem_major = GLOBALMEM_MAJOR;
#define globalmem_debug

/**
 * @brief The globalmem_dev struct - the description of the global memory
 */
struct globalmem_dev
{
    struct cdev chrdev;
    unsigned char mem[GLOBALMEM_SIZE];
};
static struct globalmem_dev *globalmem_devp = NULL;

/**
 * @brief globalmem_llseek - reposition read/write file offset
 * @param filep: the file pointer
 * @param offset: the offset
 * @param whence: SEEK_SET,SEEK_CUR,SEEK_END
 * @return: the actual offset, return -1 when failed
 */
static loff_t globalmem_llseek (struct file *filep, loff_t offset, int whence)
{
    loff_t ret = 0;
    switch (whence)
    {
        case SEEK_SET:
            if (offset < 0 || offset > GLOBALMEM_SIZE)
            {
                ret = -EINVAL;
                break;
            }
            filep->f_pos = offset;
            ret = offset;
        break;
        case SEEK_CUR:
            if (offset < 0 || filep->f_pos + offset > GLOBALMEM_SIZE)
            {
                ret = -EINVAL;
                break;
            }
            filep->f_pos += offset;
            ret = filep->f_pos;
        break;
        case SEEK_END:
            filep->f_pos = GLOBALMEM_SIZE;
            ret = filep->f_pos;
        break;
    default:
            ret = -EINVAL;
        break;
    }
    return ret;
}

/**
 * @brief globalmem_read
 * @param filep
 * @param buf
 * @param count
 * @param ppos
 * @return
 */
static ssize_t globalmem_read (struct file *filep, char __user *buf, size_t count, loff_t *ppos)
{
    ssize_t ret = 0;
    loff_t  curpos = *ppos;
    struct globalmem_dev *dev;
#ifdef globalmem_debug
    printk("globalmem_read\n");
#endif
    if (curpos >= GLOBALMEM_SIZE)
    {
        return 0;
    }
    dev = filep->private_data;
    if (!dev)
    {
        ret = -ENOMEM;
        return ret;
    }
    if (curpos + count > GLOBALMEM_SIZE)
    {
        count = GLOBALMEM_SIZE - curpos;
    }
    ret = copy_to_user(buf, dev->mem + curpos, count);
    *ppos += count;
    ret = count;
    return ret;
}

static ssize_t globalmem_write (struct file *filep, const char __user *buf, size_t count, loff_t *ppos)
{
    ssize_t ret = 0;
    loff_t  curpos = *ppos;
    struct globalmem_dev *dev;
#ifdef globalmem_debug
    printk("globalmem_write\n");
#endif
    if (curpos >= GLOBALMEM_SIZE)
    {
        ret = -ENAVAIL;
        return ret;
    }
    dev = filep->private_data;
    if (!dev)
    {
        ret = -ENOMEM;
        return ret;
    }
    if (curpos + count > GLOBALMEM_SIZE)
    {
        count = GLOBALMEM_SIZE - curpos;
    }
    ret = copy_from_user(dev->mem + curpos, buf, count);
    *ppos += count;
    ret = count;
    return ret;
}

/**
 * @brief globalmem_unlocked_ioctl
 * @param filep
 * @param cmd
 * @param arg
 * @return
 */
static long globalmem_unlocked_ioctl (struct file *filep, unsigned int cmd, unsigned long arg)
{
    struct globalmem_dev *dev = filep->private_data;
    switch (cmd)
    {
        case MEM_CLEAR:
        memset(dev->mem, 0, GLOBALMEM_SIZE);
        printk("mem clear success\n");
        break;
    default:
        break;
    }
    return 0;
}

static int globalmem_open (struct inode *inode, struct file *filep)
{
    struct globalmem_dev *dev = container_of(inode->i_cdev, struct globalmem_dev, chrdev);
    filep->private_data = dev;
#ifdef globalmem_debug
    printk("globalmem_open\n");
#endif
    return 0;
}

static int globalmem_release (struct inode *inode, struct file *filep)
{
    return 0;
}

static  struct file_operations chrdev_file_operations =
{
    .owner = THIS_MODULE,
    .llseek = globalmem_llseek,
    .read = globalmem_read,
    .write = globalmem_write,
    .unlocked_ioctl = globalmem_unlocked_ioctl,
    .open = globalmem_open,
    .release = globalmem_release,
};

/**
 * @brief globalmem_init_dev - initialize the global memory decice
 * @param chrdev: out parameter for a char device
 * @param index: the minor device id
 */
static void globalmem_init_dev(struct globalmem_dev *chrdev, int index)
{
    int ret = 0;
    dev_t devno = MKDEV(globalmem_major, index);
    cdev_init(&chrdev->chrdev, &chrdev_file_operations);
    // add the globalmem device structure to the kobj_map
    ret = cdev_add(&chrdev->chrdev, devno, 1);
    if (ret)
    {
        printk("Error code = %d when adding globalmem %d\n", ret, index);
    }
}

/**
 * @brief globalmem_init - the function of initializing the global memory
 * @return: the status of initializing the global memory
 */
static int __init globalmem_init(void)
{
    int i = 0;
    int ret = 0;
    // 1. get the device id
    dev_t devno = MKDEV(globalmem_major, 0);
#ifdef globalmem_debug
    printk(KERN_NOTICE "globalmem_init\n");
#endif
    // 2. register the DEVICE_NUM of the char device
    if (globalmem_major)
    {
        ret = register_chrdev_region(devno, DEVICE_NUM, "globalmem");
    }
    else
    {
        // automatic register char device
        ret = alloc_chrdev_region(&devno, 0, DEVICE_NUM, "globalmem");
    }
    // check the error code
    if (ret < 0)
    {
        return ret;
    }

    // 3. construct the globalmem devices structure in the heap
    globalmem_devp = kzalloc(sizeof(struct globalmem_dev) * DEVICE_NUM, GFP_KERNEL);
    if (!globalmem_devp)
    {
        ret = -ENOMEM;
        goto fail_malloc;
    }

    // 4. add the globalmem decices structure pointer to the kobjct map
    for (i = 0; i < DEVICE_NUM; i++)
    {
        globalmem_init_dev(globalmem_devp + i, i);
    }
    printk("globalmem_init success\n");
    return 0;

 fail_malloc:
    unregister_chrdev_region(devno, DEVICE_NUM);
    return ret;
}

/**
 * @brief globalmem_exit - exit the glboalmem device
 */
static void __exit globalmem_exit(void)
{
    int i = 0;
#ifdef globalmem_debug
    printk(KERN_NOTICE "globalmem_exit\n");
#endif
    // 1. remove the globalmem structure from teh kobject map
    for (i = 0; i < DEVICE_NUM; i++)
    {
        cdev_del(&(globalmem_devp + i)->chrdev);
    }

    // 2. free the glboalmem structure in the heap
    kfree(globalmem_devp);

    // 3. unregister the device id
    unregister_chrdev_region(MKDEV(globalmem_major, 0), DEVICE_NUM);

    // 4. remove the device id
    printk("globalmem_exit success\n");
}

module_init(globalmem_init)
module_exit(globalmem_exit)
// the declaration	of the author
MODULE_AUTHOR("ZhongHuan Duan <15818411038@163.com>");
// the declaration of the licence
MODULE_LICENSE("GPL v2");
