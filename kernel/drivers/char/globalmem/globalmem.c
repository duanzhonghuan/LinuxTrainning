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
static int globalmem_major = GLOBALMEM_MAJOR;

/**
 * @brief The globalmem_dev struct - the description of the global memory
 */
struct globalmem_dev
{
    struct cdev chrdev;
    unsigned char mem[GLOBALMEM_SIZE];
};
static struct globalmem_dev *globalmem_devp = NULL;


static loff_t globalmem_llseek (struct file *, loff_t, int)
{

}

ssize_t globalmem_read (struct file *, char __user *, size_t, loff_t *)
{

}
ssize_t globalmem_write (struct file *, const char __user *, size_t, loff_t *)
{

}

long globalmem_unlocked_ioctl (struct file *, unsigned int, unsigned long)
{

}

int globalmem_mmap (struct file *, struct vm_area_struct *)
{

}
int globalmem_open (struct inode *, struct file *)
{

}

int globalmem_release (struct inode *, struct file *)
{

}

static  struct file_operations chrdev_file_operations =
{
    .owner = THIS_MODULE,
    .llseek = globalmem_llseek,
    .read = globalmem_read,
    .write = globalmem_write,
    .unlocked_ioctl = globalmem_unlocked_ioctl,
    .mmap = globalmem_mmap,
    .open = globalmem_open,
    .release = globalmem_release,
};

/**
 * @brief globalmem_init_dev - initialize the global memory decice
 * @param chrdev: out parameter for a char device
 * @param index: the minor device id
 */
static globalmem_init_dev(struct globalmem_dev *chrdev, int index)
{
    int error_code = 0;
    dev_t devno = MKDEV(globalmem_major, index);
    cdev_init(&chrdev->chrdev, &chrdev_file_operations);
    // add the globalmem device structure to the kobj_map
    error_code = cdev_add(&chrdev->chrdev, devno, 1);
    if (error_code)
    {
        printk("Error code = %d when adding globalmem %d\n", error_code, index);
    }
}

/**
 * @brief globalmem_init - the function of initializing the global memory
 * @return: the status of initializing the global memory
 */
static int __init globalmem_init(void)
{
    int i = 0;
    int error_code = 0;
    // 1. get the device id
    dev_t devno = MKDEV(globalmem_major, 0);
    // 2. register the DEVICE_NUM of the char device
    if (globalmem_major)
    {
        error_code = register_chrdev_region(devno, DEVICE_NUM, "globalmem");
    }
    else
    {
        // automatic register char device
        error_code = alloc_chrdev_region(&devno, 0, DEVICE_NUM, "globalmem");
    }
    // check the error code
    if (!error_code)
    {
        return error_code;
    }

    // 3. construct the globalmem devices structure in the heap
    globalmem_devp = kzalloc(sizeof(struct globalmem_dev) * DEVICE_NUM, GFP_KERNEL);
    if (!globalmem_devp)
    {
        error_code = -ENOMEM;
        goto fail_malloc;
    }

    // 4. add the globalmem decices structure pointer to the kobjct map
    for (i = 0; i < DEVICE_NUM; i++)
    {
        globalmem_init_dev(globalmem_devp + i, i);
    }
    return 0;

 fail_malloc:
    unregister_chrdev_region(devno, DEVICE_NUM);
    return error_code;
}

/**
 * @brief globalmem_exit - exit the glboalmem device
 */
static void __exit globalmem_exit(void)
{
    int i = 0;
    // 1. remove the globalmem structure from teh kobject map
    for (i = 0; i < DEVICE_NUM; i++)
    {
        cdev_del((globalmem_devp + i)->chrdev);
    }

    // 2. free the glboalmem structure in the heap
    kfree(globalmem_devp);

    // 3. unregister the device id
    unregister_chrdev_region(MKDEV(globalmem_major, 0), DEVICE_NUM);

    // 4. remove the device id
}

module_init(globalmem_init)
module_exit(globalmem_exit)
// the declaration	of the author
MODULE_AUTHOR("ZhongHuan Duan <15818411038@163.com>");
// the declaration of the licence
MODULE_LICENSE("GPL v2");
