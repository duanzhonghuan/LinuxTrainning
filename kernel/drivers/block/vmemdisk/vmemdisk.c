/**
 ** This file is part of the LinuxTrainningHome project.
 ** Copyright(C) duanzhonghuan Co., Ltd.
 ** All Rights Reserved.
 ** Unauthorized copying of this file, via any medium is strictly prohibited
 ** Proprietary and confidential
 **
 ** Written by ZhongHuan Duan <15818411038@163.com>, 2019-06-15
 **/



#include "linux/init.h"
#include "linux/module.h"
#include "linux/kdev_t.h"
#include "linux/cdev.h"
#include "linux/fs.h"
#include "linux/slab.h"
#include "linux/uaccess.h"
#include "linux/mutex.h"
#include <linux/blkdev.h>
#include <linux/hdreg.h>

#define  DEVICE_NUM  (4)
static int vmemdisk_major = 0;
#define  NSECTORS  (1024)
#define  HARDSECTOR_SIZE  (512)
#define  KRNSECTOR_SIZE  (512)
#define  VMEMDISK_MINIORS  (6)
#define vmemdisk_debug

/**
 * @brief The vmemdisk_dev struct - the description of the virtual memory disk device
 */
struct vmemdisk_dev
{
    // device size in sectors
    int size;
    // the data array
    u8 *data;
    // for mutual exclusion
    spinlock_t lock;
    // the device request queue
    struct request_queue *queue;
    // the gendisk structure
    struct gendisk *gd;
};
static struct vmemdisk_dev *devices = NULL;

static int vmemdisk_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
    struct vmemdisk_dev *dev = bdev->bd_disk->private_data;
    long size = dev->size * (HARDSECTOR_SIZE / KRNSECTOR_SIZE);

    geo->cylinders = (size & ~0x3f) >> 6;
    geo->heads = 4;
    geo->sectors = 16;
    geo->start = 4;

    return 0;
}

static struct block_device_operations vmemdisk_ops = {
    .getgeo = vmemdisk_getgeo,
};

static void vmemdisk_transfer(struct vmemdisk_dev *dev, unsigned long sector,
                              unsigned long nsector, char *buff, int write)
{
    unsigned long offset = sector * KRNSECTOR_SIZE;
    unsigned long nbytes = nsector * KRNSECTOR_SIZE;

    if (dev->size < offset + nbytes)
    {
        return;
    }
    if (write)
    {
            memcpy(dev->data, buff, nbytes);
    }
    else
    {
            memcpy(buff, dev->data, nbytes);
    }
}

static void vmemdisk_xfer_bio(struct vmemdisk_dev *dev, struct bio *bio)
{
    struct bio_vec bvec;
    struct bvec_iter iter;
    sector_t sector = bio->bi_iter.bi_sector;

    bio_for_each_segment(bvec, bio, iter){
                char *buffer = __bio_kmap_atomic(bio, iter, 0);
                vmemdisk_transfer(dev, sector, bio_cur_bytes(bio) >> 9,
                                  buffer, bio_data_dir(bio) == WRITE);
                sector += bio_cur_bytes(bio) >> 9;
                __bio_kunmap_atomic(buffer, 0);
    }
}

void vmemdisk_make_request(struct request_queue *q, struct bio *bio)
{
    struct vmemdisk_dev *dev = q->queuedata;
    if (dev)
    {
       vmemdisk_xfer_bio(dev, bio);
    }
}

/**
 * @brief setup_device - setup the virtual memory device
 * @param device: the virtual memory device
 * @param which: which way to setup
 */
static void setup_device(struct vmemdisk_dev* device, int which)
{
    // fill the data into the @device structure
    memset(device, 0, sizeof(struct vmemdisk_dev));
    device->size = NSECTORS * HARDSECTOR_SIZE;
    device->data = kmalloc(device->size, GFP_KERNEL);
    spin_lock_init(&device->lock);

    device->queue = blk_alloc_queue(GFP_KERNEL);
    device->gd = alloc_disk(VMEMDISK_MINIORS);

    // fill the device structure into the queue data
    device->queue->queuedata = device;
    blk_queue_make_request(device->queue, vmemdisk_make_request);

    // fill the data into the gendisk structre
    device->gd->major = vmemdisk_major;
    device->gd->first_minor = 0;
    device->gd->fops = &vmemdisk_ops;
    device->gd->private_data = device;
    device->gd->queue = device->queue;
    set_capacity(device->gd, NSECTORS * (HARDSECTOR_SIZE / KRNSECTOR_SIZE));
    add_disk(device->gd);
    printk("success to setup : %s\n", __FILE__);
}


/**
 * @brief vmemdisk_init - the function of initializing the virtual memory disk
 * @return: the status of initializing the virtual memory disk
 */
static int __init vmemdisk_init(void)
{
    int ret = 0;
    int i;
    // register a new block device
    vmemdisk_major = register_blkdev(vmemdisk_major, "vmemdisk");
    if (vmemdisk_major <= 0)
    {
        printk("Error code = %d Fail to register the @vmemdisk new block device.\n", ret);
        return -EBUSY;
    }

    devices = kmalloc(DEVICE_NUM * sizeof(struct vmemdisk_dev), GFP_KERNEL);
    if (devices == NULL)
    {
        printk("Fail to malloc.\n");
        ret = -EINVAL;
        goto unregister;
    }

    for (i = 0; i < DEVICE_NUM; i++)
    {
        setup_device(devices + i * sizeof(struct vmemdisk_dev), 0);
    }

unregister:
    unregister_blkdev(vmemdisk_major, "vmemdisk");
    return ret;
}

/**
 * @brief vmemdisk_exit - exit the glboalmem device
 */
static void __exit vmemdisk_exit(void)
{
    unregister_blkdev(vmemdisk_major, "vmemdisk");
    kfree(devices);
    printk("exit success: %s\n", __FILE__);
}

module_init(vmemdisk_init)
module_exit(vmemdisk_exit)
// the declaration	of the author
MODULE_AUTHOR("ZhongHuan Duan <15818411038@163.com>");
// the declaration of the licence
MODULE_LICENSE("GPL v2");
