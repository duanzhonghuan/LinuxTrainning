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

#define  GLOBALMEM_MAJOR  (230)

static int globalmem_major = GLOBALMEM_MAJOR;

/**
 * @brief globalmem_init - the function of initting the global memory
 * @return: the status of initting the global memory
 */
static int __init globalmem_init(void)
{
    int error_code = 0;
    dev_t devno = MKDEV(globalmem_major, 0);
    if (globalmem_major)
    {
        // only register a char device
        error_code = register_chrdev_region(devno, 1, "globalmem");
    }
    else
    {
        error_code = alloc_chrdev_region(&devno, 0, 1, "globalmem");
    }
    // check the error code
    if (!error_code)
    {
        return error_code;
    }

    printk("globalmem_init:register chrdev success\n");
    return 0;
}

static void __exit globalmem_exit(void)
{

}

module_init(globalmem_init);
module_exit(globalmem_exit);
// the declaration	of the author
MODULE_AUTHOR("ZhongHuan Duan <15818411038@163.com>");
// the declaration of the licence
MODULE_LICENSE("GPL v2");
