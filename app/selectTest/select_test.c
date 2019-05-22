/**
 ** This file is part of the LinuxTrainningHome project.
 ** Copyright(C) duanzhonghuan Co., Ltd.
 ** All Rights Reserved.
 ** Unauthorized copying of this file, via any medium is strictly prohibited
 ** Proprietary and confidential
 **
 ** Written by ZhongHuan Duan <15818411038@163.com>, 2019-05-22
 **/


#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>

#define FIFO_CLEAR 0X1
#define BUFFER_LEN 20

int main(void)
{
    int fd, num;
    char rd_ch[BUFFER_LEN];
    // read/write file description
    fd_set rfds, wfds;

    fd = open("/dev/globalfifo", O_RDONLY | O_NONBLOCK);
    if (fd != -1)
    {
        if (ioctl(fd, FIFO_CLEAR, 0) != 0)
        {
            printf("ioctl command failed \n");
        }

        while(1)
        {
            FD_ZERO(&rfds);
            FD_ZERO(&wfds);

            FD_SET(fd, &rfds);
            FD_SET(fd, &wfds);

            select(fd + 1, &rfds, &wfds, 0, 0);

            if (FD_ISSET(fd, &rfds))
            {
                printf("have data to read\n");
            }
            if (FD_ISSET(fd, &wfds))
            {
                printf("have space to write\n");
            }
        }

    }
    else
    {
        printf("open /dev/globalfifo failed \n");
    }
    return 0;
}
