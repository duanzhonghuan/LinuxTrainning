/**
 ** This file is part of the LinuxTrainningHome project.
 ** Copyright(C) duanzhonghuan Co., Ltd.
 ** All Rights Reserved.
 ** Unauthorized copying of this file, via any medium is strictly prohibited
 ** Proprietary and confidential
 **
 ** Written by ZhongHuan Duan <15818411038@163.com>, 2019-05-22
 **/


#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/epoll.h>
#include <strings.h>

#define FIFO_CLEAR 0X1
#define BUFFER_LEN 20

int main(void)
{
    int fd, num;
    char rd_ch[BUFFER_LEN];
    // read/write file description
    fd_set rfds, wfds;
    int epfd;
    struct epoll_event ev_globalfifo;

    fd = open("/dev/globalfifo", O_RDONLY | O_NONBLOCK);
    if (fd != -1)
    {
        if (ioctl(fd, FIFO_CLEAR, 0) != 0)
        {
            printf("ioctl command failed \n");
        }

        epfd = epoll_create(1);
        if (epfd == -1)
        {
            printf("get epoll fd  failed \n");
        }

        bzero(&ev_globalfifo, sizeof(struct epoll_event));

        ev_globalfifo.events = EPOLLIN | EPOLLPRI;

        if (0 != epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev_globalfifo))
        {
            printf("epoll add fd failed \n");
            return 0;
        }

        if (1 == epoll_wait(epfd, &ev_globalfifo, 1, 15000))
        {
            printf("epoll wait is not empty \n");
            return 0;
        }

        if (0 != epoll_ctl(epfd, EPOLL_CTL_DEL, fd, &ev_globalfifo))
        {
            printf("epoll del fd failed \n");
            return 0;
        }
    }
    else
    {
        printf("open /dev/globalfifo failed \n");
    }
    return 0;
}
