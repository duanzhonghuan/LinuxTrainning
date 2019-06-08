/**
 ** This file is part of the LinuxTrainningCompany project.
 ** Copyright (C) duanzhonghuan Co., Ltd.
 ** All Rights Reserved.
 ** Unauthorized copying of this file, via any medium is strictly prohibited
 ** Proprietary and confidential
 **
 ** Written by duanzhonghuan <15818411038@163.com>, 2019/5/18
 **/

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "unistd.h"


void signal_handle(int signo)
{
    printf("Have caught the Signal: %d\n", signo);
    exit(0);
}

int main(void)
{
    int fd, oflags;
    fd = open("/dev/globalfifo", O_RDWR);
    if (fd != -1)
    {
        // if the signal @SIGIO is captured,
        // the function of @signal_handle is called
        signal(SIGIO, signal_handle);
        // set one of the @fd owners as the process
        fcntl(fd, F_SETOWN, getpid());
        // receive the asynchronous sinals
        oflags = fcntl(fd, F_GETFL);
        fcntl(fd, F_SETFL, oflags | O_ASYNC);
        while(1)
        {
            sleep(1);
        }
    }
    else
    {
        printf("open /dev/globalfifo failed \n");
    }
#if 0
    signal(SIGINT, signal_handle);
    signal(SIGTERM, signal_handle);
    while(1) {}
#endif
    return 0;
}
