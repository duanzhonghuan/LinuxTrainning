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

void signal_handle(int signo)
{
    printf("Have caught the Signal: %d\n", signo);
    exit(0);
}

int main(void)
{
    signal(SIGINT, signal_handle);
    signal(SIGTERM, signal_handle);
    while(1) {}
    return 0;
}
