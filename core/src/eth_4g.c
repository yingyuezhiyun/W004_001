#include "stdint.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include "glob_cfg.h"
#include "src_tty.h"
#include "src_io.h"

int8_t eth_4g_enable()
{
    // todo
    return 0;
}

int8_t eth_4g_disable()
{
    // todo
    return 0;
}

void *eth_4g_thread_func(void *arg)
{
    (void)arg;
    // todo
    while (1)
    {

        sleep(1);
    }
    return NULL;
}
