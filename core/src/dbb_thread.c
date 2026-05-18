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

void *dbb_thread_func(void *arg)
{
    (void)arg;
    int fd = open(DEV_DBB, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0)
    {
        perror("open dbb device");
        return NULL;
    }
    set_opt(fd, 115200, 8, 'N', 1);
    usleep(100000); // Sleep for 100 milliseconds to allow the device to initialize

    // todo
    while (1)
    {

        sleep(1);
    }
    return NULL;
}