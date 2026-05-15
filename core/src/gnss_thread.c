#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <termios.h>
#include <errno.h>
#include "glob_cfg.h"
#include "src_tty.h"

void *gnss_thread_func(void *arg)
{
    int fd = open(DEV_GNSS, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0)
    {
        perror("open gnss device");
        return NULL;
    }
    set_opt(fd, 115200, 8, 'N', 1);

    char buf[256];
    while (1)
    {
        int n = read(fd, buf, sizeof(buf) - 1);
        if (n > 0)
        {
            buf[n] = '\0';
            printf("GNSS data: %s\n", buf);
        }
        usleep(1000);
    }

    close(fd);

    return NULL;
}
