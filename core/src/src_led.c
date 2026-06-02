#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "src_led.h"

int set_led(char *name, int on)
{

    char path[64];
    snprintf(path, sizeof(path), "/sys/class/leds/%s/brightness", name);
    int fd = open(path, O_WRONLY);
    if (fd < 0)
    {
        perror("open LED brightness");
        return -1;
    }

    const char *value = on ? "1" : "0";
    ssize_t n = write(fd, value, 1);
    close(fd);

    if (n != 1)
    {
        perror("write LED brightness");
        return -1;
    }
    return 0;
}