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

static int lora_fd = -1;

int8_t lora_enable()
{
    // todo
    if (io_set_output_level(DEV_LoRa_IO_AUX, 1) < 0)
    {
        fprintf(stderr, "Failed to set LoRa AUX pin high\n");
        return -1;
    }
    // usleep(100000); // Sleep for 100 milliseconds
    if (io_set_output_level(DEV_LoRa_IO_RESET, 1) < 0)
    {
        fprintf(stderr, "Failed to set LoRa RESET pin high\n");
        return -1;
    }
    sleep(1); // Sleep for 1 second to allow the module to reset
    if (io_set_output_level(DEV_LoRa_IO_RESET, 0) < 0)
    {
        fprintf(stderr, "Failed to set LoRa RESET pin low\n");
        return -1;
    }
    sleep(1); // Sleep for 1 second to allow the module to initialize
    io_set_direction(DEV_LoRa_IO_AUX, IO_DIR_IN);

    return 0;
}

int8_t lora_disable()
{
    if (io_set_output_level(DEV_LoRa_IO_RESET, 1) < 0)
    {
        fprintf(stderr, "Failed to set LoRa RESET pin high\n");
        return -1;
    }
    return 0;
}

int8_t lora_send(const char *data, size_t len)
{
    if (lora_fd < 0)
    {
        fprintf(stderr, "LoRa device not opened\n");
        return -1;
    }
    ssize_t n = write(lora_fd, data, len);
    if (n < 0)
    {
        perror("write lora device");
        return -1;
    }
    return 0;
}

void *lora_thread_func(void *arg)
{
    (void)arg;
    if (lora_enable() < 0)
    {
        fprintf(stderr, "Failed to initialize LoRa module\n");
        return;
    }
    lora_fd = open(DEV_LoRa, O_RDWR | O_NOCTTY | O_NDELAY);
    if (lora_fd < 0)
    {
        perror("open lora device");
        return -1;
    }
    set_opt(lora_fd, 9600, 8, 'N', 1);
    char buf[256];
    while (1)
    {
        int n = read(lora_fd, buf, sizeof(buf) - 1);
        if (n > 0)
        {
            buf[n] = '\0';
            printf("LoRa received: %s\n", buf);
        }
        sleep(1);
    }
    return NULL;
}
