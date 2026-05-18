#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "src_io.h"

#define GPIO_SYSFS_PATH_MAX (64)

static int write_text_file(const char *path, const char *text)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0)
    {
        return -1;
    }

    ssize_t n = write(fd, text, strlen(text));
    close(fd);

    if (n < 0)
    {
        return -1;
    }
    return 0;
}

static int gpio_export_if_needed(int gpio_num)
{
    char gpio_dir[GPIO_SYSFS_PATH_MAX];
    snprintf(gpio_dir, sizeof(gpio_dir), "/sys/class/gpio/gpio%d", gpio_num);
    if (access(gpio_dir, F_OK) == 0)
    {
        return 0;
    }

    char gpio_buf[16];
    snprintf(gpio_buf, sizeof(gpio_buf), "%d", gpio_num);
    if (write_text_file("/sys/class/gpio/export", gpio_buf) < 0)
    {
        if (errno != EBUSY)
        {
            return -1;
        }
    }
    return 0;
}

int io_set_direction(int gpio_num, io_direction_t direction)
{
    if ((gpio_num < 0) || (direction != IO_DIR_IN && direction != IO_DIR_OUT))
    {
        errno = EINVAL;
        return -1;
    }

    if (gpio_export_if_needed(gpio_num) < 0)
    {
        return -1;
    }

    char path[GPIO_SYSFS_PATH_MAX];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio_num);

    if (direction == IO_DIR_OUT)
    {
        return write_text_file(path, "out");
    }
    return write_text_file(path, "in");
}

int io_set_output_level(int gpio_num, int level)
{
    if ((gpio_num < 0) || (level != 0 && level != 1))
    {
        errno = EINVAL;
        return -1;
    }

    if (io_set_direction(gpio_num, IO_DIR_OUT) < 0)
    {
        return -1;
    }

    char path[GPIO_SYSFS_PATH_MAX];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio_num);

    return write_text_file(path, level ? "1" : "0");
}

int io_get_input_level(int gpio_num, int *level)
{
    if ((gpio_num < 0) || (level == NULL))
    {
        errno = EINVAL;
        return -1;
    }

    if (io_set_direction(gpio_num, IO_DIR_IN) < 0)
    {
        return -1;
    }

    char path[GPIO_SYSFS_PATH_MAX];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio_num);

    int fd = open(path, O_RDONLY);
    if (fd < 0)
    {
        return -1;
    }

    char value = 0;
    ssize_t n = read(fd, &value, 1);
    close(fd);

    if (n != 1)
    {
        if (n >= 0)
        {
            errno = EIO;
        }
        return -1;
    }

    if (value == '1')
    {
        *level = 1;
    }
    else
    {
        *level = 0;
    }

    return 0;
}
