#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    IO_DIR_IN = 0,
    IO_DIR_OUT = 1
} io_direction_t;

int io_set_direction(int gpio_num, io_direction_t direction);
int io_set_output_level(int gpio_num, int level);
int io_get_input_level(int gpio_num, int *level);

#ifdef __cplusplus
}
#endif
