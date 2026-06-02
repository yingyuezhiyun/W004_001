#pragma once

#include "comm_service.h"

#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

int comm_tcp_configure(const comm_tcp_config_t *config);
int comm_tcp_init(void);
int comm_tcp_send(const void *data, size_t len);

#ifdef __cplusplus
}
#endif