#pragma once

#include "comm_service.h"

#ifdef __cplusplus
extern "C"
{
#endif

int comm_udp_configure(const comm_udp_config_t *config);
int comm_udp_init(void);
int comm_udp_send(const void *data, size_t len);

#ifdef __cplusplus
}
#endif