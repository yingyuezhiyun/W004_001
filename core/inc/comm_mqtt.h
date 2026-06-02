#pragma once

#include "comm_service.h"

#ifdef __cplusplus
extern "C"
{
#endif

int comm_mqtt_configure(const comm_mqtt_config_t *config);
int comm_mqtt_init(void);
int comm_mqtt_send(const void *data, size_t len);
int comm_mqtt_send_publish(const char *payload);

#ifdef __cplusplus
}
#endif