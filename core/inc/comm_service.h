#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    uint16_t port;
} comm_udp_config_t;

typedef struct
{
    uint16_t port;
} comm_tcp_config_t;

typedef struct
{
    char host[128];
    char port[16];
    char client_id[64];
    char topic[128];
    unsigned int keepalive;
} comm_mqtt_config_t;

typedef struct
{
    comm_udp_config_t udp;
    comm_tcp_config_t tcp;
    comm_mqtt_config_t mqtt;
} comm_service_config_t;

typedef enum
{
    COMM_SERVICE_UDP = 0,
    COMM_SERVICE_TCP = 1,
    COMM_SERVICE_MQTT = 2,
} comm_service_kind_t;

int comm_service_init(const comm_service_config_t *config);
int comm_service_apply_config(const comm_service_config_t *config);
int comm_service_send(comm_service_kind_t kind, const void *data, size_t len);

#ifdef __cplusplus
}
#endif
