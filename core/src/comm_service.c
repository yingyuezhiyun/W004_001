#include <string.h>

#include "comm_mqtt.h"
#include "comm_service.h"
#include "comm_tcp.h"
#include "comm_udp.h"

static const comm_service_config_t k_comm_service_default_config = {
    .udp = {
        .port = 9000,
    },
    .tcp = {
        .port = 9001,
    },
    .mqtt = {
        .host = "127.0.0.1",
        .port = "1883",
        .client_id = "ok3506-demo",
        .topic = "ok3506/demo/status",
        .keepalive = 30U,
    },
};

static comm_service_config_t g_comm_service_config = {
    .udp = {
        .port = 9000,
    },
    .tcp = {
        .port = 9001,
    },
    .mqtt = {
        .host = "127.0.0.1",
        .port = "1883",
        .client_id = "ok3506-demo",
        .topic = "ok3506/demo/status",
        .keepalive = 30U,
    },
};

static comm_service_config_t comm_service_normalize_config(const comm_service_config_t *config)
{
    if (config != NULL)
    {
        return *config;
    }

    return k_comm_service_default_config;
}

int comm_service_apply_config(const comm_service_config_t *config)
{
    g_comm_service_config = comm_service_normalize_config(config);

    (void)comm_udp_configure(&g_comm_service_config.udp);
    (void)comm_tcp_configure(&g_comm_service_config.tcp);
    (void)comm_mqtt_configure(&g_comm_service_config.mqtt);

    return 0;
}

int comm_service_init(const comm_service_config_t *config)
{
    (void)comm_service_apply_config(config);

    if (comm_udp_init() < 0)
    {
        return -1;
    }

    if (comm_tcp_init() < 0)
    {
        return -1;
    }

    if (comm_mqtt_init() < 0)
    {
        return -1;
    }

    return 0;
}

int comm_service_send(comm_service_kind_t kind, const void *data, size_t len)
{
    switch (kind)
    {
    case COMM_SERVICE_UDP:
        return comm_udp_send(data, len);
    case COMM_SERVICE_TCP:
        return comm_tcp_send(data, len);
    case COMM_SERVICE_MQTT:
        return comm_mqtt_send(data, len);
    default:
        return -1;
    }
}