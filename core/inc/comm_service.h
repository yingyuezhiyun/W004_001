#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif
#include "glob_value.h"
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

    typedef enum
    {
        REV_CMD_SET_EPH_SW = 0xA0,      // 星历开关
        REV_CMD_SET_OUTPUT_FREQ = 0xA1, // 输出频率
        REV_CMD_QUERY_EPH_SW = 0xB0,
        REV_CMD_QUERY_OUTPUT_FREQ = 0xB1,
    } NET_REV_CMD_t;

    typedef enum
    {
        SEND_CMD_POS = 0x10,         // 位置数据
        SEND_CMD_EPH_INFO = 0x11,    // 星历信息
        SEND_CMD_EPH_SW = 0x20,      // 星历开关
        SEND_CMD_OUTPUT_FREQ = 0x21, // 输出频率
    } NET_SEND_CMD_t;

    int comm_service_init(const comm_service_config_t *config);
    int comm_service_apply_config(const comm_service_config_t *config);
    int comm_service_send(comm_service_kind_t kind, const void *data, size_t len);
    int comm_service_receive(comm_service_kind_t kind, const void *buffer, size_t len);
    int net_send(comm_service_kind_t kind, const uint8_t cmd, const void *data, size_t len);
    int net_send_eph(const comm_service_kind_t kind, const EPH_Type_t eph_type, const void *data, size_t len);

#ifdef __cplusplus
}
#endif
