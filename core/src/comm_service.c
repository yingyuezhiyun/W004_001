#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include "comm_mqtt.h"
#include "comm_service.h"
#include "comm_tcp.h"
#include "comm_udp.h"
#include "glob_cfg.h"
#include "glob_value.h"
#include "gnss_func.h"

#define NET_FRAME_HEAD (0xAA55)
#define NET_FRAME_TAIL (0xFFEE)

#define NET_FRAME_HEAD_REVERSE (0x55AA)
#define NET_FRAME_TAIL_REVERSE (0xEEFF)

glob_comm_config_t glob_comm_config = {
    .output_freq = 1,
    .eph_sw.value = 0,
    .id = 1,
};

#pragma pack(1)
typedef struct
{
    uint16_t head;
    uint16_t id;
    uint8_t cmd;
    uint16_t len;
    // uint8_t payload[n];
    uint8_t check;
    uint16_t tail;
} net_frame_t;
#pragma pack()

static const comm_service_config_t k_comm_service_default_config = {
    .udp = {
        .port = 8888,
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
        .port = 8888,
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

    // if (comm_tcp_init() < 0)
    // {
    //     return -1;
    // }

    // if (comm_mqtt_init() < 0)
    // {
    //     return -1;
    // }

    return 0;
}

uint8_t check_sum(const uint8_t *data, size_t len)
{
    if (len == 0)
    {
        return 0;
    }
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i)
    {
        sum += data[i];
    }
    return sum;
}

int net_send(comm_service_kind_t kind, const uint8_t cmd, const void *data, size_t len)
{
    static char send_buf[65536] = {0};
    net_frame_t *frame = (net_frame_t *)send_buf;
    frame->head = NET_FRAME_HEAD_REVERSE;
    frame->cmd = cmd;
    frame->len = len;
    frame->id = glob_comm_config.id;
    memcpy(send_buf + 7, data, len);
    *(uint16_t *)(send_buf + 7 + len) = check_sum(send_buf + 2, len + 5);
    *(uint16_t *)(send_buf + 8 + len) = NET_FRAME_TAIL_REVERSE;
    len += sizeof(net_frame_t);
    switch (kind)
    {
    case COMM_SERVICE_UDP:
        return comm_udp_send(send_buf, len);
    case COMM_SERVICE_TCP:
        return comm_tcp_send(send_buf, len);
    case COMM_SERVICE_MQTT:
        return comm_mqtt_send(send_buf, len);
    default:
        return -1;
    }
}

int net_send_eph(const comm_service_kind_t kind, const EPH_Type_t eph_type, const void *data, size_t len)
{
    static char send_buf[65536] = {0};
    net_frame_t *frame = (net_frame_t *)send_buf;
    frame->head = NET_FRAME_HEAD_REVERSE;
    frame->cmd = SEND_CMD_EPH_INFO;
    frame->len = len + 1;
    frame->id = glob_comm_config.id;
    memcpy(send_buf + 7, &eph_type, 1);
    memcpy(send_buf + 8, data, len);
    *(uint16_t *)(send_buf + 8 + len) = check_sum(send_buf + 2, len + 6);
    *(uint16_t *)(send_buf + 9 + len) = NET_FRAME_TAIL_REVERSE;
    len = len + sizeof(net_frame_t) + 1;
    switch (kind)
    {
    case COMM_SERVICE_UDP:
        return comm_udp_send(send_buf, len);
    case COMM_SERVICE_TCP:
        return comm_tcp_send(send_buf, len);
    case COMM_SERVICE_MQTT:
        return comm_mqtt_send(send_buf, len);
    default:
        return -1;
    }
}

void update_eph_sw()
{
    if (glob_comm_config.eph_sw.content.bdxw)
    {
        gnss_cfg_dis_enable(gnss_ctrl.fd, "BDXWEPHB", 1, glob_comm_config.output_freq);
        usleep(50000); // Sleep for 50 milliseconds
    }
    else
    {
        gnss_cfg_dis_enable(gnss_ctrl.fd, "BDXWEPHB", 0, glob_comm_config.output_freq);
        usleep(50000); // Sleep for 50 milliseconds
    }
    if (glob_comm_config.eph_sw.content.bd2)
    {
        gnss_cfg_dis_enable(gnss_ctrl.fd, "BD2EPHB", 1, glob_comm_config.output_freq);
        usleep(50000); // Sleep for 50 milliseconds
    }
    else
    {
        gnss_cfg_dis_enable(gnss_ctrl.fd, "BD2EPHB", 0, glob_comm_config.output_freq);
        usleep(50000); // Sleep for 50 milliseconds
    }
    if (glob_comm_config.eph_sw.content.bd3)
    {
        gnss_cfg_dis_enable(gnss_ctrl.fd, "BD3EPHB", 1, glob_comm_config.output_freq);
        usleep(50000); // Sleep for 50 milliseconds
    }
    else
    {
        gnss_cfg_dis_enable(gnss_ctrl.fd, "BD3EPHB", 0, glob_comm_config.output_freq);
        usleep(50000); // Sleep for 50 milliseconds
    }
    if (glob_comm_config.eph_sw.content.gps)
    {
        gnss_cfg_dis_enable(gnss_ctrl.fd, "GPSEPHB", 1, glob_comm_config.output_freq);
        usleep(50000); // Sleep for 50 milliseconds
    }
    else
    {
        gnss_cfg_dis_enable(gnss_ctrl.fd, "GPSEPHB", 0, glob_comm_config.output_freq);
        usleep(50000); // Sleep for 50 milliseconds
    }
    if (glob_comm_config.eph_sw.content.gal)
    {
        gnss_cfg_dis_enable(gnss_ctrl.fd, "GALEPHB", 1, glob_comm_config.output_freq);
        usleep(50000); // Sleep for 50 milliseconds
    }
    else
    {
        gnss_cfg_dis_enable(gnss_ctrl.fd, "GALEPHB", 0, glob_comm_config.output_freq);
        usleep(50000); // Sleep for 50 milliseconds
    }
    if (glob_comm_config.eph_sw.content.glo)
    {
        gnss_cfg_dis_enable(gnss_ctrl.fd, "GLOEPHB", 1, glob_comm_config.output_freq);
        usleep(50000); // Sleep for 50 milliseconds
    }
    else
    {
        gnss_cfg_dis_enable(gnss_ctrl.fd, "GLOEPHB", 0, glob_comm_config.output_freq);
        usleep(50000); // Sleep for 50 milliseconds
    }
    if (glob_comm_config.eph_sw.content.bd3cnav2)
    {
        gnss_cfg_dis_enable(gnss_ctrl.fd, "BD3CANV2EPHB", 1, glob_comm_config.output_freq);
        usleep(50000); // Sleep for 50 milliseconds
    }
    else
    {
        gnss_cfg_dis_enable(gnss_ctrl.fd, "BD3CANV2EPHB", 0, glob_comm_config.output_freq);
        usleep(50000); // Sleep for 50 milliseconds
    }
    if (glob_comm_config.eph_sw.content.bd3cnav3)
    {
        gnss_cfg_dis_enable(gnss_ctrl.fd, "BD3CNAV3EPHB", 1, glob_comm_config.output_freq);
        usleep(50000); // Sleep for 50 milliseconds
    }
    else
    {
        gnss_cfg_dis_enable(gnss_ctrl.fd, "BD3CNAV3EPHB", 0, glob_comm_config.output_freq);
        usleep(50000); // Sleep for 50 milliseconds
    }
}

void rev_process(const uint8_t cmd, const int8_t *data, size_t len)
{
    switch (cmd)
    {
    case REV_CMD_SET_EPH_SW:
        if (len == sizeof(uint16_t))
        {
            glob_comm_config.eph_sw.value = *(uint16_t *)data;
            update_eph_sw();
        }
        net_send(COMM_SERVICE_UDP, SEND_CMD_EPH_SW, &glob_comm_config.eph_sw.value, sizeof(glob_comm_config.eph_sw.value));
        break;
    case REV_CMD_SET_OUTPUT_FREQ:
        if (len == sizeof(uint8_t))
        {
            glob_comm_config.output_freq = *(uint8_t *)data;
        }

        net_send(COMM_SERVICE_UDP, SEND_CMD_OUTPUT_FREQ, &glob_comm_config.output_freq, sizeof(glob_comm_config.output_freq));
        break;
    case REV_CMD_QUERY_EPH_SW:
        net_send(COMM_SERVICE_UDP, SEND_CMD_EPH_SW, &glob_comm_config.eph_sw.value, sizeof(glob_comm_config.eph_sw.value));
        break;
    case REV_CMD_QUERY_OUTPUT_FREQ:
        net_send(COMM_SERVICE_UDP, SEND_CMD_OUTPUT_FREQ, &glob_comm_config.output_freq, sizeof(glob_comm_config.output_freq));
        break;
    default:
        break;
    }
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

int comm_service_receive(comm_service_kind_t kind, const void *buffer, size_t len)
{
    switch (kind)
    {
    case COMM_SERVICE_UDP:
    {
        net_frame_t *frame = (net_frame_t *)buffer;
        uint8_t *data_ptr = (uint8_t *)buffer;
        if (frame->head != NET_FRAME_HEAD_REVERSE ||
            len < frame->len + sizeof(net_frame_t) ||
            *(uint8_t *)(data_ptr + 7 + frame->len) != check_sum(data_ptr + 2, frame->len + 5) ||
            *(uint16_t *)(data_ptr + 8 + frame->len) != NET_FRAME_TAIL_REVERSE)
        {
            printf("UDP frame error\n");
            return -1;
        }
        rev_process(frame->cmd, data_ptr + 7, frame->len);
    }
    break;
    case COMM_SERVICE_TCP:
        break;
    case COMM_SERVICE_MQTT:
        break;
    default:
        break;
    }
    return 0;
}
