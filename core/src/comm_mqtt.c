#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "comm_mqtt.h"

typedef struct
{
    pthread_t rx_thread;
    pthread_t tx_thread;

    pthread_mutex_t state_mutex;
    pthread_mutex_t send_mutex;

    int fd;
    int connected;
    int running;

    comm_mqtt_config_t config;
} comm_mqtt_context_t;

static comm_mqtt_context_t g_comm_mqtt_ctx = {
    .state_mutex = PTHREAD_MUTEX_INITIALIZER,
    .send_mutex = PTHREAD_MUTEX_INITIALIZER,
    .fd = -1,
    .config = {
        .host = "127.0.0.1",
        .port = "1883",
        .client_id = "ok3506-demo",
        .topic = "ok3506/demo/status",
        .keepalive = 30U,
    },
};

static int comm_mqtt_write_all(int fd, const void *buf, size_t len)
{
    const uint8_t *cursor = (const uint8_t *)buf;
    size_t written = 0;

    while (written < len)
    {
        ssize_t n = send(fd, cursor + written, len - written, 0);
        if (n < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            return -1;
        }

        written += (size_t)n;
    }

    return 0;
}

static size_t comm_mqtt_encode_remaining_length(uint8_t *buf, size_t length)
{
    size_t index = 0;

    do
    {
        uint8_t encoded = (uint8_t)(length % 128U);
        length /= 128U;
        if (length > 0)
        {
            encoded |= 0x80U;
        }
        buf[index++] = encoded;
    } while (length > 0 && index < 4U);

    return index;
}

static size_t comm_mqtt_put_utf8(uint8_t *buf, size_t offset, const char *text)
{
    size_t len = strlen(text);
    buf[offset++] = (uint8_t)((len >> 8) & 0xFFU);
    buf[offset++] = (uint8_t)(len & 0xFFU);
    memcpy(buf + offset, text, len);
    return offset + len;
}

static void comm_mqtt_sleep_short(void)
{
    struct timespec req;
    req.tv_sec = 0;
    req.tv_nsec = 100000000L;
    (void)nanosleep(&req, NULL);
}

static void comm_mqtt_set_connection(int fd, int connected)
{
    pthread_mutex_lock(&g_comm_mqtt_ctx.state_mutex);
    if (g_comm_mqtt_ctx.fd >= 0 && g_comm_mqtt_ctx.fd != fd && connected == 0)
    {
        close(g_comm_mqtt_ctx.fd);
    }

    g_comm_mqtt_ctx.fd = fd;
    g_comm_mqtt_ctx.connected = connected;
    pthread_mutex_unlock(&g_comm_mqtt_ctx.state_mutex);
}

static int comm_mqtt_get_connection(int *fd, int *connected)
{
    pthread_mutex_lock(&g_comm_mqtt_ctx.state_mutex);
    *fd = g_comm_mqtt_ctx.fd;
    *connected = g_comm_mqtt_ctx.connected;
    pthread_mutex_unlock(&g_comm_mqtt_ctx.state_mutex);
    return 0;
}

static int comm_mqtt_connect_broker(void)
{
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo *result = NULL;
    int rc = getaddrinfo(g_comm_mqtt_ctx.config.host, g_comm_mqtt_ctx.config.port, &hints, &result);
    if (rc != 0)
    {
        fprintf(stderr, "MQTT resolve failed: %s\n", gai_strerror(rc));
        return -1;
    }

    int fd = -1;
    for (struct addrinfo *item = result; item != NULL; item = item->ai_next)
    {
        fd = socket(item->ai_family, item->ai_socktype, item->ai_protocol);
        if (fd < 0)
        {
            continue;
        }

        if (connect(fd, item->ai_addr, item->ai_addrlen) == 0)
        {
            freeaddrinfo(result);
            return fd;
        }

        close(fd);
        fd = -1;
    }

    freeaddrinfo(result);
    return -1;
}

static int comm_mqtt_send_connect(int fd)
{
    uint8_t packet[256];
    uint8_t *cursor = packet;
    size_t client_id_len = strlen(g_comm_mqtt_ctx.config.client_id);
    size_t payload_len = 2U + client_id_len;
    size_t remaining_len = 10U + payload_len;

    *cursor++ = 0x10U;
    cursor += comm_mqtt_encode_remaining_length(cursor, remaining_len);

    *cursor++ = 0x00U;
    *cursor++ = 0x04U;
    *cursor++ = 'M';
    *cursor++ = 'Q';
    *cursor++ = 'T';
    *cursor++ = 'T';
    *cursor++ = 0x04U;
    *cursor++ = 0x02U;
    *cursor++ = (uint8_t)((g_comm_mqtt_ctx.config.keepalive >> 8) & 0xFFU);
    *cursor++ = (uint8_t)(g_comm_mqtt_ctx.config.keepalive & 0xFFU);
    cursor += comm_mqtt_put_utf8(cursor, 0, g_comm_mqtt_ctx.config.client_id);

    return comm_mqtt_write_all(fd, packet, (size_t)(cursor - packet));
}

static int comm_mqtt_send_publish_fd(int fd, const void *payload, size_t len)
{
    if (payload == NULL || len == 0)
    {
        errno = EINVAL;
        return -1;
    }

    uint8_t packet[512];
    uint8_t *cursor = packet;
    size_t topic_len = strlen(g_comm_mqtt_ctx.config.topic);
    size_t remaining_len = 2U + topic_len + len;

    if (remaining_len + 2U >= sizeof(packet))
    {
        errno = EMSGSIZE;
        return -1;
    }

    *cursor++ = 0x30U;
    cursor += comm_mqtt_encode_remaining_length(cursor, remaining_len);
    cursor += comm_mqtt_put_utf8(cursor, 0, g_comm_mqtt_ctx.config.topic);
    memcpy(cursor, payload, len);
    cursor += len;

    return comm_mqtt_write_all(fd, packet, (size_t)(cursor - packet));
}

static int comm_mqtt_send_pingreq(int fd)
{
    const uint8_t packet[2] = {0xC0U, 0x00U};
    return comm_mqtt_write_all(fd, packet, sizeof(packet));
}

static void comm_mqtt_parse_packet(const uint8_t *packet, size_t len)
{
    if (len == 0)
    {
        return;
    }

    uint8_t packet_type = packet[0] >> 4;
    if (packet_type == 2U && len >= 4U)
    {
        printf("[MQTT][RX] CONNACK rc=%u\n", packet[3]);
        return;
    }

    printf("[MQTT][RX] packet type=%u len=%zu\n", packet_type, len);
}

static void *comm_mqtt_rx_thread_func(void *arg)
{
    (void)arg;

    while (g_comm_mqtt_ctx.running)
    {
        int fd = -1;
        int connected = 0;
        comm_mqtt_get_connection(&fd, &connected);

        if (!connected || fd < 0)
        {
            comm_mqtt_sleep_short();
            continue;
        }

        uint8_t buffer[512];
        ssize_t n = recv(fd, buffer, sizeof(buffer), 0);
        if (n < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            perror("recv mqtt");
            comm_mqtt_set_connection(-1, 0);
            continue;
        }

        if (n == 0)
        {
            printf("[MQTT][RX] broker closed connection\n");
            comm_mqtt_set_connection(-1, 0);
            continue;
        }

        comm_mqtt_parse_packet(buffer, (size_t)n);
    }

    return NULL;
}

int comm_mqtt_configure(const comm_mqtt_config_t *config)
{
    if (config == NULL)
    {
        return 0;
    }

    pthread_mutex_lock(&g_comm_mqtt_ctx.state_mutex);
    g_comm_mqtt_ctx.config = *config;
    pthread_mutex_unlock(&g_comm_mqtt_ctx.state_mutex);

    return 0;
}

int comm_mqtt_send(const void *data, size_t len)
{
    if (data == NULL || len == 0)
    {
        errno = EINVAL;
        return -1;
    }

    int fd = -1;
    int connected = 0;
    comm_mqtt_get_connection(&fd, &connected);
    if (!connected || fd < 0)
    {
        errno = ENOTCONN;
        return -1;
    }

    pthread_mutex_lock(&g_comm_mqtt_ctx.send_mutex);
    int ret = comm_mqtt_send_publish_fd(fd, data, len);
    pthread_mutex_unlock(&g_comm_mqtt_ctx.send_mutex);

    return ret;
}

int comm_mqtt_send_publish(const char *payload)
{
    if (payload == NULL)
    {
        errno = EINVAL;
        return -1;
    }

    return comm_mqtt_send(payload, strlen(payload));
}

static void *comm_mqtt_tx_thread_func(void *arg)
{
    (void)arg;

    while (g_comm_mqtt_ctx.running)
    {
        int fd = comm_mqtt_connect_broker();
        if (fd < 0)
        {
            fprintf(stderr, "[MQTT] connect broker %s:%s failed\n", g_comm_mqtt_ctx.config.host, g_comm_mqtt_ctx.config.port);
            sleep(3);
            continue;
        }

        printf("[MQTT] connected to %s:%s\n", g_comm_mqtt_ctx.config.host, g_comm_mqtt_ctx.config.port);

        comm_mqtt_set_connection(fd, 1);

        if (comm_mqtt_send_connect(fd) < 0)
        {
            perror("mqtt connect packet");
            comm_mqtt_set_connection(-1, 0);
            sleep(3);
            continue;
        }

        uint8_t connack[4] = {0};
        (void)recv(fd, connack, sizeof(connack), 0);

        unsigned int tick = 0;
        while (g_comm_mqtt_ctx.running)
        {
            char payload[256];
            int payload_size = snprintf(payload, sizeof(payload), "{\"device\":\"ok3506\",\"tick\":%u}", tick++);
            if (payload_size < 0)
            {
                break;
            }

            if (comm_mqtt_send(payload, (size_t)payload_size) < 0)
            {
                perror("mqtt publish");
                break;
            }

            for (unsigned int elapsed = 0; elapsed < 5U; ++elapsed)
            {
                sleep(1);
            }

            if ((tick % 4U) == 0U)
            {
                if (comm_mqtt_send_pingreq(fd) < 0)
                {
                    perror("mqtt ping");
                    break;
                }
            }
        }

        comm_mqtt_set_connection(-1, 0);
        sleep(3);
    }

    return NULL;
}

int comm_mqtt_init(void)
{
    g_comm_mqtt_ctx.running = 1;

    int ret = pthread_create(&g_comm_mqtt_ctx.rx_thread, NULL, comm_mqtt_rx_thread_func, NULL);
    if (ret != 0)
    {
        fprintf(stderr, "pthread_create mqtt rx failed: %s\n", strerror(ret));
        return -1;
    }

    ret = pthread_create(&g_comm_mqtt_ctx.tx_thread, NULL, comm_mqtt_tx_thread_func, NULL);
    if (ret != 0)
    {
        fprintf(stderr, "pthread_create mqtt tx failed: %s\n", strerror(ret));
        return -1;
    }

    return 0;
}
