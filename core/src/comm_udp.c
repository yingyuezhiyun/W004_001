#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "comm_udp.h"

#define COMM_UDP_PORT 9000
#define COMM_UDP_QUEUE_CAPACITY 16

typedef struct
{
    unsigned char *data;
    size_t len;
    struct sockaddr_in peer;
    socklen_t peer_len;
} comm_udp_frame_t;

typedef struct
{
    pthread_t rx_thread;
    pthread_t tx_thread;

    pthread_mutex_t state_mutex;
    pthread_mutex_t queue_mutex;
    pthread_cond_t queue_cond;

    int fd;
    int running;
    int last_peer_valid;
    struct sockaddr_in last_peer;

    comm_udp_config_t config;

    comm_udp_frame_t queue[COMM_UDP_QUEUE_CAPACITY];
    size_t queue_head;
    size_t queue_tail;
    size_t queue_count;
} comm_udp_context_t;

static comm_udp_context_t g_comm_udp_ctx = {
    .state_mutex = PTHREAD_MUTEX_INITIALIZER,
    .queue_mutex = PTHREAD_MUTEX_INITIALIZER,
    .queue_cond = PTHREAD_COND_INITIALIZER,
    .fd = -1,
    .config = {.port = COMM_UDP_PORT},
};

static int comm_udp_queue_push_frame(const void *data, size_t len, const struct sockaddr_in *peer, socklen_t peer_len)
{
    unsigned char *copy = (unsigned char *)malloc(len);
    if (copy == NULL)
    {
        return -1;
    }

    memcpy(copy, data, len);

    pthread_mutex_lock(&g_comm_udp_ctx.queue_mutex);
    if (g_comm_udp_ctx.queue_count == COMM_UDP_QUEUE_CAPACITY)
    {
        pthread_mutex_unlock(&g_comm_udp_ctx.queue_mutex);
        free(copy);
        errno = EAGAIN;
        return -1;
    }

    g_comm_udp_ctx.queue[g_comm_udp_ctx.queue_tail].data = copy;
    g_comm_udp_ctx.queue[g_comm_udp_ctx.queue_tail].len = len;
    g_comm_udp_ctx.queue[g_comm_udp_ctx.queue_tail].peer = *peer;
    g_comm_udp_ctx.queue[g_comm_udp_ctx.queue_tail].peer_len = peer_len;
    g_comm_udp_ctx.queue_tail = (g_comm_udp_ctx.queue_tail + 1U) % COMM_UDP_QUEUE_CAPACITY;
    g_comm_udp_ctx.queue_count++;
    pthread_cond_signal(&g_comm_udp_ctx.queue_cond);
    pthread_mutex_unlock(&g_comm_udp_ctx.queue_mutex);

    return 0;
}

static int comm_udp_queue_pop_frame(comm_udp_frame_t *frame)
{
    pthread_mutex_lock(&g_comm_udp_ctx.queue_mutex);
    while (g_comm_udp_ctx.running && g_comm_udp_ctx.queue_count == 0)
    {
        pthread_cond_wait(&g_comm_udp_ctx.queue_cond, &g_comm_udp_ctx.queue_mutex);
    }

    if (!g_comm_udp_ctx.running && g_comm_udp_ctx.queue_count == 0)
    {
        pthread_mutex_unlock(&g_comm_udp_ctx.queue_mutex);
        return -1;
    }

    *frame = g_comm_udp_ctx.queue[g_comm_udp_ctx.queue_head];
    g_comm_udp_ctx.queue[g_comm_udp_ctx.queue_head].data = NULL;
    g_comm_udp_ctx.queue[g_comm_udp_ctx.queue_head].len = 0;
    g_comm_udp_ctx.queue_head = (g_comm_udp_ctx.queue_head + 1U) % COMM_UDP_QUEUE_CAPACITY;
    g_comm_udp_ctx.queue_count--;
    pthread_mutex_unlock(&g_comm_udp_ctx.queue_mutex);

    return 0;
}

static int comm_udp_create_server(void)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        perror("socket udp");
        return -1;
    }

    int reuse = 1;
    (void)setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(g_comm_udp_ctx.config.port);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind udp");
        close(fd);
        return -1;
    }

    return fd;
}

static void comm_udp_parse_frame(const unsigned char *data, size_t len)
{
    size_t copy_len = len;
    if (copy_len >= 1024U)
    {
        copy_len = 1023U;
    }

    char buffer[1024];
    memcpy(buffer, data, copy_len);
    buffer[copy_len] = '\0';
    printf("[UDP][RX] parsed: %s\n", buffer);
}

static void *comm_udp_rx_thread_func(void *arg)
{
    (void)arg;

    int fd = comm_udp_create_server();
    if (fd < 0)
    {
        return NULL;
    }

    pthread_mutex_lock(&g_comm_udp_ctx.state_mutex);
    g_comm_udp_ctx.fd = fd;
    pthread_mutex_unlock(&g_comm_udp_ctx.state_mutex);

    printf("[UDP] server listening on %u\n", g_comm_udp_ctx.config.port);

    while (g_comm_udp_ctx.running)
    {
        unsigned char buffer[1024];
        struct sockaddr_in peer;
        socklen_t peer_len = sizeof(peer);

        ssize_t n = recvfrom(fd, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&peer, &peer_len);
        if (n < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            perror("recvfrom udp");
            continue;
        }

        buffer[n] = '\0';

        char peer_ip[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET, &peer.sin_addr, peer_ip, sizeof(peer_ip));
        printf("[UDP] %s:%u -> %s\n", peer_ip, ntohs(peer.sin_port), buffer);

        pthread_mutex_lock(&g_comm_udp_ctx.state_mutex);
        g_comm_udp_ctx.last_peer = peer;
        g_comm_udp_ctx.last_peer_valid = 1;
        pthread_mutex_unlock(&g_comm_udp_ctx.state_mutex);

        comm_udp_parse_frame(buffer, (size_t)n);
        if (comm_udp_queue_push_frame(buffer, (size_t)n, &peer, peer_len) < 0)
        {
            perror("queue udp frame");
        }
    }

    close(fd);
    pthread_mutex_lock(&g_comm_udp_ctx.state_mutex);
    g_comm_udp_ctx.fd = -1;
    pthread_mutex_unlock(&g_comm_udp_ctx.state_mutex);
    return NULL;
}

static void *comm_udp_tx_thread_func(void *arg)
{
    (void)arg;

    while (g_comm_udp_ctx.running)
    {
        comm_udp_frame_t frame;
        if (comm_udp_queue_pop_frame(&frame) < 0)
        {
            continue;
        }

        pthread_mutex_lock(&g_comm_udp_ctx.state_mutex);
        int fd = g_comm_udp_ctx.fd;
        pthread_mutex_unlock(&g_comm_udp_ctx.state_mutex);

        if (fd >= 0)
        {
            if (sendto(fd, frame.data, frame.len, 0, (struct sockaddr *)&frame.peer, frame.peer_len) < 0)
            {
                perror("sendto udp");
            }
            else
            {
                printf("[UDP][TX] sent %zu bytes\n", frame.len);
            }
        }

        free(frame.data);
    }

    return NULL;
}

int comm_udp_send(const void *data, size_t len)
{
    if (data == NULL || len == 0)
    {
        errno = EINVAL;
        return -1;
    }

    pthread_mutex_lock(&g_comm_udp_ctx.state_mutex);
    if (!g_comm_udp_ctx.last_peer_valid)
    {
        pthread_mutex_unlock(&g_comm_udp_ctx.state_mutex);
        errno = ENOTCONN;
        return -1;
    }

    struct sockaddr_in peer = g_comm_udp_ctx.last_peer;
    pthread_mutex_unlock(&g_comm_udp_ctx.state_mutex);

    return comm_udp_queue_push_frame(data, len, &peer, sizeof(peer));
}

int comm_udp_init(void)
{
    g_comm_udp_ctx.running = 1;

    int ret = pthread_create(&g_comm_udp_ctx.rx_thread, NULL, comm_udp_rx_thread_func, NULL);
    if (ret != 0)
    {
        fprintf(stderr, "pthread_create udp rx failed: %s\n", strerror(ret));
        return -1;
    }

    ret = pthread_create(&g_comm_udp_ctx.tx_thread, NULL, comm_udp_tx_thread_func, NULL);
    if (ret != 0)
    {
        fprintf(stderr, "pthread_create udp tx failed: %s\n", strerror(ret));
        return -1;
    }

    return 0;
}

int comm_udp_configure(const comm_udp_config_t *config)
{
    if (config == NULL || config->port == 0U)
    {
        return 0;
    }

    pthread_mutex_lock(&g_comm_udp_ctx.state_mutex);
    g_comm_udp_ctx.config = *config;
    pthread_mutex_unlock(&g_comm_udp_ctx.state_mutex);

    return 0;
}