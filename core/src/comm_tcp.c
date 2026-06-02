#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/socket.h>
#include <unistd.h>

#include "comm_tcp.h"

#define COMM_TCP_PORT 9001
#define COMM_TCP_QUEUE_CAPACITY 16

typedef struct
{
    unsigned char *data;
    size_t len;
} comm_tcp_frame_t;

typedef struct
{
    pthread_t accept_thread;
    pthread_t rx_thread;
    pthread_t tx_thread;

    pthread_mutex_t state_mutex;
    pthread_mutex_t queue_mutex;
    pthread_cond_t queue_cond;

    int listen_fd;
    int client_fd;
    int running;

    comm_tcp_config_t config;

    comm_tcp_frame_t queue[COMM_TCP_QUEUE_CAPACITY];
    size_t queue_head;
    size_t queue_tail;
    size_t queue_count;
} comm_tcp_context_t;

static comm_tcp_context_t g_comm_tcp_ctx = {
    .state_mutex = PTHREAD_MUTEX_INITIALIZER,
    .queue_mutex = PTHREAD_MUTEX_INITIALIZER,
    .queue_cond = PTHREAD_COND_INITIALIZER,
    .listen_fd = -1,
    .client_fd = -1,
    .config = {.port = COMM_TCP_PORT},
};

static int comm_tcp_write_all(int fd, const void *buf, size_t len)
{
    const unsigned char *cursor = (const unsigned char *)buf;
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

static int comm_tcp_queue_push(const void *data, size_t len)
{
    unsigned char *copy = (unsigned char *)malloc(len);
    if (copy == NULL)
    {
        return -1;
    }

    memcpy(copy, data, len);

    pthread_mutex_lock(&g_comm_tcp_ctx.queue_mutex);
    if (g_comm_tcp_ctx.queue_count == COMM_TCP_QUEUE_CAPACITY)
    {
        pthread_mutex_unlock(&g_comm_tcp_ctx.queue_mutex);
        free(copy);
        errno = EAGAIN;
        return -1;
    }

    g_comm_tcp_ctx.queue[g_comm_tcp_ctx.queue_tail].data = copy;
    g_comm_tcp_ctx.queue[g_comm_tcp_ctx.queue_tail].len = len;
    g_comm_tcp_ctx.queue_tail = (g_comm_tcp_ctx.queue_tail + 1U) % COMM_TCP_QUEUE_CAPACITY;
    g_comm_tcp_ctx.queue_count++;
    pthread_cond_signal(&g_comm_tcp_ctx.queue_cond);
    pthread_mutex_unlock(&g_comm_tcp_ctx.queue_mutex);

    return 0;
}

static int comm_tcp_queue_pop(comm_tcp_frame_t *frame)
{
    pthread_mutex_lock(&g_comm_tcp_ctx.queue_mutex);
    while (g_comm_tcp_ctx.running && g_comm_tcp_ctx.queue_count == 0)
    {
        pthread_cond_wait(&g_comm_tcp_ctx.queue_cond, &g_comm_tcp_ctx.queue_mutex);
    }

    if (!g_comm_tcp_ctx.running && g_comm_tcp_ctx.queue_count == 0)
    {
        pthread_mutex_unlock(&g_comm_tcp_ctx.queue_mutex);
        return -1;
    }

    *frame = g_comm_tcp_ctx.queue[g_comm_tcp_ctx.queue_head];
    g_comm_tcp_ctx.queue[g_comm_tcp_ctx.queue_head].data = NULL;
    g_comm_tcp_ctx.queue[g_comm_tcp_ctx.queue_head].len = 0;
    g_comm_tcp_ctx.queue_head = (g_comm_tcp_ctx.queue_head + 1U) % COMM_TCP_QUEUE_CAPACITY;
    g_comm_tcp_ctx.queue_count--;
    pthread_mutex_unlock(&g_comm_tcp_ctx.queue_mutex);

    return 0;
}

static void comm_tcp_close_client(void)
{
    pthread_mutex_lock(&g_comm_tcp_ctx.state_mutex);
    if (g_comm_tcp_ctx.client_fd >= 0)
    {
        close(g_comm_tcp_ctx.client_fd);
        g_comm_tcp_ctx.client_fd = -1;
    }
    pthread_mutex_unlock(&g_comm_tcp_ctx.state_mutex);
}

static int comm_tcp_get_client_fd(void)
{
    int client_fd;

    pthread_mutex_lock(&g_comm_tcp_ctx.state_mutex);
    client_fd = g_comm_tcp_ctx.client_fd;
    pthread_mutex_unlock(&g_comm_tcp_ctx.state_mutex);

    return client_fd;
}

static int comm_tcp_create_server(void)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
        perror("socket tcp");
        return -1;
    }

    int reuse = 1;
    (void)setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(g_comm_tcp_ctx.config.port);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind tcp");
        close(fd);
        return -1;
    }

    if (listen(fd, 4) < 0)
    {
        perror("listen tcp");
        close(fd);
        return -1;
    }

    return fd;
}

static void comm_tcp_sleep_for_poll(void)
{
    struct timespec req;
    req.tv_sec = 0;
    req.tv_nsec = 100000000L;
    (void)nanosleep(&req, NULL);
}

static void comm_tcp_parse_frame(const unsigned char *data, size_t len)
{
    size_t copy_len = len;
    if (copy_len >= 1024U)
    {
        copy_len = 1023U;
    }

    char buffer[1024];
    memcpy(buffer, data, copy_len);
    buffer[copy_len] = '\0';
    printf("[TCP][RX] parsed: %s\n", buffer);
}

static void *comm_tcp_rx_thread_func(void *arg)
{
    (void)arg;

    while (g_comm_tcp_ctx.running)
    {
        int client_fd = comm_tcp_get_client_fd();
        if (client_fd < 0)
        {
            comm_tcp_sleep_for_poll();
            continue;
        }

        unsigned char buffer[1024];
        ssize_t n = recv(client_fd, buffer, sizeof(buffer), 0);
        if (n < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            perror("recv tcp");
            comm_tcp_close_client();
            continue;
        }

        if (n == 0)
        {
            printf("[TCP][RX] client disconnected\n");
            comm_tcp_close_client();
            continue;
        }

        comm_tcp_parse_frame(buffer, (size_t)n);
    }

    return NULL;
}

static void *comm_tcp_tx_thread_func(void *arg)
{
    (void)arg;

    while (g_comm_tcp_ctx.running)
    {
        comm_tcp_frame_t frame;
        if (comm_tcp_queue_pop(&frame) < 0)
        {
            continue;
        }

        int client_fd = comm_tcp_get_client_fd();
        if (client_fd >= 0)
        {
            if (comm_tcp_write_all(client_fd, frame.data, frame.len) < 0)
            {
                perror("send tcp");
                comm_tcp_close_client();
            }
            else
            {
                printf("[TCP][TX] sent %zu bytes\n", frame.len);
            }
        }

        free(frame.data);
    }

    return NULL;
}

static void *comm_tcp_accept_thread_func(void *arg)
{
    (void)arg;

    int fd = comm_tcp_create_server();
    if (fd < 0)
    {
        return NULL;
    }

    pthread_mutex_lock(&g_comm_tcp_ctx.state_mutex);
    g_comm_tcp_ctx.listen_fd = fd;
    pthread_mutex_unlock(&g_comm_tcp_ctx.state_mutex);

    printf("[TCP] server listening on %u\n", g_comm_tcp_ctx.config.port);

    while (g_comm_tcp_ctx.running)
    {
        struct sockaddr_in peer;
        socklen_t peer_len = sizeof(peer);
        int client_fd = accept(fd, (struct sockaddr *)&peer, &peer_len);
        if (client_fd < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            perror("accept tcp");
            continue;
        }

        pthread_mutex_lock(&g_comm_tcp_ctx.state_mutex);
        if (g_comm_tcp_ctx.client_fd >= 0)
        {
            close(g_comm_tcp_ctx.client_fd);
        }
        g_comm_tcp_ctx.client_fd = client_fd;
        pthread_mutex_unlock(&g_comm_tcp_ctx.state_mutex);

        char peer_ip[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET, &peer.sin_addr, peer_ip, sizeof(peer_ip));
        printf("[TCP] client connected from %s:%u\n", peer_ip, ntohs(peer.sin_port));
    }

    close(fd);
    pthread_mutex_lock(&g_comm_tcp_ctx.state_mutex);
    g_comm_tcp_ctx.listen_fd = -1;
    pthread_mutex_unlock(&g_comm_tcp_ctx.state_mutex);
    return NULL;
}

int comm_tcp_init(void)
{
    g_comm_tcp_ctx.running = 1;

    int ret = pthread_create(&g_comm_tcp_ctx.accept_thread, NULL, comm_tcp_accept_thread_func, NULL);
    if (ret != 0)
    {
        fprintf(stderr, "pthread_create tcp accept failed: %s\n", strerror(ret));
        return -1;
    }

    ret = pthread_create(&g_comm_tcp_ctx.rx_thread, NULL, comm_tcp_rx_thread_func, NULL);
    if (ret != 0)
    {
        fprintf(stderr, "pthread_create tcp rx failed: %s\n", strerror(ret));
        return -1;
    }

    ret = pthread_create(&g_comm_tcp_ctx.tx_thread, NULL, comm_tcp_tx_thread_func, NULL);
    if (ret != 0)
    {
        fprintf(stderr, "pthread_create tcp tx failed: %s\n", strerror(ret));
        return -1;
    }

    return 0;
}

int comm_tcp_configure(const comm_tcp_config_t *config)
{
    if (config == NULL || config->port == 0U)
    {
        return 0;
    }

    pthread_mutex_lock(&g_comm_tcp_ctx.state_mutex);
    g_comm_tcp_ctx.config = *config;
    pthread_mutex_unlock(&g_comm_tcp_ctx.state_mutex);

    return 0;
}

int comm_tcp_send(const void *data, size_t len)
{
    if (data == NULL || len == 0)
    {
        errno = EINVAL;
        return -1;
    }

    return comm_tcp_queue_push(data, len);
}