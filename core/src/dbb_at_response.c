#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <iconv.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include "dbb_func.h"
#include "gnss_func.h"






int dbb_capture_response(const char *cmd, char *response, size_t response_size, int timeout_ms)
{
    if (response == NULL || response_size == 0)
    {
        errno = EINVAL;
        return -1;
    }

    response[0] = '\0';

    if (dbb_open_device() < 0)
    {
        return -1;
    }

    if (cmd != NULL)
    {
        char command[256];
        int written = snprintf(command, sizeof(command), "%s\r", cmd);
        if (written < 0 || (size_t)written >= sizeof(command))
        {
            errno = EINVAL;
            return -1;
        }

        if (dbb_write_all(command, (size_t)written) < 0)
        {
            return -1;
        }
    }

    size_t used = 0;
    int idle_ms = 0;
    int total_ms = 0;

    while (total_ms < timeout_ms && used + 1 < response_size)
    {
        fd_set read_fds;
        struct timeval timeout;

        FD_ZERO(&read_fds);
        FD_SET(dbb_ctrl.fd, &read_fds);

        timeout.tv_sec = 0;
        timeout.tv_usec = 200 * 1000;

        int ready = select(dbb_ctrl.fd + 1, &read_fds, NULL, NULL, &timeout);
        if (ready < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            perror("select dbb response");
            return -1;
        }

        if (ready == 0)
        {
            total_ms += 200;
            if (used > 0)
            {
                idle_ms += 200;
                if (idle_ms >= 400)
                {
                    break;
                }
            }
            continue;
        }

        ssize_t n = read(dbb_ctrl.fd, response + used, response_size - 1 - used);
        if (n < 0)
        {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
            {
                continue;
            }
            perror("read dbb response");
            return -1;
        }

        if (n == 0)
        {
            continue;
        }

        used += (size_t)n;
        response[used] = '\0';
        idle_ms = 0;
    }

    return used > 0 ? 0 : -1;
}


int dbb_at_expect(const char *name, const char *cmd, const char *expect)
{
    char response[DBB_MAX_RESPONSE];
    dbb_debug_info("[DBB] %s", name);

    if (dbb_capture_response(cmd, response, sizeof(response), DBB_AT_TIMEOUT_MS) < 0)
    {
        dbb_debug_err("%s failed: no AT response", name);
        return -1;
    }

    dbb_dump_response(response);

    if (strstr(response, "OK") == NULL)
    {
        dbb_debug_err("%s failed: missing OK", name);
        return -1;
    }

    if (expect != NULL && strstr(response, expect) == NULL)
    {
        dbb_debug_err("%s failed: expected '%s'", name, expect);
        return -1;
    }

    return 0;
}


int dbb_wait_for_text(const char *expect, char *response, size_t response_size, int timeout_ms)
{
    if (expect == NULL || expect[0] == '\0')
    {
        errno = EINVAL;
        return -1;
    }

    if (dbb_capture_response(NULL, response, response_size, timeout_ms) < 0)
    {
        return -1;
    }

    if (strstr(response, expect) == NULL)
    {
        return -1;
    }

    return 0;
}



void dbb_dump_response(const char *response)
{
    if (response == NULL || response[0] == '\0')
    {
        return;
    }
    dbb_debug_info("%s", response);
}

