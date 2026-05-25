#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <sys/select.h>
#include <time.h>
#include <termios.h>
#include <unistd.h>

#include "stdint.h"
#include "glob_cfg.h"
#include "src_io.h"
#include "src_tty.h"

#define DBB_DEBUG_INFO (1)
#define DBB_DEBUG_ERR (1)

#define DBB_MAX_RESPONSE 2048
#define DBB_AT_TIMEOUT_MS 2000
#define DBB_WAIT_EVENT_TIMEOUT_MS 30000
#define DBB_MONITOR_INTERVAL_SEC 10

typedef enum
{
    DEV_DBB_IDLE,
    DEV_DBB_INIT,
    DEV_DBB_POWER_ON,
    DEV_DBB_SIM_READY,
    DEV_DBB_CFUN_OK,
    DEV_DBB_CREG_OK,
    DEV_DBB_CNMI_OK,
    DEV_DBB_DSCI_OK,
    DEV_DBB_CGDCONT_OK,
    DEV_DBB_PSDATA_OK,
    DEV_DBB_WAIT_CREGXW,
    DEV_DBB_WAIT_CREV,
    DEV_DBB_WAIT_CIREG,
    DEV_DBB_ONLINE,
    DEV_DBB_POWER_OFF,
} dbb_status_t;

typedef struct
{
    uint8_t enabled;
    int fd;
    dbb_status_t status;
    char device[64];
} dbb_ctrl_t;

static dbb_ctrl_t dbb_ctrl = {
    .enabled = 0,
    .fd = -1,
    .status = DEV_DBB_IDLE,
    .device = DEV_DBB,
};

static void debug_err_dbb(char *fmt, ...)
{
#if DBB_DEBUG_ERR
    va_list args;
    va_start(args, fmt);
    fprintf(stderr, "[DBB][ERROR] ");
    vfprintf(stderr, fmt, args);
    fprintf(stderr, "\n");
    va_end(args);
#endif
}

static void debug_info_dbb(char *fmt, ...)
{
    (void)fmt;
#if DBB_DEBUG_INFO
    va_list args;
    va_start(args, fmt);
    fprintf(stdout, "[DBB][INFO] ");
    vfprintf(stdout, fmt, args);
    fprintf(stdout, "\n");
    va_end(args);
#endif
}

static void dbb_close_device(void)
{
    if (dbb_ctrl.fd >= 0)
    {
        close(dbb_ctrl.fd);
        dbb_ctrl.fd = -1;
    }
}

static int dbb_open_device(void)
{
    if (dbb_ctrl.fd >= 0)
    {
        return 0;
    }

    dbb_ctrl.fd = open(dbb_ctrl.device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (dbb_ctrl.fd < 0)
    {
        perror("open dbb device");
        return -1;
    }

    if (set_opt(dbb_ctrl.fd, 115200, 8, 'N', 1) < 0)
    {
        perror("configure dbb device");
        dbb_close_device();
        return -1;
    }

    tcflush(dbb_ctrl.fd, TCIOFLUSH);
    return 0;
}

static int dbb_capture_response(const char *cmd, char *response, size_t response_size, int timeout_ms)
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
        char command[128];
        int written = snprintf(command, sizeof(command), "%s\r", cmd);
        if (written < 0 || (size_t)written >= sizeof(command))
        {
            errno = EINVAL;
            return -1;
        }

        size_t command_len = strlen(command);
        size_t offset = 0;
        while (offset < command_len)
        {
            ssize_t n = write(dbb_ctrl.fd, command + offset, command_len - offset);
            if (n < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }
                perror("write dbb AT command");
                return -1;
            }
            offset += (size_t)n;
        }
    }

    size_t used = 0;
    int idle_ms = 0;
    int total_ms = 0;
    while (total_ms < timeout_ms && used + 1 < response_size)
    {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(dbb_ctrl.fd, &read_fds);

        struct timeval timeout;
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

static int dbb_wait_for_text(const char *expect, char *response, size_t response_size, int timeout_ms)
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

static void dbb_dump_response(const char *response)
{
    if (response == NULL || response[0] == '\0')
    {
        return;
    }
    debug_info_dbb("%s", response);
}

static int dbb_at_expect(const char *name, const char *cmd, const char *expect)
{
    char response[DBB_MAX_RESPONSE];
    debug_info_dbb("[DBB] %s", name);

    if (dbb_capture_response(cmd, response, sizeof(response), DBB_AT_TIMEOUT_MS) < 0)
    {
        debug_err_dbb("%s failed: no AT response", name);
        return -1;
    }

    dbb_dump_response(response);

    if (strstr(response, "OK") == NULL)
    {
        debug_err_dbb("%s failed: missing OK", name);
        return -1;
    }

    if (expect != NULL && strstr(response, expect) == NULL)
    {
        debug_err_dbb("%s failed: expected '%s'", name, expect);
        return -1;
    }

    return 0;
}

static int dbb_boot_sequence_ok(void)
{
    char response[DBB_MAX_RESPONSE];

    if (dbb_at_expect("query SIM", "AT+CIMI", NULL) < 0)
    {
        debug_err_dbb("CIMI query failed");
        return -1;
    }

    if (dbb_at_expect("activate SIM", "AT+CFUN=5", NULL) < 0)
    {
        debug_err_dbb("CFUN=5 failed");
        return -1;
    }

    if (dbb_at_expect("enable network registration report", "AT+CREG=1", NULL) < 0)
    {
        debug_err_dbb("CREG=1 failed");
        return -1;
    }

    if (dbb_at_expect("enable sms report", "AT+CNMI=2,2,0,1,0", NULL) < 0)
    {
        debug_err_dbb("CNMI=2,2,0,1,0 failed");
        return -1;
    }

    if (dbb_at_expect("enable voice report", "AT+DSCI=1", NULL) < 0)
    {
        debug_err_dbb("DSCI=1 failed");
        return -1;
    }

    if (dbb_at_expect("set packet data context", "AT+CGDCONT=1,\"IP\"", NULL) < 0)
    {
        debug_err_dbb("CGDCONT failed");
        return -1;
    }

    if (dbb_at_expect("enable AT ip transport", "AT^PSDATA", NULL) < 0)
    {
        debug_err_dbb("PSDATA failed");
        return -1;
    }

    if (dbb_wait_for_text("+CREGXW: 1", response, sizeof(response), DBB_WAIT_EVENT_TIMEOUT_MS) < 0)
    {
        debug_err_dbb("wait +CREGXW: 1 failed");
        return -1;
    }
    dbb_dump_response(response);

    if (dbb_wait_for_text("+CREV: ME PDN ACT 1", response, sizeof(response), DBB_WAIT_EVENT_TIMEOUT_MS) < 0)
    {
        debug_err_dbb("wait +CREV: ME PDN ACT 1 failed");
        return -1;
    }
    dbb_dump_response(response);

    if (dbb_wait_for_text("+CIREG: 1", response, sizeof(response), DBB_WAIT_EVENT_TIMEOUT_MS) < 0)
    {
        debug_err_dbb("wait +CIREG: 1 failed");
        return -1;
    }
    dbb_dump_response(response);

    return 0;
}

static int dbb_cfg_once()
{
    dbb_at_expect("CFG", "AT^PLMNSELMODE=1", NULL);
    dbb_at_expect("CFG", "AT^AUTHKEY=\"00112233445566778899AABBCCDDEEFF\"", NULL);
    dbb_at_expect("CFG", "AT^AUTHOPC=\"000102030405060708090A0B0C0D0E0F\"", NULL);
    dbb_at_expect("CFG", "AT^DNN=\"CSCNNET\"", NULL);
    dbb_at_expect("CFG", "AT^MISWITCH=1", NULL);
    dbb_at_expect("CFG", "AT^RETXFORBIDDENSWITCH=0", NULL);
    dbb_at_expect("CFG", "AT^WORKMODE=0,0", NULL);
    dbb_at_expect("CFG", "AT^MESTYPE=1", NULL);
    dbb_at_expect("CFG", "AT^TXPOWER=330", NULL);
    dbb_at_expect("CFG", "AT^SELFTESTSWITCH=0", NULL);
    dbb_at_expect("CFG", "AT^LFLOWXWSWITCH=0", NULL);
    dbb_at_expect("CFG", "AT^REESTDEREGCLOSE=1", NULL);
    dbb_at_expect("CFG", "AT^AUTODEREGREGCLOSE=1", NULL);
    dbb_at_expect("CFG", "AT^MISWITCH=1", NULL);
    dbb_at_expect("CFG", "AT^BMCARDSWITCH=1", NULL);
}

void dbb_online_func(void)
{
    char response[DBB_MAX_RESPONSE];
    switch (dbb_ctrl.status)
    {
    case DEV_DBB_IDLE:
        sleep(1);
        break;
    case DEV_DBB_INIT:
        if (dbb_open_device() == 0)
        {
            dbb_ctrl.status = DEV_DBB_POWER_ON;
        }
        else
        {
            sleep(1);
        }
        break;
    case DEV_DBB_POWER_ON:
        if (dbb_at_expect("query SIM", "AT+CIMI", "OK") == 0)
        {
            dbb_ctrl.status = DEV_DBB_SIM_READY;
            debug_info_dbb("dbb module is online");
        }
        break;
    case DEV_DBB_SIM_READY:
        if (dbb_at_expect("activate SIM", "AT+CFUN=5", "OK") == 0)
        {
            dbb_ctrl.status = DEV_DBB_CFUN_OK;
        }
        break;
    case DEV_DBB_CFUN_OK:
        if (dbb_at_expect("enable network registration report", "AT+CREG=1", "OK") == 0)
        {
            dbb_ctrl.status = DEV_DBB_CREG_OK;
        }
        break;
    case DEV_DBB_CREG_OK:
        if (dbb_at_expect("enable sms report", "AT+CNMI=2,2,0,1,0", "OK") == 0)
        {
            dbb_ctrl.status = DEV_DBB_CNMI_OK;
        }
        break;
    case DEV_DBB_CNMI_OK:
        if (dbb_at_expect("enable voice report", "AT+DSCI=1", "OK") == 0)
        {
            dbb_ctrl.status = DEV_DBB_DSCI_OK;
        }
        break;
    case DEV_DBB_DSCI_OK:
        if (dbb_at_expect("set packet data context", "AT+CGDCONT=1,\"IP\"", "OK") == 0)
        {
            dbb_ctrl.status = DEV_DBB_CGDCONT_OK;
        }
        break;
    case DEV_DBB_CGDCONT_OK:
        if (dbb_at_expect("enable AT ip transport", "AT^PSDATA", "OK") == 0)
        {
            if (dbb_wait_for_text("+CREGXW: 1", response, sizeof(response), DBB_WAIT_EVENT_TIMEOUT_MS) < 0)
            {
                debug_err_dbb("wait +CREGXW: 1 failed");
                return;
            }
            dbb_dump_response(response);

            if (dbb_wait_for_text("+CREV: ME PDN ACT 1", response, sizeof(response), DBB_WAIT_EVENT_TIMEOUT_MS) < 0)
            {
                debug_err_dbb("wait +CREV: ME PDN ACT 1 failed");
                return;
            }
            dbb_dump_response(response);

            if (dbb_wait_for_text("+CIREG: 1", response, sizeof(response), DBB_WAIT_EVENT_TIMEOUT_MS) < 0)
            {
                debug_err_dbb("wait +CIREG: 1 failed");
                return;
            }
            dbb_dump_response(response);
            dbb_ctrl.status = DEV_DBB_ONLINE;
        }
        break;
    case DEV_DBB_ONLINE:
        for (int i = 0; i < DBB_MONITOR_INTERVAL_SEC && dbb_ctrl.enabled; ++i)
        {
            sleep(1);
        }
        break;
    case DEV_DBB_POWER_OFF:
        dbb_ctrl.status = DEV_DBB_IDLE;
        break;
    default:
        dbb_ctrl.status = DEV_DBB_INIT;
        break;
    }
}

void *dbb_thread_func(void *arg)
{
    (void)arg;
    sleep(5); // Sleep for 5 seconds before starting DBB operations
    dbb_ctrl.enabled = 1;
    dbb_ctrl.status = DEV_DBB_INIT;
    dbb_close_device();

    while (1)
    {
        dbb_online_func();
        struct timespec delay = {0, 1000 * 1000};
        nanosleep(&delay, NULL);
    }

    return NULL;
}