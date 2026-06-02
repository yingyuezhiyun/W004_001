#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <stdarg.h>

#include "stdint.h"
#include "glob_cfg.h"
#include "src_io.h"
#include "src_tty.h"

#define ETH_4G_DEBUG_INFO (0)
#define ETH_4G_DEBUG_ERR (1)

#define ETH_4G_DEFAULT_APN "internet"
#define ETH_4G_DEFAULT_NET_IF "usb0"
#define ETH_4G_DEFAULT_PING_HOST "1.1.1.1"
#define ETH_4G_MAX_RESPONSE 2048
#define ETH_4G_MONITOR_INTERVAL_SEC 10
#define ETH_4G_STARTUP_RETRY_COUNT 3

typedef enum
{
    DEV_4G_IDLE,
    DEV_4G_INIT,
    DEV_4G_POWER_ON,
    DEV_4G_AT_READY,
    DEV_4G_SIM_READY,
    DEV_4G_SIGNAL_OK,
    DEV_4G_CFUN_OK,
    DEV_4G_REGISTERED,
    DEV_4G_APN_OK,
    DEV_4G_ATTACHED,
    DEV_4G_IFACE_UP,
    DEV_4G_LINK_UP,
    DEV_4G_DHCP_OK,
    DEV_4G_IP_OK,
    DEV_4G_ONLINE,
    DEV_4G_POWER_OFF,
} eth_4g_Status_t;

typedef struct
{
    uint8_t enabled;
    int fd;
    eth_4g_Status_t status;
    char device[64];
    char apn[64];
    char net_if[32];
    char ping_host[64];
} eth_4g_ctrl_t;

eth_4g_ctrl_t eth_4g_ctrl = {
    .enabled = 0,
    .status = DEV_4G_IDLE,
    .fd = -1,
    .apn = ETH_4G_DEFAULT_APN,
    .net_if = ETH_4G_DEFAULT_NET_IF,
    .ping_host = ETH_4G_DEFAULT_PING_HOST,
    .device = DEV_4G};

void debug_err_4g(char *fmt, ...)
{
#if ETH_4G_DEBUG_ERR
    va_list args;
    va_start(args, fmt);
    fprintf(stderr, "[4G][ERROR] ");
    vfprintf(stderr, fmt, args);
    fprintf(stderr, "\n");
    va_end(args);
#endif
}

void debug_info_4g(char *fmt, ...)
{
#if ETH_4G_DEBUG_INFO
    va_list args;
    va_start(args, fmt);
    fprintf(stdout, "[4G][INFO] ");
    vfprintf(stdout, fmt, args);
    fprintf(stdout, "\n");
    va_end(args);
#endif
}

/// @brief Load environment variables for 4G configuration
/// @param
static void eth_4g_load_env(void)
{
    const char *value = getenv("APN");
    if (value != NULL && value[0] != '\0')
    {
        snprintf(eth_4g_ctrl.apn, sizeof(eth_4g_ctrl.apn), "%s", value);
    }

    value = getenv("NET_IF");
    if (value != NULL && value[0] != '\0')
    {
        snprintf(eth_4g_ctrl.net_if, sizeof(eth_4g_ctrl.net_if), "%s", value);
    }

    value = getenv("PING_HOST");
    if (value != NULL && value[0] != '\0')
    {
        snprintf(eth_4g_ctrl.ping_host, sizeof(eth_4g_ctrl.ping_host), "%s", value);
    }

    value = getenv("AT_PORT");
    if (value != NULL && value[0] != '\0')
    {
        snprintf(eth_4g_ctrl.device, sizeof(eth_4g_ctrl.device), "%s", value);
    }
}

/// @brief Close the 4G device
/// @param
static void eth_4g_close_device(void)
{
    if (eth_4g_ctrl.fd >= 0)
    {
        close(eth_4g_ctrl.fd);
        eth_4g_ctrl.fd = -1;
    }
}

/// @brief Open the 4G device
/// @param
/// @return
static int eth_4g_open_device(void)
{
    if (eth_4g_ctrl.fd >= 0)
    {
        return 0;
    }

    eth_4g_ctrl.fd = open(eth_4g_ctrl.device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (eth_4g_ctrl.fd < 0)
    {
        perror("open 4G device");
        return -1;
    }

    if (set_opt(eth_4g_ctrl.fd, 115200, 8, 'N', 1) < 0)
    {
        perror("configure 4G device");
        eth_4g_close_device();
        return -1;
    }

    tcflush(eth_4g_ctrl.fd, TCIOFLUSH);
    return 0;
}

/// @brief Send an AT command and capture the response with timeout
/// @param cmd The AT command to send (without trailing CR)
/// @param response Buffer to store the response (null-terminated)
/// @param response_size Size of the response buffer
/// @param timeout_ms Timeout in milliseconds
/// @return 0 on success, -1 on failure
static int eth_4g_capture_at_response(const char *cmd, char *response, size_t response_size, int timeout_ms)
{
    if (response == NULL || response_size == 0)
    {
        errno = EINVAL;
        return -1;
    }

    response[0] = '\0';

    if (eth_4g_open_device() < 0)
    {
        return -1;
    }

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
        ssize_t n = write(eth_4g_ctrl.fd, command + offset, command_len - offset);
        if (n < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            perror("write 4G AT command");
            return -1;
        }
        offset += (size_t)n;
    }

    size_t used = 0;
    int idle_ms = 0;
    int total_ms = 0;
    while (total_ms < timeout_ms && used + 1 < response_size)
    {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(eth_4g_ctrl.fd, &read_fds);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 200 * 1000;

        int ready = select(eth_4g_ctrl.fd + 1, &read_fds, NULL, NULL, &timeout);
        if (ready < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            perror("select 4G AT response");
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

        ssize_t n = read(eth_4g_ctrl.fd, response + used, response_size - 1 - used);
        if (n < 0)
        {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
            {
                continue;
            }
            perror("read 4G AT response");
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

/// @brief Dump the response from the 4G device
/// @param response The response to dump
static void eth_4g_dump_response(const char *response)
{
    if (response == NULL || response[0] == '\0')
    {
        return;
    }
    debug_info_4g("%s", response);
}

/// @brief Expect a specific response to an AT command
/// @param name The name of the command
/// @param cmd The AT command to send
/// @param expect The expected response
/// @return 0 on success, -1 on failure
static int eth_4g_at_expect(const char *name, const char *cmd, const char *expect)
{
    char response[ETH_4G_MAX_RESPONSE];
    debug_info_4g("[4G] %s", name);

    if (eth_4g_capture_at_response(cmd, response, sizeof(response), 2000) < 0)
    {
        debug_err_4g("%s failed: no AT response", name);
        return -1;
    }

    eth_4g_dump_response(response);

    if (strstr(response, "OK") == NULL)
    {
        debug_err_4g("%s failed: missing OK", name);
        return -1;
    }

    if (expect != NULL && strstr(response, expect) == NULL)
    {
        debug_err_4g("%s failed: expected '%s'", name, expect);
        return -1;
    }

    return 0;
}

/// @brief Check if the SIM is ready
/// @param
/// @return 0 if ready, -1 otherwise
static int eth_4g_sim_ready(void)
{
    char response[ETH_4G_MAX_RESPONSE];
    if (eth_4g_capture_at_response("AT+CPIN?", response, sizeof(response), 2000) < 0)
    {
        debug_err_4g("CPIN query failed");
        return -1;
    }

    eth_4g_dump_response(response);
    if (strstr(response, "+CPIN: READY") == NULL)
    {
        debug_err_4g("SIM is not ready");
        return -1;
    }

    return 0;
}

/// @brief Check the signal quality of the 4G connection
/// @param
/// @return 0 on success, -1 on failure
static int eth_4g_signal_quality(void)
{
    char response[ETH_4G_MAX_RESPONSE];
    if (eth_4g_capture_at_response("AT+CSQ", response, sizeof(response), 2000) < 0)
    {
        debug_err_4g("CSQ query failed");
        return -1;
    }

    eth_4g_dump_response(response);
    if (strstr(response, "+CSQ: 99,") != NULL)
    {
        debug_err_4g("signal quality is unknown or unavailable");
    }

    return strstr(response, "OK") != NULL ? 0 : -1;
}

/// @brief Check if the 4G module is registered on the network
/// @param
/// @return 0 if registered, -1 otherwise
static int eth_4g_registration_ok(void)
{
    char response[ETH_4G_MAX_RESPONSE];
    if (eth_4g_capture_at_response("AT+CEREG?", response, sizeof(response), 2000) < 0)
    {
        debug_err_4g("CEREG query failed");
        return -1;
    }

    eth_4g_dump_response(response);

    if (strstr(response, "+CEREG: 0,1") != NULL ||
        strstr(response, "+CEREG: 0,5") != NULL ||
        strstr(response, "+CEREG: 1,1") != NULL ||
        strstr(response, "+CEREG: 1,5") != NULL)
    {
        return 0;
    }

    debug_err_4g("module is not registered on the network");
    return -1;
}

/// @brief Check if the 4G module has attached to the packet network
/// @param
/// @return 0 if attached, -1 otherwise
static int eth_4g_packet_attached(void)
{
    char response[ETH_4G_MAX_RESPONSE];
    if (eth_4g_capture_at_response("AT+CGATT?", response, sizeof(response), 2000) < 0)
    {
        debug_err_4g("CGATT query failed");
        return -1;
    }

    eth_4g_dump_response(response);
    if (strstr(response, "+CGATT: 1") == NULL)
    {
        debug_err_4g("packet service is not attached");
        return -1;
    }

    return 0;
}

/// @brief Set the APN for the 4G connection
/// @param
/// @return 0 on success, -1 on failure
static int eth_4g_set_apn(void)
{
    char command[128];
    int written = snprintf(command, sizeof(command), "AT+CGDCONT=1,\"IP\",\"%s\"", eth_4g_ctrl.apn);
    if (written < 0 || (size_t)written >= sizeof(command))
    {
        debug_err_4g("APN is too long");
        return -1;
    }

    return eth_4g_at_expect("set APN", command, "OK");
}

/// @brief Bring the 4G network interface up
/// @param
/// @return 0 on success, -1 on failure
static int eth_4g_bring_iface_up(void)
{
    char command[128];
    int written = snprintf(command, sizeof(command),
                           "ip link set %s up >/dev/null 2>&1 || ifconfig %s up >/dev/null 2>&1",
                           eth_4g_ctrl.net_if,
                           eth_4g_ctrl.net_if);
    if (written < 0 || (size_t)written >= sizeof(command))
    {
        return -1;
    }

    return system(command) == 0 ? 0 : -1;
}

/// @brief Wait for the 4G network interface to appear
/// @param timeout_sec Timeout in seconds
/// @return 0 on success, -1 on failure
static int eth_4g_wait_for_iface(int timeout_sec)
{
    char path[96];
    int written = snprintf(path, sizeof(path), "/sys/class/net/%s", eth_4g_ctrl.net_if);
    if (written < 0 || (size_t)written >= sizeof(path))
    {
        return -1;
    }

    for (int i = 0; i < timeout_sec; ++i)
    {
        if (access(path, F_OK) == 0)
        {
            return 0;
        }
        sleep(1);
    }

    return -1;
}

/// @brief Start the DHCP client for the 4G interface
/// @param
/// @return 0 on success, -1 on failure
static int eth_4g_start_dhcp(void)
{
    char command[256];

    if (system("command -v udhcpc >/dev/null 2>&1") == 0)
    {
        int written = snprintf(command, sizeof(command),
                               "udhcpc -i %s -n -q -t 10 -T 3 >/dev/null 2>&1",
                               eth_4g_ctrl.net_if);
        if (written < 0 || (size_t)written >= sizeof(command))
        {
            return -1;
        }
        return system(command) == 0 ? 0 : -1;
    }

    if (system("command -v dhclient >/dev/null 2>&1") == 0)
    {
        int written = snprintf(command, sizeof(command),
                               "dhclient %s >/dev/null 2>&1",
                               eth_4g_ctrl.net_if);
        if (written < 0 || (size_t)written >= sizeof(command))
        {
            return -1;
        }
        return system(command) == 0 ? 0 : -1;
    }

    debug_err_4g("no DHCP client found");
    return -1;
}

/// @brief Wait for an IP address to be assigned to the 4G interface
/// @param timeout_sec Timeout in seconds
/// @return 0 on success, -1 on failure
static int eth_4g_wait_for_ip(int timeout_sec)
{
    char command[256];
    for (int i = 0; i < timeout_sec; ++i)
    {
        int written = snprintf(command, sizeof(command),
                               "command -v ip >/dev/null 2>&1 && ip addr show dev %s | grep -q 'inet '",
                               eth_4g_ctrl.net_if);
        if (written < 0 || (size_t)written >= sizeof(command))
        {
            return -1;
        }

        if (system(command) == 0)
        {
            return 0;
        }

        written = snprintf(command, sizeof(command),
                           "command -v ifconfig >/dev/null 2>&1 && ifconfig %s | grep -q 'inet '",
                           eth_4g_ctrl.net_if);
        if (written < 0 || (size_t)written >= sizeof(command))
        {
            return -1;
        }

        if (system(command) == 0)
        {
            return 0;
        }

        sleep(1);
    }

    return -1;
}

/// @brief Test internet connectivity
/// @param
/// @return 0 on success, -1 on failure
static int eth_4g_test_internet(void)
{
    char command[256];
    int written = snprintf(command, sizeof(command),
                           "command -v ping >/dev/null 2>&1 && ping -c 3 -W 2 %s >/dev/null 2>&1",
                           eth_4g_ctrl.ping_host);
    if (written < 0 || (size_t)written >= sizeof(command))
    {
        return -1;
    }

    if (system(command) == 0)
    {
        return 0;
    }

    debug_err_4g("connectivity test failed");
    return -1;
}

/// @brief Show the status of the 4G interface
/// @param
static void eth_4g_show_status(void)
{
    char command[256];

    int written = snprintf(command, sizeof(command), "command -v ip >/dev/null 2>&1 && ip addr show dev %s", eth_4g_ctrl.net_if);
    if (written > 0 && (size_t)written < sizeof(command))
    {
        system(command);
    }

    written = snprintf(command, sizeof(command), "command -v ip >/dev/null 2>&1 && ip route show default");
    if (written > 0 && (size_t)written < sizeof(command))
    {
        system(command);
    }
}

/// @brief Check if the 4G interface is in a healthy state
/// @param
/// @return 0 if healthy, -1 otherwise
static int eth_4g_status_ok(void)
{
    if (eth_4g_sim_ready() < 0)
    {
        return -1;
    }

    if (eth_4g_registration_ok() < 0)
    {
        return -1;
    }

    if (eth_4g_packet_attached() < 0)
    {
        return -1;
    }

    // if (eth_4g_wait_for_ip(1) < 0)
    // {
    //     return -1;
    // }
    // if (eth_4g_test_internet() < 0)
    // {
    //     return -1;
    // }

    return 0;
}

int8_t eth_4g_power_on(void)
{
    if (io_set_output_level(DEV_4G_IO_RESET, 0) < 0)
    {
        perror("release 4G reset");
        return -1;
    }
    if (io_set_output_level(DEV_4G_IO_POWER, 1) < 0)
    {
        perror("set 4G power high");
        return -1;
    }
    if (io_set_output_level(DEV_USB_IO_SW1, 1) < 0)
    {
        perror("set USB switch for 4G");
        return -1;
    }
    sleep(2);
    return 0;
}

/// @brief Disable the 4G interface
/// @return 0 on success, -1 on failure
int8_t eth_4g_power_off()
{
    eth_4g_close_device();
    io_set_output_level(DEV_4G_IO_RESET, 1);
    io_set_output_level(DEV_4G_IO_POWER, 0);
    io_set_output_level(DEV_USB_IO_SW1, 0);
    return 0;
}

/// @brief Attempt a soft recovery of the 4G module
/// @param
/// @return 0 on success, -1 on failure
static int eth_4g_soft_recover(void)
{
    if (eth_4g_open_device() < 0)
    {
        return -1;
    }
    debug_info_4g("trying soft recovery");
    if (eth_4g_at_expect("modem standby", "AT+CFUN=0", "OK") < 0)
    {
        return -1;
    }
    sleep(2);
    eth_4g_ctrl.status = DEV_4G_AT_READY;
    return 0;
}

/// @brief Attempt a hard recovery of the 4G module
/// @param
/// @return 0 on success, -1 on failure
static int eth_4g_hard_recover(void)
{
    debug_info_4g("trying hard recovery");
    if (eth_4g_power_off() < 0)
    {
        return -1;
    }
    sleep(2);
    eth_4g_ctrl.status = DEV_4G_INIT;
    return 0;
}

void eth_4g_online_func()
{
    if (eth_4g_ctrl.enabled == 0 && eth_4g_ctrl.status != DEV_4G_IDLE)
    {
        eth_4g_ctrl.status = DEV_4G_POWER_OFF;
    }
    else if (eth_4g_ctrl.enabled == 1 && eth_4g_ctrl.status == DEV_4G_IDLE)
    {
        eth_4g_ctrl.status = DEV_4G_INIT;
    }
    switch (eth_4g_ctrl.status)
    {
    case DEV_4G_IDLE:
        set_led(DEV_4G_LED, 0);
        sleep(1);
        break;
    case DEV_4G_INIT:
        eth_4g_load_env();
        if (eth_4g_power_on() == 0)
        {
            set_led(DEV_4G_LED, 0);
            eth_4g_ctrl.status = DEV_4G_POWER_ON;
            eth_4g_close_device();
        }
        else
        {
            debug_err_4g("power on failed");
        }
        break;
    case DEV_4G_POWER_ON:
        eth_4g_open_device();
        if (eth_4g_at_expect("AT", "AT", "OK") >= 0 && eth_4g_at_expect("echo off", "ATE0", "OK") >= 0)
        {
            eth_4g_ctrl.status = DEV_4G_AT_READY;
            debug_info_4g("module is responsive");
        }
        break;
    case DEV_4G_AT_READY:
        if (eth_4g_sim_ready() == 0)
        {
            eth_4g_ctrl.status = DEV_4G_SIM_READY;
            debug_info_4g("SIM is ready");
        }
        break;
    case DEV_4G_SIM_READY:
        if (eth_4g_signal_quality() == 0)
        {
            eth_4g_ctrl.status = DEV_4G_SIGNAL_OK;
            debug_info_4g("signal quality is OK");
        }
        break;
    case DEV_4G_SIGNAL_OK:
        if (eth_4g_at_expect("full functionality", "AT+CFUN=1", "OK") >= 0 &&
            eth_4g_at_expect("confirm functionality", "AT+CFUN?", "+CFUN: 1") >= 0)
        {
            eth_4g_ctrl.status = DEV_4G_CFUN_OK;
            debug_info_4g("module is in full functionality mode");
        }
        break;
    case DEV_4G_CFUN_OK:
        if (eth_4g_registration_ok() == 0)
        {
            eth_4g_ctrl.status = DEV_4G_REGISTERED;
            debug_info_4g("module is registered on the network");
        }
        break;
    case DEV_4G_REGISTERED:
        if (eth_4g_set_apn() == 0)
        {
            eth_4g_ctrl.status = DEV_4G_APN_OK;
            debug_info_4g("APN set to '%s'", eth_4g_ctrl.apn);
        }
        break;
    case DEV_4G_APN_OK:
        if (eth_4g_at_expect("attach packet service", "AT+CGATT=1", "OK") == 0 &&
            eth_4g_packet_attached() == 0)
        {
            eth_4g_ctrl.status = DEV_4G_ATTACHED;
            debug_info_4g("packet service is attached");
        }
        break;
    case DEV_4G_ATTACHED:
        if (eth_4g_bring_iface_up() == 0)
        {
            eth_4g_ctrl.status = DEV_4G_IFACE_UP;
            debug_info_4g("network interface %s is up", eth_4g_ctrl.net_if);
        }
        break;
    case DEV_4G_IFACE_UP:
        if (eth_4g_wait_for_iface(10) == 0)
        {
            eth_4g_ctrl.status = DEV_4G_LINK_UP;
            debug_info_4g("network interface %s is ready", eth_4g_ctrl.net_if);
        }
        break;
    case DEV_4G_LINK_UP:
        if (eth_4g_start_dhcp() >= 0)
        {
            eth_4g_ctrl.status = DEV_4G_DHCP_OK;
            debug_info_4g("DHCP started on %s", eth_4g_ctrl.net_if);
        }
        break;
    case DEV_4G_DHCP_OK:
        if (eth_4g_wait_for_ip(30) == 0)
        {
            eth_4g_show_status();
            eth_4g_ctrl.status = DEV_4G_ONLINE;
            debug_info_4g("module is online with IP address");
        }
        break;
    case DEV_4G_IP_OK:
        if (eth_4g_wait_for_ip(30) == 0)
        {
            eth_4g_show_status();
            eth_4g_ctrl.status = DEV_4G_ONLINE;
            debug_info_4g("module is online with IP address");
            set_led(DEV_4G_LED, 1);
        }
        break;
    case DEV_4G_ONLINE:
        for (int i = 0; i < ETH_4G_MONITOR_INTERVAL_SEC && eth_4g_ctrl.enabled; ++i)
        {
            sleep(1);
        }
        if (eth_4g_status_ok() < 0)
        {
            debug_info_4g("status degraded, attempting recovery");
            eth_4g_soft_recover();
        }
        break;
    case DEV_4G_POWER_OFF:
        eth_4g_power_off();
        eth_4g_ctrl.status = DEV_4G_IDLE;
        break;
    default:
        break;
    }
    usleep(1000);    
}

/// @brief Thread function for the 4G monitoring thread
/// @param arg
/// @return
void *eth_4g_thread_func(void *arg)
{
    (void)arg;
    eth_4g_ctrl.enabled = 1;
    while (1)
    {
        eth_4g_online_func();
    }
    eth_4g_power_off();
    return NULL;
}
