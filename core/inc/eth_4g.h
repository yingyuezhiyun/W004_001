#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

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
    typedef enum
    {
        ETH_4G_PRINT_OFF,
        ETH_4G_PRINT_ON,
    } eth4g_PrintType_t;
    typedef struct
    {
        uint8_t enabled;
        int fd;
        eth_4g_Status_t status;
        char device[64];
        char apn[64];
        char net_if[32];
        char ping_host[64];
        struct
        {
            eth4g_PrintType_t info;
            eth4g_PrintType_t err;
        } print;
    } eth_4g_ctrl_t;

    extern eth_4g_ctrl_t eth_4g_ctrl;

#ifdef __cplusplus
}
#endif