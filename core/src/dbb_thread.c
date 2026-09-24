#define _POSIX_C_SOURCE 200809L

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

#include "glob_cfg.h"
#include "gnss_func.h"
#include "src_io.h"
#include "src_tty.h"
#include "src_led.h"

dbb_ctrl_t dbb_ctrl = {
    .enabled = 0,
    .fd = -1,
    .status = DEV_DBB_IDLE,
    .device = DEV_DBB,
    .uplink_interval_sec = 30,
    .last_uplink_ts = 0,
    .sms_send_once_done = 0,
    .downlink_raw_len = 0,
    .downlink_text = {0},
    .cfg_once_done = 0,
};

void dbb_debug_err(const char *fmt, ...)
{
#if DBB_DEBUG_ERR
    va_list args;
    va_start(args, fmt);
    fprintf(stderr, "[DBB][ERROR] ");
    vfprintf(stderr, fmt, args);
    fprintf(stderr, "\n");
    va_end(args);
#else
    (void)fmt;
#endif
}

void dbb_debug_info(const char *fmt, ...)
{
#if DBB_DEBUG_INFO
    va_list args;
    va_start(args, fmt);
    fprintf(stdout, "[DBB][INFO] ");
    vfprintf(stdout, fmt, args);
    fprintf(stdout, "\n");
    va_end(args);
#else
    (void)fmt;
#endif
}

void dbb_close_device(void)
{
    if (dbb_ctrl.fd >= 0)
    {
        close(dbb_ctrl.fd);
        dbb_ctrl.fd = -1;
    }
}

int dbb_open_device(void)
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

int dbb_write_all(const char *buf, size_t count)
{
    size_t offset = 0;

    while (offset < count)
    {
        ssize_t n = write(dbb_ctrl.fd, buf + offset, count - offset);
        if (n < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            perror("write dbb device");
            return -1;
        }
        offset += (size_t)n;
    }

    return 0;
}

static int dbb_cfg_once(void)
{
    if (dbb_ctrl.cfg_once_done)
    {
        return 0;
    }

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

    dbb_ctrl.cfg_once_done = 1;
    return 0;
}

static int dbb_cfg_broadcast_once(void)
{
    if (dbb_ctrl.cfg_once_done)
    {
        return 0;
    }

    if (dbb_at_expect("disable satellite service", "AT^MISWITCH=0", "OK") < 0 ||
        dbb_at_expect("disable BM card", "AT^BMCARDSWITCH=0", "OK") < 0 ||
        dbb_at_expect("enable dummy USIM", "AT^DUMMYUSIM=1", "OK") < 0 /* ||
        dbb_at_expect("restart DBB for broadcast mode", "AT+CFUN=1", "OK") < 0 */
    )
    {
        return -1;
    }
    dbb_at_expect("enable BBIC reset", "AT^BBICRSTSW=1", NULL);
    sleep(1);
    if (dbb_at_expect("enable SSR broadcast report", "AT^SSRINFOXW=1", "OK") < 0 ||
        dbb_at_expect("restart DBB after broadcast setup", "AT+CFUN=1", "OK") < 0)
    {
        return -1;
    }
    sleep(5);

    dbb_ctrl.cfg_once_done = 1;
    return 0;
}

static void dbb_online_service(void)
{
    char response[DBB_MAX_RESPONSE];
    time_t now = time(NULL);
    static const uint8_t demo_uplink_raw[] = {
        'D', 'B', 'B', '-', 'U', 'P', '-', 'D', 'E', 'M', 'O'};

    if (dbb_capture_response(NULL, response, sizeof(response), DBB_MONITOR_INTERVAL_SEC * 1000) == 0)
    {
        dbb_dump_response(response);
        dbb_handle_urc_blob(response);
    }

    if (dbb_ctrl.last_uplink_ts == 0 ||
        now - dbb_ctrl.last_uplink_ts >= dbb_ctrl.uplink_interval_sec)
    {
        if (dbb_send_uplink(demo_uplink_raw, sizeof(demo_uplink_raw), DBB_DATA_CODEC_BASE64) == 0)
        {
            dbb_ctrl.last_uplink_ts = now;
        }
    }

    if (!dbb_ctrl.sms_send_once_done && dbb_ctrl.sms_target[0] != '\0' && dbb_ctrl.sms_text[0] != '\0')
    {
        if (dbb_send_sms(dbb_ctrl.sms_target, dbb_ctrl.sms_text) == 0)
        {
            dbb_ctrl.sms_send_once_done = 1;
        }
    }
}

static void dbb_online_broadcast_service(void)
{
    char response[DBB_MAX_RESPONSE];
    if (dbb_capture_response(NULL, response, sizeof(response), DBB_MONITOR_INTERVAL_SEC * 1000) == 0)
    {
        dbb_dump_response(response);
        dbb_handle_urc_blob(response);
    }
}

void dbb_online_func(void)
{
    char response[DBB_MAX_RESPONSE];

    switch (dbb_ctrl.status)
    {
    case DEV_DBB_IDLE:
        set_led(DEV_DBB_LED, 0);
        sleep(1);
        break;
    case DEV_DBB_INIT:
        dbb_open_device();
        if (DBB_RECEIVE_MODE == DBB_RECEIVE_MODE_BROADCAST)
        {
            if (dbb_cfg_broadcast_once() == 0)
            {
                dbb_ctrl.status = DEV_DBB_ONLINE_BROADCAST;
                set_led(DEV_DBB_LED, 1);
            }
            else
            {
                sleep(1);
            }
            break;
        }
        dbb_ctrl.status = DEV_DBB_POWER_ON;
        set_led(DEV_DBB_LED, 0);
        break;
    case DEV_DBB_POWER_ON:
        if (dbb_at_expect("query SIM", "AT+CIMI", "OK") == 0)
        {
            dbb_ctrl.status = DEV_DBB_SIM_READY;
            dbb_debug_info("dbb module is online");
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
        if (dbb_at_expect("enable AT ip transport", "AT^PSDATA=2", "OK") == 0)
        {
            // if (dbb_wait_for_text("+CREGXW: 1", response, sizeof(response), DBB_WAIT_EVENT_TIMEOUT_MS) < 0)
            // {
            //     dbb_debug_err("wait +CREGXW: 1 failed");
            //     return;
            // }
            // dbb_dump_response(response);

            // if (dbb_wait_for_text("+CREV: ME PDN ACT 1", response, sizeof(response), DBB_WAIT_EVENT_TIMEOUT_MS) < 0)
            // {
            //     dbb_debug_err("wait +CREV: ME PDN ACT 1 failed");
            //     return;
            // }
            // dbb_dump_response(response);

            // if (dbb_wait_for_text("+CIREG: 1", response, sizeof(response), DBB_WAIT_EVENT_TIMEOUT_MS) < 0)
            // {
            //     dbb_debug_err("wait +CIREG: 1 failed");
            //     return;
            // }
            // dbb_dump_response(response);
            dbb_ctrl.status = DEV_DBB_WAIT_CREGXW;
        }
        break;
    case DEV_DBB_WAIT_CREGXW:
        if (dbb_at_expect("wait for CREGXW", "AT+CREGXW?", "+CREGXW:1") == 0)
        {
            dbb_ctrl.status = DEV_DBB_WAIT_CREV;
        }
        break;
    case DEV_DBB_WAIT_CREV:
        if (dbb_wait_for_text("+CREV: ME PDN ACT 1", response, sizeof(response), DBB_WAIT_EVENT_TIMEOUT_MS) == 0)
        {
            dbb_dump_response(response);
            dbb_ctrl.status = DEV_DBB_WAIT_CIREG;
        }
        break;
    case DEV_DBB_WAIT_CIREG:
        if (dbb_at_expect("wait for CIREG", "AT+CIREG?", "+CIREG:1") == 0)
        {
            dbb_ctrl.status = DEV_DBB_ONLINE;
            set_led(DEV_DBB_LED, 1);
        }
        break;
    case DEV_DBB_ONLINE:
        dbb_online_service();
        break;
    case DEV_DBB_ONLINE_BROADCAST:
        dbb_online_broadcast_service();
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

    // char target[DBB_MAX_SMS_TARGET_LEN];
    // char text[DBB_MAX_SMS_TEXT_LEN];
    // char pdu_hex[DBB_MAX_SMS_PDU_HEX];

    // int cmgs_len;
    // snprintf(target, sizeof(target), "%s", "8616194441530");
    // snprintf(text, sizeof(text), "%s", "卫星通信终端短信测试123abc");
    // dbb_build_sms_submit_pdu(target, text, pdu_hex, sizeof(pdu_hex), &cmgs_len);
    // printf("PDU HEX for empty SMS: %s\n", pdu_hex);

    // char response[DBB_MAX_RESPONSE];
    // snprintf(response, sizeof(response), "+CMT:\"HEX\",24\n0891686191004105F0040D91686191441425F8000852709201123323044F60597D\n", pdu_hex);
    // dbb_handle_urc_blob(response);

    // bytes_to_hex((const uint8_t *)"Hello, 世界!", 13, pdu_hex, sizeof(pdu_hex));
    // printf("HEX of 'Hello, 世界!': %s\n", pdu_hex);

    // snprintf(response, sizeof(response), "4500002E00000000FF11242F0A1041B80A1041B8C0011389001A949F000000000000000000000000000000000000", pdu_hex);
    // hex_to_bytes(response, (uint8_t *)pdu_hex, sizeof(pdu_hex), &cmgs_len);
    // printf("Bytes of PDU HEX:%s \n",pdu_hex);
    // for (int i = 0; i < cmgs_len; ++i)
    // {
    //     printf("%02X ", (unsigned char)response[i]);
    // }
    // printf("\n");

    // return NULL;
    (void)arg;
    sleep(10); // Sleep for 10 seconds before starting DBB operations
    dbb_ctrl.enabled = 1;
    dbb_ctrl.status = DEV_DBB_INIT;
    dbb_close_device();
    // if (DBB_RECEIVE_MODE == DBB_RECEIVE_MODE_CARD)
    // {
    //     dbb_cfg_once();
    // }

    while (1)
    {
        dbb_online_func();
        struct timespec delay = {0, 1000 * 1000};
        nanosleep(&delay, NULL);
    }

    return NULL;
}
