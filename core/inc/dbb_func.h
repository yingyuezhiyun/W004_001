#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#define DBB_DEBUG_INFO (0)
#define DBB_DEBUG_ERR (0)

#define DBB_MAX_RESPONSE 2048
#define DBB_AT_TIMEOUT_MS 2000
#define DBB_WAIT_EVENT_TIMEOUT_MS 30000
#define DBB_MONITOR_INTERVAL_SEC 1
#define DBB_MIN_UL_INTERVAL_SEC 5

#define DBB_MAX_UL_DATA_HEX 1024
#define DBB_MAX_UL_DATA_RAW 1024
#define DBB_MAX_B64_DATA 1536
#define DBB_MAX_SMS_TARGET_LEN 32
#define DBB_MAX_SMS_TEXT_LEN 160
#define DBB_MAX_SMS_PDU_HEX 512
#define DBB_MAX_SMS_UD_BYTES 140

    typedef enum
    {
        DBB_DATA_CODEC_HEX,
        DBB_DATA_CODEC_BASE64,
    } dbb_data_codec_t;

    typedef enum
    {
        DBB_PRINT_OFF,
        DBB_PRINT_ON,
    } dbb_PrintType_t;

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
        DEV_DBB_ONLINE_BROADCAST,
        DEV_DBB_POWER_OFF,
    } dbb_status_t;

    typedef struct
    {
        uint8_t enabled;
        int fd;
        dbb_status_t status;
        char device[64];

        int uplink_interval_sec;
        time_t last_uplink_ts;

        char sms_target[DBB_MAX_SMS_TARGET_LEN];
        char sms_text[DBB_MAX_SMS_TEXT_LEN];
        uint8_t sms_send_once_done;

        uint8_t downlink_raw[DBB_MAX_UL_DATA_RAW];
        size_t downlink_raw_len;
        char downlink_text[DBB_MAX_UL_DATA_RAW + 1];
        struct
        {
            uint8_t info;
            uint8_t err;
        } print;

    } dbb_ctrl_t;

    extern dbb_ctrl_t dbb_ctrl;
    int dbb_wait_for_text(const char *expect, char *response, size_t response_size, int timeout_ms);
    int dbb_at_expect(const char *name, const char *cmd, const char *expect);
    void dbb_dump_response(const char *response);
    int dbb_capture_response(const char *cmd, char *response, size_t response_size, int timeout_ms);

    void dbb_debug_info(const char *fmt, ...);
    void dbb_debug_err(const char *fmt, ...);

    void dbb_handle_urc_blob(const char *urc);
    int dbb_send_uplink(const uint8_t *raw_payload, size_t raw_payload_len, dbb_data_codec_t codec);
    int dbb_send_sms(const char *target, const char *text);
#ifdef __cplusplus
}
#endif
