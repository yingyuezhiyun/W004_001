#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "third_party/minmea.h"
    typedef enum
    {
        GNSS_DATA_NMEA,
        GNSS_DATA_RAW,
    } gnss_DataType_t;

    typedef struct
    {
        int fd;
        gnss_DataType_t data_type;
        char Nmea_sentence[MINMEA_MAX_SENTENCE_LENGTH + 16];
        uint16_t Nmea_len;
        uint8_t data_raw[1024];
        uint16_t data_raw_len;
    } gnss_ctrl_t;

    extern gnss_ctrl_t gnss_ctrl;

    void handle_gnss_nmea(const char *sentence);
    int handle_gnss_raw(const uint8_t *data, size_t len);
    void gnss_cfg_close_all(int fd);
    void gnss_cfg_eable_onchange(int fd, char *type);
    void gnss_cfg(int fd, char *type, uint8_t enable, uint8_t per_second);
    int8_t gnss_bdd_enable();
    int8_t gnss_bdd_disable();

#ifdef __cplusplus
}
#endif
