#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

#include "glob_cfg.h"
#include <stdint.h>

    typedef enum
    {
        BDXWEPH = 1,
        BD2EPH,
        BD3EPH,
        BD3CNAV2EPH,
        BD3CNAV3EPH,
        GPSEPH,
        GALEPH,
        GLOEPH,
    } EPH_Type_t;

    typedef union
    {
        struct
        {
            uint8_t bdxw : 1;
            uint8_t bd2 : 1;
            uint8_t bd3 : 1;
            uint8_t bd3cnav2 : 1;
            uint8_t bd3cnav3 : 1;
            uint8_t gps : 1;
            uint8_t gal : 1;
            uint8_t glo : 1;
            uint8_t remain;
        } content;
        uint16_t value;
    } eph_sw_t;

    typedef struct
    {
        eph_sw_t eph_sw;
        uint8_t output_freq; // 输出频率 默认1Hz
        uint16_t id;
    } glob_comm_config_t;

    extern glob_comm_config_t glob_comm_config;

#ifdef __cplusplus
}
#endif
