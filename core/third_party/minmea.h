/*
 * Copyright © 2014 Kosma Moczek <kosma@cloudyourcar.com>
 * This program is free software. It comes without any warranty, to the extent
 * permitted by applicable law. You can redistribute it and/or modify it under
 * the terms of the Do What The Fuck You Want To Public License, Version 2, as
 * published by Sam Hocevar. See the COPYING file for more details.
 */

#ifndef MINMEA_H
#define MINMEA_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#ifdef MINMEA_INCLUDE_COMPAT
#include <minmea_compat.h>
#endif

#ifndef MINMEA_MAX_SENTENCE_LENGTH
#define MINMEA_MAX_SENTENCE_LENGTH 80
#endif

    enum minmea_sentence_id
    {
        MINMEA_INVALID = -1,
        MINMEA_UNKNOWN = 0,
        MINMEA_SENTENCE_GBS,
        MINMEA_SENTENCE_GGA,
        MINMEA_SENTENCE_GLL,
        MINMEA_SENTENCE_GSA,
        MINMEA_SENTENCE_GST,
        MINMEA_SENTENCE_GSV,
        MINMEA_SENTENCE_RMC,
        MINMEA_SENTENCE_VTG,
        MINMEA_SENTENCE_ZDA,
    };

    struct minmea_float
    {
        int_least32_t value; // 原始数值，带固定小数位缩放
        int_least32_t scale; // 缩放因子，例如 100 表示保留两位小数
    };

    struct minmea_date
    {
        int day;   // 日
        int month; // 月
        int year;  // 年
    };

    struct minmea_time
    {
        int hours;        // 时
        int minutes;      // 分
        int seconds;      // 秒
        int microseconds; // 微秒
    };

    // provide backwards compatibility to users expecting a null-terminated string
    // instead of a struct
    union minmea_type
    {
        char buf[6]; // 完整句型标识，如 GPRMC
        struct
        {
            char talker_id[2];   // 说话者标识，如 GP/GN
            char sentence_id[3]; // 句型标识，如 RMC/GGA
            char null_terminator; // 字符串结束符
        };
    };

    struct minmea_sentence_gbs
    {
        union minmea_type type;          // 句型类型
        struct minmea_time time;         // 时间
        struct minmea_float err_latitude;  // 纬度误差
        struct minmea_float err_longitude; // 经度误差
        struct minmea_float err_altitude;  // 高程误差
        int svid;                        // 卫星编号
        struct minmea_float prob;        // 置信概率
        struct minmea_float bias;        // 偏差
        struct minmea_float stddev;      // 标准差
    };

    struct minmea_sentence_rmc
    {
        union minmea_type type;        // 句型类型
        struct minmea_time time;       // UTC 时间
        bool valid;                    // 定位是否有效
        struct minmea_float latitude;  // 纬度
        struct minmea_float longitude; // 经度
        struct minmea_float speed;     // 速度，单位节
        struct minmea_float course;    // 航向角
        struct minmea_date date;       // 日期
        struct minmea_float variation; // 磁偏角
    };

    struct minmea_sentence_gga
    {
        union minmea_type type;      // 句型类型
        struct minmea_time time;     // UTC 时间
        struct minmea_float latitude;  // 纬度
        struct minmea_float longitude; // 经度
        int fix_quality;             // 定位质量
        int satellites_tracked;      // 跟踪卫星数
        struct minmea_float hdop;    // 水平精度因子
        struct minmea_float altitude; // 海拔高度
        char altitude_units;         // 高度单位
        struct minmea_float height;   // 椭球高
        char height_units;           // 椭球高度单位
        struct minmea_float dgps_age; // 差分数据龄期
    };

    enum minmea_gll_status
    {
        MINMEA_GLL_STATUS_DATA_VALID = 'A',
        MINMEA_GLL_STATUS_DATA_NOT_VALID = 'V',
    };

    // FAA mode added to some fields in NMEA 2.3.
    enum minmea_faa_mode
    {
        MINMEA_FAA_MODE_AUTONOMOUS = 'A',
        MINMEA_FAA_MODE_DIFFERENTIAL = 'D',
        MINMEA_FAA_MODE_ESTIMATED = 'E',
        MINMEA_FAA_MODE_MANUAL = 'M',
        MINMEA_FAA_MODE_SIMULATED = 'S',
        MINMEA_FAA_MODE_NOT_VALID = 'N',
        MINMEA_FAA_MODE_PRECISE = 'P',
    };

    struct minmea_sentence_gll
    {
        union minmea_type type;      // 句型类型
        struct minmea_float latitude;  // 纬度
        struct minmea_float longitude; // 经度
        struct minmea_time time;     // UTC 时间
        char status;                 // 状态 A/V
        char mode;                   // 定位模式
    };

    struct minmea_sentence_gst
    {
        union minmea_type type;              // 句型类型
        struct minmea_time time;             // UTC 时间
        struct minmea_float rms_deviation;   // 均方根偏差
        struct minmea_float semi_major_deviation; // 长半轴偏差
        struct minmea_float semi_minor_deviation; // 短半轴偏差
        struct minmea_float semi_major_orientation; // 长半轴方向角
        struct minmea_float latitude_error_deviation;  // 纬度误差偏差
        struct minmea_float longitude_error_deviation; // 经度误差偏差
        struct minmea_float altitude_error_deviation;   // 高度误差偏差
    };

    enum minmea_gsa_mode
    {
        MINMEA_GPGSA_MODE_AUTO = 'A',
        MINMEA_GPGSA_MODE_FORCED = 'M',
    };

    enum minmea_gsa_fix_type
    {
        MINMEA_GPGSA_FIX_NONE = 1,
        MINMEA_GPGSA_FIX_2D = 2,
        MINMEA_GPGSA_FIX_3D = 3,
    };

    struct minmea_sentence_gsa
    {
        union minmea_type type; // 句型类型
        char mode;              // 模式 A/M
        int fix_type;           // 定位类型 1/2/3
        int sats[12];           // 参与定位的卫星号
        struct minmea_float pdop; // 位置精度因子
        struct minmea_float hdop; // 水平精度因子
        struct minmea_float vdop; // 垂直精度因子
    };

    struct minmea_sat_info
    {
        int nr;                  // 卫星号
        int elevation;           // 仰角
        int azimuth;             // 方位角
        struct minmea_float snr; // 信噪比
    };

    struct minmea_sentence_gsv
    {
        union minmea_type type;     // 句型类型
        int total_msgs;             // 总消息数
        int msg_nr;                 // 当前消息序号
        int total_sats;             // 可见卫星总数
        struct minmea_sat_info sats[4]; // 当前消息中的卫星信息
    };

    struct minmea_sentence_vtg
    {
        union minmea_type type;              // 句型类型
        struct minmea_float true_track_degrees;     // 真航向角
        struct minmea_float magnetic_track_degrees; // 磁航向角
        struct minmea_float speed_knots;             // 速度，单位节
        struct minmea_float speed_kph;               // 速度，单位千米每小时
        enum minmea_faa_mode faa_mode;               // FAA 模式
    };

    struct minmea_sentence_zda
    {
        union minmea_type type;  // 句型类型
        struct minmea_time time; // UTC 时间
        struct minmea_date date;  // 日期
        int hour_offset;         // 时区偏移小时
        int minute_offset;       // 时区偏移分钟
    };

    /**
     * Calculate raw sentence checksum. Does not check sentence integrity.
     */
    uint8_t minmea_checksum(const char *sentence);

    /**
     * Check sentence validity and checksum. Returns true for valid sentences.
     */
    bool minmea_check(const char *sentence, bool strict);

    /**
     * Determine talker identifier.
     */
    bool minmea_talker_id(char talker[3], const char *sentence);

    /**
     * Get sentence id as string.
     */
    const char *minmea_sentence(enum minmea_sentence_id id);

    /**
     * Determine sentence identifier.
     */
    enum minmea_sentence_id minmea_sentence_id(const char *sentence, bool strict);

    /**
     * Scanf-like processor for NMEA sentences. Supports the following formats:
     * c - single character (char *)
     * d - direction, returned as 1/-1, default 0 (int *)
     * f - fractional, returned as value + scale (struct minmea_float *)
     * i - decimal, default zero (int *)
     * s - string (char *)
     * t - talker identifier and type (union minmea_type *)
     * D - date (struct minmea_date *)
     * T - time stamp (struct minmea_time *)
     * _ - ignore this field
     * ; - following fields are optional
     * Returns true on success. See library source code for details.
     */
    bool minmea_scan(const char *sentence, const char *format, ...);

    /*
     * Parse a specific type of sentence. Return true on success.
     */
    bool minmea_parse_gbs(struct minmea_sentence_gbs *frame, const char *sentence);
    bool minmea_parse_rmc(struct minmea_sentence_rmc *frame, const char *sentence);
    bool minmea_parse_gga(struct minmea_sentence_gga *frame, const char *sentence);
    bool minmea_parse_gsa(struct minmea_sentence_gsa *frame, const char *sentence);
    bool minmea_parse_gll(struct minmea_sentence_gll *frame, const char *sentence);
    bool minmea_parse_gst(struct minmea_sentence_gst *frame, const char *sentence);
    bool minmea_parse_gsv(struct minmea_sentence_gsv *frame, const char *sentence);
    bool minmea_parse_vtg(struct minmea_sentence_vtg *frame, const char *sentence);
    bool minmea_parse_zda(struct minmea_sentence_zda *frame, const char *sentence);

    /**
     * Convert GPS UTC date/time representation to a UNIX calendar time.
     */
    int minmea_getdatetime(struct tm *tm, const struct minmea_date *date, const struct minmea_time *time_);

    /**
     * Convert GPS UTC date/time representation to a UNIX timestamp.
     */
    int minmea_gettime(struct timespec *ts, const struct minmea_date *date, const struct minmea_time *time_);

    /**
     * Rescale a fixed-point value to a different scale. Rounds towards zero.
     */
    static inline int_least32_t minmea_rescale(const struct minmea_float *f, int_least32_t new_scale)
    {
        if (f->scale == 0)
            return 0;
        if (f->scale == new_scale)
            return f->value;
        if (f->scale > new_scale)
            return (f->value + ((f->value > 0) - (f->value < 0)) * f->scale / new_scale / 2) / (f->scale / new_scale);
        else
            return f->value * (new_scale / f->scale);
    }

    /**
     * Convert a fixed-point value to a floating-point value.
     * Returns NaN for "unknown" values.
     */
    static inline float minmea_tofloat(const struct minmea_float *f)
    {
        if (f->scale == 0)
            return NAN;
        return (float)f->value / (float)f->scale;
    }

    /**
     * Convert a raw coordinate to a floating point DD.DDD... value.
     * Returns NaN for "unknown" values.
     */
    static inline float minmea_tocoord(const struct minmea_float *f)
    {
        if (f->scale == 0)
            return NAN;
        if (f->scale > (INT_LEAST32_MAX / 100))
            return NAN;
        if (f->scale < (INT_LEAST32_MIN / 100))
            return NAN;
        int_least32_t degrees = f->value / (f->scale * 100);
        int_least32_t minutes = f->value % (f->scale * 100);
        return (float)degrees + (float)minutes / (60 * f->scale);
    }

    /**
     * Check whether a character belongs to the set of characters allowed in a
     * sentence data field.
     */
    static inline bool minmea_isfield(char c)
    {
        return isprint((unsigned char)c) && c != ',' && c != '*';
    }

#ifdef __cplusplus
}
#endif

#endif /* MINMEA_H */

/* vim: set ts=4 sw=4 et: */
