#include "stdint.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include "glob_cfg.h"
#include "src_tty.h"
#include "src_io.h"
#include "third_party/minmea.h"
#include "gnss_func.h"
#include <time.h>
#include <sys/time.h>
#include "glob_value.h"
#include "comm_service.h"
#include "stdarg.h"

typedef struct
{
    double longitude;   // 经度
    double latitude;    // 纬度
    uint64_t timestamp; // 时间戳，单位毫秒
} pos_info_t;

void GNSS_NMEA_LOG(const char *format, ...)
{
    if (gnss_ctrl.print.nmea != GNSS_PRINT_NONE)
    {
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
    }
}

static void print_coord(const char *label, const struct minmea_float *coord)
{
    double value = minmea_tocoord(coord);
    if (isnan(value))
    {
        GNSS_NMEA_LOG("%s=nan", label);
        return;
    }
    GNSS_NMEA_LOG("%s=%.6f", label, value);
}

static void print_float_field(const char *label, const struct minmea_float *value)
{
    double number = minmea_tofloat(value);
    if (isnan(number))
    {
        GNSS_NMEA_LOG("%s=nan", label);
        return;
    }
    GNSS_NMEA_LOG("%s=%.2f", label, number);
}

static void print_time_field(const struct minmea_time *time_)
{
    GNSS_NMEA_LOG("%02d:%02d:%02d.%06d",
                  time_->hours,
                  time_->minutes,
                  time_->seconds,
                  time_->microseconds);
}

static void print_date_field(const struct minmea_date *date)
{
    GNSS_NMEA_LOG("%02d-%02d-%04d", date->day, date->month, date->year);
}

static const char *gsv_talker_name(const char *talker)
{
    if (strcmp(talker, "GB") == 0)
        return "BDS";
    if (strcmp(talker, "GP") == 0)
        return "GPS";
    if (strcmp(talker, "GL") == 0)
        return "GLONASS";
    if (strcmp(talker, "GA") == 0)
        return "Galileo";
    if (strcmp(talker, "GQ") == 0)
        return "QZSS";
    if (strcmp(talker, "GX") == 0)
        return "XW";
    return "UNKNOWN";
}

static const char *gsv_signal_name(const char *talker, const char *signal_id)
{
    if (signal_id[0] == '\0')
        return "";

    if (strcmp(talker, "GB") == 0)
    {
        if (strcmp(signal_id, "0") == 0)
            return "ALL SIGNALS";
        if (strcmp(signal_id, "1") == 0)
            return "B1I";
        if (strcmp(signal_id, "3") == 0)
            return "B1C";
        if (strcmp(signal_id, "5") == 0)
            return "B2a";
        if (strcmp(signal_id, "6") == 0)
            return "B2b";
        if (strcmp(signal_id, "8") == 0)
            return "B3I";
        if (strcmp(signal_id, "B") == 0)
            return "B2I";
        return "RESERVED";
    }

    if (strcmp(talker, "GP") == 0)
    {
        if (strcmp(signal_id, "0") == 0)
            return "ALL SIGNALS";
        if (strcmp(signal_id, "1") == 0)
            return "L1CA";
        if (strcmp(signal_id, "3") == 0)
            return "L1C";
        if (strcmp(signal_id, "4") == 0)
            return "L2P";
        if (strcmp(signal_id, "5") == 0)
            return "L2C";
        if (strcmp(signal_id, "7") == 0)
            return "L5";
        return "RESERVED";
    }

    if (strcmp(talker, "GL") == 0)
    {
        if (strcmp(signal_id, "0") == 0)
            return "ALL SIGNALS";
        if (strcmp(signal_id, "1") == 0)
            return "G1";
        if (strcmp(signal_id, "3") == 0)
            return "G2";
        return "RESERVED";
    }

    if (strcmp(talker, "GA") == 0)
    {
        if (strcmp(signal_id, "0") == 0)
            return "ALL SIGNALS";
        if (strcmp(signal_id, "1") == 0)
            return "E5a";
        if (strcmp(signal_id, "2") == 0)
            return "E5b";
        if (strcmp(signal_id, "7") == 0)
            return "E1";
        return "RESERVED";
    }

    if (strcmp(talker, "GQ") == 0)
    {
        if (strcmp(signal_id, "0") == 0)
            return "ALL SIGNALS";
        if (strcmp(signal_id, "1") == 0)
            return "L1";
        if (strcmp(signal_id, "5") == 0)
            return "L2";
        if (strcmp(signal_id, "7") == 0)
            return "L5";
        return "RESERVED";
    }

    if (strcmp(talker, "GX") == 0)
    {
        if (strcmp(signal_id, "1") == 0)
            return "XW-CNL";
        if (strcmp(signal_id, "2") == 0)
            return "XW-B2b";
        return "RESERVED";
    }

    return "UNKNOWN";
}

void handle_gnss_nmea(const char *sentence)
{
    if (!minmea_check(sentence, false))
    {
        return;
    }

    switch (minmea_sentence_id(sentence, false))
    {
    case MINMEA_SENTENCE_RMC:
    {
        struct minmea_sentence_rmc frame;
        if (minmea_parse_rmc(&frame, sentence))
        {
            double speed_knots = minmea_tofloat(&frame.speed);
            double speed_kph = speed_knots * 1.852;
            GNSS_NMEA_LOG("RMC %02d:%02d:%02d.%06d ",
                          frame.time.hours,
                          frame.time.minutes,
                          frame.time.seconds,
                          frame.time.microseconds);
            struct timeval tv;
            struct tm tm_now;
            tm_now.tm_year = frame.date.year + 2000 - 1900; // tm结构的年份是从1900年开始计算的
            tm_now.tm_mon = frame.date.month - 1;
            tm_now.tm_mday = frame.date.day;
            tm_now.tm_hour = frame.time.hours;
            tm_now.tm_min = frame.time.minutes;
            tm_now.tm_sec = frame.time.seconds;
            tm_now.tm_isdst = -1; // 不确定是否夏令时

            if (frame.valid)
            {
                gettimeofday(&tv, NULL); // 获取当前系统时间
                // mktime
                if (fabs(tv.tv_sec - mktime(&tm_now)) > 10)
                {
                    tv.tv_sec = mktime(&tm_now); // 转换为UTC时间
                    tv.tv_usec = frame.time.microseconds;
                    if (settimeofday(&tv, NULL) == -1) // 设置系统时间
                    {
                        // perror("settimeofday");
                        char command[128];
                        snprintf(command, sizeof(command), "date -s \"%04d-%02d-%02d %02d:%02d:%02d\"",
                                 tm_now.tm_year + 1900,
                                 tm_now.tm_mon + 1,
                                 tm_now.tm_mday,
                                 tm_now.tm_hour,
                                 tm_now.tm_min,
                                 tm_now.tm_sec);
                        system(command);
                    }
                }
            }

            // if (fabs(tv.tv_sec - timegm(&tm_now)) > 10)
            // {
            //     tv.tv_sec = timegm(&tm_now); // 转换为UTC时间
            //     tv.tv_usec = frame.time.microseconds;
            //     settimeofday(&tv, NULL); // 设置系统时间
            // }

            // time_t utc_time = timegm(&tm_now); // 转换为UTC时间
            // double time_diff = difftime(tv.tv_sec, utc_time) + (tv.tv_usec / 1e6);
            // GNSS_NMEA_LOG("time_diff=%.2f seconds ", time_diff);
            print_coord("lat", &frame.latitude);
            GNSS_NMEA_LOG(" ");
            print_coord("lon", &frame.longitude);
            GNSS_NMEA_LOG(" speed=%.2fkn(%.2fkm/h) course=%.2f valid=%c date=%02d-%02d-%04d\n",
                          speed_knots,
                          speed_kph,
                          minmea_tofloat(&frame.course),
                          frame.valid ? 'A' : 'V',
                          frame.date.year,
                          frame.date.month,
                          frame.date.day);
            pos_info_t pos;
            pos.latitude = minmea_tocoord(&frame.latitude);
            pos.longitude = minmea_tocoord(&frame.longitude);
            pos.timestamp = (uint64_t)timegm(&tm_now) * 1000 + frame.time.microseconds / 1000;
            net_send(COMM_SERVICE_UDP, SEND_CMD_POS, &pos, sizeof(pos_info_t));
        }
    }
    break;
    case MINMEA_SENTENCE_GGA:
    {
        struct minmea_sentence_gga frame;
        if (minmea_parse_gga(&frame, sentence))
        {
            GNSS_NMEA_LOG("GGA ");
            print_time_field(&frame.time);
            GNSS_NMEA_LOG(" ");
            print_coord("lat", &frame.latitude);
            GNSS_NMEA_LOG(" ");
            print_coord("lon", &frame.longitude);
            GNSS_NMEA_LOG(" fix=%d sats=%d ",
                          frame.fix_quality,
                          frame.satellites_tracked);
            print_float_field("hdop", &frame.hdop);
            GNSS_NMEA_LOG(" ");
            print_float_field("alt", &frame.altitude);
            GNSS_NMEA_LOG("%c\n", frame.altitude_units);
        }
    }
    break;
    case MINMEA_SENTENCE_GLL:
    {
        struct minmea_sentence_gll frame;
        if (minmea_parse_gll(&frame, sentence))
        {
            GNSS_NMEA_LOG("GLL ");
            print_coord("lat", &frame.latitude);
            GNSS_NMEA_LOG(" ");
            print_coord("lon", &frame.longitude);
            GNSS_NMEA_LOG(" ");
            print_time_field(&frame.time);
            GNSS_NMEA_LOG(" status=%c mode=%c\n", frame.status, frame.mode);
        }
    }
    break;
    case MINMEA_SENTENCE_GSA:
    {
        struct minmea_sentence_gsa frame;
        if (minmea_parse_gsa(&frame, sentence))
        {
            GNSS_NMEA_LOG("GSA mode=%c fix=%d sats=", frame.mode, frame.fix_type);
            for (int i = 0; i < 12; i++)
            {
                if (frame.sats[i] > 0)
                {
                    GNSS_NMEA_LOG("%s%d", (i == 0) ? "" : ",", frame.sats[i]);
                }
            }
            GNSS_NMEA_LOG(" ");
            print_float_field("pdop", &frame.pdop);
            GNSS_NMEA_LOG(" ");
            print_float_field("hdop", &frame.hdop);
            GNSS_NMEA_LOG(" ");
            print_float_field("vdop", &frame.vdop);
            GNSS_NMEA_LOG("\n");
        }
    }
    break;
    case MINMEA_SENTENCE_GSV:
    {
        struct minmea_sentence_gsv frame;
        char talker[3] = {0};
        if (minmea_parse_gsv(&frame, sentence))
        {
            if (!minmea_talker_id(talker, sentence))
            {
                strcpy(talker, "??");
            }

            const char *signal_name = gsv_signal_name(talker, frame.signal_id);

            if (frame.signal_id[0])
            {
                GNSS_NMEA_LOG("GSV talker=%s(%s) signal_id=%s(%s) msgs=%d/%d sats=%d",
                              talker,
                              gsv_talker_name(talker),
                              frame.signal_id,
                              signal_name,
                              frame.msg_nr,
                              frame.total_msgs,
                              frame.total_sats);
            }
            else
            {
                GNSS_NMEA_LOG("GSV talker=%s(%s) signal_id=- msgs=%d/%d sats=%d",
                              talker,
                              gsv_talker_name(talker),
                              frame.msg_nr,
                              frame.total_msgs,
                              frame.total_sats);
            }
            for (int i = 0; i < 4; i++)
            {
                if (frame.sats[i].nr > 0)
                {
                    GNSS_NMEA_LOG(" [prn=%d elev=%d az=%d ",
                                  frame.sats[i].nr,
                                  frame.sats[i].elevation,
                                  frame.sats[i].azimuth);
                    print_float_field("snr", &frame.sats[i].snr);
                    GNSS_NMEA_LOG("]");
                }
            }
            GNSS_NMEA_LOG("\n");
        }
    }
    break;
    case MINMEA_SENTENCE_VTG:
    {
        struct minmea_sentence_vtg frame;
        if (minmea_parse_vtg(&frame, sentence))
        {
            GNSS_NMEA_LOG("VTG ");
            print_float_field("true", &frame.true_track_degrees);
            GNSS_NMEA_LOG(" ");
            print_float_field("mag", &frame.magnetic_track_degrees);
            GNSS_NMEA_LOG(" ");
            print_float_field("knots", &frame.speed_knots);
            GNSS_NMEA_LOG(" ");
            print_float_field("kph", &frame.speed_kph);
            GNSS_NMEA_LOG(" faa=%c\n", (char)frame.faa_mode);
        }
    }
    break;
    case MINMEA_SENTENCE_ZDA:
    {
        struct minmea_sentence_zda frame;
        if (minmea_parse_zda(&frame, sentence))
        {
            GNSS_NMEA_LOG("ZDA ");
            print_time_field(&frame.time);
            GNSS_NMEA_LOG(" ");
            print_date_field(&frame.date);
            GNSS_NMEA_LOG(" offset=%+03d:%02d\n", frame.hour_offset, frame.minute_offset);
        }
    }
    break;
    case MINMEA_SENTENCE_GBS:
    {
        struct minmea_sentence_gbs frame;
        if (minmea_parse_gbs(&frame, sentence))
        {
            GNSS_NMEA_LOG("GBS ");
            print_time_field(&frame.time);
            GNSS_NMEA_LOG(" svid=%d ", frame.svid);
            print_float_field("err_lat", &frame.err_latitude);
            GNSS_NMEA_LOG(" ");
            print_float_field("err_lon", &frame.err_longitude);
            GNSS_NMEA_LOG(" ");
            print_float_field("err_alt", &frame.err_altitude);
            GNSS_NMEA_LOG(" prob=%f bias=%f stddev=%f\n",
                          minmea_tofloat(&frame.prob),
                          minmea_tofloat(&frame.bias),
                          minmea_tofloat(&frame.stddev));
        }
    }
    break;
    case MINMEA_SENTENCE_GST:
    {
        struct minmea_sentence_gst frame;
        if (minmea_parse_gst(&frame, sentence))
        {
            GNSS_NMEA_LOG("GST ");
            print_time_field(&frame.time);
            print_float_field(" rms_dev", &frame.rms_deviation);
            GNSS_NMEA_LOG(" ");
            print_float_field("semi_major_dev", &frame.semi_major_deviation);
            GNSS_NMEA_LOG(" ");
            print_float_field("semi_minor_dev", &frame.semi_minor_deviation);
            GNSS_NMEA_LOG(" ");
            print_float_field("semi_major_orientation", &frame.semi_major_orientation);
            GNSS_NMEA_LOG(" ");
            print_float_field("lat_error_dev", &frame.latitude_error_deviation);
            GNSS_NMEA_LOG(" ");
            print_float_field("lon_error_dev", &frame.longitude_error_deviation);
            GNSS_NMEA_LOG(" ");
            print_float_field("alt_error_dev", &frame.altitude_error_deviation);
            GNSS_NMEA_LOG("\n");
        }
    }
    break;
    default:
        break;
    }
}
