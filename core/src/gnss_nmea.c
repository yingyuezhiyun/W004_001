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

static void print_coord(const char *label, const struct minmea_float *coord)
{
    double value = minmea_tocoord(coord);
    if (isnan(value))
    {
        printf("%s=nan", label);
        return;
    }
    printf("%s=%.6f", label, value);
}

static void print_float_field(const char *label, const struct minmea_float *value)
{
    double number = minmea_tofloat(value);
    if (isnan(number))
    {
        printf("%s=nan", label);
        return;
    }
    printf("%s=%.2f", label, number);
}

static void print_time_field(const struct minmea_time *time_)
{
    printf("%02d:%02d:%02d.%06d",
           time_->hours,
           time_->minutes,
           time_->seconds,
           time_->microseconds);
}

static void print_date_field(const struct minmea_date *date)
{
    printf("%02d-%02d-%04d", date->day, date->month, date->year);
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
            printf("RMC %02d:%02d:%02d.%06d ",
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
            tm_now.tm_isdst = -1;        // 不确定是否夏令时

            gettimeofday(&tv, NULL); // 获取当前系统时间
            //mktime
            if (fabs(tv.tv_sec - mktime(&tm_now)) > 10)
            {
                tv.tv_sec = mktime(&tm_now); // 转换为UTC时间
                tv.tv_usec = frame.time.microseconds;
                if (settimeofday(&tv, NULL) == -1)// 设置系统时间
                {
                    // perror("settimeofday");
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
            // printf("time_diff=%.2f seconds ", time_diff);
            print_coord("lat", &frame.latitude);
            printf(" ");
            print_coord("lon", &frame.longitude);
            printf(" speed=%.2fkn(%.2fkm/h) course=%.2f valid=%c date=%02d-%02d-%04d\n",
                   speed_knots,
                   speed_kph,
                   minmea_tofloat(&frame.course),
                   frame.valid ? 'A' : 'V',
                   frame.date.year,
                   frame.date.month,
                   frame.date.day);
        }
    }
    break;
    case MINMEA_SENTENCE_GGA:
    {
        struct minmea_sentence_gga frame;
        if (minmea_parse_gga(&frame, sentence))
        {
            printf("GGA ");
            print_time_field(&frame.time);
            printf(" ");
            print_coord("lat", &frame.latitude);
            printf(" ");
            print_coord("lon", &frame.longitude);
            printf(" fix=%d sats=%d ",
                   frame.fix_quality,
                   frame.satellites_tracked);
            print_float_field("hdop", &frame.hdop);
            printf(" ");
            print_float_field("alt", &frame.altitude);
            printf("%c\n", frame.altitude_units);
        }
    }
    break;
    case MINMEA_SENTENCE_GLL:
    {
        struct minmea_sentence_gll frame;
        if (minmea_parse_gll(&frame, sentence))
        {
            printf("GLL ");
            print_coord("lat", &frame.latitude);
            printf(" ");
            print_coord("lon", &frame.longitude);
            printf(" ");
            print_time_field(&frame.time);
            printf(" status=%c mode=%c\n", frame.status, frame.mode);
        }
    }
    break;
    case MINMEA_SENTENCE_GSA:
    {
        struct minmea_sentence_gsa frame;
        if (minmea_parse_gsa(&frame, sentence))
        {
            printf("GSA mode=%c fix=%d sats=", frame.mode, frame.fix_type);
            for (int i = 0; i < 12; i++)
            {
                if (frame.sats[i] > 0)
                {
                    printf("%s%d", (i == 0) ? "" : ",", frame.sats[i]);
                }
            }
            printf(" ");
            print_float_field("pdop", &frame.pdop);
            printf(" ");
            print_float_field("hdop", &frame.hdop);
            printf(" ");
            print_float_field("vdop", &frame.vdop);
            printf("\n");
        }
    }
    break;
    case MINMEA_SENTENCE_GSV:
    {
        struct minmea_sentence_gsv frame;
        if (minmea_parse_gsv(&frame, sentence))
        {
            printf("GSV msgs=%d/%d sats=%d",
                   frame.msg_nr,
                   frame.total_msgs,
                   frame.total_sats);
            for (int i = 0; i < 4; i++)
            {
                if (frame.sats[i].nr > 0)
                {
                    printf(" [prn=%d elev=%d az=%d ",
                           frame.sats[i].nr,
                           frame.sats[i].elevation,
                           frame.sats[i].azimuth);
                    print_float_field("snr", &frame.sats[i].snr);
                    printf("]");
                }
            }
            printf("\n");
        }
    }
    break;
    case MINMEA_SENTENCE_VTG:
    {
        struct minmea_sentence_vtg frame;
        if (minmea_parse_vtg(&frame, sentence))
        {
            printf("VTG ");
            print_float_field("true", &frame.true_track_degrees);
            printf(" ");
            print_float_field("mag", &frame.magnetic_track_degrees);
            printf(" ");
            print_float_field("knots", &frame.speed_knots);
            printf(" ");
            print_float_field("kph", &frame.speed_kph);
            printf(" faa=%c\n", (char)frame.faa_mode);
        }
    }
    break;
    case MINMEA_SENTENCE_ZDA:
    {
        struct minmea_sentence_zda frame;
        if (minmea_parse_zda(&frame, sentence))
        {
            printf("ZDA ");
            print_time_field(&frame.time);
            printf(" ");
            print_date_field(&frame.date);
            printf(" offset=%+03d:%02d\n", frame.hour_offset, frame.minute_offset);
        }
    }
    break;
    case MINMEA_SENTENCE_GBS:
    {
        struct minmea_sentence_gbs frame;
        if (minmea_parse_gbs(&frame, sentence))
        {
            printf("GBS ");
            print_time_field(&frame.time);
            printf(" svid=%d ", frame.svid);
            print_float_field("err_lat", &frame.err_latitude);
            printf(" ");
            print_float_field("err_lon", &frame.err_longitude);
            printf(" ");
            print_float_field("err_alt", &frame.err_altitude);
            printf(" prob=%f bias=%f stddev=%f\n",
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
            printf("GST ");
            print_time_field(&frame.time);
            print_float_field(" rms_dev", &frame.rms_deviation);
            printf(" ");
            print_float_field("semi_major_dev", &frame.semi_major_deviation);
            printf(" ");
            print_float_field("semi_minor_dev", &frame.semi_minor_deviation);
            printf(" ");
            print_float_field("semi_major_orientation", &frame.semi_major_orientation);
            printf(" ");
            print_float_field("lat_error_dev", &frame.latitude_error_deviation);
            printf(" ");
            print_float_field("lon_error_dev", &frame.longitude_error_deviation);
            printf(" ");
            print_float_field("alt_error_dev", &frame.altitude_error_deviation);
            printf("\n");
        }
    }
    break;
    default:
        break;
    }
}
