#define _POSIX_C_SOURCE 200809L

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

static void handle_nmea_sentence(const char *sentence)
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

int8_t gnss_bdd_enable()
{
    if (io_set_output_level(DEV_GNSS_DBB_IO_NRESET, 1) < 0)
    {
        fprintf(stderr, "Failed to enable GNSS BDD\n");
        return -1;
    }
    if (io_set_output_level(DEV_GNSS_DBB_IO_POWER, 1) < 0)
    {
        fprintf(stderr, "Failed to enable GNSS BDD power\n");
        return -1;
    }
    return 0;
}

int8_t gnss_bdd_disable()
{
    if (io_set_output_level(DEV_GNSS_DBB_IO_POWER, 0) < 0)
    {
        fprintf(stderr, "Failed to disable GNSS BDD power\n");
        return -1;
    }
    return 0;
}

/// @brief
/// @param type NMEA sentence type, e.g. "RMC", "GGA", "GLL", "GSA", "GST", "GSV", "VTG", "ZDA" , "GBS" ,"HDT", "NTR", "ORI", "ROT", "TRA", "DTM"
/// @param enable 0 to disable, 1 to enable
/// @param per_second > 0 , number of sentences to output per second
void gnss_cfg(int fd, char *type, uint8_t enable, uint8_t per_second)
{
    if (fd < 0)
    {
        fprintf(stderr, "Invalid file descriptor\n");
        return;
    }
    char buff[128];
    int result = 0;
    if (enable)
    {
        snprintf(buff, sizeof(buff), "CSHG OPEN COM3 %s ONTIME %d \r\n", type, per_second);
        result = write(fd, buff, strlen(buff));
    }
    else
    {
        snprintf(buff, sizeof(buff), "CSHG CLOSE COM3 %s \r\n", type);
        result = write(fd, buff, strlen(buff));
    }
    if (result < 0)
    {
        perror("write gnss device");
    }
    else
    {
        printf("GNSS: %s %s %d/s\n", enable ? "enabled" : "disabled", type, per_second);
    }
}

void *gnss_thread_func(void *arg)
{
    (void)arg;

    if (gnss_bdd_enable() < 0)
    {
        fprintf(stderr, "Failed to enable GNSS BDD\n");
        return NULL;
    }
    sleep(2); // Sleep for 2 seconds to allow the BDD to power up
    int fd = open(DEV_GNSS, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0)
    {
        perror("open gnss device");
        return NULL;
    }
    set_opt(fd, 115200, 8, 'N', 1);
    usleep(100000); // Sleep for 100 milliseconds to allow the device to initialize
    gnss_cfg(fd, "RMC", 1, 1);
    usleep(100000); // Sleep for 100 milliseconds
    gnss_cfg(fd, "GGA", 1, 1);
    usleep(100000); // Sleep for 100 milliseconds
    gnss_cfg(fd, "GSA", 1, 1);
    usleep(100000); // Sleep for 100 milliseconds
    gnss_cfg(fd, "GST", 1, 1);
    usleep(100000); // Sleep for 100 milliseconds
    gnss_cfg(fd, "GBS", 0, 0);
    usleep(100000); // Sleep for 100 milliseconds
    gnss_cfg(fd, "GSV", 0, 0);
    usleep(100000); // Sleep for 100 milliseconds
    gnss_cfg(fd, "VTG", 0, 0);
    usleep(100000); // Sleep for 100 milliseconds
    gnss_cfg(fd, "ZDA", 0, 0);
    usleep(100000); // Sleep for 100 milliseconds
    gnss_cfg(fd, "HDT", 0, 0);
    usleep(100000); // Sleep for 100 milliseconds
    gnss_cfg(fd, "NTR", 0, 0);
    usleep(100000); // Sleep for 100 milliseconds
    gnss_cfg(fd, "ORI", 0, 0);
    usleep(100000); // Sleep for 100 milliseconds
    gnss_cfg(fd, "ROT", 0, 0);
    usleep(100000); // Sleep for 100 milliseconds
    gnss_cfg(fd, "TRA", 0, 0);
    usleep(100000); // Sleep for 100 milliseconds
    gnss_cfg(fd, "DTM", 0, 0);
    usleep(100000); // Sleep for 100 milliseconds

    char buf[256];
    char sentence[MINMEA_MAX_SENTENCE_LENGTH + 16];
    size_t sentence_len = 0;
    while (1)
    {
        int n = read(fd, buf, sizeof(buf) - 1);
        if (n > 0)
        {
            for (int i = 0; i < n; i++)
            {
                char c = buf[i];
                if (c == '\r')
                {
                    continue;
                }
                if (c == '\n')
                {
                    if (sentence_len > 0)
                    {
                        sentence[sentence_len] = '\0';
                        handle_nmea_sentence(sentence);
                        sentence_len = 0;
                    }
                    continue;
                }
                if (sentence_len < sizeof(sentence) - 1)
                {
                    sentence[sentence_len++] = c;
                }
                else
                {
                    sentence_len = 0;
                }
            }
        }
        else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            perror("read gnss device");
            break;
        }
        usleep(1000); // Sleep for 1 millisecond
        // struct timespec sleep_time = {0, 1000000L};
        // nanosleep(&sleep_time, NULL);
    }

    close(fd);

    return NULL;
}
