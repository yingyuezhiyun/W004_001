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
#include "gnss_func.h"

gnss_ctrl_t gnss_ctrl = {
    .fd = -1,
    // .data_type = GNSS_DATA_NMEA,
    .data_type = GNSS_DATA_RAW,
    .Nmea_sentence = {0},
    .Nmea_len = 0,
    .data_raw = {0},
    .data_raw_len = 0};

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

void gnss_cfg_eable_onchange(int fd, char *type)
{
    if (fd < 0)
    {
        fprintf(stderr, "Invalid file descriptor\n");
        return;
    }
    char buff[128];
    int result = 0;
    snprintf(buff, sizeof(buff), "CSHG ONCHANGE COM3 %s ONCHANGED \r\n", type);
    result = write(fd, buff, strlen(buff));
    if (result < 0)
    {
        perror("write gnss device");
    }
    else
    {
        printf("GNSS: %s on change enabled\n", type);
    }
}

void gnss_cfg_close_all(int fd)
{
    char buff[128];
    snprintf(buff, sizeof(buff), "CSHG CLOSEALL COM3 \r\n");
    int result = write(fd, buff, strlen(buff));
}

void *gnss_thread_func(void *arg)
{
    (void)arg;
    gnss_bdd_disable();
    sleep(1);
    if (gnss_bdd_enable() < 0)
    {
        fprintf(stderr, "Failed to enable GNSS BDD\n");
        return NULL;
    }
    sleep(2); // Sleep for 2 seconds to allow the BDD to power up
    gnss_ctrl.fd = open(DEV_GNSS, O_RDWR | O_NOCTTY | O_NDELAY);
    if (gnss_ctrl.fd < 0)
    {
        perror("open gnss device");
        return NULL;
    }
    set_opt(gnss_ctrl.fd, 115200, 8, 'N', 1);
    usleep(100000); // Sleep for 100 milliseconds to allow the device to initialize
    gnss_cfg_close_all(gnss_ctrl.fd);
    usleep(100000); // Sleep for 100 milliseconds

    // gnss_ctrl.data_type = GNSS_DATA_NMEA;

    // gnss_cfg(gnss_ctrl.fd, "RMC", 1, 1);
    // usleep(100000); // Sleep for 100 milliseconds
    // gnss_cfg(gnss_ctrl.fd, "GGA", 1, 1);
    // usleep(100000); // Sleep for 100 milliseconds
    // gnss_cfg(gnss_ctrl.fd, "GSA", 1, 1);
    // usleep(100000); // Sleep for 100 milliseconds
    // gnss_cfg(gnss_ctrl.fd, "GST", 1, 1);

    // gnss_cfg_eable_onchange(gnss_ctrl.fd, "GPSEPHB");
    gnss_cfg(gnss_ctrl.fd, "GPSEPHB", 1, 1);

    char buf[256];

    while (1)
    {
        int n = read(gnss_ctrl.fd, buf, sizeof(buf) - 1);
        if (n > 0)
        {
            if (gnss_ctrl.data_type == GNSS_DATA_NMEA)
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
                        if (gnss_ctrl.Nmea_len > 0)
                        {
                            gnss_ctrl.Nmea_sentence[gnss_ctrl.Nmea_len] = '\0';
                            handle_gnss_nmea(gnss_ctrl.Nmea_sentence);
                            gnss_ctrl.Nmea_len = 0;
                        }
                        continue;
                    }
                    if (gnss_ctrl.Nmea_len < sizeof(gnss_ctrl.Nmea_sentence) - 1)
                    {
                        gnss_ctrl.Nmea_sentence[gnss_ctrl.Nmea_len++] = c;
                    }
                    else
                    {
                        gnss_ctrl.Nmea_len = 0;
                    }
                }
            }
            else
            {
                if (sizeof(gnss_ctrl.data_raw) == gnss_ctrl.data_raw_len)
                {
                    fprintf(stderr, "GNSS raw data buffer overflow, dropping data\n");
                    gnss_ctrl.data_raw_len = 0;
                }

                int copy_len = (n < sizeof(gnss_ctrl.data_raw) - gnss_ctrl.data_raw_len) ? n : (sizeof(gnss_ctrl.data_raw) - gnss_ctrl.data_raw_len);
                memcpy(gnss_ctrl.data_raw + gnss_ctrl.data_raw_len, buf, copy_len);
                gnss_ctrl.data_raw_len += copy_len;
                uint16_t handle_len = handle_gnss_raw(gnss_ctrl.data_raw, gnss_ctrl.data_raw_len);
                if (handle_len > 0)
                {
                    if (handle_len < gnss_ctrl.data_raw_len)
                    {
                        memmove(gnss_ctrl.data_raw, gnss_ctrl.data_raw + handle_len, gnss_ctrl.data_raw_len - handle_len);
                    }
                    gnss_ctrl.data_raw_len -= handle_len;
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

    close(gnss_ctrl.fd);

    return NULL;
}
