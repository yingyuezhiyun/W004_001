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
#include "src_led.h"
#include <time.h>
#include <sys/time.h>

gnss_ctrl_t gnss_ctrl = {
    .fd = -1,
    .data_type = GNSS_DATA_AUTO,
    .data_nmea = {0},
    .data_nmea_len = 0,
    .data_raw = {0},
    .data_raw_len = 0};

static void gnss_handle_nmea_bytes(const uint8_t *buf, size_t len, int require_sentence_start)
{
    for (size_t i = 0; i < len; i++)
    {
        char c = (char)buf[i];

        if (c == '\r')
        {
            continue;
        }

        if (gnss_ctrl.data_nmea_len == 0)
        {
            if (require_sentence_start && c != '$')
            {
                continue;
            }

            if (!require_sentence_start && c == '\n')
            {
                continue;
            }
        }

        if (c == '\n')
        {
            if (gnss_ctrl.data_nmea_len > 0)
            {
                gnss_ctrl.data_nmea[gnss_ctrl.data_nmea_len] = '\0';
                handle_gnss_nmea(gnss_ctrl.data_nmea);
                gnss_ctrl.data_nmea_len = 0;
            }
            continue;
        }

        if (gnss_ctrl.data_nmea_len < sizeof(gnss_ctrl.data_nmea) - 1)
        {
            gnss_ctrl.data_nmea[gnss_ctrl.data_nmea_len++] = c;
        }
        else
        {
            gnss_ctrl.data_nmea_len = 0;
        }
    }
}

static void gnss_handle_raw_bytes(const uint8_t *buf, size_t len)
{
    if (gnss_ctrl.data_raw_len == sizeof(gnss_ctrl.data_raw))
    {
        fprintf(stderr, "GNSS raw data buffer overflow, dropping data\n");
        gnss_ctrl.data_raw_len = 0;
    }

    size_t room = sizeof(gnss_ctrl.data_raw) - gnss_ctrl.data_raw_len;
    size_t copy_len = (len < room) ? len : room;
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

int gnss_dev_write(int fd, const void *buf, size_t count)
{
    if (fd < 0)
    {
        fprintf(stderr, "Invalid file descriptor\n");
        return -1;
    }
    ssize_t result = write(fd, buf, count);
    if (result < 0)
    {
        perror("write gnss device");
        return -1;
    }
    return result;
}

/// @brief
/// @param type NMEA sentence type, e.g. "RMC", "GGA", "GLL", "GSA", "GST", "GSV", "VTG", "ZDA" , "GBS" ,"HDT", "NTR", "ORI", "ROT", "TRA", "DTM"
/// @param enable 0 to disable, 1 to enable
/// @param per_second > 0 , number of sentences to output per second
void gnss_cfg_dis_enable(int fd, char *type, uint8_t enable, uint8_t per_second)
{

    char buff[128];
    int result = 0;
    if (enable)
    {
        snprintf(buff, sizeof(buff), "CSHG OPEN COM3 %s ONTIME %d \r\n", type, per_second);
        result = gnss_dev_write(fd, buff, strlen(buff));
    }
    else
    {
        snprintf(buff, sizeof(buff), "CSHG CLOSE COM3 %s \r\n", type);
        result = gnss_dev_write(fd, buff, strlen(buff));
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

void gnss_cfg_enable_onchange(int fd, char *type)
{

    char buff[128];
    int result = 0;
    snprintf(buff, sizeof(buff), "CSHG ONCHANGE COM3 %s ONCHANGED \r\n", type);
    result = gnss_dev_write(fd, buff, strlen(buff));
    if (result < 0)
    {
        perror("write gnss device");
    }
    else
    {
        printf("GNSS: %s on change enabled\n", type);
    }
}

void gnss_cfg_disable_all(int fd)
{
    char buff[128];
    snprintf(buff, sizeof(buff), "CSHG CLOSEALL COM3 \r\n");
    int result = gnss_dev_write(fd, buff, strlen(buff));
}

/// @brief 设置GNSS工作模式，设置模式后，模块会重启，需要再次开启相关协议数据输出
/// @param fd
/// @param workMode 工作模式 BASE:基准站模式 ROVER:流动站模式
/// @param calcType 解算类型 RTD/RTK/PPP/DPPP/FPPP
/// @param freqCode 工作频点代码 1：全频点模式  2：低功耗模式 10：高性能模式 13：导航增强模式
void gnss_cfg_mode(int fd, char *workMode, char *calcType, uint8_t freqCode)
{
    char buff[128];
    snprintf(buff, sizeof(buff), "CSHG MODE %s %s %d \r\n", workMode, calcType, freqCode);
    int result = gnss_dev_write(fd, buff, strlen(buff));
    if (result < 0)
    {
        perror("write gnss device");
    }
}

void *gnss_thread_func(void *arg)
{
    (void)arg;
    // set_led(DEV_4G_LED, 1);
    // set_led(DEV_LoRa_LED, 1);
    // set_led(DEV_DBB_LED, 1);
    gnss_bdd_disable();
    sleep(1);
    if (gnss_bdd_enable() < 0)
    {
        // fprintf(stderr, "Failed to enable GNSS BDD\n");
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
    // gnss_cfg_disable_all(gnss_ctrl.fd);
    usleep(100000); // Sleep for 100 milliseconds

    gnss_ctrl.data_type = GNSS_DATA_AUTO;

    // gnss_cfg_dis_enable(gnss_ctrl.fd, "RMC", 1, 1);
    // usleep(100000); // Sleep for 100 milliseconds
    // gnss_cfg_dis_enable(gnss_ctrl.fd, "GGA", 1, 1);
    // usleep(100000); // Sleep for 100 milliseconds
    // gnss_cfg_dis_enable(gnss_ctrl.fd, "GSA", 1, 1);
    // usleep(100000); // Sleep for 100 milliseconds
    // gnss_cfg_dis_enable(gnss_ctrl.fd, "GST", 1, 1);

    // gnss_cfg_enable_onchange(gnss_ctrl.fd, "GPSEPHB");
    // gnss_cfg_dis_enable(gnss_ctrl.fd, "GPSEPHB", 1, 1);
    // gpsephb_file_header();
    // gnss_cfg_dis_enable(gnss_ctrl.fd, "BD2EPHB", 1, 1);
    // gnss_cfg_dis_enable(gnss_ctrl.fd, "BD3EPHB", 1, 1);
    // gnss_cfg_dis_enable(gnss_ctrl.fd, "GLOEPHB", 1, 1);//todo 无数据
    // gnss_cfg_dis_enable(gnss_ctrl.fd, "GALEPHB", 1, 1);
    // gnss_cfg_dis_enable(gnss_ctrl.fd, "BD3CANV1EPHB", 1, 1); // todo 无数据
    // gnss_cfg_dis_enable(gnss_ctrl.fd, "BD3CANV2EPHB", 1, 1);//todo 无数据
    // gnss_cfg_dis_enable(gnss_ctrl.fd, "BD3CNAV3EPHB", 1, 1);//todo 无数据 解析错误
    // gnss_cfg_dis_enable(gnss_ctrl.fd, "PRANGEB", 1, 1);//
    // char *enable_ins = "CSHG INS ON\r\n"; // 启用组合导航功能
    // gnss_dev_write(gnss_ctrl.fd, enable_ins, strlen(enable_ins));
    // gnss_cfg_dis_enable(gnss_ctrl.fd, "POSDATAB", 1, 1);//最优定位信息输出
    // gnss_cfg_dis_enable(gnss_ctrl.fd, "BDXWEPHB", 1, 1);
    char buf[256];

    while (1)
    {
        int n = read(gnss_ctrl.fd, buf, sizeof(buf) - 1);
        if (n > 0)
        {
            if (gnss_ctrl.data_type == GNSS_DATA_AUTO)
            {
                gnss_handle_raw_bytes((const uint8_t *)buf, (size_t)n);
                gnss_handle_nmea_bytes((const uint8_t *)buf, (size_t)n, 1);
            }
            else if (gnss_ctrl.data_type == GNSS_DATA_NMEA)
            {
                gnss_handle_nmea_bytes((const uint8_t *)buf, (size_t)n, 0);
            }
            else
            {
                gnss_handle_raw_bytes((const uint8_t *)buf, (size_t)n);
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
