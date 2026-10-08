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
#include "string.h"

gnss_ctrl_t gnss_ctrl = {
    .fd = -1,
    .parseDataType = GNSS_DATA_AUTO,
    .data_nmea = {0},
    .data_nmea_len = 0,
    .data_raw = {0},
    .data_raw_len = 0,
    .print = {
        .nmea = GNSS_PRINT_SUMMARY,
        .ephb = GNSS_PRINT_SUMMARY,
        .nmea_raw = GNSS_PRINT_NONE,
    },
    .file = {.gpsephb = {0}, .bd2ephb = {0}, .bd3ephb = {0}, .gloephb = {0}, .galephb = {0}, .bdxwephb = {0}, .bd3cnav2ephb = {0}, .bd3cnav3ephb = {0}, .gbs = {0}, .gga = {0}, .gll = {0}, .gsa = {0}, .gst = {0}, .gsv = {0}, .rmc = {0}, .vtg = {0}, .zda = {0}},
    .fd_mutex = PTHREAD_MUTEX_INITIALIZER,
};

static void gnss_handle_nmea_bytes(const uint8_t *buf, size_t len, int require_sentence_start)
{
    for (size_t i = 0; i < len; i++)
    {
        char c = (char)buf[i];

        if (c == '\r')
        {
            continue;
        }
        if (require_sentence_start && c == '$')
        {
            gnss_ctrl.data_nmea_len = 0;
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
        fprintf(stderr, "GNSS ephb data buffer overflow, dropping data\n");
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

int gnss_dev_write(const void *buf, size_t count)
{
    pthread_mutex_lock(&gnss_ctrl.fd_mutex);
    if (gnss_ctrl.fd < 0)
    {
        fprintf(stderr, "Invalid file descriptor\n");
        pthread_mutex_unlock(&gnss_ctrl.fd_mutex);
        return -1;
    }
    ssize_t result = write(gnss_ctrl.fd, buf, count);
    if (result < 0)
    {
        perror("write gnss device");
        pthread_mutex_unlock(&gnss_ctrl.fd_mutex);
        return -1;
    }
    pthread_mutex_unlock(&gnss_ctrl.fd_mutex);
    return result;
}

/// @brief Reset the GNSS module
void gnss_reset()
{
    char buff[128];
    int result = 0;

    snprintf(buff, sizeof(buff), "CSHG FRESET\r\n");
    result = gnss_dev_write(buff, strlen(buff));

    if (result < 0)
    {
        perror("write gnss device");
    }
    else
    {
        printf("\033[32mGNSS:  reset\n\033[0m");
    }
}

/// @brief Save the GNSS module configuration
void gnss_cfg_save()
{
    char buff[128];
    int result = 0;

    snprintf(buff, sizeof(buff), "CSHG SAVECONFIG\r\n");
    result = gnss_dev_write(buff, strlen(buff));

    if (result < 0)
    {
        perror("write gnss device");
    }
    else
    {
        printf("\033[32mGNSS:  save\n\033[0m");
    }
}

void CfgdataType_from_name(char *type, uint8_t enable)
{
    if (strcasecmp(type, "gbs") == 0)
        gnss_ctrl.dataType.nmea.gbs = enable;
    else if (strcasecmp(type, "gga") == 0)
        gnss_ctrl.dataType.nmea.gga = enable;
    else if (strcasecmp(type, "gll") == 0)
        gnss_ctrl.dataType.nmea.gll = enable;
    else if (strcasecmp(type, "gsa") == 0)
        gnss_ctrl.dataType.nmea.gsa = enable;
    else if (strcasecmp(type, "gst") == 0)
        gnss_ctrl.dataType.nmea.gst = enable;
    else if (strcasecmp(type, "gsv") == 0)
        gnss_ctrl.dataType.nmea.gsv = enable;
    else if (strcasecmp(type, "rmc") == 0)
        gnss_ctrl.dataType.nmea.rmc = enable;
    else if (strcasecmp(type, "vtg") == 0)
        gnss_ctrl.dataType.nmea.vtg = enable;
    else if (strcasecmp(type, "zda") == 0)
        gnss_ctrl.dataType.nmea.zda = enable;
    else if (strcasecmp(type, "bdxwephb") == 0)
        gnss_ctrl.dataType.ephb.bdxw = enable;
    else if (strcasecmp(type, "gpsephb") == 0)
        gnss_ctrl.dataType.ephb.gps = enable;
    else if (strcasecmp(type, "bd2ephb") == 0)
        gnss_ctrl.dataType.ephb.bd2 = enable;
    else if (strcasecmp(type, "bd3ephb") == 0)
        gnss_ctrl.dataType.ephb.bd3 = enable;
    else if (strcasecmp(type, "bd3cnav2ephb") == 0)
        gnss_ctrl.dataType.ephb.bd3cnav2 = enable;
    else if (strcasecmp(type, "bd3cnav3ephb") == 0)
        gnss_ctrl.dataType.ephb.bd3cnav3 = enable;
    else if (strcasecmp(type, "gloephb") == 0)
        gnss_ctrl.dataType.ephb.glo = enable;
    else if (strcasecmp(type, "galephb") == 0)
        gnss_ctrl.dataType.ephb.gal = enable;
}

/// @brief 配置GNSS模块输出数据类型和频率
/// @param type NMEA sentence type, e.g. "RMC", "GGA", "GLL", "GSA", "GST", "GSV", "VTG", "ZDA" , "GBS" ,"HDT", "NTR", "ORI", "ROT", "TRA", "DTM"
/// @param enable 0 to disable, 1 to enable
/// @param per_second > 0 , number of sentences to output per second
void gnss_cfg_dataType(char *type, uint8_t enable, uint8_t per_second)
{

    char buff[128];
    int result = 0;
    if (enable)
    {
        snprintf(buff, sizeof(buff), "CSHG OPEN COM1 %s ONTIME %d \r\n", type, per_second);
        result = gnss_dev_write(buff, strlen(buff));
    }
    else
    {
        snprintf(buff, sizeof(buff), "CSHG CLOSE COM1 %s \r\n", type);
        result = gnss_dev_write(buff, strlen(buff));
    }
    if (result < 0)
    {
        perror("write gnss device");
    }
    else
    {
        printf("\033[32mGNSS: %s %s %d/s\n\033[0m", enable ? "enabled" : "disabled", type, per_second);
        CfgdataType_from_name(type, enable);
    }
}

/// @brief 配置GNSS模块输出数据类型的变化触发事件
/// @param type 
void gnss_cfg_dataType_onchange(char *type)
{

    char buff[128];
    int result = 0;
    snprintf(buff, sizeof(buff), "CSHG ONCHANGE COM1 %s ONCHANGED \r\n", type);
    result = gnss_dev_write(buff, strlen(buff));
    if (result < 0)
    {
        perror("write gnss device");
    }
    else
    {
        printf("\033[32mGNSS: %s on change enabled\n\033[0m", type);
        CfgdataType_from_name(type, 1);
    }
}

void CfgsysType_from_name(char *sys, uint8_t enable)
{
    if (strcasecmp(sys, "bds") == 0)
        gnss_ctrl.sysType.content.bds = enable;
    else if (strcasecmp(sys, "gps") == 0)
        gnss_ctrl.sysType.content.gps = enable;
    else if (strcasecmp(sys, "gal") == 0)
        gnss_ctrl.sysType.content.gal = enable;
    else if (strcasecmp(sys, "glo") == 0)
        gnss_ctrl.sysType.content.glo = enable;
    else if (strcasecmp(sys, "all") == 0)
    {
        gnss_ctrl.sysType.content.bds = enable;
        gnss_ctrl.sysType.content.gps = enable;
        gnss_ctrl.sysType.content.gal = enable;
        gnss_ctrl.sysType.content.glo = enable;
    }   
}

/// @brief 配置GNSS的卫星系统
/// @param sys 
/// @param enable 
void gnss_cfg_sys(char *sys, uint8_t enable)
{
    char buff[128];
    int result = 0;
    if (enable)
    {
        snprintf(buff, sizeof(buff), "CSHG SYSEN %s ON \r\n", sys);
        result = gnss_dev_write(buff, strlen(buff));
    }
    else
    {
        snprintf(buff, sizeof(buff), "CSHG SYSEN %s OFF \r\n", sys);
        result = gnss_dev_write(buff, strlen(buff));
    }
    if (result < 0)
    {
        perror("write gnss device");
    }
    else
    {
        printf("GNSS: %s %s\n", enable ? "enabled" : "disabled", sys);
        CfgsysType_from_name(sys, enable);
    }
}

/// @brief Disable all GNSS module outputs
void gnss_cfg_disable_out()
{
    char buff[128];
    snprintf(buff, sizeof(buff), "CSHG CLOSEALL COM1 \r\n");
    int result = gnss_dev_write(buff, strlen(buff));
}

/// @brief 设置GNSS工作模式，设置模式后，模块会重启，需要再次开启相关协议数据输出
/// @param workMode 工作模式 BASE:基准站模式 ROVER:流动站模式
/// @param calcType 解算类型 RTD/RTK/PPP/DPPP/FPPP
/// @param freqCode 工作频点代码 1：全频点模式  2：低功耗模式 10：高性能模式 13：导航增强模式
void gnss_cfg_mode(char *workMode, char *calcType, uint8_t freqCode)
{
    char buff[128];
    snprintf(buff, sizeof(buff), "CSHG MODE %s %s %d \r\n", workMode, calcType, freqCode);
    int result = gnss_dev_write(buff, strlen(buff));
    if (result < 0)
    {
        perror("write gnss device");
    }
    else
    {
        if (strcasecmp(calcType, "ppp") == 0)
        {
            gnss_ctrl.calcType = GNSS_CALC_PPP;
        }
        else if (strcasecmp(calcType, "fppp") == 0)
        {
            gnss_ctrl.calcType = GNSS_CALC_FPPP;
        }
        else if (strcasecmp(calcType, "rtk") == 0)
        {
            gnss_ctrl.calcType = GNSS_CALC_RTK;
        }
        else if (strcasecmp(calcType, "rtd") == 0)
        {
            gnss_ctrl.calcType = GNSS_CALC_RTD;
        }
        else if (strcasecmp(calcType, "dppp") == 0)
        {
            gnss_ctrl.calcType = GNSS_CALC_DPPP;
        }
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

    pthread_mutex_lock(&gnss_ctrl.fd_mutex);
    gnss_ctrl.fd = open(DEV_GNSS, O_RDWR | O_NOCTTY | O_NDELAY);
    if (gnss_ctrl.fd < 0)
    {
        perror("open gnss device");
        pthread_mutex_unlock(&gnss_ctrl.fd_mutex);
        return NULL;
    }
    set_opt(gnss_ctrl.fd, 115200, 8, 'N', 1);
    pthread_mutex_unlock(&gnss_ctrl.fd_mutex);

    usleep(100000); // Sleep for 100 milliseconds to allow the device to initialize
    // gnss_cfg_disable_out();
    usleep(100000); // Sleep for 100 milliseconds

    gnss_ctrl.parseDataType = GNSS_DATA_AUTO;
    gnss_cfg_mode("ROVER", "FPPP", 13);
    sleep(2);
    gnss_cfg_dataType("RMC", 1, 1);
    // usleep(100000); // Sleep for 100 milliseconds
    // gnss_cfg_dataType( "GGA", 1, 1);
    // usleep(100000); // Sleep for 100 milliseconds
    // gnss_cfg_dataType( "GSA", 1, 1);
    // usleep(100000); // Sleep for 100 milliseconds
    // gnss_cfg_dataType( "GST", 1, 1);

    // gnss_cfg_dataType_onchange("GPSEPHB");
    // gnss_cfg_dataType( "GPSEPHB", 1, 1);
    // gpsephb_file_header();
    // gnss_cfg_dataType( "BD2EPHB", 1, 1);
    // gnss_cfg_dataType( "BD3EPHB", 1, 1);
    // gnss_cfg_dataType( "GLOEPHB", 1, 1);//todo 无数据
    // gnss_cfg_dataType( "GALEPHB", 1, 1);
    // gnss_cfg_dataType( "BD3CANV1EPHB", 1, 1); // todo 无数据
    // gnss_cfg_dataType( "BD3CANV2EPHB", 1, 1);//todo 无数据
    // gnss_cfg_dataType( "BD3CNAV3EPHB", 1, 1);//todo 无数据 解析错误
    // gnss_cfg_dataType( "PRANGEB", 1, 1);//
    // char *enable_ins = "CSHG INS ON\r\n"; // 启用组合导航功能
    // gnss_dev_write(enable_ins, strlen(enable_ins));
    // gnss_cfg_dataType("POSDATAB", 1, 1);//最优定位信息输出
    // gnss_cfg_dataType("BDXWEPHB", 1, 1);
    char buf[4096];

    while (1)
    {
        int drained = 0;

        while (1)
        {
            pthread_mutex_lock(&gnss_ctrl.fd_mutex);
            int n = read(gnss_ctrl.fd, buf, sizeof(buf));
            pthread_mutex_unlock(&gnss_ctrl.fd_mutex);
            if (n > 0)
            {
                drained = 1;

                if (gnss_ctrl.parseDataType == GNSS_DATA_AUTO)
                {
                    gnss_handle_raw_bytes((const uint8_t *)buf, (size_t)n);
                    gnss_handle_nmea_bytes((const uint8_t *)buf, (size_t)n, 1);
                }
                else if (gnss_ctrl.parseDataType == GNSS_DATA_NMEA)
                {
                    gnss_handle_nmea_bytes((const uint8_t *)buf, (size_t)n, 0);
                }
                else
                {
                    gnss_handle_raw_bytes((const uint8_t *)buf, (size_t)n);
                }

                continue;
            }

            if (n < 0 && errno == EINTR)
            {
                continue;
            }

            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            {
                break;
            }

            if (n < 0)
            {
                perror("read gnss device");
                pthread_mutex_lock(&gnss_ctrl.fd_mutex);
                close(gnss_ctrl.fd);
                pthread_mutex_unlock(&gnss_ctrl.fd_mutex);
                return NULL;
            }
            break;
        }
        if (!drained)
        {
            usleep(1000); // Sleep briefly when no data is available
        }
        // struct timespec sleep_time = {0, 1000000L};
        // nanosleep(&sleep_time, NULL);
    }

    close(gnss_ctrl.fd);
    return NULL;
}
