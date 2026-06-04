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
#include "gnss_func.h"
#include <time.h>
#include <sys/time.h>

#define TEST_GPSEPHB_FILE_PATH "/root/gps_ephe.csv"
#define TEST_BD2EPHB_FILE_PATH "/root/bd2_ephe.csv"
#define TEST_BD3EPHB_FILE_PATH "/root/bd3_ephe.csv"
#define TEST_BDXWEPHB_FILE_PATH "/root/bdxw_ephe.csv"
#define TEST_BD3CNAV2EPHB_FILE_PATH "/root/bd3cnav2_ephe.csv"
#define TEST_BD3CNAV3EPHB_FILE_PATH "/root/bd3cnav3_ephe.csv"

// #ifndef M_PI
// #define M_PI 3.14159265358979323846
// #endif

#pragma region "helper functions"

static const char *gps_l2_code_desc(uint8_t code)
{
    switch (code)
    {
    case 0:
        return "reserved";
    case 1:
        return "P";
    case 2:
        return "C/A";
    case 3:
        return "L2C";
    default:
        return "unknown";
    }
}

static const char *gps_fit_desc(uint8_t fit)
{
    return fit ? "curve fit interval > 4 h" : "curve fit interval = 4 h";
}

static const char *pos_time_system_desc(uint8_t time_system)
{
    switch (time_system)
    {
    case 0:
        return "invalid";
    case 1:
        return "BDS";
    case 2:
        return "GPS";
    case 3:
        return "GAL";
    case 4:
        return "GLO";
    default:
        return "reserved";
    }
}

static const char *pos_sys_status_desc(uint8_t sys_status)
{
    switch (sys_status)
    {
    case 0:
        return "not positioned";
    case 1:
        return "GNSS only";
    case 2:
        return "combined nav";
    case 3:
        return "INS only";
    default:
        return "reserved";
    }
}

static const char *pos_fix_type_desc(uint8_t type)
{
    switch (type)
    {
    case 0:
        return "invalid";
    case 1:
        return "single point";
    case 2:
        return "differential";
    case 3:
        return "PPS";
    case 4:
        return "fixed";
    case 5:
        return "float";
    case 6:
        return "dead reckoning";
    case 7:
        return "user input";
    case 8:
        return "PPP";
    default:
        return "reserved";
    }
}

static const char *pos_gear_desc(uint8_t gear)
{
    switch (gear)
    {
    case 1:
        return "N/D";
    case 2:
        return "R";
    case 3:
        return "P";
    default:
        return "reserved";
    }
}

char *gps_week_sec_to_utc(uint16_t gps_week, uint32_t gps_sec)
{
    static char buf[64];
    // GPS时间起点是1980-01-06 00:00:00 UTC
    const time_t gps_epoch = 315964800; // 1980-01-06 00:00:00 UTC对应的Unix时间戳
    time_t utc_time = gps_epoch + (gps_week * 7 * 24 * 3600) + gps_sec;
    struct tm *tm_info = gmtime(&utc_time);
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", tm_info);
    return buf;
}

#pragma endregion "helper functions"

#pragma region "data print"

void print_bd2ephb(const BD2EPHB_Decoded_t *eph, uint32_t payload_crc_calc)
{
    printf("BD2EPHB decoded frame:\n");
    printf("  head: ");
    for (size_t i = 0; i < sizeof(eph->head); ++i)
    {
        printf("%02X%s", eph->head[i], (i + 1 == sizeof(eph->head)) ? "" : " ");
    }
    printf("\n");

    printf("  gps_week_count  : raw=%u\n", eph->gps_week_count);
    printf("  gps_tow_s       : raw=%u s\n", eph->gps_tow_s);
    printf("  bd2_satid       : raw=%u\n", eph->bd2_satid);
    printf("  bd2_sv_urai     : raw=%u\n", eph->bd2_sv_urai);
    printf("  bd2_sv_health   : raw=%u\n", eph->bd2_sv_health);
    printf("  bd2_week        : raw=%u\n", eph->bd2_week);
    printf("  bd2_toe         : raw=%u s\n", eph->bd2_toe);
    printf("  bd2_toc         : raw=%u s\n", eph->bd2_toc);
    printf("  bd2_af0         : =% .12e s\n", eph->bd2_af0);
    printf("  bd2_af1         : =% .12e s/s\n", eph->bd2_af1);
    printf("  bd2_af2         : =% .12e s/s^2\n", eph->bd2_af2);
    printf("  bd2_aode        : raw=%u\n", eph->bd2_aode);
    printf("  bd2_aodc        : raw=%u\n", eph->bd2_aodc);
    printf("  bd2_idot        : =% .12e rad/s\n", eph->bd2_idot);
    printf("  bd2_crs         : =% .12e m\n", eph->bd2_crs);
    printf("  bd2_crc         : =% .12e m\n", eph->bd2_crc);
    printf("  bd2_cus         : =% .12e rad\n", eph->bd2_cus);
    printf("  bd2_cuc         : =% .12e rad\n", eph->bd2_cuc);
    printf("  bd2_cis         : =% .12e rad\n", eph->bd2_cis);
    printf("  bd2_cic         : =% .12e rad\n", eph->bd2_cic);
    printf("  bd2_delta_n     : =% .12e rad/s\n", eph->bd2_delta_n);
    printf("  bd2_m0          : =% .12e rad\n", eph->bd2_m0);
    printf("  bd2_ecc         : =% .12e\n", eph->bd2_ecc);
    printf("  bd2_a_half      : =% .12e m^1/2\n", eph->bd2_a_half);
    printf("  bd2_omega0      : =% .12e rad\n", eph->bd2_omega0);
    printf("  bd2_i0          : =% .12e rad\n", eph->bd2_i0);
    printf("  bd2_omega       : =% .12e rad\n", eph->bd2_omega);
    printf("  bd2_omegadot    : =% .12e rad/s\n", eph->bd2_omegadot);
    printf("  bd2_tgd1        : =% .12e s\n", eph->bd2_tgd1);
    printf("  bd2_tgd2        : =% .12e s\n", eph->bd2_tgd2);
    printf("  bd2_reserved    : raw=%u\n", eph->bd2_reserved);
    printf("  payload_crc24   : raw=%06X calc=%06X [%s]\n",
           eph->crc24,
           payload_crc_calc,
           (eph->crc24 == payload_crc_calc) ? "OK" : "BAD");
}

void print_bd3ephb(const BD3EPHB_Decoded_t *eph, uint32_t payload_crc_calc)
{
    printf("BD3EPHB decoded frame:\n");
    printf("  head: ");
    for (size_t i = 0; i < sizeof(eph->head); ++i)
    {
        printf("%02X%s", eph->head[i], (i + 1 == sizeof(eph->head)) ? "" : " ");
    }
    printf("\n");

    printf("  gps_week_count : raw=%u\n", eph->gps_week_count);
    printf("  gps_tow_s      : raw=%u s\n", eph->gps_tow_s);
    printf("  bd3_satid      : raw=%u\n", eph->bd3_satid);
    printf("  bd3_sattype    : raw=%u\n", eph->bd3_sattype);
    printf("  bd3_week       : raw=%u\n", eph->bd3_week);
    printf("  bd3_toe        : =%u s \n", eph->bd3_toe);
    printf("  bd3_toc        : =%u s \n", eph->bd3_toc);
    printf("  bd3_af0        : =% .12e s\n", eph->bd3_af0);
    printf("  bd3_af1        : =% .12e s/s\n", eph->bd3_af1);
    printf("  bd3_af2        : =% .12e s/s^2\n", eph->bd3_af2);
    printf("  bd3_iode       : raw=%u\n", eph->bd3_iode);
    printf("  bd3_iodc       : raw=%u\n", eph->bd3_iodc);
    printf("  bd3_idot       : =% .12e rad/s\n", eph->bd3_idot);
    printf("  bd3_crs        : =% .12e m\n", eph->bd3_crs);
    printf("  bd3_crc        : =% .12e m\n", eph->bd3_crc);
    printf("  bd3_cus        : =% .12e rad\n", eph->bd3_cus);
    printf("  bd3_cuc        : =% .12e rad\n", eph->bd3_cuc);
    printf("  bd3_cis        : =% .12e rad\n", eph->bd3_cis);
    printf("  bd3_cic        : =% .12e rad\n", eph->bd3_cic);
    printf("  bd3_delta_n0   : =% .12e rad/s\n", eph->bd3_delta_n0);
    printf("  bd3_delta_n0_dot: =% .12e rad/s^2\n", eph->bd3_delta_n0_dot);
    printf("  bd3_m0         : =% .12e rad\n", eph->bd3_m0);
    printf("  bd3_ecc        : =% .12e\n", eph->bd3_ecc);
    printf("  bd3_deltaA     : =% .12e m\n", eph->bd3_deltaA);
    printf("  bd3_adot       : =% .12e m/s\n", eph->bd3_adot);
    printf("  bd3_omega0     : =% .12e rad\n", eph->bd3_omega0);
    printf("  bd3_i0         : =% .12e rad\n", eph->bd3_i0);
    printf("  bd3_omega      : =% .12e rad\n", eph->bd3_omega);
    printf("  bd3_omegadot   : =% .12e rad/s\n", eph->bd3_omegadot);
    printf("  bd3_tgdb1cp    : =% .12e s\n", eph->bd3_tgdb1cp);
    printf("  bd3_tgdb2ap    : =% .12e s\n", eph->bd3_tgdb2ap);
    printf("  bd3_iscb1cd    : =% .12e s\n", eph->bd3_iscb1cd);
    printf("  bd3_reserved   : raw=%u\n", eph->bd3_reserved);
    printf("  payload_crc24  : raw=%06X calc=%06X [%s]\n",
           eph->crc24,
           payload_crc_calc,
           (eph->crc24 == payload_crc_calc) ? "OK" : "BAD");
}

void print_bdxwephb(const BDXWEPHB_Decoded_t *eph, uint32_t payload_crc_calc)
{
    printf("BDXWEPHB decoded frame:\n");
    printf("  gps_week_count  : %u\n", eph->gps_week_count);
    printf("  gps_tow_s       : %u\n", eph->gps_tow_s);
    printf("  xws_satid       : %u\n", eph->xws_satid);
    printf("  xws_sattype     : %u\n", eph->xws_sattype);
    printf("  xws_week        : %u\n", eph->xws_week);
    printf("  xws_toe         : %u s\n", eph->xws_toe);
    printf("  xws_toc         : %u s\n", eph->xws_toc);
    printf("  xws_af0         : %.12e s\n", eph->xws_af0);
    printf("  xws_af1         : %.12e s/s\n", eph->xws_af1);
    printf("  xws_iodc        : %u\n", eph->xws_iodc);
    printf("  xws_iode        : %u\n", eph->xws_iode);
    printf("  xws_crs         : %.12e m\n", eph->xws_crs);
    printf("  xws_crc         : %.12e m\n", eph->xws_crc);
    printf("  xws_cus         : %.12e rad\n", eph->xws_cus);
    printf("  xws_cuc         : %.12e rad\n", eph->xws_cuc);
    printf("  xws_cis         : %.12e rad\n", eph->xws_cis);
    printf("  xws_cic         : %.12e rad\n", eph->xws_cic);
    printf("  xws_delta_n0    : %.12e rad/s\n", eph->xws_delta_n0);
    printf("  xws_delta_n0_dot: %.12e rad/s^2\n", eph->xws_delta_n0_dot);
    printf("  xws_m0          : %.12e rad\n", eph->xws_m0);
    printf("  xws_ecc         : %.12e\n", eph->xws_ecc);
    printf("  xws_deltaA0     : %.12e m\n", eph->xws_deltaA0);
    printf("  xws_i0_dot      : %.12e rad/s\n", eph->xws_i0_dot);
    printf("  xws_a_dot       : %.12e m/s\n", eph->xws_a_dot);
    printf("  xws_delta_i0    : %.12e rad\n", eph->xws_delta_i0);
    printf("  xws_omega0      : %.12e rad\n", eph->xws_omega0);
    printf("  xws_omega_dot   : %.12e rad/s\n", eph->xws_omega_dot);
    printf("  xws_omega       : %.12e rad\n", eph->xws_omega);
    printf("  reserved        : 0x%llx\n", (unsigned long long)eph->xws_reserved);
    printf("  crc24           : 0x%06x (calc:0x%06x)%s\n", (unsigned)eph->crc24, payload_crc_calc,
           ((uint32_t)eph->crc24 == payload_crc_calc) ? " OK" : " ERR");
}

void print_bd3cnav2ephb(const BD3CNAV2EPHB_Decoded_t *eph, uint32_t payload_crc_calc)
{
    printf("BD3CNAV2EPHB decoded frame:\n");
    printf("  head: ");
    for (size_t i = 0; i < sizeof(eph->head); ++i)
        printf("%02X%s", eph->head[i], (i + 1 == sizeof(eph->head)) ? "" : " ");
    printf("\n");

    printf("  gps_week_count : raw=%u\n", eph->gps_week_count);
    printf("  gps_tow_s      : raw=%u s\n", eph->gps_tow_s);
    printf("  bds_satid      : raw=%u\n", eph->bds_satid);
    printf("  bds_sattype    : raw=%u\n", eph->bds_sattype);
    printf("  bds_week       : raw=%u\n", eph->bds_week);
    printf("  bds_toe        : raw=%u s \n", eph->bds_toe);
    printf("  bds_toc        : raw=%u s \n", eph->bds_toc);
    printf("  bds_af0        : =% .12e s\n", eph->bds_af0);
    printf("  bds_af1        : =% .12e s/s\n", eph->bds_af1);
    printf("  bds_af2        : =% .12e s/s^2\n", eph->bds_af2);
    printf("  bds_iode       : raw=%u\n", eph->bds_iode);
    printf("  bds_iodc       : raw=%u\n", eph->bds_iodc);
    printf("  bds_idot       : =% .12e rad/s\n", eph->bds_idot);
    printf("  bds_crs        : =% .12e m\n", eph->bds_crs);
    printf("  bds_crc        : =% .12e m\n", eph->bds_crc);
    printf("  bds_cus        : =% .12e rad\n", eph->bds_cus);
    printf("  bds_cuc        : =% .12e rad\n", eph->bds_cuc);
    printf("  bds_cis        : =% .12e rad\n", eph->bds_cis);
    printf("  bds_cic        : =% .12e rad\n", eph->bds_cic);
    printf("  bds_delta_n0   : =% .12e rad/s\n", eph->bds_delta_n0);
    printf("  bds_delta_n0_dot: =% .12e\n", eph->bds_delta_n0_dot);
    printf("  bds_m0         : =% .12e rad\n", eph->bds_m0);
    printf("  bds_ecc        : =% .12e\n", eph->bds_ecc);
    printf("  bds_AA         : =% .12e m\n", eph->bds_AA);
    printf("  bds_adot       : =% .12e m/s\n", eph->bds_adot);
    printf("  bds_omega0     : =% .12e rad\n", eph->bds_omega0);
    printf("  bds_i0         : =% .12e rad\n", eph->bds_i0);
    printf("  bds_omega      : =% .12e rad\n", eph->bds_omega);
    printf("  bds_omegadot   : =% .12e rad/s\n", eph->bds_omegadot);
    printf("  bds_tgdb1cp    : =% .12e s\n", eph->bds_tgdb1cp);
    printf("  bds_tgdb2ap    : =% .12e s\n", eph->bds_tgdb2ap);
    printf("  bds_iscb2ad    : =% .12e s\n", eph->bds_iscb2ad);
    printf("  bds_reserved   : raw=%u\n", eph->bds_reserved);
    printf("  payload_crc24  : raw=%06X calc=%06X [%s]\n",
           eph->crc24,
           payload_crc_calc,
           (eph->crc24 == payload_crc_calc) ? "OK" : "BAD");
}

void print_bd3cnav3ephb(const BD3CNAV3EPHB_Decoded_t *eph, uint32_t payload_crc_calc)
{
    printf("BD3CNAV3EPHB decoded frame:\n");
    printf("  head: ");
    for (size_t i = 0; i < sizeof(eph->head); ++i)
        printf("%02X%s", eph->head[i], (i + 1 == sizeof(eph->head)) ? "" : " ");
    printf("\n");

    printf("  gps_week_count : raw=%u\n", eph->gps_week_count);
    printf("  gps_tow_s      : raw=%u s\n", eph->gps_tow_s);
    printf("  bds_satid      : raw=%u\n", eph->bds_satid);
    printf("  bds_sattype    : raw=%u\n", eph->bds_sattype);
    printf("  bds_week       : raw=%u\n", eph->bds_week);
    printf("  bds_toe        : raw=%u s \n", eph->bds_toe);
    printf("  bds_toc        : raw=%u s \n", eph->bds_toc);
    printf("  bds_af0        : =% .12e s\n", eph->bds_af0);
    printf("  bds_af1        : =% .12e s/s\n", eph->bds_af1);
    printf("  bds_af2        : =% .12e s/s^2\n", eph->bds_af2);
    printf("  bds_iode       : raw=%u\n", eph->bds_iode);
    printf("  bds_iodc       : raw=%u\n", eph->bds_iodc);
    printf("  bds_idot       : =% .12e rad/s\n", eph->bds_idot);
    printf("  bds_crs        : =% .12e m\n", eph->bds_crs);
    printf("  bds_crc        : =% .12e m\n", eph->bds_crc);
    printf("  bds_cus        : =% .12e rad\n", eph->bds_cus);
    printf("  bds_cuc        : =% .12e rad\n", eph->bds_cuc);
    printf("  bds_cis        : =% .12e rad\n", eph->bds_cis);
    printf("  bds_cic        : =% .12e rad\n", eph->bds_cic);
    printf("  bds_delta_n0   : =% .12e rad/s\n", eph->bds_delta_n0);
    printf("  bds_delta_n0_dot: =% .12e\n", eph->bds_delta_n0_dot);
    printf("  bds_m0         : =% .12e rad\n", eph->bds_m0);
    printf("  bds_ecc        : =% .12e\n", eph->bds_ecc);
    printf("  bds_deltaA     : =% .12e m\n", eph->bds_deltaA);
    printf("  bds_adot       : =% .12e m/s\n", eph->bds_adot);
    printf("  bds_omega0     : =% .12e rad\n", eph->bds_omega0);
    printf("  bds_i0         : =% .12e rad\n", eph->bds_i0);
    printf("  bds_omega      : =% .12e rad\n", eph->bds_omega);
    printf("  bds_omegadot   : =% .12e rad/s\n", eph->bds_omegadot);
    printf("  bds_tgdb2bI    : =% .12e s\n", eph->bds_tgdb2bI);
    printf("  bds_reserved   : raw=%u\n", eph->bds_reserved);
    printf("  payload_crc24  : raw=%06X calc=%06X [%s]\n",
           eph->crc24,
           payload_crc_calc,
           (eph->crc24 == payload_crc_calc) ? "OK" : "BAD");
}

void print_gpsephb(const GPSEPHB_Decoded_t *eph, uint32_t payload_crc_calc)
{
    printf("GPSEPHB decoded frame:\n");
    printf("  head: ");
    for (size_t i = 0; i < sizeof(eph->head); ++i)
    {
        printf("%02X%s", eph->head[i], (i + 1 == sizeof(eph->head)) ? "" : " ");
    }
    printf("\n");

    printf("  gps_week_count   : raw=%u\n", eph->gps_week_count);
    printf("  gps_tow_s        : raw=%u s\n", eph->gps_tow_s);
    printf("  gps_satid        : raw=%u\n", eph->gps_satid);
    printf("  gps_sv_accuracy   : raw=%u m\n", eph->gps_sv_accuracy);
    printf("  gps_sv_health    : raw=%u\n", eph->gps_sv_health);
    printf("  gps_week         : raw=%u\n", eph->gps_week);
    printf("  gps_toe          : raw=%u s\n", eph->gps_toe);
    printf("  gps_toc          : raw=%u s\n", eph->gps_toc);
    printf("  gps_af0          : =% .12e s\n", eph->gps_af0);
    printf("  gps_af1          : =% .12e s/s\n", eph->gps_af1);
    printf("  gps_af2          : =% .12e s/s^2\n", eph->gps_af2);
    printf("  gps_iode         : raw=%u\n", eph->gps_iode);
    printf("  gps_iodc         : raw=%u\n", eph->gps_iodc);
    printf("  gps_idot         : =% .12e rad/s\n", eph->gps_idot);
    printf("  gps_crs          : =% .12e m\n", eph->gps_crs);
    printf("  gps_crc          : =% .12e m\n", eph->gps_crc);
    printf("  gps_cus          : =% .12e rad\n", eph->gps_cus);
    printf("  gps_cuc          : =% .12e rad\n", eph->gps_cuc);
    printf("  gps_cis          : =% .12e rad\n", eph->gps_cis);
    printf("  gps_cic          : =% .12e rad\n", eph->gps_cic);
    printf("  gps_delta_n      : =% .12e rad/s\n", eph->gps_delta_n);
    printf("  gps_m0           : =% .12e rad\n", eph->gps_m0);
    printf("  gps_ecc          : =% .12e\n", eph->gps_ecc);
    printf("  gps_a_half       : =% .12e m^1/2\n", eph->gps_a_half);
    printf("  gps_omega0       : =% .12e rad\n", eph->gps_omega0);
    printf("  gps_i0           : =% .12e rad\n", eph->gps_i0);
    printf("  gps_omega        : =% .12e rad\n", eph->gps_omega);
    printf("  gps_omegadot     : =% .12e rad/s\n", eph->gps_omegadot);
    printf("  gps_tgd          : =% .12e s\n", eph->gps_tgd);
    printf("  gps_code_on_l2   : raw=%u (%s)\n", eph->gps_code_on_l2, gps_l2_code_desc(eph->gps_code_on_l2));
    printf("  gps_l2p_data_flag: raw=%u\n", eph->gps_l2p_data_flag);
    printf("  gps_fit          : raw=%u (%s)\n", eph->gps_fit, gps_fit_desc(eph->gps_fit));
    printf("  reserved         : raw=%u\n", eph->reserved);
    printf("  payload_crc24    : raw=%06X calc=%06X [%s]\n",
           eph->crc24,
           payload_crc_calc,
           (eph->crc24 == payload_crc_calc) ? "OK" : "BAD");
}

void print_gpsephb_simple(const GPSEPHB_Decoded_t *eph)
{
    // printf("%d % .12e % .12e % .12e % .12e % .12e % .12e\n",
    //        eph->gps_satid, eph->gps_i0, eph->gps_omega0, eph->gps_ecc, eph->gps_omega, eph->gps_m0, eph->gps_delta_n);
    // printf("%d %u %u % .12e % .12e % .12e % .12e % .12e % .12e\n\n",
    //        eph->gps_satid, eph->gps_toe, eph->gps_toc, eph->gps_cuc, eph->gps_cus, eph->gps_crc, eph->gps_crs, eph->gps_omegadot);

    printf("%d %u %u % .12e % .12e % .12e % .12e % .12e % .12e % .12e % .12e %u % .12e \n",
           eph->gps_satid, eph->gps_toe, eph->gps_toc, eph->gps_af0, eph->gps_af1, eph->gps_cuc,
           eph->gps_cus, eph->gps_crc, eph->gps_crs, eph->gps_cic, eph->gps_cis, eph->gps_iodc, eph->gps_omegadot);
    printf("%d % .12e % .12e % .12e % .12e % .12e % .12e\n\n",
           eph->gps_satid, eph->gps_i0, eph->gps_omega0, eph->gps_ecc, eph->gps_omega, eph->gps_m0, eph->gps_delta_n);
}

void print_gloephb(const GLOEPHB_Decoded_t *eph, uint32_t payload_crc_calc)
{
    printf("GLOEPHB decoded frame:\n");
    printf("  head: ");
    for (size_t i = 0; i < sizeof(eph->head); ++i)
        printf("%02X%s", eph->head[i], (i + 1 == sizeof(eph->head)) ? "" : " ");
    printf("\n");

    printf("  gps_week_count : raw=%u\n", eph->gps_week_count);
    printf("  gps_tow_s      : raw=%u s\n", eph->gps_tow_s);
    printf("  glo_satid      : raw=%u\n", eph->glo_satid);
    printf("  glo_freq       : raw=%u\n", eph->glo_freq);
    printf("  glo_bn_msb     : raw=%u\n", eph->glo_bn_msb);
    printf("  glo_n4         : raw=%u (4-year cycles since 1996)\n", eph->glo_n4);
    printf("  glo_nt         : raw=%u (days since leap-year start)\n", eph->glo_nt);
    /* decode tk into hh:mm:ss */
    uint16_t tk = eph->glo_tk;
    uint8_t tk_hour = (tk >> 7) & 0x1F;
    uint8_t tk_min = (tk >> 1) & 0x3F;
    uint8_t tk_sec = (tk & 0x1) ? 30 : 0;
    printf("  glo_tk         : raw=%u => %02u:%02u:%02u\n", eph->glo_tk, tk_hour, tk_min, tk_sec);
    printf("  glo_tb         : raw=%u => %u min (tb*15min)\n", eph->glo_tb, eph->glo_tb * 15);
    printf("  glo_gamma      : =% .12e\n", eph->glo_gamma);
    printf("  glo_tau        : =% .12e s\n", eph->glo_tau);
    printf("  glo_x          : =% .12e km\n", eph->glo_x);
    printf("  glo_x_dot      : =% .12e km/s\n", eph->glo_x_dot);
    printf("  glo_x_ddot     : =% .12e km/s^2\n", eph->glo_x_ddot);
    printf("  glo_y          : =% .12e km\n", eph->glo_y);
    printf("  glo_y_dot      : =% .12e km/s\n", eph->glo_y_dot);
    printf("  glo_y_ddot     : =% .12e km/s^2\n", eph->glo_y_ddot);
    printf("  glo_z          : =% .12e km\n", eph->glo_z);
    printf("  glo_z_dot      : =% .12e km/s\n", eph->glo_z_dot);
    printf("  glo_z_ddot     : =% .12e km/s^2\n", eph->glo_z_ddot);
    printf("  payload_crc24  : raw=%06X calc=%06X [%s]\n",
           eph->crc24,
           payload_crc_calc,
           (eph->crc24 == payload_crc_calc) ? "OK" : "BAD");
}

void print_galephb(const GALEPHB_Decoded_t *eph, uint32_t payload_crc_calc)
{
    printf("GALEPHB decoded frame:\n");
    printf("  head: ");
    for (size_t i = 0; i < sizeof(eph->head); ++i)
        printf("%02X%s", eph->head[i], (i + 1 == sizeof(eph->head)) ? "" : " ");
    printf("\n");

    printf("  gps_week_count : raw=%u\n", eph->gps_week_count);
    printf("  gps_tow_s      : raw=%u s\n", eph->gps_tow_s);
    printf("  gal_satid      : raw=%u\n", eph->gal_satid);
    printf("  gal_sisa       : raw=%u\n", eph->gal_sisa);
    printf("  gal_e5b_sv_health: raw=%u\n", eph->gal_e5b_sv_health);
    printf("  gal_e5b_valid  : raw=%u\n", eph->gal_e5b_valid);
    printf("  gal_e1b_health : raw=%u\n", eph->gal_e1b_health);
    printf("  gal_e1b_valid  : raw=%u\n", eph->gal_e1b_valid);
    printf("  gal_week       : raw=%u\n", eph->gal_week);
    printf("  gal_toe        : raw=%u s \n", eph->gal_toe);
    printf("  gal_toc        : raw=%u s \n", eph->gal_toc);
    printf("  gal_af0        : =% .12e s\n", eph->gal_af0);
    printf("  gal_af1        : =% .12e s/s\n", eph->gal_af1);
    printf("  gal_af2        : =% .12e s/s^2\n", eph->gal_af2);
    printf("  gal_iodnav     : raw=%u\n", eph->gal_iodnav);
    printf("  gal_idot       : =% .12e π/s\n", eph->gal_idot);
    printf("  gal_crs        : =% .12e m\n", eph->gal_crs);
    printf("  gal_crc        : =% .12e m\n", eph->gal_crc);
    printf("  gal_cus        : =% .12e rad\n", eph->gal_cus);
    printf("  gal_cuc        : =% .12e rad\n", eph->gal_cuc);
    printf("  gal_cis        : =% .12e rad\n", eph->gal_cis);
    printf("  gal_cic        : =% .12e rad\n", eph->gal_cic);
    printf("  gal_delta_n    : =% .12e π/s\n", eph->gal_delta_n);
    printf("  gal_m0         : =% .12e π\n", eph->gal_m0);
    printf("  gal_ecc        : =% .12e\n", eph->gal_ecc);
    printf("  gal_a_half     : =% .12e m^1/2\n", eph->gal_a_half);
    printf("  gal_omega0     : =% .12e π\n", eph->gal_omega0);
    printf("  gal_i0         : =% .12e π\n", eph->gal_i0);
    printf("  gal_omega      : =% .12e π\n", eph->gal_omega);
    printf("  gal_omegadot   : =% .12e π/s\n", eph->gal_omegadot);
    printf("  gal_bgd_e5a_e1 : =% .12e s\n", eph->gal_bgd_e5a_e1);
    printf("  gal_bgd_e5b_e1 : =% .12e s\n", eph->gal_bgd_e5b_e1);
    printf("  gal_navtype    : raw=%u\n", eph->gal_navtype);
    printf("  gal_reserved   : raw=%u\n", eph->gal_reserved);
    printf("  payload_crc24  : raw=%06X calc=%06X [%s]\n",
           eph->crc24,
           payload_crc_calc,
           (eph->crc24 == payload_crc_calc) ? "OK" : "BAD");
}

void print_prangeb(const PRANGEB_Decoded_t *d, uint32_t payload_crc_calc)
{
    printf("PRANGEB decoded frame:\n");
    printf("  head: ");
    for (size_t i = 0; i < sizeof(d->head); ++i)
        printf("%02X%s", d->head[i], (i + 1 == sizeof(d->head)) ? "" : " ");
    printf("\n");
    printf("  gps_week_count: %u\n", d->gps_week_count);
    printf("  gps_tow_s     : %u s\n", d->gps_tow_s);
    printf("  ms_count      : %u\n", d->ms_count);
    printf("  sync_flag     : %u\n", d->sync_flag);
    printf("  system_id     : %u\n", d->system_id);
    printf("  sat_count     : %u\n", d->sat_count);

    for (size_t i = 0; i < d->sat_count; i++)
    {
        printf("  --- sat %zu ---\n", i);
        printf("    sat_id          : %u\n", d->sat_info[i].sat_id);
        printf("    signal_count    : %u\n", d->sat_info[i].signal_count);
        printf("    apd_ms          : % .12e ms\n", d->sat_info[i].apd_ms);
        printf("    approx_phase_rate: %d (Sint14 raw)\n", d->sat_info[i].approx_phase_rate);
        for (size_t j = 0; j < d->sat_info[i].signal_count; j++)
        {
            printf("      --- signal %zu ---\n", j);
            printf("        signal_id       : %u\n", d->sat_info[i].signal_info[j].signal_id);
            printf("        phase_lock_flag : %u\n", d->sat_info[i].signal_info[j].phase_lock_flag);
            printf("        precise_pr      : % .12e ms\n", d->sat_info[i].signal_info[j].precise_pr);
            printf("        precise_phase   : % .12e ms\n", d->sat_info[i].signal_info[j].precise_phase);
            printf("        precise_phase_rate: % .12e m/s\n", d->sat_info[i].signal_info[j].precise_phase_rate);
            printf("        cn0             : % .12e dB\n", d->sat_info[i].signal_info[j].cn0);
            printf("        half_cycle      : %u\n", d->sat_info[i].signal_info[j].half_cycle);
        }
    }

    printf("  embedded_crc24  : %06X calc=%06X [%s]\n",
           d->crc24, payload_crc_calc, (d->crc24 == payload_crc_calc) ? "OK" : "BAD");
}

void print_posdatab(const POSDATAB_Decoded_t *pos, uint32_t payload_crc_calc)
{
    printf("POSDATAB decoded frame:\n");
    printf("  head: ");
    for (size_t i = 0; i < sizeof(pos->head); ++i)
        printf("%02X%s", pos->head[i], (i + 1 == sizeof(pos->head)) ? "" : " ");
    printf("\n");

    printf("  time_system     : raw=%u (%s)\n", pos->time_system, pos_time_system_desc(pos->time_system));
    printf("  week_num        : raw=%u\n", pos->week_num);
    printf("  sec_in_week     : =% .12e s\n", pos->sec_in_week);
    printf("  leap_second     : raw=%d s\n", pos->leap_second);
    printf("  sys_status      : raw=%u (%s)\n", pos->sys_status, pos_sys_status_desc(pos->sys_status));
    printf("  pos_type        : raw=%u (%s)\n", pos->pos_type, pos_fix_type_desc(pos->pos_type));
    printf("  azi_type        : raw=%u (%s)\n", pos->azi_type, pos_fix_type_desc(pos->azi_type));
    printf("  latitude        : =% .12e deg\n", pos->latitude);
    printf("  longitude       : =% .12e deg\n", pos->longitude);
    printf("  altitude        : =% .12e m\n", pos->altitude);
    printf("  east_velocity   : =% .12e m/s\n", pos->east_velocity);
    printf("  north_velocity  : =% .12e m/s\n", pos->north_velocity);
    printf("  up_velocity     : =% .12e m/s\n", pos->up_velocity);
    printf("  pitch           : =% .12e deg\n", pos->pitch);
    printf("  roll            : =% .12e deg\n", pos->roll);
    printf("  azimuth         : =% .12e deg\n", pos->azimuth);
    printf("  lat_sigma       : =% .12e m\n", pos->lat_sigma);
    printf("  lon_sigma       : =% .12e m\n", pos->lon_sigma);
    printf("  altitude_sigma  : =% .12e m\n", pos->altitude_sigma);
    printf("  east_vel_sigma  : =% .12e m/s\n", pos->east_vel_sigma);
    printf("  north_vel_sigma : =% .12e m/s\n", pos->north_vel_sigma);
    printf("  up_vel_sigma    : =% .12e m/s\n", pos->up_vel_sigma);
    printf("  pitch_sigma     : =% .12e deg\n", pos->pitch_sigma);
    printf("  roll_sigma      : =% .12e deg\n", pos->roll_sigma);
    printf("  azimuth_sigma   : =% .12e deg\n", pos->azimuth_sigma);
    printf("  gnss_sat_m      : raw=%u\n", pos->gnss_sat_m);
    printf("  gnss_sat_s      : raw=%u\n", pos->gnss_sat_s);
    printf("  diff_age        : =% .12e s\n", pos->diff_age);
    printf("  odo_flag        : raw=%u\n", pos->odo_flag);
    printf("  gear            : raw=%u (%s)\n", pos->gear, pos_gear_desc(pos->gear));
    printf("  fl_wheel_speed  : =% .12e m/s\n", pos->fl_wheel_speed);
    printf("  fr_wheel_speed  : =% .12e m/s\n", pos->fr_wheel_speed);
    printf("  rl_wheel_speed  : =% .12e m/s\n", pos->rl_wheel_speed);
    printf("  rr_wheel_speed  : =% .12e m/s\n", pos->rr_wheel_speed);
    printf("  reserved0       : raw=%08X\n", pos->reserved0);
    printf("  reserved1       : raw=%08X\n", pos->reserved1);
    printf("  payload_crc24   : raw=%06X calc=%06X [%s]\n",
           pos->crc24,
           payload_crc_calc,
           (pos->crc24 == payload_crc_calc) ? "OK" : "BAD");
}

#pragma endregion "data print"

/*****************************************************************************************************************************************************************************************************/

#pragma region "file save"

char* gnss_raw_info_file_header(char *type, uint8_t enable)
{
    static char file_path[256];
    char time_str[64];
    // struct timeval tv; 
    // gettimeofday(&tv, NULL);
    time_t nowtime ;
    time(&nowtime);
    struct tm *nowtm = localtime(&nowtime);    
    strftime(time_str, sizeof(time_str), "%Y-%m-%d_%H-%M-%S", nowtm);
    snprintf(file_path, sizeof(file_path), "/root/%s_%s.csv", type, time_str);
    FILE *fp = NULL;
    if (strcmp(type, "gpsephb") == 0)
    {
        if (enable && (fp = fopen(file_path, "w")) && fp != NULL)
        {
            memcpy(ephb_file_sw.gpsephb.path, file_path, sizeof(ephb_file_sw.gpsephb.path));
            fprintf(fp, "gps_week_count,gps_tow_s,gps_satid,gps_sv_accuracy,gps_sv_health,gps_week,gps_toe,gps_toc,"
                        "gps_af0,gps_af1,gps_af2,"
                        "gps_iode,gps_iodc,"
                        "gps_idot,gps_crs,gps_crc,gps_cus,gps_cuc,gps_cis,gps_cic,gps_delta_n,"
                        "gps_m0,gps_ecc,gps_a_half,gps_omega0,gps_i0,gps_omega,gps_omegadot,gps_tgd,"
                        "gps_code_on_l2,gps_l2p_data_flag,gps_fit\n");
        }
        ephb_file_sw.gpsephb.en = enable ? 1 : 0;
    }
    else if (strcmp(type, "gloephb") == 0)
    {
        if (enable && (fp = fopen(file_path, "w")) && fp != NULL)
        {
            memcpy(ephb_file_sw.gloephb.path, file_path, sizeof(ephb_file_sw.gloephb.path));
            fprintf(fp, "gps_week_count,gps_tow_s,glo_satid,glo_freq,glo_bn_msb,glo_n4,glo_nt,glo_tk_hour:glo_tk_min:glo_tk_sec,glo_tb_minute:glo_tb_minute*glo_tb, glo_gamma, glo_tau, glo_x, glo_x_dot, glo_x_ddot, glo_y, glo_y_dot, glo_y_ddot, glo_z, glo_z_dot, glo_z_ddot\n");
        }
        ephb_file_sw.gloephb.en = enable ? 1 : 0;
    }
    else if (strcmp(type, "galephb") == 0)
    {
        if (enable && (fp = fopen(file_path, "w")) && fp != NULL)
        {
            memcpy(ephb_file_sw.galephb.path, file_path, sizeof(ephb_file_sw.galephb.path));
            fprintf(fp, "gps_week_count,gps_tow_s,gal_satid,gal_sisa,gal_e5b_sv_health,gal_e5b_valid,gal_e1b_health,gal_e1b_valid,gal_week,gal_toe,gal_toc,gal_af0,gal_af1,gal_af2,gal_iodnav,gal_idot,gal_crs,gal_crc,gal_cus,gal_cuc,gal_cis,gal_cic,gal_delta_n,gal_m0,gal_ecc,gal_a_half,gal_omega0,gal_i0,gal_omega,gal_omegadot,gal_bgd_e5a_e1,gal_bgd_e5b_e1,gal_navtype\n");
        }
        ephb_file_sw.galephb.en = enable ? 1 : 0;
    }
    else if (strcmp(type, "bd2ephb") == 0)
    {
        if (enable && (fp = fopen(file_path, "w")) && fp != NULL)
        {
            memcpy(ephb_file_sw.bd2ephb.path, file_path, sizeof(ephb_file_sw.bd2ephb.path));
            fprintf(fp, "gps_week_count,gps_tow_s,bd2_satid,bd2_sv_urai,bd2_sv_health,bd2_week,bd2_toe,bd2_toc,bd2_af0,bd2_af1,bd2_af2,bd2_aode,bd2_aodc,bd2_idot,bd2_crs,bd2_crc,bd2_cus,bd2_cuc,bd2_cis,bd2_cic,bd2_delta_n,bd2_m0,bd2_ecc,bd2_a_half,bd2_omega0,bd2_i0,bd2_omega,bd2_omegadot,bd2_tgd1,bd2_tgd2\n");
        }
        ephb_file_sw.bd2ephb.en = enable ? 1 : 0;
    }
    else if (strcmp(type, "bd3ephb") == 0)
    {
        if (enable && (fp = fopen(file_path, "w")) && fp != NULL)
        {
            memcpy(ephb_file_sw.bd3ephb.path, file_path, sizeof(ephb_file_sw.bd3ephb.path));
            fprintf(fp, "gps_week_count,gps_tow_s,bd3_satid,bd3_sattype,bd3_week,bd3_toe,bd3_toc,bd3_af0,bd3_af1,bd3_af2,bd3_iode,bd3_iodc,bd3_idot,bd3_crs,bd3_crc,bd3_cus,bd3_cuc,bd3_cis,bd3_cic	bd3_delta_n0	bд3_delta_n0_dot	bд3_m0	bд3_ecc	bд3_deltaA	bд3_adot	bд3_omega0	bд3_i0	bд3_omega	bд3_omegadot	bд3_tgdb1cp	bд3_tgdb2ap	bд3_iscb1cd\n");
        }
        ephb_file_sw.bd3ephb.en = enable ? 1 : 0;
    }
    else if (strcmp(type, "bdxwephb") == 0)
    {
        if (enable && (fp = fopen(file_path, "w")) && fp != NULL)
        {
            memcpy(ephb_file_sw.bdxwephb.path, file_path, sizeof(ephb_file_sw.bdxwephb.path));
            fprintf(fp, "gps_week_count,gps_tow_s,xws_satid,xws_sattype,xws_week,xws_toe,xws_toc,xws_af0,xws_af1,xws_iodc,xws_iode,xws_crs,xws_crc,xws_cus,xws_cuc,xws_cis,xws_cic,xws_delta_n0,xws_delta_n0_dot,xws_m0,xws_ecc,xws_deltaA0,xws_i0_dot,xws_a_dot,xws_delta_i0,xws_omega0,xws_omega_dot,xws_omega\n");
        }
        ephb_file_sw.bdxwephb.en = enable ? 1 : 0;
    }
    else if (strcmp(type, "bd3cnav2ephb") == 0)
    {
        if (enable && (fp = fopen(file_path, "w")) && fp != NULL)
        {
            memcpy(ephb_file_sw.bd3cnav2ephb.path, file_path, sizeof(ephb_file_sw.bd3cnav2ephb.path));
            fprintf(fp, "gps_week_count,gps_tow_s,bds_satid,bds_sattype,bds_week,bds_toe,bds_toc,bds_af0,bds_af1,bds_af2,bds_iode,bds_iodc,bds_idot,bds_crs,bds_crc,bds_cus,bds_cuc,bds_cis,bds_cic,bds_delta_n0,bds_delta_n0_dot,bds_m0,bds_ecc,bds_AA,bds_adot,bds_omega0,bds_i0,bds_omega,bds_omegadot,bds_tgdb1cp,bds_tgdb2ap,bds_iscb2ad\n");
        }
        ephb_file_sw.bd3cnav2ephb.en = enable ? 1 : 0;
    }
    else if (strcmp(type, "bd3cnav3ephb") == 0)
    {
        if (enable && (fp = fopen(file_path, "w")) && fp != NULL)
        {
            memcpy(ephb_file_sw.bd3cnav3ephb.path, file_path, sizeof(ephb_file_sw.bd3cnav3ephb.path));
            fprintf(fp, "gps_week_count,gps_tow_s,bds_satid,bds_sattype,bds_week,bds_toe,bds_toc,bds_af0,bds_af1,bds_af2,bds_iode,bds_iodc,but bgs_idot,but bgs_crs,but bgs_crc,but bgs_cus,but bgs_cuc,but bgs_cis,but bgs_cic,but bgs_delta_n0,but bgs_delta_n0_dot,but bgs_m0,but bgs_ecc,but bgs_deltaA,but bgs_adot,but bgs_omega0,but bgs_i0,but bgs_omega,but bgs_omegadot,but bgs_tgdb2bI\n");
        }
        ephb_file_sw.bd3cnav3ephb.en = enable ? 1 : 0;
    }
    if (fp)
    fclose(fp);
    gnss_cfg_dis_enable(gnss_ctrl.fd, type, enable, 1);
    return file_path;
}

void bd2ephb_file_header()
{
    FILE *fp = fopen(TEST_BD2EPHB_FILE_PATH, "w");
    if (fp)
    {
        fprintf(fp, "gps_week_count,gps_tow_s,bd2_satid,bd2_sv_urai,bd2_sv_health,bd2_week,bd2_toe,bd2_toc,bd2_af0,bd2_af1,bd2_af2,bd2_aode,bd2_aodc,bd2_idot,bd2_crs,bd2_crc,bd2_cus,bd2_cuc,bd2_cis,bd2_cic,bd2_delta_n,bd2_m0,bd2_ecc,bd2_a_half,bd2_omega0,bd2_i0,bd2_omega,bd2_omegadot,bd2_tgd1,bd2_tgd2\n");
        fclose(fp);
    }
}

void bd2ephb_file_append(const BD2EPHB_Decoded_t *eph)
{
    FILE *fp = fopen(ephb_file_sw.bd2ephb.path, "a");
    if (fp)
    {
        fprintf(fp, "%u,%u,%u,%u,%u,%u,%u,%u,% .12e,% .12e,% .12e,%u,%u,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e\n",
                eph->gps_week_count,
                eph->gps_tow_s,
                eph->bd2_satid,
                eph->bd2_sv_urai,
                eph->bd2_sv_health,
                eph->bd2_week,
                eph->bd2_toe,
                eph->bd2_toc,
                eph->bd2_af0,
                eph->bd2_af1,
                eph->bd2_af2,
                eph->bd2_aode,
                eph->bd2_aodc,
                eph->bd2_idot,
                eph->bd2_crs,
                eph->bd2_crc,
                eph->bd2_cus,
                eph->bd2_cuc,
                eph->bd2_cis,
                eph->bd2_cic,
                eph->bd2_delta_n,
                eph->bd2_m0,
                eph->bd2_ecc,
                eph->bd2_a_half,
                eph->bd2_omega0,
                eph->bd2_i0,
                eph->bd2_omega,
                eph->bd2_omegadot,
                eph->bd2_tgd1,
                eph->bd2_tgd2);
        fclose(fp);
    }
}

void bd3ephb_file_header()
{
    FILE *fp = fopen(TEST_BD3EPHB_FILE_PATH, "w");
    if (fp)
    {
        fprintf(fp, "gps_week_count,gps_tow_s,bd3_satid,bd3_sattype,bd3_week,bd3_toe,bd3_toc,bd3_af0,bd3_af1,bd3_af2,bd3_iode,bd3_iodc,bd3_idot,bd3_crs,bd3_crc,bd3_cus,bd3_cuc,bd3_cis,bd3_cic,bd3_delta_n0,bd3_delta_n0_dot,bd3_m0,bd3_ecc,bd3_deltaA,bd3_adot,bd3_omega0,bd3_i0,bd3_omega,bd3_omegadot,bd3_tgdb1cp,bd3_tgdb2ap,bd3_iscb1cd\n");
        fclose(fp);
    }
}

void bd3ephb_file_append(const BD3EPHB_Decoded_t *eph)
{
    FILE *fp = fopen(ephb_file_sw.bd3ephb.path, "a");
    if (fp)
    {
        fprintf(fp, "%u,%u,%u,%u,%u,%u,%u,"
                    "% .12e,% .12e,% .12e,%u,%u,"
                    "% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,"
                    "% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e\n",
                eph->gps_week_count,
                eph->gps_tow_s,
                eph->bd3_satid,
                eph->bd3_sattype,
                eph->bd3_week,
                eph->bd3_toe,
                eph->bd3_toc,
                eph->bd3_af0,
                eph->bd3_af1,
                eph->bd3_af2,
                eph->bd3_iode,
                eph->bd3_iodc,
                eph->bd3_idot,
                eph->bd3_crs,
                eph->bd3_crc,
                eph->bd3_cus,
                eph->bd3_cuc,
                eph->bd3_cis,
                eph->bd3_cic,
                eph->bd3_delta_n0,
                eph->bd3_delta_n0_dot,
                eph->bd3_m0,
                eph->bd3_ecc,
                eph->bd3_deltaA,
                eph->bd3_adot,
                eph->bd3_omega0,
                eph->bd3_i0,
                eph->bd3_omega,
                eph->bd3_omegadot,
                eph->bd3_tgdb1cp,
                eph->bd3_tgdb2ap,
                eph->bd3_iscb1cd);
        fclose(fp);
    }
}

void bdxwephb_file_header()
{
    FILE *fp = fopen(TEST_BDXWEPHB_FILE_PATH, "w");
    if (fp)
    {
        fprintf(fp, "gps_week_count,gps_tow_s,xws_satid,xws_sattype,xws_week,xws_toe,xws_toc,xws_af0,xws_af1,xws_iodc,xws_iode,xws_crs,xws_crc,xws_cus,xws_cuc,xws_cis,xws_cic,xws_delta_n0,xws_delta_n0_dot,xws_m0,xws_ecc,xws_deltaA0,xws_i0_dot,xws_a_dot,xws_delta_i0,xws_omega0,xws_omega_dot,xws_omega\n");
        fclose(fp);
    }
}

void bdxwephb_file_append(const BDXWEPHB_Decoded_t *eph)
{
    FILE *fp = fopen(ephb_file_sw.bdxwephb.path, "a");
    if (fp)
    {
        fprintf(fp, "%u,%u,%u,%u,%u,%u,%u,%.12e,%.12e,%u,%u,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e\n",
                eph->gps_week_count,
                eph->gps_tow_s,
                eph->xws_satid,
                eph->xws_sattype,
                eph->xws_week,
                eph->xws_toe,
                eph->xws_toc,
                eph->xws_af0,
                eph->xws_af1,
                eph->xws_iodc,
                eph->xws_iode,
                eph->xws_crs,
                eph->xws_crc,
                eph->xws_cus,
                eph->xws_cuc,
                eph->xws_cis,
                eph->xws_cic,
                eph->xws_delta_n0,
                eph->xws_delta_n0_dot,
                eph->xws_m0,
                eph->xws_ecc,
                eph->xws_deltaA0,
                eph->xws_i0_dot,
                eph->xws_a_dot,
                eph->xws_delta_i0,
                eph->xws_omega0,
                eph->xws_omega_dot,
                eph->xws_omega);
        fclose(fp);
    }
}

void bd3cnav2ephb_file_header()
{
    FILE *fp = fopen(TEST_BD3CNAV2EPHB_FILE_PATH, "w");
    if (fp)
    {
        fprintf(fp, "gps_week_count,gps_tow_s,bds_satid,bds_sattype,bds_week,bds_toe,bds_toc,bds_af0,bds_af1,bds_af2,bds_iode,bds_iodc,bds_idot,bds_crs,bds_crc,bds_cus,bds_cuc,bds_cis,bds_cic,bds_delta_n0,bds_delta_n0_dot,bds_m0,bds_ecc,bds_AA,bds_adot,bds_omega0,bds_i0,bds_omega,bds_omegadot,bds_tgdb1cp,bds_tgdb2ap,bds_iscb2ad\n");
        fclose(fp);
    }
}

void bd3cnav2ephb_file_append(const BD3CNAV2EPHB_Decoded_t *eph)
{
    FILE *fp = fopen(ephb_file_sw.bd3cnav2ephb.path, "a");
    if (fp)
    {
        fprintf(fp, "%u,%u,%u,%u,%u,%u,%u,"
                    "% .12e,% .12e,% .12e,%u,%u,"
                    "% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,"
                    "% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e\n",
                eph->gps_week_count,
                eph->gps_tow_s,
                eph->bds_satid,
                eph->bds_sattype,
                eph->bds_week,
                eph->bds_toe,
                eph->bds_toc,
                eph->bds_af0,
                eph->bds_af1,
                eph->bds_af2,
                eph->bds_iode,
                eph->bds_iodc,
                eph->bds_idot,
                eph->bds_crs,
                eph->bds_crc,
                eph->bds_cus,
                eph->bds_cuc,
                eph->bds_cis,
                eph->bds_cic,
                eph->bds_delta_n0,
                eph->bds_delta_n0_dot,
                eph->bds_m0,
                eph->bds_ecc,
                eph->bds_AA,
                eph->bds_adot,
                eph->bds_omega0,
                eph->bds_i0,
                eph->bds_omega,
                eph->bds_omegadot,
                eph->bds_tgdb1cp,
                eph->bds_tgdb2ap,
                eph->bds_iscb2ad);
        fclose(fp);
    }
}

void bd3cnav3ephb_file_header()
{
    FILE *fp = fopen(TEST_BD3CNAV3EPHB_FILE_PATH, "w");
    if (fp)
    {
        fprintf(fp, "gps_week_count,gps_tow_s,bds_satid,bds_sattype,bds_week,bds_toe,bds_toc,bds_af0,bds_af1,bds_af2,bds_iode,bds_iodc,bds_idot,bds_crs,bds_crc,bds_cus,bds_cuc,bds_cis,bds_cic,bds_delta_n0,bds_delta_n0_dot,bds_m0,bds_ecc,bds_deltaA,bds_adot,bds_omega0,bds_i0,bds_omega,bds_omegadot,bds_tgdb2bI\n");
        fclose(fp);
    }
}

void bd3cnav3ephb_file_append(const BD3CNAV3EPHB_Decoded_t *eph)
{
    FILE *fp = fopen(ephb_file_sw.bd3cnav3ephb.path, "a");
    if (fp)
    {
        fprintf(fp, "%u,%u,%u,%u,%u,%u,%u,"
                    "% .12e,% .12e,% .12e,%u,%u,"
                    "% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,"
                    "% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e\n",
                eph->gps_week_count,
                eph->gps_tow_s,
                eph->bds_satid,
                eph->bds_sattype,
                eph->bds_week,
                eph->bds_toe,
                eph->bds_toc,
                eph->bds_af0,
                eph->bds_af1,
                eph->bds_af2,
                eph->bds_iode,
                eph->bds_iodc,
                eph->bds_idot,
                eph->bds_crs,
                eph->bds_crc,
                eph->bds_cus,
                eph->bds_cuc,
                eph->bds_cis,
                eph->bds_cic,
                eph->bds_delta_n0,
                eph->bds_delta_n0_dot,
                eph->bds_m0,
                eph->bds_ecc,
                eph->bds_deltaA,
                eph->bds_adot,
                eph->bds_omega0,
                eph->bds_i0,
                eph->bds_omega,
                eph->bds_omegadot,
                eph->bds_tgdb2bI);
        fclose(fp);
    }
}

void gpsephb_file_header()
{
    FILE *f = fopen(ephb_file_sw.gpsephb.path, "w");
    if (f)
    {
        fprintf(f, "gps_week_count,gps_tow_s,gps_satid,gps_sv_accuracy,gps_sv_health,gps_week,gps_toe,gps_toc,"
                   "gps_af0,gps_af1,gps_af2,"
                   "gps_iode,gps_iodc,"
                   "gps_idot,gps_crs,gps_crc,gps_cus,gps_cuc,gps_cis,gps_cic,gps_delta_n,"
                   "gps_m0,gps_ecc,gps_a_half,gps_omega0,gps_i0,gps_omega,gps_omegadot,gps_tgd,"
                   "gps_code_on_l2,gps_l2p_data_flag,gps_fit\n");
    }
    fclose(f);
}

void gpsephb_file_header2()
{
    FILE *f = fopen(ephb_file_sw.gpsephb.path, "w");
    if (f)
    {
        fprintf(f, "周计数,周内秒,卫星号,用户等效距离精度(m),星自主健康标识,时间周计数,卫星星历参考时间(s),卫星钟参考时刻(s),"
                   "卫星钟钟差改正参数(s),卫星钟钟速改正参数(s/s),卫星钟钟漂改正参数(s/s^2),"
                   "卫星星历数据期号,卫星钟参数期卷号,"
                   "卫星轨道倾角变化率(π/s),卫星轨道半径正弦调和改正项的振幅(m),卫星轨道半径余弦调和改正项的振幅(m),"
                   "卫星纬度幅角正弦调和改正项的振幅(rad),卫星纬度幅角余弦调和改正项的振幅(rad),"
                   "卫星轨道倾角正弦调和改正项的振幅(rad),卫星轨道倾角余弦调和改正项的振幅(rad),"
                   "卫星平均运动速率与计算值之差(π/s),"
                   "卫星参考时间的平近点角(π),卫星轨道偏心率,卫星轨道长半轴的平方根(m^1/2),卫星按参考时间计算的升交点赤经(π),卫星参考时间的轨道倾角(π),卫星近地点幅角(π),卫星升交点赤经变化率(π/s),卫星L1和L2信号频率的群延迟差(s),"
                   "L2测距码标志,L2P码导航电文可用状态,曲线拟合标志\n");
    }
    fclose(f);
}

void gpsephb_file_append(const GPSEPHB_Decoded_t *eph)
{
    FILE *f = fopen(ephb_file_sw.gpsephb.path, "a+");
    if (f)
    {
        char buffer[2048];
        snprintf(buffer, sizeof(buffer), "%u,%u,%u,%u,%u,%u,%u,%u,"
                                         "% .12e,% .12e,% .12e,"
                                         "%u,%u,"
                                         "% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,"
                                         "% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,% .12e,"
                                         "%u,%u,%u\n",
                 eph->gps_week_count, eph->gps_tow_s, eph->gps_satid, eph->gps_sv_accuracy, eph->gps_sv_health, eph->gps_week, eph->gps_toe, eph->gps_toc,
                 eph->gps_af0, eph->gps_af1, eph->gps_af2,
                 eph->gps_iode, eph->gps_iodc,
                 eph->gps_idot, eph->gps_crs, eph->gps_crc, eph->gps_cus, eph->gps_cuc, eph->gps_cis, eph->gps_cic, eph->gps_delta_n,
                 eph->gps_m0, eph->gps_ecc, eph->gps_a_half, eph->gps_omega0, eph->gps_i0, eph->gps_omega, eph->gps_omegadot, eph->gps_tgd,
                 eph->gps_code_on_l2, eph->gps_l2p_data_flag, eph->gps_fit);
        fputs(buffer, f);
    }
    fclose(f);
}

#pragma endregion "file save"
