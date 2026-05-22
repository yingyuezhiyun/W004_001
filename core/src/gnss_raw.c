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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// #define Header (0x43534847)
#define Header (0x47485343) //

#define RAW_PACKET_HEADER_LEN 8
#define RAW_PACKET_CRC_LEN 3
#define GPSEPHB_PAYLOAD_LEN 64
#define BD2EPHB_PAYLOAD_LEN 67

#define BD3CNAV3EPHB_PAYLOAD_LEN 76

// type类型和length字节长度采用小端在前输出方式，其他参数采用大端在前输出方式
#pragma pack(push, 1)
typedef struct
{
    uint32_t header;
    uint16_t type;
    uint16_t length;
} gns_raw_packet_header_t;
#pragma pack(pop)

enum
{
    RAW_BD2EPHB = 01, // BDS BD2 星历
    RAW_BD3EPHB = 03, // BDS BD3 星历
    RAW_GPSEPHB = 11, // GPS 星历
    RAW_GLOEPHB = 21, // GLONASS 星历
    RAW_GALEPHB = 32, // Galileo 星历
    RAW_BD3CNAV1EPHB = 23,
    RAW_BD3CNAV2EPHB = 24,
    RAW_BD3CNAV3EPHB = 25,
    RAW_PRANGEB = 61,
    RAW_PRANGE2B = 62,
};
#pragma pack(push, 1)
typedef struct
{
    uint32_t header;
    uint16_t type;
    uint16_t length;
    // uint8_t payload[0];
    uint8_t checksum[3];
} gns_raw_data_packet_t;
#pragma pack(pop)

const uint32_t tbl_CRC24Q[] = {

    0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17,
    0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E,
    0xC54E89, 0x430272, 0x4F9B84, 0xC9D77F, 0x56A868, 0xD0E493, 0xDC7D65, 0x5A319E,
    0x64CFB0, 0xE2834B, 0xEE1ABD, 0x685646, 0xF72951, 0x7165AA, 0x7DFC5C, 0xFBB0A7,
    0x0CD1E9, 0x8A9D12, 0x8604E4, 0x00481F, 0x9F3708, 0x197BF3, 0x15E205, 0x93AEFE,
    0xAD50D0, 0x2B1C2B, 0x2785DD, 0xA1C926, 0x3EB631, 0xB8FACA, 0xB4633C, 0x322FC7,
    0xC99F60, 0x4FD39B, 0x434A6D, 0xC50696, 0x5A7981, 0xDC357A, 0xD0AC8C, 0x56E077,
    0x681E59, 0xEE52A2, 0xE2CB54, 0x6487AF, 0xFBF8B8, 0x7DB443, 0x712DB5, 0xF7614E,
    0x19A3D2, 0x9FEF29, 0x9376DF, 0x153A24, 0x8A4533, 0x0C09C8, 0x00903E, 0x86DCC5,
    0xB822EB, 0x3E6E10, 0x32F7E6, 0xB4BB1D, 0x2BC40A, 0xAD88F1, 0xA11107, 0x275DFC,
    0xDCED5B, 0x5AA1A0, 0x563856, 0xD074AD, 0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C,
    0x7D6C62, 0xFB2099, 0xF7B96F, 0x71F594, 0xEE8A83, 0x68C678, 0x645F8E, 0xE21375,
    0x15723B, 0x933EC0, 0x9FA736, 0x19EBCD, 0x8694DA, 0x00D821, 0x0C41D7, 0x8A0D2C,
    0xB4F302, 0x32BFF9, 0x3E260F, 0xB86AF4, 0x2715E3, 0xA15918, 0xADC0EE, 0x2B8C15,
    0xD03CB2, 0x567049, 0x5AE9BF, 0xDCA544, 0x43DA53, 0xC596A8, 0xC90F5E, 0x4F43A5,
    0x71BD8B, 0xF7F170, 0xFB6886, 0x7D247D, 0xE25B6A, 0x641791, 0x688E67, 0xEEC29C,
    0x3347A4, 0xB50B5F, 0xB992A9, 0x3FDE52, 0xA0A145, 0x26EDBE, 0x2A7448, 0xAC38B3,
    0x92C69D, 0x148A66, 0x181390, 0x9E5F6B, 0x01207C, 0x876C87, 0x8BF571, 0x0DB98A,
    0xF6092D, 0x7045D6, 0x7CDC20, 0xFA90DB, 0x65EFCC, 0xE3A337, 0xEF3AC1, 0x69763A,
    0x578814, 0xD1C4EF, 0xDD5D19, 0x5B11E2, 0xC46EF5, 0x42220E, 0x4EBBF8, 0xC8F703,
    0x3F964D, 0xB9DAB6, 0xB54340, 0x330FBB, 0xAC70AC, 0x2A3C57, 0x26A5A1, 0xA0E95A,
    0x9E1774, 0x185B8F, 0x14C279, 0x928E82, 0x0DF195, 0x8BBD6E, 0x872498, 0x016863,
    0xFAD8C4, 0x7C943F, 0x700DC9, 0xF64132, 0x693E25, 0xEF72DE, 0xE3EB28, 0x65A7D3,
    0x5B59FD, 0xDD1506, 0xD18CF0, 0x57C00B, 0xC8BF1C, 0x4EF3E7, 0x426A11, 0xC426EA,
    0x2AE476, 0xACA88D, 0xA0317B, 0x267D80, 0xB90297, 0x3F4E6C, 0x33D79A, 0xB59B61,
    0x8B654F, 0x0D29B4, 0x01B042, 0x87FCB9, 0x1883AE, 0x9ECF55, 0x9256A3, 0x141A58,
    0xEFAAFF, 0x69E604, 0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5, 0xF69913, 0x70D5E8,
    0x4E2BC6, 0xC8673D, 0xC4FECB, 0x42B230, 0xDDCD27, 0x5B81DC, 0x57182A, 0xD154D1,
    0x26359F, 0xA07964, 0xACE092, 0x2AAC69, 0xB5D37E, 0x339F85, 0x3F0673, 0xB94A88,
    0x87B4A6, 0x01F85D, 0x0D61AB, 0x8B2D50, 0x145247, 0x921EBC, 0x9E874A, 0x18CBB1,
    0xE37B16, 0x6537ED, 0x69AE1B, 0xEFE2E0, 0x709DF7, 0xF6D10C, 0xFA48FA, 0x7C0401,
    0x42FA2F, 0xC4B6D4, 0xC82F22, 0x4E63D9, 0xD11CCE, 0x575035, 0x5BC9C3, 0xDD8538};

uint32_t rtk_crc24q(const uint8_t *buff, int len)
{
    uint32_t crc = 0;
    int i;
    for (i = 0; i < len; i++)
        crc = ((crc << 8) & 0xFFFFFF) ^ tbl_CRC24Q[(crc >> 16) ^ buff[i]];
    return crc;
}

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

static void print_bd2ephb(const BD2EPHB_Decoded_t *eph, uint32_t payload_crc_calc)
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

static void print_bd3ephb(const BD3EPHB_Decoded_t *eph, uint32_t payload_crc_calc)
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

static void print_gloephb(const GLOEPHB_Decoded_t *eph, uint32_t payload_crc_calc)
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

static void print_galephb(const GALEPHB_Decoded_t *eph, uint32_t payload_crc_calc)
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
    printf("  gal_toe        : raw=%u s (需按协议乘比例)\n", eph->gal_toe);
    printf("  gal_toc        : raw=%u s (需按协议乘比例)\n", eph->gal_toc);
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

static void print_bd3cnav1ephb(const BD3CNAV1EPHB_Decoded_t *eph, uint32_t payload_crc_calc)
{
    printf("BD3CNAV1EPHB decoded frame:\n");
    printf("  head: ");
    for (size_t i = 0; i < sizeof(eph->head); ++i)
        printf("%02X%s", eph->head[i], (i + 1 == sizeof(eph->head)) ? "" : " ");
    printf("\n");

    printf("  gps_week_count : raw=%u\n", eph->gps_week_count);
    printf("  gps_tow_s      : raw=%u s\n", eph->gps_tow_s);
    printf("  bds_satid      : raw=%u\n", eph->bds_satid);
    printf("  bds_sattype    : raw=%u\n", eph->bds_sattype);
    printf("  bds_week       : raw=%u\n", eph->bds_week);
    printf("  bds_toe        : raw=%u s (需乘300s)\n", eph->bds_toe);
    printf("  bds_toc        : raw=%u s (需乘300s)\n", eph->bds_toc);
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
    printf("  bds_tgdb1cp    : =% .12e s\n", eph->bds_tgdb1cp);
    printf("  bds_tgdb2ap    : =% .12e s\n", eph->bds_tgdb2ap);
    printf("  bds_iscb1cd    : =% .12e s\n", eph->bds_iscb1cd);
    printf("  bds_reserved2  : raw=%u\n", eph->bds_reserved2);
    printf("  payload_crc24  : raw=%06X calc=%06X [%s]\n",
           eph->crc24,
           payload_crc_calc,
           (eph->crc24 == payload_crc_calc) ? "OK" : "BAD");
}

static void print_bd3cnav2ephb(const BD3CNAV2EPHB_Decoded_t *eph, uint32_t payload_crc_calc)
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
    printf("  bds_toe        : raw=%u s (需乘300s)\n", eph->bds_toe);
    printf("  bds_toc        : raw=%u s (需乘300s)\n", eph->bds_toc);
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


static void print_prangeb(const PRANGEB_Decoded_t *d, uint32_t payload_crc_calc)
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


static void print_bd3cnav3ephb(const BD3CNAV3EPHB_Decoded_t *eph, uint32_t payload_crc_calc)
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
    printf("  bds_toe        : raw=%u s (需乘300s)\n", eph->bds_toe);
    printf("  bds_toc        : raw=%u s (需乘300s)\n", eph->bds_toc);
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

static void print_gpsephb(const GPSEPHB_Decoded_t *eph, uint32_t payload_crc_calc)
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

static void print_gpsephb_simple(const GPSEPHB_Decoded_t *eph)
{
    printf("%d % .12e % .12e % .12e % .12e % .12e % .12e\n",
           eph->gps_satid, eph->gps_i0, eph->gps_omega0, eph->gps_ecc, eph->gps_omega, eph->gps_m0, eph->gps_delta_n);
    printf("%d %u %u % .12e % .12e % .12e % .12e % .12e % .12e\n\n",
           eph->gps_satid, eph->gps_toe, eph->gps_toc, eph->gps_cuc, eph->gps_cus, eph->gps_crc, eph->gps_crs, eph->gps_omegadot);
}

int handle_gnss_raw(const uint8_t *data, size_t len)
{
    int handle_cnt = 0;
    for (size_t i = 0; i < len - 3; i++)
    {
        gns_raw_data_packet_t *packet = (gns_raw_data_packet_t *)(data + i);
        if (packet->header == Header)
        {
            // printf("Packet header found at offset %zu\n", i);
            if (packet->length > len - i - sizeof(gns_raw_data_packet_t))
            {
                continue;
            }
            uint32_t crc_calculated = rtk_crc24q((const uint8_t *)packet, packet->length + 8);
            uint32_t crc_received = data[i + sizeof(gns_raw_data_packet_t) + packet->length - 3] << 16 |
                                    data[i + sizeof(gns_raw_data_packet_t) + packet->length - 2] << 8 |
                                    data[i + sizeof(gns_raw_data_packet_t) + packet->length - 1];
            if (crc_calculated == crc_received)
            {
                // printf("Valid packet found at offset %zu: type=%u length=%u\n", i, packet->type, packet->length);
                // 这里可以根据 packet->type 进一步
                // 解析 packet->payload 数据
                // const uint8_t *payload = (uint8_t *)packet + RAW_PACKET_HEADER_LEN;
                switch (packet->type)
                {
                case RAW_BD3EPHB:
                {
                    BD3EPHB_Decoded_t eph;
                    if (packet->length > 0)
                    {
                        decode_bd3ephb((uint8_t *)packet, packet->length, &eph);
                        print_bd3ephb(&eph, crc_calculated);
                    }
                }
                break;
                case RAW_GLOEPHB:
                {
                    GLOEPHB_Decoded_t eph;
                    if (packet->length > 0)
                    {
                        decode_gloephb((uint8_t *)packet, packet->length, &eph);
                        print_gloephb(&eph, crc_calculated);
                    }
                }
                break;
                case RAW_GALEPHB:
                {
                    GALEPHB_Decoded_t eph;
                    if (packet->length > 0)
                    {
                        decode_galephb((uint8_t *)packet, packet->length, &eph);
                        print_galephb(&eph, crc_calculated);
                    }
                }
                break;
                case RAW_BD3CNAV3EPHB:
                {
                    BD3CNAV3EPHB_Decoded_t eph;
                    if (packet->length > 0)
                    {
                        decode_bd3cnav3ephb((uint8_t *)packet, packet->length, &eph);
                        print_bd3cnav3ephb(&eph, crc_calculated);
                    }
                }
                break;
                case RAW_BD3CNAV1EPHB:
                {
                    BD3CNAV1EPHB_Decoded_t eph;
                    if (packet->length > 0)
                    {
                        decode_bd3cnav1ephb((uint8_t *)packet, packet->length, &eph);
                        print_bd3cnav1ephb(&eph, crc_calculated);
                    }
                }
                break;
                case RAW_BD3CNAV2EPHB:
                {
                    BD3CNAV2EPHB_Decoded_t eph;
                    if (packet->length > 0)
                    {
                        decode_bd3cnav2ephb((uint8_t *)packet, packet->length, &eph);
                        print_bd3cnav2ephb(&eph, crc_calculated);
                    }
                }
                break;
                case RAW_PRANGEB:
                case RAW_PRANGE2B:
                {
                    PRANGEB_Decoded_t p;
                    if (packet->length > 0)
                    {
                        decode_prangeb((uint8_t *)packet, packet->length, &p);                       
                        print_prangeb(&p, crc_calculated);
                    }
                    
                }
                break;                
                case RAW_BD2EPHB:
                {
                    BD2EPHB_Decoded_t eph;
                    if (packet->length >= BD2EPHB_PAYLOAD_LEN)
                    {
                        decode_bd2ephb((uint8_t *)packet, packet->length, &eph);
                        print_bd2ephb(&eph, crc_calculated);
                    }
                }
                break;
                case RAW_GPSEPHB:               
                {
                    GPSEPHB_Decoded_t eph;
                    decode_gpsephb((uint8_t *)packet, packet->length, &eph);
                    // print_gpsephb(&eph, crc_calculated);
                    print_gpsephb_simple(&eph);
                }
                break;
                default:
                    printf("Unknown packet type %u \n", packet->type);
                    break;
                }
                handle_cnt = i + packet->length; // 移动到下一个可能的包位置
                i += packet->length - 1;         // 跳过当前包的 payload 和 checksum
            }
        }
    }
    return handle_cnt;
}
