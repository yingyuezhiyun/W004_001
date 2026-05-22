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

typedef struct
{
    uint8_t head[8];
    uint16_t gps_week_count;
    uint32_t gps_tow_s;
    uint8_t gps_satid;
    uint8_t gps_sv_accuracy;
    uint8_t gps_sv_health;
    uint16_t gps_week;
    uint16_t gps_toe;
    uint16_t gps_toc;
    int32_t gps_af0;
    int32_t gps_af1;
    int32_t gps_af2;
    uint8_t gps_iode;
    uint16_t gps_iodc;
    int32_t gps_idot;
    int32_t gps_crs;
    int32_t gps_crc;
    int32_t gps_cus;
    int32_t gps_cuc;
    int32_t gps_cis;
    int32_t gps_cic;
    int32_t gps_delta_n;
    int32_t gps_m0;
    uint32_t gps_ecc;
    uint32_t gps_a_half;
    int32_t gps_omega0;
    int32_t gps_i0;
    int32_t gps_omega;
    int32_t gps_omegadot;
    int32_t gps_tgd;
    uint8_t gps_code_on_l2;
    uint8_t gps_l2p_data_flag;
    uint8_t gps_fit;
    uint8_t reserved;
    uint32_t crc24;
} GPSEPHB_Decoded_t;

//type类型和length字节长度采用小端在前输出方式，其他参数采用大端在前输出方式 
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
    RAW_BD3EPHB = 02, // BDS BD3 星历
    RAW_GPSEPHB = 11, // GPS 星历
    RAW_GLOEPHB = 21, // GLONASS 星历
    RAW_GALEPHB = 32, // Galileo 星历
    // BD3CNAV1EPHB=23,
    //  BD3CNAV2EPHB =24,
    //   BD3CNAV3EPHB =25,
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

static uint32_t read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t read_u24_be(const uint8_t *p)
{
    return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | (uint32_t)p[2];
}

static uint32_t read_bits_be(const uint8_t *buf, size_t bit_offset, unsigned bit_len)
{
    uint32_t value = 0;

    for (unsigned bit = 0; bit < bit_len; ++bit)
    {
        size_t idx = bit_offset + bit;
        if (buf[idx / 8] & (uint8_t)(1u << (7 - (idx % 8))))
        {
            value |= 1u << (bit_len - 1 - bit);
        }
    }

    return value;
}

static uint32_t read_bits_le(const uint8_t *buf, size_t bit_offset, unsigned bit_len)
{
    uint32_t value = 0;

    for (unsigned bit = 0; bit < bit_len; ++bit)
    {
        size_t idx = bit_offset + bit;
        if (buf[idx / 8] & (uint8_t)(1u << (idx % 8)))
        {
            value |= 1u << bit;
        }
    }

    return value;
}

static uint32_t read_signed_bits_be(const uint8_t *buf, size_t bit_offset, unsigned bit_len)
{
    uint32_t raw = read_bits_be(buf, bit_offset, bit_len);

    if (bit_len == 32)
    {
        return (int32_t)raw;
    }

    uint32_t sign_bit = 1u << (bit_len - 1);
    return (int32_t)((raw ^ sign_bit) - sign_bit);
}

static int32_t read_signed_bits_le(const uint8_t *buf, size_t bit_offset, unsigned bit_len)
{
    uint32_t raw = read_bits_le(buf, bit_offset, bit_len);

    if (bit_len == 32)
    {
        return (int32_t)raw;
    }

    uint32_t sign_bit = 1u << (bit_len - 1);
    return (int32_t)((raw ^ sign_bit) - sign_bit);
}

static double scale_pow2(int32_t raw, int exp)
{
    return ldexp((double)raw, exp);
}

static double scale_pow2_u(uint32_t raw, int exp)
{
    return ldexp((double)raw, exp);
}

static double scale_pi_pow2(int32_t raw, int exp)
{
    return ldexp((double)raw, exp) * M_PI;
}

static double scale_pi_pow2_u(uint32_t raw, int exp)
{
    return ldexp((double)raw, exp) * M_PI;
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

static void decode_gpsephb(const uint8_t *payload, size_t payload_len, GPSEPHB_Decoded_t *out)
{
    size_t bit = 0;

    memset(out, 0, sizeof(*out));
    memcpy(out->head, payload, sizeof(out->head));
    bit += 64;

    out->gps_week_count = (uint16_t)read_bits_be(payload, bit, 12);
    bit += 12;
    out->gps_tow_s = read_bits_be(payload, bit, 20);
    bit += 20;
    out->gps_satid = (uint8_t)read_bits_be(payload, bit, 6);
    bit += 6;
    out->gps_sv_accuracy = (uint8_t)read_bits_be(payload, bit, 4);
    bit += 4;
    out->gps_sv_health = (uint8_t)read_bits_be(payload, bit, 6);
    bit += 6;
    out->gps_week = (uint16_t)read_bits_be(payload, bit, 10);
    bit += 10;
    out->gps_toe = (uint16_t)read_bits_be(payload, bit, 16);
    bit += 16;
    out->gps_toc = (uint16_t)read_bits_be(payload, bit, 16);
    bit += 16;
    out->gps_af0 = read_signed_bits_be(payload, bit, 22);
    bit += 22;
    out->gps_af1 = read_signed_bits_be(payload, bit, 16);
    bit += 16;
    out->gps_af2 = read_signed_bits_be(payload, bit, 8);
    bit += 8;
    out->gps_iode = (uint8_t)read_bits_be(payload, bit, 8);
    bit += 8;
    out->gps_iodc = (uint16_t)read_bits_be(payload, bit, 10);
    bit += 10;
    out->gps_idot = read_signed_bits_be(payload, bit, 14);
    bit += 14;
    out->gps_crs = read_signed_bits_be(payload, bit, 16);
    bit += 16;
    out->gps_crc = read_signed_bits_be(payload, bit, 16);
    bit += 16;
    out->gps_cus = read_signed_bits_be(payload, bit, 16);
    bit += 16;
    out->gps_cuc = read_signed_bits_be(payload, bit, 16);
    bit += 16;
    out->gps_cis = read_signed_bits_be(payload, bit, 16);
    bit += 16;
    out->gps_cic = read_signed_bits_be(payload, bit, 16);
    bit += 16;
    out->gps_delta_n = read_signed_bits_be(payload, bit, 16);
    bit += 16;
    out->gps_m0 = read_signed_bits_be(payload, bit, 32);
    bit += 32;
    out->gps_ecc = read_bits_be(payload, bit, 32);
    bit += 32;
    out->gps_a_half = read_bits_be(payload, bit, 32);
    bit += 32;
    out->gps_omega0 = read_signed_bits_be(payload, bit, 32);
    bit += 32;
    out->gps_i0 = read_signed_bits_be(payload, bit, 32);
    bit += 32;
    out->gps_omega = read_signed_bits_be(payload, bit, 32);
    bit += 32;
    out->gps_omegadot = read_signed_bits_be(payload, bit, 24);
    bit += 24;
    out->gps_tgd = read_signed_bits_be(payload, bit, 8);
    bit += 8;
    out->gps_code_on_l2 = (uint8_t)read_bits_be(payload, bit, 2);
    bit += 2;
    out->gps_l2p_data_flag = (uint8_t)read_bits_be(payload, bit, 1);
    bit += 1;
    out->gps_fit = (uint8_t)read_bits_be(payload, bit, 1);
    bit += 1;
    out->reserved = (uint8_t)read_bits_be(payload, bit, 4);
    bit += 4;
    out->crc24 = read_bits_be(payload, bit, 24);

    (void)payload_len;
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
    printf("  gps_af0          : raw=%d  scaled=% .12e s\n", eph->gps_af0, scale_pow2(eph->gps_af0, -31));
    printf("  gps_af1          : raw=%d  scaled=% .12e s/s\n", eph->gps_af1, scale_pow2(eph->gps_af1, -43));
    printf("  gps_af2          : raw=%d  scaled=% .12e s/s^2\n", eph->gps_af2, scale_pow2(eph->gps_af2, -55));
    printf("  gps_iode         : raw=%u\n", eph->gps_iode);
    printf("  gps_iodc         : raw=%u\n", eph->gps_iodc);
    printf("  gps_idot         : raw=%d  scaled=% .12e rad/s\n", eph->gps_idot, scale_pi_pow2(eph->gps_idot, -43));
    printf("  gps_crs          : raw=%d  scaled=% .12e m\n", eph->gps_crs, scale_pow2(eph->gps_crs, -5));
    printf("  gps_crc          : raw=%d  scaled=% .12e m\n", eph->gps_crc, scale_pow2(eph->gps_crc, -5));
    printf("  gps_cus          : raw=%d  scaled=% .12e rad\n", eph->gps_cus, scale_pi_pow2(eph->gps_cus, -29));
    printf("  gps_cuc          : raw=%d  scaled=% .12e rad\n", eph->gps_cuc, scale_pi_pow2(eph->gps_cuc, -29));
    printf("  gps_cis          : raw=%d  scaled=% .12e rad\n", eph->gps_cis, scale_pi_pow2(eph->gps_cis, -29));
    printf("  gps_cic          : raw=%d  scaled=% .12e rad\n", eph->gps_cic, scale_pi_pow2(eph->gps_cic, -29));
    printf("  gps_delta_n      : raw=%d  scaled=% .12e rad/s\n", eph->gps_delta_n, scale_pi_pow2(eph->gps_delta_n, -43));
    printf("  gps_m0           : raw=%d  scaled=% .12e rad\n", eph->gps_m0, scale_pi_pow2(eph->gps_m0, -31));
    printf("  gps_ecc          : raw=%u  scaled=% .12e\n", eph->gps_ecc, scale_pow2_u(eph->gps_ecc, -33));
    printf("  gps_a_half       : raw=%u  scaled=% .12e m^1/2\n", eph->gps_a_half, scale_pow2_u(eph->gps_a_half, -19));
    printf("  gps_omega0       : raw=%d  scaled=% .12e rad\n", eph->gps_omega0, scale_pi_pow2(eph->gps_omega0, -31));
    printf("  gps_i0           : raw=%d  scaled=% .12e rad\n", eph->gps_i0, scale_pi_pow2(eph->gps_i0, -31));
    printf("  gps_omega        : raw=%d  scaled=% .12e rad\n", eph->gps_omega, scale_pi_pow2(eph->gps_omega, -31));
    printf("  gps_omegadot     : raw=%d  scaled=% .12e rad/s\n", eph->gps_omegadot, scale_pi_pow2(eph->gps_omegadot, -43));
    printf("  gps_tgd          : raw=%d  scaled=% .12e s\n", eph->gps_tgd, scale_pow2(eph->gps_tgd, -31));
    printf("  gps_code_on_l2   : raw=%u (%s)\n", eph->gps_code_on_l2, gps_l2_code_desc(eph->gps_code_on_l2));
    printf("  gps_l2p_data_flag: raw=%u\n", eph->gps_l2p_data_flag);
    printf("  gps_fit          : raw=%u (%s)\n", eph->gps_fit, gps_fit_desc(eph->gps_fit));
    printf("  reserved         : raw=%u\n", eph->reserved);
    printf("  payload_crc24    : raw=%06X calc=%06X [%s]\n",
           eph->crc24,
           payload_crc_calc,
           (eph->crc24 == payload_crc_calc) ? "OK" : "BAD");
}

int handle_gnss_raw(const uint8_t *data, size_t len)
{
    int handle_cnt = 0;
    for (size_t i = 0; i < len - 3; i++)
    {
        gns_raw_data_packet_t *packet = (gns_raw_data_packet_t *)(data + i);
        if (packet->header == Header)
        {
            printf("Packet header found at offset %zu\n", i);
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
                printf("Valid packet found at offset %zu: type=%u length=%u\n", i, packet->type, packet->length);
                // 这里可以根据 packet->type 进一步
                // 解析 packet->payload 数据
                // const uint8_t *payload = (uint8_t *)packet + RAW_PACKET_HEADER_LEN;
                switch (packet->type)
                {
                case RAW_GPSEPHB:
                {
                    GPSEPHB_Decoded_t eph;
                    decode_gpsephb((uint8_t *)packet, packet->length, &eph);
                    print_gpsephb(&eph, crc_calculated);
                }
                break;
                default:
                    break;
                }
                handle_cnt = i + packet->length; // 移动到下一个可能的包位置
                i += packet->length - 1;         // 跳过当前包的 payload 和 checksum
            }
        }
    }
    return handle_cnt;
}
