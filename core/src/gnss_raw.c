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

EPHB_File_sw_t ephb_file_sw = {
    .gpsephb = 0,
    .bd2ephb = 0,
    .bd3ephb = 0,
    .gloephb = 0,
    .galephb = 0,
    .bdxwephb = 0,
    .bd3cnav2ephb = 0,
    .bd3cnav3ephb = 0};



#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TEST_GPSEPHB_FILE_PATH "/root/gps_ephe.csv"
#define TEST_BD2EPHB_FILE_PATH "/root/bd2_ephe.csv"
#define TEST_BD3EPHB_FILE_PATH "/root/bd3_ephe.csv"
#define TEST_BDXWEPHB_FILE_PATH "/root/bdxw_ephe.csv"
#define TEST_BD3CNAV2EPHB_FILE_PATH "/root/bd3cnav2_ephe.csv"
#define TEST_BD3CNAV3EPHB_FILE_PATH "/root/bd3cnav3_ephe.csv"


// #define Header1 (0x43534847)
#define Header1 (0x47485343) //CSHG

// #define Header2 (0x2448455844415441)
#define Header2 (0x4154414458454824)//$HEXDATA

#define GPSEPHB_PAYLOAD_LEN (64)
#define BD2EPHB_PAYLOAD_LEN (67)
#define BD3CNAV3EPHB_PAYLOAD_LEN (76)
#define POSDATAB_PAYLOAD_LEN (70)
#define BD3EPHB_PAYLOAD_LEN (76)
#define GALEPHB_PAYLOAD_LEN (66)

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
    RAW_BD2EPHB = 01,      // BDS BD2 星历
    RAW_BD2IonoB = 205,    // BDS BD2 电离层参数
    RAW_BD3EPHB = 02,      // BDS BD3 星历
    RAW_BD3CNAV1EPHB = 02, // BDS BD3 CNAV1 星历
    RAW_BD3CNAV2EPHB = 03, // BDS BD3 CNAV2 星历
    RAW_BD3CNAV3EPHB = 04, // BDS BD3 CNAV3 星历
    RAW_BDXWEPHB = 41,     // BDXW/XWS 星历（扩展）
    RAW_BD3IonoB = 206,    // BDS BD3 电离层参数
    RAW_GPSEPHB = 11,      // GPS 星历 / GPS LNAV 星历
    RAW_GPSCNAV = 12,      // GPS CNAV 星历
    RAW_GPSCNAV2 = 13,     // GPS CNAV2 星历
    RAW_GPSIonoB = 214,    // GPS 电离层参数
    RAW_GLOEPHB = 21,      // GLONASS 星历
    RAW_GALEPHB = 31,      // Galileo 星历 / GAL FNAV星历
    RAW_GALINAV = 32,      // GAL INAV星历
    RAW_QZSS = 51,         // QZSS 星历
    RAW_PRANGEB = 61,      // 主天线观测量
    RAW_PRANGE2B = 62,     // 副天线观测量
    RAW_POSDATAB = 0xF090, // 最优定位信息
};
#pragma pack(push, 1)
typedef struct
{
    uint32_t header;
    uint16_t type;
    uint16_t length;
    // uint8_t payload[0];
    uint8_t crc[3];
} gns_raw_data1_packet_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    uint64_t header;
    uint16_t type;
    uint16_t length;
    // uint8_t payload[0];
    uint8_t crc[3];
} gns_raw_data2_packet_t;
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

int handle_gnss_raw(const uint8_t *data, size_t len)
{
    int handle_cnt = 0;
    if (len < sizeof(gns_raw_data1_packet_t))
    {
        return handle_cnt;
    }    
    for (size_t i = 0; i < len - 3; i++)
    {
        gns_raw_data1_packet_t *packet = (gns_raw_data1_packet_t *)(data + i);
        gns_raw_data2_packet_t *packet2 = (gns_raw_data2_packet_t *)(data + i);
        if (len - i > sizeof(gns_raw_data1_packet_t) && packet->header == Header1)
        {
            // printf("Packet header found at offset %zu\n", i);
            if (packet->length > len - i - sizeof(gns_raw_data1_packet_t))
            {
                continue;
            }
            uint32_t crc_calculated = rtk_crc24q((const uint8_t *)packet, packet->length + 8);
            uint32_t crc_received = data[i + sizeof(gns_raw_data1_packet_t) + packet->length - 3] << 16 |
                                    data[i + sizeof(gns_raw_data1_packet_t) + packet->length - 2] << 8 |
                                    data[i + sizeof(gns_raw_data1_packet_t) + packet->length - 1];
            if (crc_calculated == crc_received)
            {
                // printf("Valid packet found at offset %zu: type=%u length=%u\n", i, packet->type, packet->length);
                switch (packet->type)
                {
                case RAW_BD2EPHB:
                {
                    BD2EPHB_Decoded_t eph;
                    if (packet->length >= BD2EPHB_PAYLOAD_LEN) // 67
                    {
                        decode_bd2ephb((uint8_t *)packet, packet->length, &eph);
                        printf("date:%s BDS-2 EPHB: satid=%u \n", gps_week_sec_to_utc(eph.gps_week_count, eph.gps_tow_s), eph.bd2_satid);
                        // print_bd2ephb(&eph, crc_calculated);
                        if (ephb_file_sw.bd2ephb)
                        {
                            bd2ephb_file_append(&eph);  
                        }
                    }
                }
                break;
                case RAW_BD3EPHB:
                {
                    BD3EPHB_Decoded_t eph;
                    if (packet->length >= BD3EPHB_PAYLOAD_LEN) // 76
                    {
                        decode_bd3ephb((uint8_t *)packet, packet->length, &eph);
                        printf("date:%s BDS-3 EPHB: satid=%u \n", gps_week_sec_to_utc(eph.gps_week_count, eph.gps_tow_s), eph.bd3_satid);
                        // print_bd3ephb(&eph, crc_calculated);
                        if (ephb_file_sw.bd3ephb)
                        {
                            bd3ephb_file_append(&eph);
                        }
                    }
                }
                break;
                case RAW_BD3CNAV2EPHB:
                {
                    BD3CNAV2EPHB_Decoded_t eph;
                    if (packet->length > 0)
                    {
                        decode_bd3cnav2ephb((uint8_t *)packet, packet->length, &eph);
                        printf("date:%s BDS-3 CNAV2 Ephemeris: satid=%u \n", gps_week_sec_to_utc(eph.gps_week_count, eph.gps_tow_s), eph.bds_satid);
                        // print_bd3cnav2ephb(&eph, crc_calculated);
                        if (ephb_file_sw.bd3cnav2ephb)
                        {
                            bd3cnav2ephb_file_append(&eph);
                        }
                    }
                }
                break;
                case RAW_BD3CNAV3EPHB:
                {
                    BD3CNAV3EPHB_Decoded_t eph;
                    if (packet->length > 0)
                    {
                        decode_bd3cnav3ephb((uint8_t *)packet, packet->length, &eph);
                        printf("date:%s BDS-3 CNAV3 Ephemeris: satid=%u \n", gps_week_sec_to_utc(eph.gps_week_count, eph.gps_tow_s), eph.bds_satid);
                        // print_bd3cnav3ephb(&eph, crc_calculated);
                        if (ephb_file_sw.bd3cnav3ephb)
                        {
                            bd3cnav3ephb_file_append(&eph);
                        }
                        
                    }
                }
                break;
                case RAW_BDXWEPHB:
                {
                    BDXWEPHB_Decoded_t eph;
                    if (packet->length > 0)
                    {
                        decode_bdxwephb((uint8_t *)packet, packet->length, &eph);
                        print_bdxwephb(&eph, crc_calculated);
                        if (ephb_file_sw.bdxwephb)
                        {
                            bdxwephb_file_append(&eph);
                        }
                        
                    }
                }
                break;
                case RAW_GPSEPHB:
                {
                    GPSEPHB_Decoded_t eph;
                    if (packet->length >= GPSEPHB_PAYLOAD_LEN) // 64
                    {
                        decode_gpsephb((uint8_t *)packet, packet->length, &eph); // 64
                        // print_gpsephb(&eph, crc_calculated);
                        // print_gpsephb_simple(&eph);                    
                        printf("date:%s GPS Ephemeris: satid=%u \n", gps_week_sec_to_utc(eph.gps_week_count, eph.gps_tow_s), eph.gps_satid);
                        if (ephb_file_sw.gpsephb)
                        {
                            gpsephb_file_append(&eph);
                        }
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
                case RAW_GALINAV:
                {
                    GALEPHB_Decoded_t eph;
                    if (packet->length >= GALEPHB_PAYLOAD_LEN) // 66
                    {
                        decode_galephb((uint8_t *)packet, packet->length, &eph);
                        print_galephb(&eph, crc_calculated);
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
                case RAW_BD2IonoB:
                case RAW_BD3IonoB:
                case RAW_GPSIonoB:
                    break;
                default:
                    printf("Unknown packet type %u \n", packet->type);
                    break;
                }
                handle_cnt = i + packet->length; // 移动到下一个可能的包位置
                i += packet->length - 1;         // 跳过当前包的 payload 和 checksum
            }
        }
        else if (len - i > sizeof(gns_raw_data2_packet_t) && packet2->header == Header2)
        {
            // printf("Packet2 header found at offset %zu\n", i);
            if (packet2->length > len - i - sizeof(gns_raw_data2_packet_t))
            {
                continue;
            }
            uint32_t crc_calculated = rtk_crc24q((const uint8_t *)packet2, packet2->length + 12);
            uint32_t crc_received = data[i + sizeof(gns_raw_data2_packet_t) + packet2->length - 3] << 16 |
                                    data[i + sizeof(gns_raw_data2_packet_t) + packet2->length - 2] << 8 |
                                    data[i + sizeof(gns_raw_data2_packet_t) + packet2->length - 1];
            if (crc_calculated == crc_received)
            {
                // printf("Valid packet2 found at offset %zu: type=%u length=%u\n", i, packet2->type, packet2->length);
                switch (packet2->type)
                {
                case RAW_POSDATAB:
                {
                    POSDATAB_Decoded_t pos;
                    if (packet2->length >= POSDATAB_PAYLOAD_LEN)
                    {
                        decode_posdatab((uint8_t *)packet2, packet2->length, &pos);
                        print_posdatab(&pos, crc_calculated);
                    }
                }
                break;
                default:
                    printf("Unknown packet2 type %u \n", packet2->type);
                    break;
                }
            }
        }
    }
    return handle_cnt;
}
