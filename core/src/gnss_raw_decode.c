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

static uint64_t read_bits_be64(const uint8_t *buf, size_t bit_offset, unsigned bit_len)
{
    uint64_t value = 0;

    for (unsigned bit = 0; bit < bit_len; ++bit)
    {
        size_t idx = bit_offset + bit;
        if (buf[idx / 8] & (uint8_t)(1u << (7 - (idx % 8))))
        {
            value |= (uint64_t)1u << (bit_len - 1 - bit);
        }
    }

    return value;
}

static int64_t read_signed_bits_be64(const uint8_t *buf, size_t bit_offset, unsigned bit_len)
{
    uint64_t raw = read_bits_be64(buf, bit_offset, bit_len);
    if (bit_len == 64)
    {
        return (int64_t)raw;
    }
    uint64_t sign_bit = (uint64_t)1u << (bit_len - 1);
    return (int64_t)((raw ^ sign_bit) - sign_bit);
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

static double scale_tenth_ns(int32_t raw)
{
    return (double)raw * 1.0e-10;
}

static double scale_pow2_64(int64_t raw, int exp)
{
    return ldexp((double)raw, exp);
}

static double scale_pow2_u64(uint64_t raw, int exp)
{
    return ldexp((double)raw, exp);
}

static double scale_pi_pow2_64(int64_t raw, int exp)
{
    return ldexp((double)raw, exp) * M_PI;
}

void decode_bd2ephb(const uint8_t *payload, size_t payload_len, BD2EPHB_Decoded_t *out)
{
    size_t bit = 0;

    memset(out, 0, sizeof(*out));
    memcpy(out->head, payload, sizeof(out->head));
    bit += 64;

    out->gps_week_count = (uint16_t)read_bits_be(payload, bit, 12);
    bit += 12;
    out->gps_tow_s = read_bits_be(payload, bit, 20);
    bit += 20;
    out->bd2_satid = (uint8_t)read_bits_be(payload, bit, 6);
    bit += 6;
    out->bd2_sv_urai = (uint8_t)read_bits_be(payload, bit, 4);
    bit += 4;
    out->bd2_sv_health = (uint8_t)read_bits_be(payload, bit, 1);
    bit += 1;
    out->bd2_week = (uint16_t)read_bits_be(payload, bit, 13);
    bit += 13;
    out->bd2_toe = read_bits_be(payload, bit, 17);
    bit += 17;
    out->bd2_toc = read_bits_be(payload, bit, 17);
    bit += 17;
    out->bd2_af0 = scale_pow2(read_signed_bits_be(payload, bit, 24), -33);
    bit += 24;
    out->bd2_af1 = scale_pow2(read_signed_bits_be(payload, bit, 22), -50);
    bit += 22;
    out->bd2_af2 = scale_pow2(read_signed_bits_be(payload, bit, 11), -66);
    bit += 11;
    out->bd2_aode = (uint8_t)read_bits_be(payload, bit, 5);
    bit += 5;
    out->bd2_aodc = (uint8_t)read_bits_be(payload, bit, 5);
    bit += 5;
    out->bd2_idot = scale_pi_pow2(read_signed_bits_be(payload, bit, 14), -43);
    bit += 14;
    out->bd2_crs = scale_pow2(read_signed_bits_be(payload, bit, 18), -6);
    bit += 18;
    out->bd2_crc = scale_pow2(read_signed_bits_be(payload, bit, 18), -6);
    bit += 18;
    out->bd2_cus = scale_pow2(read_signed_bits_be(payload, bit, 18), -31);
    bit += 18;
    out->bd2_cuc = scale_pow2(read_signed_bits_be(payload, bit, 18), -31);
    bit += 18;
    out->bd2_cis = scale_pow2(read_signed_bits_be(payload, bit, 18), -31);
    bit += 18;
    out->bd2_cic = scale_pow2(read_signed_bits_be(payload, bit, 18), -31);
    bit += 18;
    out->bd2_delta_n = scale_pi_pow2(read_signed_bits_be(payload, bit, 16), -43);
    bit += 16;
    out->bd2_m0 = scale_pi_pow2(read_signed_bits_be(payload, bit, 32), -31);
    bit += 32;
    out->bd2_ecc = scale_pow2_u(read_bits_be(payload, bit, 32), -33);
    bit += 32;
    out->bd2_a_half = scale_pow2_u(read_bits_be(payload, bit, 32), -19);
    bit += 32;
    out->bd2_omega0 = scale_pi_pow2(read_signed_bits_be(payload, bit, 32), -31);
    bit += 32;
    out->bd2_i0 = scale_pi_pow2(read_signed_bits_be(payload, bit, 32), -31);
    bit += 32;
    out->bd2_omega = scale_pi_pow2(read_signed_bits_be(payload, bit, 32), -31);
    bit += 32;
    out->bd2_omegadot = scale_pi_pow2(read_signed_bits_be(payload, bit, 24), -43);
    bit += 24;
    out->bd2_tgd1 = scale_tenth_ns(read_signed_bits_be(payload, bit, 10));
    bit += 10;
    out->bd2_tgd2 = scale_tenth_ns(read_signed_bits_be(payload, bit, 10));
    bit += 10;
    out->bd2_reserved = (uint8_t)read_bits_be(payload, bit, 5);
    bit += 5;
    out->crc24 = read_bits_be(payload, bit, 24);

    (void)payload_len;
}

void decode_bd3ephb(const uint8_t *payload, size_t payload_len, BD3EPHB_Decoded_t *out)
{
    size_t bit = 0;

    memset(out, 0, sizeof(*out));
    memcpy(out->head, payload, sizeof(out->head));
    bit += 64;

    out->gps_week_count = (uint16_t)read_bits_be(payload, bit, 12);
    bit += 12;
    out->gps_tow_s = read_bits_be(payload, bit, 20);
    bit += 20;
    out->bd3_satid = (uint8_t)read_bits_be(payload, bit, 6);
    bit += 6;
    out->bd3_sattype = (uint8_t)read_bits_be(payload, bit, 3);
    bit += 3;
    out->bd3_week = (uint16_t)read_bits_be(payload, bit, 13);
    bit += 13;
    out->bd3_toe = read_bits_be(payload, bit, 11) * 300u;
    bit += 11;
    out->bd3_toc = read_bits_be(payload, bit, 11) * 300u;
    bit += 11;
    out->bd3_af0 = scale_pow2_64(read_signed_bits_be64(payload, bit, 25), -34);
    bit += 25;
    out->bd3_af1 = scale_pow2_64(read_signed_bits_be64(payload, bit, 22), -50);
    bit += 22;
    out->bd3_af2 = scale_pow2_64(read_signed_bits_be64(payload, bit, 11), -66);
    bit += 11;
    out->bd3_iode = (uint8_t)read_bits_be(payload, bit, 8);
    bit += 8;
    out->bd3_iodc = (uint16_t)read_bits_be(payload, bit, 16);
    bit += 16;
    out->bd3_idot = scale_pi_pow2_64(read_signed_bits_be64(payload, bit, 15), -44);
    bit += 15;
    out->bd3_crs = scale_pow2_64(read_signed_bits_be64(payload, bit, 24), -8);
    bit += 24;
    out->bd3_crc = scale_pow2_64(read_signed_bits_be64(payload, bit, 24), -8);
    bit += 24;
    out->bd3_cus = scale_pow2_64(read_signed_bits_be64(payload, bit, 21), -30);
    bit += 21;
    out->bd3_cuc = scale_pow2_64(read_signed_bits_be64(payload, bit, 21), -30);
    bit += 21;
    out->bd3_cis = scale_pow2_64(read_signed_bits_be64(payload, bit, 16), -30);
    bit += 16;
    out->bd3_cic = scale_pow2_64(read_signed_bits_be64(payload, bit, 16), -30);
    bit += 16;
    out->bd3_delta_n0 = scale_pi_pow2_64(read_signed_bits_be64(payload, bit, 17), -44);
    bit += 17;
    out->bd3_delta_n0_dot = scale_pi_pow2_64(read_signed_bits_be64(payload, bit, 23), -57);
    bit += 23;
    out->bd3_m0 = scale_pi_pow2_64((int64_t)read_bits_be64(payload, bit, 33), -32);
    bit += 33;
    out->bd3_ecc = scale_pow2_u64(read_bits_be64(payload, bit, 33), -34);
    bit += 33;
    out->bd3_deltaA = scale_pow2_64((int64_t)read_bits_be64(payload, bit, 26), -9);
    bit += 26;
    out->bd3_adot = scale_pow2_64(read_signed_bits_be64(payload, bit, 25), -21);
    bit += 25;
    out->bd3_omega0 = scale_pi_pow2_64((int64_t)read_bits_be64(payload, bit, 33), -32);
    bit += 33;
    out->bd3_i0 = scale_pi_pow2_64((int64_t)read_bits_be64(payload, bit, 33), -32);
    bit += 33;
    out->bd3_omega = scale_pi_pow2_64((int64_t)read_bits_be64(payload, bit, 33), -32);
    bit += 33;
    out->bd3_omegadot = scale_pi_pow2_64(read_signed_bits_be64(payload, bit, 19), -44);
    bit += 19;
    out->bd3_tgdb1cp = scale_pow2_64(read_signed_bits_be64(payload, bit, 12), -34);
    bit += 12;
    out->bd3_tgdb2ap = scale_pow2_64(read_signed_bits_be64(payload, bit, 12), -34);
    bit += 12;
    out->bd3_iscb1cd = scale_pow2_64(read_signed_bits_be64(payload, bit, 12), -34);
    bit += 12;
    out->bd3_reserved = (uint8_t)read_bits_be(payload, bit, 2);
    bit += 2;
    out->crc24 = read_bits_be(payload, bit, 24);

    (void)payload_len;
}

void decode_gloephb(const uint8_t *payload, size_t payload_len, GLOEPHB_Decoded_t *out)
{
    size_t bit = 0;

    memset(out, 0, sizeof(*out));
    memcpy(out->head, payload, sizeof(out->head));
    bit += 64;

    out->gps_week_count = (uint16_t)read_bits_be(payload, bit, 12);
    bit += 12;
    out->gps_tow_s = read_bits_be(payload, bit, 20);
    bit += 20;
    out->glo_satid = (uint8_t)read_bits_be(payload, bit, 6);
    bit += 6;
    out->glo_freq = (uint8_t)read_bits_be(payload, bit, 5);
    bit += 5;
    out->glo_bn_msb = (uint8_t)read_bits_be(payload, bit, 1);
    bit += 1;
    out->glo_n4 = (uint8_t)read_bits_be(payload, bit, 5);
    bit += 5;
    out->glo_nt = (uint16_t)read_bits_be(payload, bit, 11);
    bit += 11;
    out->glo_tk = (uint16_t)read_bits_be(payload, bit, 12);
    bit += 12;
    out->glo_tb = (uint8_t)read_bits_be(payload, bit, 7);
    bit += 7;
    out->glo_gamma = scale_pow2(read_signed_bits_be(payload, bit, 11), -40);
    bit += 11;
    out->glo_tau = scale_pow2(read_signed_bits_be(payload, bit, 22), -30);
    bit += 22;
    out->glo_x = scale_pow2(read_signed_bits_be(payload, bit, 27), -11);
    bit += 27;
    out->glo_x_dot = scale_pow2(read_signed_bits_be(payload, bit, 24), -20);
    bit += 24;
    out->glo_x_ddot = scale_pow2(read_signed_bits_be(payload, bit, 5), -30);
    bit += 5;
    out->glo_y = scale_pow2(read_signed_bits_be(payload, bit, 27), -11);
    bit += 27;
    out->glo_y_dot = scale_pow2(read_signed_bits_be(payload, bit, 24), -20);
    bit += 24;
    out->glo_y_ddot = scale_pow2(read_signed_bits_be(payload, bit, 5), -30);
    bit += 5;
    out->glo_z = scale_pow2(read_signed_bits_be(payload, bit, 27), -11);
    bit += 27;
    out->glo_z_dot = scale_pow2(read_signed_bits_be(payload, bit, 24), -20);
    bit += 24;
    out->glo_z_ddot = scale_pow2(read_signed_bits_be(payload, bit, 5), -30);
    bit += 5;
    /* ID21 Reserved: 0 bits according to table */
    out->crc24 = read_bits_be(payload, bit, 24);

    (void)payload_len;
}

void decode_galephb(const uint8_t *payload, size_t payload_len, GALEPHB_Decoded_t *out)
{
    size_t bit = 0;

    memset(out, 0, sizeof(*out));
    memcpy(out->head, payload, sizeof(out->head));
    bit += 64;

    out->gps_week_count = (uint16_t)read_bits_be(payload, bit, 12);
    bit += 12;
    out->gps_tow_s = read_bits_be(payload, bit, 20);
    bit += 20;
    out->gal_satid = (uint8_t)read_bits_be(payload, bit, 6);
    bit += 6;
    out->gal_sisa = (uint8_t)read_bits_be(payload, bit, 8);
    bit += 8;
    out->gal_e5b_sv_health = (uint8_t)read_bits_be(payload, bit, 2);
    bit += 2;
    out->gal_e5b_valid = (uint8_t)read_bits_be(payload, bit, 1);
    bit += 1;
    out->gal_e1b_health = (uint8_t)read_bits_be(payload, bit, 2);
    bit += 2;
    out->gal_e1b_valid = (uint8_t)read_bits_be(payload, bit, 1);
    bit += 1;
    out->gal_week = (uint16_t)read_bits_be(payload, bit, 12);
    bit += 12;
    out->gal_toe = read_bits_be(payload, bit, 14) * 60u;
    bit += 14;
    out->gal_toc = read_bits_be(payload, bit, 14) * 60u;
    bit += 14;
    out->gal_af0 = scale_pow2(read_signed_bits_be(payload, bit, 31), -34);
    bit += 31;
    out->gal_af1 = scale_pow2(read_signed_bits_be(payload, bit, 21), -46);
    bit += 21;
    out->gal_af2 = scale_pow2(read_signed_bits_be(payload, bit, 6), -59);
    bit += 6;
    out->gal_iodnav = (uint16_t)read_bits_be(payload, bit, 10);
    bit += 10;
    out->gal_idot = scale_pi_pow2(read_signed_bits_be(payload, bit, 14), -43);
    bit += 14;
    out->gal_crs = scale_pow2(read_signed_bits_be(payload, bit, 16), -5);
    bit += 16;
    out->gal_crc = scale_pow2(read_signed_bits_be(payload, bit, 16), -5);
    bit += 16;
    out->gal_cus = scale_pow2(read_signed_bits_be(payload, bit, 16), -29);
    bit += 16;
    out->gal_cuc = scale_pow2(read_signed_bits_be(payload, bit, 16), -29);
    bit += 16;
    out->gal_cis = scale_pow2(read_signed_bits_be(payload, bit, 16), -29);
    bit += 16;
    out->gal_cic = scale_pow2(read_signed_bits_be(payload, bit, 16), -29);
    bit += 16;
    out->gal_delta_n = scale_pi_pow2(read_signed_bits_be(payload, bit, 16), -43);
    bit += 16;
    out->gal_m0 = scale_pi_pow2(read_signed_bits_be(payload, bit, 32), -31);
    bit += 32;
    out->gal_ecc = scale_pow2_u(read_bits_be(payload, bit, 32), -33);
    bit += 32;
    out->gal_a_half = scale_pow2_u(read_bits_be(payload, bit, 32), -19);
    bit += 32;
    out->gal_omega0 = scale_pi_pow2(read_signed_bits_be(payload, bit, 32), -31);
    bit += 32;
    out->gal_i0 = scale_pi_pow2(read_signed_bits_be(payload, bit, 32), -31);
    bit += 32;
    out->gal_omega = scale_pi_pow2(read_signed_bits_be(payload, bit, 32), -31);
    bit += 32;
    out->gal_omegadot = scale_pi_pow2(read_signed_bits_be(payload, bit, 24), -43);
    bit += 24;
    out->gal_bgd_e5a_e1 = scale_pow2(read_signed_bits_be(payload, bit, 10), -32);
    bit += 10;
    out->gal_bgd_e5b_e1 = scale_pow2(read_signed_bits_be(payload, bit, 10), -32);
    bit += 10;
    out->gal_navtype = (uint8_t)read_bits_be(payload, bit, 2);
    bit += 2;
    out->gal_reserved = (uint8_t)read_bits_be(payload, bit, 4);
    bit += 4;
    out->crc24 = read_bits_be(payload, bit, 24);

    (void)payload_len;
}

void decode_bd3cnav3ephb(const uint8_t *payload, size_t payload_len, BD3CNAV3EPHB_Decoded_t *out)
{
    size_t bit = 0;
    memset(out, 0, sizeof(*out));
    memcpy(out->head, payload, sizeof(out->head));
    bit += 64;
    out->gps_week_count = (uint16_t)read_bits_be(payload, bit, 12);
    bit += 12;
    out->gps_tow_s = read_bits_be(payload, bit, 20);
    bit += 20;
    out->bds_satid = (uint8_t)read_bits_be(payload, bit, 6);
    bit += 6;
    out->bds_sattype = (uint8_t)read_bits_be(payload, bit, 3);
    bit += 3;
    out->bds_week = (uint16_t)read_bits_be(payload, bit, 13);
    bit += 13;
    out->bds_toe = read_bits_be(payload, bit, 11) * 300u;
    bit += 11;
    out->bds_toc = read_bits_be(payload, bit, 11) * 300u;
    bit += 11;
    out->bds_af0 = scale_pow2_64(read_signed_bits_be64(payload, bit, 25), -34);
    bit += 25;
    out->bds_af1 = scale_pow2_64(read_signed_bits_be64(payload, bit, 22), -50);
    bit += 22;
    out->bds_af2 = scale_pow2_64(read_signed_bits_be64(payload, bit, 11), -66);
    bit += 11;
    out->bds_iode = (uint8_t)read_bits_be(payload, bit, 8);
    bit += 8;
    out->bds_iodc = (uint16_t)read_bits_be(payload, bit, 16);
    bit += 16;
    out->bds_idot = scale_pi_pow2_64(read_signed_bits_be64(payload, bit, 15), -44);
    bit += 15;
    out->bds_crs = scale_pow2_64(read_signed_bits_be64(payload, bit, 24), -8);
    bit += 24;
    out->bds_crc = scale_pow2_64(read_signed_bits_be64(payload, bit, 24), -8);
    bit += 24;
    out->bds_cus = scale_pow2_64(read_signed_bits_be64(payload, bit, 21), -30);
    bit += 21;
    out->bds_cuc = scale_pow2_64(read_signed_bits_be64(payload, bit, 21), -30);
    bit += 21;
    out->bds_cis = scale_pow2_64(read_signed_bits_be64(payload, bit, 16), -30);
    bit += 16;
    out->bds_cic = scale_pow2_64(read_signed_bits_be64(payload, bit, 16), -30);
    bit += 16;
    out->bds_delta_n0 = scale_pi_pow2_64(read_signed_bits_be64(payload, bit, 17), -44);
    bit += 17;
    out->bds_delta_n0_dot = scale_pow2_64(read_signed_bits_be64(payload, bit, 23), -57);
    bit += 23;
    out->bds_m0 = scale_pi_pow2_64((int64_t)read_bits_be64(payload, bit, 33), -32);
    bit += 33;
    out->bds_ecc = scale_pow2_u64(read_bits_be64(payload, bit, 33), -34);
    bit += 33;
    out->bds_deltaA = scale_pow2_64((int64_t)read_bits_be64(payload, bit, 26), -9);
    bit += 26;
    out->bds_adot = scale_pow2_64(read_signed_bits_be64(payload, bit, 25), -21);
    bit += 25;
    out->bds_omega0 = scale_pi_pow2_64((int64_t)read_bits_be64(payload, bit, 33), -32);
    bit += 33;
    out->bds_i0 = scale_pi_pow2_64((int64_t)read_bits_be64(payload, bit, 33), -32);
    bit += 33;
    out->bds_omega = scale_pi_pow2_64((int64_t)read_bits_be64(payload, bit, 33), -32);
    bit += 33;
    out->bds_omegadot = scale_pi_pow2_64(read_signed_bits_be64(payload, bit, 19), -44);
    bit += 19;
    out->bds_tgdb2bI = scale_pow2_64(read_signed_bits_be64(payload, bit, 12), -34);
    bit += 12;
    out->bds_reserved = (uint32_t)read_bits_be(payload, bit, 26);
    bit += 26;
    out->crc24 = read_bits_be(payload, bit, 24);
}

void decode_bd3cnav2ephb(const uint8_t *payload, size_t payload_len, BD3CNAV2EPHB_Decoded_t *out)
{
    size_t bit = 0;

    memset(out, 0, sizeof(*out));
    memcpy(out->head, payload, sizeof(out->head));
    bit += 64;

    out->gps_week_count = (uint16_t)read_bits_be(payload, bit, 12);
    bit += 12;
    out->gps_tow_s = read_bits_be(payload, bit, 20);
    bit += 20;
    out->bds_satid = (uint8_t)read_bits_be(payload, bit, 6);
    bit += 6;
    out->bds_sattype = (uint8_t)read_bits_be(payload, bit, 3);
    bit += 3;
    out->bds_week = (uint16_t)read_bits_be(payload, bit, 13);
    bit += 13;
    out->bds_toe = read_bits_be(payload, bit, 11) * 300u;
    bit += 11;
    out->bds_toc = read_bits_be(payload, bit, 11) * 300u;
    bit += 11;
    out->bds_af0 = scale_pow2_64(read_signed_bits_be64(payload, bit, 25), -34);
    bit += 25;
    out->bds_af1 = scale_pow2_64(read_signed_bits_be64(payload, bit, 22), -50);
    bit += 22;
    out->bds_af2 = scale_pow2_64(read_signed_bits_be64(payload, bit, 11), -66);
    bit += 11;
    out->bds_iode = (uint8_t)read_bits_be(payload, bit, 8);
    bit += 8;
    out->bds_iodc = (uint16_t)read_bits_be(payload, bit, 16);
    bit += 16;
    out->bds_idot = scale_pi_pow2_64(read_signed_bits_be64(payload, bit, 15), -44);
    bit += 15;
    out->bds_crs = scale_pow2_64(read_signed_bits_be64(payload, bit, 24), -8);
    bit += 24;
    out->bds_crc = scale_pow2_64(read_signed_bits_be64(payload, bit, 24), -8);
    bit += 24;
    out->bds_cus = scale_pow2_64(read_signed_bits_be64(payload, bit, 21), -30);
    bit += 21;
    out->bds_cuc = scale_pow2_64(read_signed_bits_be64(payload, bit, 21), -30);
    bit += 21;
    out->bds_cis = scale_pow2_64(read_signed_bits_be64(payload, bit, 16), -30);
    bit += 16;
    out->bds_cic = scale_pow2_64(read_signed_bits_be64(payload, bit, 16), -30);
    bit += 16;
    out->bds_delta_n0 = scale_pi_pow2_64(read_signed_bits_be64(payload, bit, 17), -44);
    bit += 17;
    out->bds_delta_n0_dot = scale_pow2_64(read_signed_bits_be64(payload, bit, 23), -57);
    bit += 23;
    out->bds_m0 = scale_pi_pow2_64((int64_t)read_bits_be64(payload, bit, 33), -32);
    bit += 33;
    out->bds_ecc = scale_pow2_u64(read_bits_be64(payload, bit, 33), -34);
    bit += 33;
    out->bds_AA = scale_pow2_64((int64_t)read_bits_be64(payload, bit, 26), -9);
    bit += 26;
    out->bds_adot = scale_pow2_64(read_signed_bits_be64(payload, bit, 25), -21);
    bit += 25;
    out->bds_omega0 = scale_pi_pow2_64((int64_t)read_bits_be64(payload, bit, 33), -32);
    bit += 33;
    out->bds_i0 = scale_pi_pow2_64((int64_t)read_bits_be64(payload, bit, 33), -32);
    bit += 33;
    out->bds_omega = scale_pi_pow2_64((int64_t)read_bits_be64(payload, bit, 33), -32);
    bit += 33;
    out->bds_omegadot = scale_pi_pow2_64(read_signed_bits_be64(payload, bit, 19), -44);
    bit += 19;
    out->bds_tgdb1cp = scale_pow2_64(read_signed_bits_be64(payload, bit, 12), -34);
    bit += 12;
    out->bds_tgdb2ap = scale_pow2_64(read_signed_bits_be64(payload, bit, 12), -34);
    bit += 12;
    out->bds_iscb2ad = scale_pow2_64(read_signed_bits_be64(payload, bit, 12), -34);
    bit += 12;
    out->bds_reserved = (uint8_t)read_bits_be(payload, bit, 2);
    bit += 2;
    out->crc24 = read_bits_be(payload, bit, 24);

    (void)payload_len;
}

void decode_bd3cnav1ephb(const uint8_t *payload, size_t payload_len, BD3CNAV1EPHB_Decoded_t *out)
{
    size_t bit = 0;

    memset(out, 0, sizeof(*out));
    memcpy(out->head, payload, sizeof(out->head));
    bit += 64;

    out->gps_week_count = (uint16_t)read_bits_be(payload, bit, 12);
    bit += 12;
    out->gps_tow_s = read_bits_be(payload, bit, 20);
    bit += 20;
    out->bds_satid = (uint8_t)read_bits_be(payload, bit, 6);
    bit += 6;
    out->bds_sattype = (uint8_t)read_bits_be(payload, bit, 3);
    bit += 3;
    out->bds_week = (uint16_t)read_bits_be(payload, bit, 13);
    bit += 13;
    out->bds_toe = read_bits_be(payload, bit, 11) * 300u;
    bit += 11;
    out->bds_toc = read_bits_be(payload, bit, 11) * 300u;
    bit += 11;
    out->bds_af0 = scale_pow2_64(read_signed_bits_be64(payload, bit, 25), -34);
    bit += 25;
    out->bds_af1 = scale_pow2_64(read_signed_bits_be64(payload, bit, 22), -50);
    bit += 22;
    out->bds_af2 = scale_pow2_64(read_signed_bits_be64(payload, bit, 11), -66);
    bit += 11;
    out->bds_iode = (uint8_t)read_bits_be(payload, bit, 8);
    bit += 8;
    out->bds_iodc = (uint16_t)read_bits_be(payload, bit, 16);
    bit += 16;
    out->bds_idot = scale_pi_pow2_64(read_signed_bits_be64(payload, bit, 15), -44);
    bit += 15;
    out->bds_crs = scale_pow2_64(read_signed_bits_be64(payload, bit, 24), -8);
    bit += 24;
    out->bds_crc = scale_pow2_64(read_signed_bits_be64(payload, bit, 24), -8);
    bit += 24;
    out->bds_cus = scale_pow2_64(read_signed_bits_be64(payload, bit, 21), -30);
    bit += 21;
    out->bds_cuc = scale_pow2_64(read_signed_bits_be64(payload, bit, 21), -30);
    bit += 21;
    out->bds_cis = scale_pow2_64(read_signed_bits_be64(payload, bit, 16), -30);
    bit += 16;
    out->bds_cic = scale_pow2_64(read_signed_bits_be64(payload, bit, 16), -30);
    bit += 16;
    out->bds_delta_n0 = scale_pi_pow2_64(read_signed_bits_be64(payload, bit, 17), -44);
    bit += 17;
    out->bds_delta_n0_dot = scale_pow2_64(read_signed_bits_be64(payload, bit, 23), -57);
    bit += 23;
    out->bds_m0 = scale_pi_pow2_64((int64_t)read_bits_be64(payload, bit, 33), -32);
    bit += 33;
    out->bds_ecc = scale_pow2_u64(read_bits_be64(payload, bit, 33), -34);
    bit += 33;
    out->bds_deltaA = scale_pow2_64((int64_t)read_bits_be64(payload, bit, 26), -9);
    bit += 26;
    out->bds_adot = scale_pow2_64(read_signed_bits_be64(payload, bit, 25), -21);
    bit += 25;
    out->bds_omega0 = scale_pi_pow2_64((int64_t)read_bits_be64(payload, bit, 33), -32);
    bit += 33;
    out->bds_i0 = scale_pi_pow2_64((int64_t)read_bits_be64(payload, bit, 33), -32);
    bit += 33;
    out->bds_omega = scale_pi_pow2_64((int64_t)read_bits_be64(payload, bit, 33), -32);
    bit += 33;
    out->bds_omegadot = scale_pi_pow2_64(read_signed_bits_be64(payload, bit, 19), -44);
    bit += 19;
    out->bds_tgdb1cp = scale_pow2_64(read_signed_bits_be64(payload, bit, 12), -34);
    bit += 12;
    out->bds_tgdb2ap = scale_pow2_64(read_signed_bits_be64(payload, bit, 12), -34);
    bit += 12;
    out->bds_iscb1cd = scale_pow2_64(read_signed_bits_be64(payload, bit, 12), -34);
    bit += 12;
    out->bds_reserved2 = (uint8_t)read_bits_be(payload, bit, 2);
    bit += 2;
    out->crc24 = read_bits_be(payload, bit, 24);
}

void decode_prangeb(const uint8_t *payload, size_t payload_len, PRANGEB_Decoded_t *out)
{
    size_t bit = 0;
    size_t i;
    const size_t available_bits = (payload_len > 3) ? (payload_len - 3) * 8 : 0; // exclude trailing CRC

    memset(out, 0, sizeof(*out));
    if (payload_len < 3)
        return;

    memcpy(out->head, payload, sizeof(out->head));
    bit += 64;

    out->gps_week_count = (uint16_t)read_bits_be(payload, bit, 12);
    bit += 12;
    out->gps_tow_s = read_bits_be(payload, bit, 20);
    bit += 20;
    out->ms_count = (uint8_t)read_bits_be(payload, bit, 7);
    bit += 7;
    out->sync_flag = (uint8_t)read_bits_be(payload, bit, 1);
    bit += 1;
    out->system_id = (uint8_t)read_bits_be(payload, bit, 3);
    bit += 3;
    out->sat_count = (uint8_t)read_bits_be(payload, bit, 6);
    bit += 6;

    /* skip reserved 7 bits */
    bit += 7;

    for (i = 0; i < out->sat_count /* && (bit + 122) <= available_bits */ && i < PRANGEB_MAX_SATS; ++i)
    {
        out->sat_id[i] = (uint8_t)read_bits_be(payload, bit, 6);
        bit += 6;
        out->signal_count[i] = (uint8_t)read_bits_be(payload, bit, 4);
        bit += 4;
        out->int_ms[i] = (uint8_t)read_bits_be(payload, bit, 8);
        bit += 8;
        out->frac_ms[i] = (uint16_t)read_bits_be(payload, bit, 10);
        bit += 10;
        out->approx_phase_rate[i] = (int16_t)read_signed_bits_be(payload, bit, 14);
        bit += 14;

        /* reserved 6 bits (ID13) */
        bit += 6;

        out->signal_id[i] = (uint8_t)read_bits_be(payload, bit, 6);
        bit += 6;
        out->phase_lock_flag[i] = (uint8_t)read_bits_be(payload, bit, 4);
        bit += 4;
        out->precise_pr[i] = (int16_t)read_signed_bits_be(payload, bit, 15);
        bit += 15;
        out->precise_phase[i] = (int32_t)read_signed_bits_be(payload, bit, 22);
        bit += 22;
        out->precise_phase_rate[i] = (int16_t)read_signed_bits_be(payload, bit, 15);
        bit += 15;
        out->cn0[i] = (uint16_t)read_bits_be(payload, bit, 13);
        bit += 13;
        out->half_cycle[i] = (uint8_t)read_bits_be(payload, bit, 1);
        bit += 1;
        out->reserved[i] = (uint8_t)read_bits_be(payload, bit, 4);
        bit += 4;
    }

    /* embedded CRC24 is stored in last 3 bytes of payload */
    out->crc24 = read_u24_be(payload + payload_len - 3);
}

void decode_prange2b(const uint8_t *payload, size_t payload_len, PRANGE2B_Decoded_t *out)
{
    size_t bit = 0;
    size_t i;
    const size_t available_bits = (payload_len > 3) ? (payload_len - 3) * 8 : 0;

    memset(out, 0, sizeof(*out));
    if (payload_len < 3)
        return;

    memcpy(out->head, payload, sizeof(out->head));
    bit += 64;

    out->gps_week_count = (uint16_t)read_bits_be(payload, bit, 12);
    bit += 12;
    out->gps_tow_s = read_bits_be(payload, bit, 20);
    bit += 20;
    out->ms_count = (uint8_t)read_bits_be(payload, bit, 7);
    bit += 7;
    out->sync_flag = (uint8_t)read_bits_be(payload, bit, 1);
    bit += 1;
    out->system_id = (uint8_t)read_bits_be(payload, bit, 3);
    bit += 3;
    out->sat_count = (uint8_t)read_bits_be(payload, bit, 6);
    bit += 6;

    /* skip reserved 7 bits */
    bit += 7;

    for (i = 0; i < out->sat_count && (bit + 122) <= available_bits && i < PRANGEB_MAX_SATS; ++i)
    {
        out->sat_id[i] = (uint8_t)read_bits_be(payload, bit, 6);
        bit += 6;
        out->signal_count[i] = (uint8_t)read_bits_be(payload, bit, 4);
        bit += 4;
        out->int_ms[i] = (uint8_t)read_bits_be(payload, bit, 8);
        bit += 8;
        out->frac_ms[i] = (uint16_t)read_bits_be(payload, bit, 10);
        bit += 10;
        out->approx_phase_rate[i] = (int16_t)read_signed_bits_be(payload, bit, 14);
        bit += 14;

        /* PRANGE2B-specific ext flag */
        out->ext_flag[i] = (uint8_t)read_bits_be(payload, bit, 2);
        bit += 2;

        out->signal_id[i] = (uint8_t)read_bits_be(payload, bit, 6);
        bit += 6;
        out->phase_lock_flag[i] = (uint8_t)read_bits_be(payload, bit, 4);
        bit += 4;
        out->precise_pr[i] = (int16_t)read_signed_bits_be(payload, bit, 15);
        bit += 15;
        out->precise_phase[i] = (int32_t)read_signed_bits_be(payload, bit, 22);
        bit += 22;
        out->precise_phase_rate[i] = (int16_t)read_signed_bits_be(payload, bit, 15);
        bit += 15;
        out->cn0[i] = (uint16_t)read_bits_be(payload, bit, 13);
        bit += 13;
        out->half_cycle[i] = (uint8_t)read_bits_be(payload, bit, 1);
        bit += 1;
        out->reserved[i] = (uint8_t)read_bits_be(payload, bit, 4);
        bit += 4;
    }

    out->crc24 = read_u24_be(payload + payload_len - 3);
}

void decode_gpsephb(const uint8_t *payload, size_t payload_len, GPSEPHB_Decoded_t *out)
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
    out->gps_af0 = scale_pow2(read_signed_bits_be(payload, bit, 22), -31);
    bit += 22;
    out->gps_af1 = scale_pow2(read_signed_bits_be(payload, bit, 16), -43);
    bit += 16;
    out->gps_af2 = scale_pow2(read_signed_bits_be(payload, bit, 8), -55);
    bit += 8;
    out->gps_iode = (uint8_t)read_bits_be(payload, bit, 8);
    bit += 8;
    out->gps_iodc = (uint16_t)read_bits_be(payload, bit, 10);
    bit += 10;
    out->gps_idot = scale_pow2(read_signed_bits_be(payload, bit, 14), -43);
    bit += 14;
    out->gps_crs = scale_pow2(read_signed_bits_be(payload, bit, 16), -5);
    bit += 16;
    out->gps_crc = scale_pow2(read_signed_bits_be(payload, bit, 16), -5);
    bit += 16;
    out->gps_cus = scale_pow2(read_signed_bits_be(payload, bit, 16), -29);
    bit += 16;
    out->gps_cuc = scale_pow2(read_signed_bits_be(payload, bit, 16), -29);
    bit += 16;
    out->gps_cis = scale_pow2(read_signed_bits_be(payload, bit, 16), -29);
    bit += 16;
    out->gps_cic = scale_pow2(read_signed_bits_be(payload, bit, 16), -29);
    bit += 16;
    out->gps_delta_n = scale_pow2(read_signed_bits_be(payload, bit, 16), -43);
    bit += 16;
    out->gps_m0 = scale_pow2(read_signed_bits_be(payload, bit, 22), -31);
    bit += 32;
    out->gps_ecc = scale_pow2_u(read_bits_be(payload, bit, 32), -33);
    bit += 32;
    out->gps_a_half = scale_pow2_u(read_bits_be(payload, bit, 32), -19);
    bit += 32;
    out->gps_omega0 = scale_pow2(read_signed_bits_be(payload, bit, 32), -31);
    bit += 32;
    out->gps_i0 = scale_pow2(read_signed_bits_be(payload, bit, 32), -31);
    bit += 32;
    out->gps_omega = scale_pow2(read_signed_bits_be(payload, bit, 32), -31);
    bit += 32;
    out->gps_omegadot = scale_pow2(read_signed_bits_be(payload, bit, 24), -43);
    bit += 24;
    out->gps_tgd = scale_pow2(read_signed_bits_be(payload, bit, 8), -31);
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
