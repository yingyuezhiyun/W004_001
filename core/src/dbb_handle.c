#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <iconv.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include "dbb_func.h"
#include "gnss_func.h"






static int dbb_hex_encode(const uint8_t *input, size_t input_len, char *output, size_t output_size)
{
    static const char lut[] = "0123456789ABCDEF";

    if (input == NULL || output == NULL || output_size < input_len * 2 + 1)
    {
        return -1;
    }

    for (size_t i = 0; i < input_len; ++i)
    {
        output[i * 2] = lut[(input[i] >> 4) & 0x0F];
        output[i * 2 + 1] = lut[input[i] & 0x0F];
    }

    output[input_len * 2] = '\0';
    return 0;
}

static int dbb_base64_value(char c)
{
    if (c >= 'A' && c <= 'Z')
    {
        return c - 'A';
    }
    if (c >= 'a' && c <= 'z')
    {
        return c - 'a' + 26;
    }
    if (c >= '0' && c <= '9')
    {
        return c - '0' + 52;
    }
    if (c == '+')
    {
        return 62;
    }
    if (c == '/')
    {
        return 63;
    }
    return -1;
}

static int dbb_base64_encode(const uint8_t *input, size_t input_len, char *output, size_t output_size)
{
    static const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t out_len = ((input_len + 2) / 3) * 4;

    if (input == NULL || output == NULL || output_size < out_len + 1)
    {
        return -1;
    }

    size_t i = 0;
    size_t j = 0;
    while (i < input_len)
    {
        size_t remain = input_len - i;
        uint32_t octet_a = input[i++];
        uint32_t octet_b = remain > 1 ? input[i++] : 0;
        uint32_t octet_c = remain > 2 ? input[i++] : 0;
        uint32_t triple = (octet_a << 16) | (octet_b << 8) | octet_c;

        output[j++] = table[(triple >> 18) & 0x3F];
        output[j++] = table[(triple >> 12) & 0x3F];
        output[j++] = remain > 1 ? table[(triple >> 6) & 0x3F] : '=';
        output[j++] = remain > 2 ? table[triple & 0x3F] : '=';
    }

    output[out_len] = '\0';
    return 0;
}

static int dbb_base64_decode(const char *input, uint8_t *output, size_t output_size, size_t *output_len)
{
    size_t len = 0;

    if (input == NULL || output == NULL || output_len == NULL)
    {
        return -1;
    }

    const char *p = input;
    while (1)
    {
        int quartet[4] = {0};
        int q = 0;

        while (*p != '\0' && q < 4)
        {
            if (isspace((unsigned char)*p))
            {
                ++p;
                continue;
            }

            if (*p == '=')
            {
                quartet[q++] = -2;
                ++p;
                continue;
            }

            int val = dbb_base64_value(*p);
            if (val < 0)
            {
                return -1;
            }
            quartet[q++] = val;
            ++p;
        }

        if (q == 0)
        {
            break;
        }

        if (q != 4)
        {
            return -1;
        }

        int pad = 0;
        for (int i = 0; i < 4; ++i)
        {
            if (quartet[i] == -2)
            {
                pad++;
                quartet[i] = 0;
            }
        }

        uint32_t triple = ((uint32_t)quartet[0] << 18) |
                          ((uint32_t)quartet[1] << 12) |
                          ((uint32_t)quartet[2] << 6) |
                          (uint32_t)quartet[3];

        if (len >= output_size)
        {
            return -1;
        }
        output[len++] = (uint8_t)((triple >> 16) & 0xFF);

        if (pad < 2)
        {
            if (len >= output_size)
            {
                return -1;
            }
            output[len++] = (uint8_t)((triple >> 8) & 0xFF);
        }

        if (pad < 1)
        {
            if (len >= output_size)
            {
                return -1;
            }
            output[len++] = (uint8_t)(triple & 0xFF);
        }

        if (pad > 0)
        {
            break;
        }
    }

    *output_len = len;
    return 0;
}




static int hex_char_to_nibble(char c)
{
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F')
    {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f')
    {
        return c - 'a' + 10;
    }
    return -1;
}

static int is_hex_string(const char *s)
{
    size_t len;

    if (s == NULL)
    {
        return 0;
    }

    len = strlen(s);
    if (len == 0 || (len % 2) != 0)
    {
        return 0;
    }

    for (size_t i = 0; i < len; ++i)
    {
        if (!isxdigit((unsigned char)s[i]))
        {
            return 0;
        }
    }

    return 1;
}

static int hex_to_bytes(const char *hex, uint8_t *out, size_t out_size, size_t *out_len)
{
    size_t hex_len;
    size_t bytes_len;

    if (!is_hex_string(hex) || out == NULL || out_len == NULL)
    {
        return -1;
    }

    hex_len = strlen(hex);
    bytes_len = hex_len / 2;

    if (bytes_len > out_size)
    {
        return -1;
    }

    for (size_t i = 0; i < bytes_len; ++i)
    {
        int hi = hex_char_to_nibble(hex[2 * i]);
        int lo = hex_char_to_nibble(hex[2 * i + 1]);
        if (hi < 0 || lo < 0)
        {
            return -1;
        }
        out[i] = (uint8_t)((hi << 4) | lo);
    }

    *out_len = bytes_len;
    return 0;
}

static int bytes_to_hex(const uint8_t *data, size_t data_len, char *hex_out, size_t hex_out_size)
{
    static const char lut[] = "0123456789ABCDEF";

    if (data == NULL || hex_out == NULL || hex_out_size < data_len * 2 + 1)
    {
        return -1;
    }

    for (size_t i = 0; i < data_len; ++i)
    {
        hex_out[2 * i] = lut[(data[i] >> 4) & 0x0F];
        hex_out[2 * i + 1] = lut[data[i] & 0x0F];
    }

    hex_out[data_len * 2] = '\0';
    return 0;
}

static int utf8_to_ucs2be_iconv(const char *utf8, uint8_t *ucs2_out, size_t ucs2_out_size, size_t *ucs2_len)
{
    iconv_t cd;
    char *in_buf;
    char *out_buf;
    size_t in_left;
    size_t out_left;

    if (utf8 == NULL || ucs2_out == NULL || ucs2_len == NULL)
    {
        return -1;
    }

    cd = iconv_open("UCS-2BE", "UTF-8");
    if (cd == (iconv_t)-1)
    {
        perror("iconv_open UTF-8->UCS-2BE");
        return -1;
    }

    in_buf = (char *)utf8;
    out_buf = (char *)ucs2_out;
    in_left = strlen(utf8);
    out_left = ucs2_out_size;

    if (iconv(cd, &in_buf, &in_left, &out_buf, &out_left) == (size_t)-1)
    {
        perror("iconv UTF-8->UCS-2BE");
        iconv_close(cd);
        return -1;
    }

    *ucs2_len = ucs2_out_size - out_left;
    iconv_close(cd);

    return 0;
}

static int ucs2be_to_utf8_iconv(const uint8_t *ucs2, size_t ucs2_len, char *utf8_out, size_t utf8_out_size)
{
    iconv_t cd;
    char *in_buf;
    char *out_buf;
    size_t in_left;
    size_t out_left;

    if (ucs2 == NULL || utf8_out == NULL || utf8_out_size == 0 || (ucs2_len % 2) != 0)
    {
        return -1;
    }

    cd = iconv_open("UTF-8", "UCS-2BE");
    if (cd == (iconv_t)-1)
    {
        perror("iconv_open UCS-2BE->UTF-8");
        return -1;
    }

    in_buf = (char *)ucs2;
    out_buf = utf8_out;
    in_left = ucs2_len;
    out_left = utf8_out_size - 1;

    if (iconv(cd, &in_buf, &in_left, &out_buf, &out_left) == (size_t)-1)
    {
        perror("iconv UCS-2BE->UTF-8");
        iconv_close(cd);
        return -1;
    }

    *out_buf = '\0';
    iconv_close(cd);

    return 0;
}

static int encode_bcd_number(const char *number, uint8_t *bcd_out, size_t bcd_out_size, uint8_t *digits_len, uint8_t *toa)
{
    char digits[DBB_MAX_SMS_TARGET_LEN];
    size_t digits_count = 0;
    size_t bytes_needed;

    if (number == NULL || bcd_out == NULL || digits_len == NULL || toa == NULL)
    {
        return -1;
    }

    for (size_t i = 0; number[i] != '\0'; ++i)
    {
        if (number[i] == '+')
        {
            *toa = 0x91;
            continue;
        }

        if (!isdigit((unsigned char)number[i]))
        {
            return -1;
        }

        if (digits_count >= sizeof(digits) - 1)
        {
            return -1;
        }

        digits[digits_count++] = number[i];
    }

    if (digits_count == 0)
    {
        return -1;
    }

    if (*toa != 0x91)
    {
        if (digits_count >= 11 && digits[0] == '8' && digits[1] == '6')
        {
            *toa = 0x91;
        }
        else
        {
            *toa = 0x81;
        }
    }

    bytes_needed = (digits_count + 1) / 2;
    if (bytes_needed > bcd_out_size)
    {
        return -1;
    }

    for (size_t i = 0; i < bytes_needed; ++i)
    {
        uint8_t low = (uint8_t)(digits[2 * i] - '0');
        uint8_t high = 0x0F;

        if ((2 * i + 1) < digits_count)
        {
            high = (uint8_t)(digits[2 * i + 1] - '0');
        }

        bcd_out[i] = (uint8_t)((high << 4) | low);
    }

    *digits_len = (uint8_t)digits_count;
    return (int)bytes_needed;
}

static int decode_bcd_number(const uint8_t *bcd, size_t bcd_len, uint8_t digits_len, char *out, size_t out_size)
{
    size_t pos = 0;

    if (bcd == NULL || out == NULL || out_size == 0)
    {
        return -1;
    }

    for (size_t i = 0; i < bcd_len; ++i)
    {
        uint8_t low = bcd[i] & 0x0F;
        uint8_t high = (bcd[i] >> 4) & 0x0F;

        if (pos < digits_len)
        {
            if (low > 9 || pos + 1 >= out_size)
            {
                return -1;
            }
            out[pos++] = (char)('0' + low);
        }

        if (pos < digits_len)
        {
            if (high > 9 || pos + 1 >= out_size)
            {
                return -1;
            }
            out[pos++] = (char)('0' + high);
        }
    }

    out[pos] = '\0';
    return 0;
}

static int dbb_build_sms_submit_pdu(const char *target, const char *text, char *pdu_hex, size_t pdu_hex_size, int *cmgs_len)
{
    uint8_t pdu[256];
    uint8_t da_bcd[16];
    uint8_t ud[DBB_MAX_SMS_UD_BYTES];
    uint8_t da_digits = 0;
    uint8_t toa = 0;
    size_t ud_len = 0;
    size_t p = 0;
    int da_bcd_len;

    if (target == NULL || text == NULL || pdu_hex == NULL || cmgs_len == NULL)
    {
        return -1;
    }

    da_bcd_len = encode_bcd_number(target, da_bcd, sizeof(da_bcd), &da_digits, &toa);
    if (da_bcd_len <= 0)
    {
        dbb_debug_err("invalid sms target number: %s", target);
        return -1;
    }

    if (utf8_to_ucs2be_iconv(text, ud, sizeof(ud), &ud_len) < 0)
    {
        dbb_debug_err("sms text encoding failed");
        return -1;
    }

    if (ud_len > DBB_MAX_SMS_UD_BYTES)
    {
        dbb_debug_err("sms text too long");
        return -1;
    }

    pdu[p++] = 0x00;
    pdu[p++] = 0x01;
    pdu[p++] = 0x00;
    pdu[p++] = da_digits;
    pdu[p++] = toa;

    memcpy(pdu + p, da_bcd, (size_t)da_bcd_len);
    p += (size_t)da_bcd_len;

    pdu[p++] = 0x00;
    pdu[p++] = 0x08;
    pdu[p++] = (uint8_t)ud_len;

    memcpy(pdu + p, ud, ud_len);
    p += ud_len;

    if (bytes_to_hex(pdu, p, pdu_hex, pdu_hex_size) < 0)
    {
        return -1;
    }

    *cmgs_len = (int)p - 1;
    return 0;
}

static int dbb_decode_sms_deliver_pdu(const char *pdu_hex, char *from, size_t from_size, char *text, size_t text_size)
{
    uint8_t pdu[256];
    size_t pdu_len = 0;
    size_t idx = 0;
    uint8_t sca_len;
    uint8_t oa_digits;
    size_t oa_bytes;
    uint8_t dcs;
    uint8_t udl;

    if (hex_to_bytes(pdu_hex, pdu, sizeof(pdu), &pdu_len) < 0)
    {
        return -1;
    }

    if (pdu_len < 8)
    {
        return -1;
    }

    sca_len = pdu[idx++];
    if (idx + sca_len > pdu_len)
    {
        return -1;
    }
    idx += sca_len;

    if (idx + 2 > pdu_len)
    {
        return -1;
    }

    idx++;
    oa_digits = pdu[idx++];

    if (idx + 1 > pdu_len)
    {
        return -1;
    }

    idx++;

    oa_bytes = (oa_digits + 1) / 2;
    if (idx + oa_bytes + 1 + 1 + 7 + 1 > pdu_len)
    {
        return -1;
    }

    if (decode_bcd_number(pdu + idx, oa_bytes, oa_digits, from, from_size) < 0)
    {
        return -1;
    }
    idx += oa_bytes;

    idx++;
    dcs = pdu[idx++];

    idx += 7;

    udl = pdu[idx++];
    if (idx + udl > pdu_len)
    {
        return -1;
    }

    if (dcs == 0x08)
    {
        if (ucs2be_to_utf8_iconv(pdu + idx, udl, text, text_size) < 0)
        {
            return -1;
        }
    }
    else
    {
        snprintf(text, text_size, "[unsupported DCS=0x%02X]", dcs);
    }

    return 0;
}


int dbb_send_sms(const char *target, const char *text)
{
    char pdu_hex[DBB_MAX_SMS_PDU_HEX];
    char command[64];
    char response[DBB_MAX_RESPONSE];
    int cmgs_len;
    char end_char = 0x1A;

    if (dbb_build_sms_submit_pdu(target, text, pdu_hex, sizeof(pdu_hex), &cmgs_len) < 0)
    {
        return -1;
    }

    snprintf(command, sizeof(command), "AT+CMGS=%d", cmgs_len);

    if (dbb_capture_response(command, response, sizeof(response), DBB_AT_TIMEOUT_MS) < 0)
    {
        dbb_debug_err("AT+CMGS command failed");
        return -1;
    }

    dbb_dump_response(response);

    if (strchr(response, '>') == NULL)
    {
        dbb_debug_err("AT+CMGS prompt not received");
        return -1;
    }

    if (dbb_write_all(pdu_hex, strlen(pdu_hex)) < 0)
    {
        return -1;
    }

    if (dbb_write_all(&end_char, 1) < 0)
    {
        return -1;
    }

    if (dbb_capture_response(NULL, response, sizeof(response), 10000) < 0)
    {
        dbb_debug_err("wait sms send result timeout");
        return -1;
    }

    dbb_dump_response(response);

    if (strstr(response, "OK") == NULL || strstr(response, "+CMGS:") == NULL)
    {
        dbb_debug_err("sms send failed");
        return -1;
    }

    dbb_debug_info("sms sent to %s", target);
    return 0;
}




static int dbb_data_prepare_uplink(const uint8_t *raw_input,
                                   size_t raw_input_len,
                                   dbb_data_codec_t codec,
                                   char *payload_out,
                                   size_t payload_out_size,
                                   size_t *raw_len_out,
                                   const char **chset_out)
{
    if (raw_input == NULL || payload_out == NULL || raw_len_out == NULL || chset_out == NULL)
    {
        return -1;
    }

    if (raw_input_len > 0)
    {
        *raw_len_out = raw_input_len;
        if (codec == DBB_DATA_CODEC_BASE64)
        {
            if (dbb_base64_encode(raw_input, raw_input_len, payload_out, payload_out_size) < 0)
            {
                return -1;
            }
            *chset_out = "BASE64";
        }
        else
        {
            if (dbb_hex_encode(raw_input, raw_input_len, payload_out, payload_out_size) < 0)
            {
                return -1;
            }
            *chset_out = "HEX";
        }

        return 0;
    }

    return -1;
}




int dbb_send_uplink(const uint8_t *raw_payload, size_t raw_payload_len, dbb_data_codec_t codec)
{
    char command[DBB_MAX_UL_DATA_HEX + 64];
    char payload[DBB_MAX_B64_DATA];
    size_t bytes_len;
    const char *chset = NULL;

    if (dbb_data_prepare_uplink(raw_payload, raw_payload_len, codec, payload, sizeof(payload), &bytes_len, &chset) < 0)
    {
        return -1;
    }

    if (snprintf(command, sizeof(command), "AT+ULNETDATA=%zu,\"%s\",\"%s\"", bytes_len, chset, payload) >= (int)sizeof(command))
    {
        dbb_debug_err("uplink payload too long");
        return -1;
    }

    if (dbb_at_expect("send uplink data", command, NULL) < 0)
    {
        return -1;
    }

    dbb_debug_info("uplink sent: %zu bytes via %s", bytes_len, chset);
    return 0;
}




static void dbb_handle_sms_cmt(const char *urc)
{
    const char *line = strstr(urc, "+CMT:");
    char format[16] = {0};
    int pdu_len = 0;
    const char *pdu_start;
    const char *pdu_end;
    size_t pdu_hex_len;
    char pdu_hex[DBB_MAX_SMS_PDU_HEX];
    char from[64] = {0};
    char text[256] = {0};

    if (line == NULL)
    {
        return;
    }

    if (sscanf(line, "+CMT:\"%15[^\"]\",%d", format, &pdu_len) != 2)
    {
        dbb_debug_err("parse +CMT header failed");
        return;
    }

    pdu_start = strchr(line, '\n');
    if (pdu_start == NULL)
    {
        dbb_debug_err("parse +CMT body failed");
        return;
    }

    while (*pdu_start == '\r' || *pdu_start == '\n')
    {
        ++pdu_start;
    }

    pdu_end = pdu_start;
    while (*pdu_end != '\0' && *pdu_end != '\r' && *pdu_end != '\n')
    {
        ++pdu_end;
    }

    pdu_hex_len = (size_t)(pdu_end - pdu_start);
    if (pdu_hex_len == 0 || pdu_hex_len >= sizeof(pdu_hex))
    {
        dbb_debug_err("+CMT pdu length invalid");
        return;
    }

    memcpy(pdu_hex, pdu_start, pdu_hex_len);
    pdu_hex[pdu_hex_len] = '\0';

    if (strcmp(format, "HEX") != 0)
    {
        dbb_debug_info("+CMT format=%s not handled", format);
        return;
    }

    if (dbb_decode_sms_deliver_pdu(pdu_hex, from, sizeof(from), text, sizeof(text)) < 0)
    {
        dbb_debug_err("decode +CMT pdu failed: %s", pdu_hex);
        return;
    }

    dbb_debug_info("SMS recv from %s: %s", from, text);
    (void)pdu_len;
}

static void dbb_handle_dlnetdata(const char *urc)
{
    const char *p = urc;

    while (1)
    {
        int datalen = 0;
        char chset[16] = {0};
        char datastr[DBB_MAX_UL_DATA_HEX] = {0};

        p = strstr(p, "+DLNETDATA=");
        if (p == NULL)
        {
            break;
        }

        if (sscanf(p, "+DLNETDATA=%d,\"%15[^\"]\",\"%1023[^\"]\"", &datalen, chset, datastr) == 3)
        {
            size_t raw_len = 0;
            int decode_rc = -1;

            if (strcasecmp(chset, "HEX") == 0)
            {
                decode_rc = hex_to_bytes(datastr, dbb_ctrl.downlink_raw, sizeof(dbb_ctrl.downlink_raw), &raw_len);
            }
            else if (strcasecmp(chset, "BASE64") == 0)
            {
                decode_rc = dbb_base64_decode(datastr, dbb_ctrl.downlink_raw, sizeof(dbb_ctrl.downlink_raw), &raw_len);
            }

            if (decode_rc == 0)
            {
                dbb_ctrl.downlink_raw_len = raw_len;
                if (raw_len < sizeof(dbb_ctrl.downlink_text))
                {
                    memcpy(dbb_ctrl.downlink_text, dbb_ctrl.downlink_raw, raw_len);
                    dbb_ctrl.downlink_text[raw_len] = '\0';
                }
                else
                {
                    dbb_ctrl.downlink_text[0] = '\0';
                }
                dbb_debug_info("downlink data: len=%d chset=%s raw_len=%zu", datalen, chset, raw_len);
            }
            else
            {
                dbb_debug_err("decode +DLNETDATA failed: chset=%s data=%s", chset, datastr);
            }
        }
        else
        {
            dbb_debug_err("parse +DLNETDATA failed");
        }

        p += 11;
    }
}

static void dbb_handle_ssrinfoxw(const char *urc)
{
    const char *line;
    const char *payload_start;
    const char *payload_end;
    size_t payload_len;
    size_t raw_len = 0;
    uint8_t raw[DBB_MAX_RESPONSE];

    line = strstr(urc, "^SSRINFOXW:");
    if (line == NULL)
    {
        return;
    }

    payload_start = strchr(line, '"');
    if (payload_start == NULL)
    {
        dbb_debug_err("parse ^SSRINFOXW failed: missing opening quote");
        return;
    }
    payload_start++;

    payload_end = strchr(payload_start, '"');
    if (payload_end == NULL)
    {
        dbb_debug_err("parse ^SSRINFOXW failed: missing closing quote");
        return;
    }

    payload_len = (size_t)(payload_end - payload_start);
    if (payload_len == 0 || payload_len >= DBB_MAX_RESPONSE)
    {
        dbb_debug_err("invalid ^SSRINFOXW payload length: %zu", payload_len);
        return;
    }

    {
        char payload[DBB_MAX_RESPONSE];
        memcpy(payload, payload_start, payload_len);
        payload[payload_len] = '\0';

        if (hex_to_bytes(payload, raw, sizeof(raw), &raw_len) < 0)
        {
            dbb_debug_err("decode ^SSRINFOXW hex payload failed");
            return;
        }
    }

    if (gnss_ctrl.fd < 0)
    {
        dbb_debug_err("drop ^SSRINFOXW payload: GNSS device is not open");
        return;
    }

    if (gnss_dev_write(gnss_ctrl.fd, raw, raw_len) != (int)raw_len)
    {
        dbb_debug_err("forward ^SSRINFOXW payload to GNSS failed: %zu bytes", raw_len);
        return;
    }

    dbb_debug_info("forwarded ^SSRINFOXW payload to GNSS: %zu bytes", raw_len);
}

static void dbb_handle_cgev(const char *urc)
{
    const char *p = strstr(urc, "+CGEV:");
    if (p != NULL)
    {
        dbb_debug_info("network event: %s", p);
    }
}

void dbb_handle_urc_blob(const char *urc)
{
    if (urc == NULL || urc[0] == '\0')
    {
        return;
    }

    dbb_handle_dlnetdata(urc);
    dbb_handle_ssrinfoxw(urc);
    dbb_handle_sms_cmt(urc);
    dbb_handle_cgev(urc);
}


