#include "ah_protocol.h"

#include <string.h>

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void wr_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

uint16_t ah_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    size_t i;
    int bit;
    for (i = 0; i < len; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

size_t ah_encode_frame(uint8_t type, uint8_t seq, const uint8_t *payload,
                       uint8_t len, uint8_t *out, size_t out_cap)
{
    size_t total;
    uint16_t crc;
    if (len > AH_MAX_PAYLOAD || out == NULL || (len > 0 && payload == NULL)) {
        return 0;
    }
    total = (size_t)len + 8u;
    if (out_cap < total) {
        return 0;
    }
    out[0] = 0xAAu;
    out[1] = 0x55u;
    out[2] = AH_PROTO_VERSION;
    out[3] = type;
    out[4] = seq;
    out[5] = len;
    if (len) {
        memcpy(&out[6], payload, len);
    }
    crc = ah_crc16(&out[2], (size_t)len + 4u);
    out[6 + len] = (uint8_t)(crc & 0xFFu);
    out[7 + len] = (uint8_t)((crc >> 8) & 0xFFu);
    return total;
}

void ah_parser_init(ah_parser_t *parser)
{
    if (parser != NULL) {
        parser->len = 0;
    }
}

size_t ah_parser_feed(ah_parser_t *parser, const uint8_t *data, size_t len,
                      ah_frame_t *out, size_t max_frames)
{
    size_t count = 0;
    if (parser == NULL || data == NULL || out == NULL) {
        return 0;
    }
    if (parser->len + len > sizeof(parser->buf)) {
        parser->len = 0; /* overflow: drop and resync */
        if (len > sizeof(parser->buf)) {
            data += len - sizeof(parser->buf);
            len = sizeof(parser->buf);
        }
    }
    memcpy(&parser->buf[parser->len], data, len);
    parser->len += len;

    while (count < max_frames) {
        size_t start = 0;
        uint8_t flen;
        size_t total;
        uint16_t got;
        while (start + 1 < parser->len &&
               !(parser->buf[start] == 0xAAu && parser->buf[start + 1] == 0x55u)) {
            ++start;
        }
        if (start + 1 >= parser->len) {
            if (parser->len > 1) {
                parser->buf[0] = parser->buf[parser->len - 1];
                parser->len = 1;
            }
            break;
        }
        if (start > 0) {
            memmove(parser->buf, &parser->buf[start], parser->len - start);
            parser->len -= start;
        }
        if (parser->len < 6) {
            break;
        }
        flen = parser->buf[5];
        if (flen > AH_MAX_PAYLOAD) {
            memmove(parser->buf, &parser->buf[2], parser->len - 2);
            parser->len -= 2;
            continue;
        }
        total = (size_t)flen + 8u;
        if (parser->len < total) {
            break;
        }
        got = (uint16_t)((uint16_t)parser->buf[6 + flen] |
                         ((uint16_t)parser->buf[7 + flen] << 8));
        if (got != ah_crc16(&parser->buf[2], (size_t)flen + 4u)) {
            memmove(parser->buf, &parser->buf[2], parser->len - 2);
            parser->len -= 2;
            continue;
        }
        out[count].type = parser->buf[3];
        out[count].seq = parser->buf[4];
        out[count].len = flen;
        if (flen) {
            memcpy(out[count].payload, &parser->buf[6], flen);
        }
        ++count;
        memmove(parser->buf, &parser->buf[total], parser->len - total);
        parser->len -= total;
    }
    return count;
}

size_t ah_build_info(const ah_info_t *info, uint8_t *out, size_t cap)
{
    if (info == NULL || out == NULL || cap < AH_INFO_PAYLOAD) {
        return 0;
    }
    out[0] = info->proto;
    out[1] = info->fw_major;
    out[2] = info->fw_minor;
    out[3] = info->n_servos;
    wr_u16(&out[4], info->telemetry_ms);
    wr_u16(&out[6], info->watchdog_ms);
    return AH_INFO_PAYLOAD;
}

size_t ah_build_telemetry(const ah_telemetry_t *t, uint8_t *out, size_t cap)
{
    size_t i;
    if (t == NULL || out == NULL || cap < AH_TELEMETRY_PAYLOAD) {
        return 0;
    }
    out[0] = t->state;
    out[1] = t->fault;
    out[2] = t->flags;
    out[3] = t->online_mask;
    out[4] = t->seq_ack;
    wr_u32(&out[5], t->tick_ms);
    for (i = 0; i < 8; ++i) {
        wr_u16(&out[9 + i * 2], t->pos_raw[i]);
        wr_u16(&out[25 + i * 2], t->load_raw[i]);
        out[41 + i] = t->temp_c[i];
        out[49 + i] = t->error_bits[i];
    }
    wr_u16(&out[57], t->command_count);
    wr_u16(&out[59], t->feedback_count);
    wr_u16(&out[61], t->max_age_ms);
    return AH_TELEMETRY_PAYLOAD;
}

int ah_parse_telemetry(const uint8_t *payload, size_t len, ah_telemetry_t *out)
{
    size_t i;
    if (payload == NULL || out == NULL || len != AH_TELEMETRY_PAYLOAD) {
        return -1;
    }
    out->state = payload[0];
    out->fault = payload[1];
    out->flags = payload[2];
    out->online_mask = payload[3];
    out->seq_ack = payload[4];
    out->tick_ms = rd_u32(&payload[5]);
    for (i = 0; i < 8; ++i) {
        out->pos_raw[i] = rd_u16(&payload[9 + i * 2]);
        out->load_raw[i] = rd_u16(&payload[25 + i * 2]);
        out->temp_c[i] = payload[41 + i];
        out->error_bits[i] = payload[49 + i];
    }
    out->command_count = rd_u16(&payload[57]);
    out->feedback_count = rd_u16(&payload[59]);
    out->max_age_ms = rd_u16(&payload[61]);
    return 0;
}

size_t ah_build_command(const ah_command_t *c, uint8_t *out, size_t cap)
{
    size_t i;
    if (c == NULL || out == NULL || cap < AH_COMMAND_PAYLOAD) {
        return 0;
    }
    out[0] = c->cmd;
    wr_u16(&out[1], c->speed_raw);
    for (i = 0; i < 8; ++i) {
        out[3 + i] = c->torque_levels[i];
        wr_u16(&out[11 + i * 2], c->pregrasp_raw[i]);
        wr_u16(&out[27 + i * 2], c->close_raw[i]);
        wr_u16(&out[43 + i * 2], c->load_threshold_raw[i]);
    }
    wr_u16(&out[59], c->closing_timeout_ms);
    return AH_COMMAND_PAYLOAD;
}

int ah_parse_command(const uint8_t *payload, size_t len, ah_command_t *out)
{
    size_t i;
    if (payload == NULL || out == NULL || len != AH_COMMAND_PAYLOAD) {
        return -1;
    }
    out->cmd = payload[0];
    out->speed_raw = rd_u16(&payload[1]);
    for (i = 0; i < 8; ++i) {
        out->torque_levels[i] = payload[3 + i];
        out->pregrasp_raw[i] = rd_u16(&payload[11 + i * 2]);
        out->close_raw[i] = rd_u16(&payload[27 + i * 2]);
        out->load_threshold_raw[i] = rd_u16(&payload[43 + i * 2]);
    }
    out->closing_timeout_ms = rd_u16(&payload[59]);
    return 0;
}

size_t ah_build_direct(const ah_direct_t *d, uint8_t *out, size_t cap)
{
    size_t i;
    if (d == NULL || out == NULL || cap < AH_DIRECT_PAYLOAD) {
        return 0;
    }
    wr_u16(&out[0], d->speed_raw);
    for (i = 0; i < 8; ++i) {
        wr_u16(&out[2 + i * 2], d->target_raw[i]);
    }
    return AH_DIRECT_PAYLOAD;
}

int ah_parse_direct(const uint8_t *payload, size_t len, ah_direct_t *out)
{
    size_t i;
    if (payload == NULL || out == NULL || len != AH_DIRECT_PAYLOAD) {
        return -1;
    }
    out->speed_raw = rd_u16(&payload[0]);
    for (i = 0; i < 8; ++i) {
        out->target_raw[i] = rd_u16(&payload[2 + i * 2]);
    }
    return 0;
}

int ah_parse_config(const uint8_t *payload, size_t len, ah_config_t *out)
{
    if (payload == NULL || out == NULL || len != AH_CONFIG_PAYLOAD) {
        return -1;
    }
    out->watchdog_ms = rd_u16(&payload[0]);
    out->telemetry_ms = rd_u16(&payload[2]);
    out->slot_ms = rd_u16(&payload[4]);
    out->pregrasp_ms = rd_u16(&payload[6]);
    out->closing_timeout_ms = rd_u16(&payload[8]);
    out->release_ms = rd_u16(&payload[10]);
    return 0;
}

size_t ah_build_error(uint8_t code, uint8_t arg, uint8_t *out, size_t cap)
{
    if (out == NULL || cap < 2) {
        return 0;
    }
    out[0] = code;
    out[1] = arg;
    return 2;
}
