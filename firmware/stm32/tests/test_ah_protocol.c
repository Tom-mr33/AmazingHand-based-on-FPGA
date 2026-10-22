#include "ah_protocol.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        ++failures; \
    } \
} while (0)

static void test_crc(void)
{
    const uint8_t s[] = "123456789";
    CHECK(ah_crc16(s, 9) == 0x29B1u);
    CHECK(ah_crc16(NULL, 0) == 0xFFFFu);
}

static void test_command_roundtrip(void)
{
    ah_command_t in;
    ah_command_t out;
    uint8_t payload[AH_COMMAND_PAYLOAD];
    uint8_t frame[AH_MAX_FRAME];
    size_t flen;
    size_t total;
    ah_parser_t parser;
    ah_frame_t frames[2];
    size_t n;
    size_t i;

    memset(&in, 0, sizeof(in));
    in.cmd = AH_CMD_GRASP;
    in.speed_raw = 300;
    in.closing_timeout_ms = 3000;
    for (i = 0; i < 8; ++i) {
        in.torque_levels[i] = 7;
        in.pregrasp_raw[i] = (uint16_t)(100 + i);
        in.close_raw[i] = (uint16_t)(200 + i);
        in.load_threshold_raw[i] = 384;
    }
    flen = ah_build_command(&in, payload, sizeof(payload));
    CHECK(flen == AH_COMMAND_PAYLOAD);
    total = ah_encode_frame(AH_MSG_COMMAND, 9, payload, (uint8_t)flen, frame, sizeof(frame));
    CHECK(total == (size_t)flen + 8u);

    ah_parser_init(&parser);
    n = ah_parser_feed(&parser, frame, total, frames, 2);
    CHECK(n == 1);
    CHECK(frames[0].type == AH_MSG_COMMAND);
    CHECK(frames[0].seq == 9);
    CHECK(ah_parse_command(frames[0].payload, frames[0].len, &out) == 0);
    CHECK(out.cmd == AH_CMD_GRASP);
    CHECK(out.speed_raw == 300);
    CHECK(out.close_raw[7] == 207);
    CHECK(out.closing_timeout_ms == 3000);
}

static void test_telemetry_roundtrip(void)
{
    ah_telemetry_t in;
    ah_telemetry_t out;
    uint8_t payload[AH_TELEMETRY_PAYLOAD];
    uint8_t frame[AH_MAX_FRAME];
    size_t plen;
    size_t total;
    ah_parser_t parser;
    ah_frame_t frames[2];
    size_t i;

    memset(&in, 0, sizeof(in));
    in.state = AH_STATE_CLOSING;
    in.fault = AH_FAULT_OVERLOAD;
    in.flags = AH_FLAG_TORQUE | AH_FLAG_MOVING;
    in.online_mask = 0xFF;
    in.seq_ack = 42;
    in.tick_ms = 123456;
    for (i = 0; i < 8; ++i) {
        in.pos_raw[i] = (uint16_t)(2048 + i);
        in.load_raw[i] = (uint16_t)i;
        in.temp_c[i] = (uint8_t)(25 + i);
        in.error_bits[i] = 0;
    }
    in.command_count = 7;
    in.feedback_count = 99;
    in.max_age_ms = 12;

    plen = ah_build_telemetry(&in, payload, sizeof(payload));
    CHECK(plen == AH_TELEMETRY_PAYLOAD);
    total = ah_encode_frame(AH_MSG_TELEMETRY, 1, payload, (uint8_t)plen, frame, sizeof(frame));
    ah_parser_init(&parser);
    CHECK(ah_parser_feed(&parser, frame, total, frames, 2) == 1);
    CHECK(ah_parse_telemetry(frames[0].payload, frames[0].len, &out) == 0);
    CHECK(out.state == AH_STATE_CLOSING);
    CHECK(out.load_raw[7] == 7);
    CHECK(out.tick_ms == 123456);
    CHECK(out.max_age_ms == 12);
}

static void test_fragment_and_bad_crc(void)
{
    uint8_t frame[AH_MAX_FRAME];
    size_t total = ah_encode_frame(AH_MSG_HEARTBEAT, 3, NULL, 0, frame, sizeof(frame));
    ah_parser_t parser;
    ah_frame_t frames[2];
    CHECK(total == 8);
    ah_parser_init(&parser);
    CHECK(ah_parser_feed(&parser, frame, 3, frames, 2) == 0);
    CHECK(ah_parser_feed(&parser, frame + 3, total - 3, frames, 2) == 1);
    CHECK(frames[0].type == AH_MSG_HEARTBEAT);

    frame[total - 1] ^= 0xFF;
    ah_parser_init(&parser);
    CHECK(ah_parser_feed(&parser, frame, total, frames, 2) == 0);
}

static void test_direct(void)
{
    ah_direct_t in;
    ah_direct_t out;
    uint8_t payload[AH_DIRECT_PAYLOAD];
    uint8_t frame[AH_MAX_FRAME];
    size_t i;
    size_t total;
    ah_parser_t parser;
    ah_frame_t frames[2];

    memset(&in, 0, sizeof(in));
    in.speed_raw = 300;
    for (i = 0; i < 8; ++i) {
        in.target_raw[i] = (uint16_t)(2048 + i);
    }
    CHECK(ah_build_direct(&in, payload, sizeof(payload)) == AH_DIRECT_PAYLOAD);
    total = ah_encode_frame(AH_MSG_DIRECT_TARGETS, 1, payload, AH_DIRECT_PAYLOAD, frame, sizeof(frame));
    ah_parser_init(&parser);
    CHECK(ah_parser_feed(&parser, frame, total, frames, 2) == 1);
    CHECK(ah_parse_direct(frames[0].payload, frames[0].len, &out) == 0);
    CHECK(out.speed_raw == 300);
    CHECK(out.target_raw[7] == 2055);
}

int main(void)
{
    test_crc();
    test_command_roundtrip();
    test_telemetry_roundtrip();
    test_fragment_and_bad_crc();
    test_direct();
    if (failures) {
        printf("AH_PROTOCOL_TEST_FAILED (%d)\n", failures);
        return 1;
    }
    printf("AH_PROTOCOL_TEST_OK\n");
    return 0;
}
