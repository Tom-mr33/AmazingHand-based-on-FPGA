#ifndef AH_PROTOCOL_H
#define AH_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AH_PROTO_VERSION 1
#define AH_MAX_PAYLOAD 64
#define AH_MAX_FRAME (AH_MAX_PAYLOAD + 8)
#define AH_PARSER_BUF 160

/* message types */
#define AH_MSG_HELLO 0x01
#define AH_MSG_INFO 0x02
#define AH_MSG_COMMAND 0x03
#define AH_MSG_TELEMETRY 0x04
#define AH_MSG_RAW_SCS 0x05
#define AH_MSG_HEARTBEAT 0x06
#define AH_MSG_CONFIG 0x07
#define AH_MSG_ERROR 0x08
#define AH_MSG_DIRECT_TARGETS 0x09
#define AH_MSG_SET_TORQUE 0x0A

/* commands */
#define AH_CMD_OPEN 1
#define AH_CMD_GRASP 2
#define AH_CMD_HOLD 3
#define AH_CMD_RELEASE 4
#define AH_CMD_STOP 5
#define AH_CMD_RESET 6

/* states */
#define AH_STATE_IDLE 0
#define AH_STATE_PREGRASP 1
#define AH_STATE_CLOSING 2
#define AH_STATE_HOLD 3
#define AH_STATE_RELEASING 4
#define AH_STATE_PROTECT 5

/* fault bits */
#define AH_FAULT_WATCHDOG (1u << 0)
#define AH_FAULT_STOP (1u << 1)
#define AH_FAULT_OFFLINE (1u << 2)
#define AH_FAULT_SEQ (1u << 3)
#define AH_FAULT_OVERLOAD (1u << 4)
#define AH_FAULT_OVERTEMP (1u << 5)
#define AH_FAULT_CRC (1u << 6)

/* telemetry flags */
#define AH_FLAG_TORQUE (1u << 0)
#define AH_FLAG_MOVING (1u << 1)
#define AH_FLAG_TIMEOUT (1u << 2)
#define AH_FLAG_GRASPED (1u << 3)

#define AH_COMMAND_PAYLOAD 61
#define AH_TELEMETRY_PAYLOAD 63
#define AH_CONFIG_PAYLOAD 12
#define AH_INFO_PAYLOAD 8
#define AH_DIRECT_PAYLOAD 18

typedef struct {
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[AH_MAX_PAYLOAD];
} ah_frame_t;

typedef struct {
    uint8_t buf[AH_PARSER_BUF];
    size_t len;
} ah_parser_t;

typedef struct {
    uint8_t proto;
    uint8_t fw_major;
    uint8_t fw_minor;
    uint8_t n_servos;
    uint16_t telemetry_ms;
    uint16_t watchdog_ms;
} ah_info_t;

typedef struct {
    uint8_t cmd;
    uint16_t speed_raw;
    uint8_t torque_levels[8];
    uint16_t pregrasp_raw[8];
    uint16_t close_raw[8];
    uint16_t load_threshold_raw[8];
    uint16_t closing_timeout_ms;
} ah_command_t;

typedef struct {
    uint8_t state;
    uint8_t fault;
    uint8_t flags;
    uint8_t online_mask;
    uint8_t seq_ack;
    uint32_t tick_ms;
    uint16_t pos_raw[8];
    uint16_t load_raw[8];
    uint8_t temp_c[8];
    uint8_t error_bits[8];
    uint16_t command_count;
    uint16_t feedback_count;
    uint16_t max_age_ms;
} ah_telemetry_t;

typedef struct {
    uint16_t speed_raw;
    uint16_t target_raw[8];
} ah_direct_t;

typedef struct {
    uint16_t watchdog_ms;
    uint16_t telemetry_ms;
    uint16_t slot_ms;
    uint16_t pregrasp_ms;
    uint16_t closing_timeout_ms;
    uint16_t release_ms;
} ah_config_t;

uint16_t ah_crc16(const uint8_t *data, size_t len);
size_t ah_encode_frame(uint8_t type, uint8_t seq, const uint8_t *payload,
                       uint8_t len, uint8_t *out, size_t out_cap);
void ah_parser_init(ah_parser_t *parser);
size_t ah_parser_feed(ah_parser_t *parser, const uint8_t *data, size_t len,
                      ah_frame_t *out, size_t max_frames);

size_t ah_build_info(const ah_info_t *info, uint8_t *out, size_t cap);
size_t ah_build_telemetry(const ah_telemetry_t *t, uint8_t *out, size_t cap);
int ah_parse_telemetry(const uint8_t *payload, size_t len, ah_telemetry_t *out);
size_t ah_build_command(const ah_command_t *c, uint8_t *out, size_t cap);
int ah_parse_command(const uint8_t *payload, size_t len, ah_command_t *out);
size_t ah_build_direct(const ah_direct_t *d, uint8_t *out, size_t cap);
int ah_parse_direct(const uint8_t *payload, size_t len, ah_direct_t *out);
int ah_parse_config(const uint8_t *payload, size_t len, ah_config_t *out);
size_t ah_build_error(uint8_t code, uint8_t arg, uint8_t *out, size_t cap);

#ifdef __cplusplus
}
#endif

#endif /* AH_PROTOCOL_H */
