#pragma once

#include "AP_Beacon_Backend.h"

#if AP_BEACON_RTLSLINK_ENABLED

class AP_Beacon_RTLSLink : public AP_Beacon_Backend
{
public:
    using AP_Beacon_Backend::AP_Beacon_Backend;

    bool healthy() override;
    void update() override;

private:
    static constexpr uint8_t FRAME_MAGIC_1 = 0x52; // 'R'
    static constexpr uint8_t FRAME_MAGIC_2 = 0x42; // 'B'
    static constexpr uint8_t PROTOCOL_VERSION = 1;
    static constexpr uint8_t PAYLOAD_LEN_MAX = 32;

    enum class MsgId : uint8_t {
        HELLO = 1,
        ANCHOR = 2,
        POSITION = 3,
        TDOA = 4,
        CONFIG_END = 5,
        ACK = 0x80,
    };

    enum class AckStatus : uint8_t {
        OK = 0,
        UNSUPPORTED_VERSION = 1,
        BAD_CONFIG = 2,
        BAD_FRAME = 3,
    };

    enum class ParseState : uint8_t {
        MAGIC_1,
        MAGIC_2,
        MSG_ID,
        LEN,
        SEQ,
        PAYLOAD,
        CRC_LOW,
        CRC_HIGH,
    } parse_state = ParseState::MAGIC_1;

    void reset_parser();
    void parse_byte(uint8_t b);
    void handle_frame();
    void handle_hello();
    void handle_anchor();
    void handle_position();
    void handle_tdoa();
    void handle_config_end();
    void send_ack(MsgId msg_id, AckStatus status);

    static uint16_t crc16_update(uint16_t crc, uint8_t b);
    static int32_t read_i32_le(const uint8_t *p);
    static uint16_t read_u16_le(const uint8_t *p);
    static void write_u16_le(uint8_t *p, uint16_t v);

    uint8_t msg_id = 0;
    uint8_t payload_len = 0;
    uint8_t seq = 0;
    uint8_t payload[PAYLOAD_LEN_MAX] {};
    uint8_t payload_idx = 0;
    uint16_t crc = 0xffff;
    uint16_t frame_crc = 0;

    uint8_t expected_anchor_count = 0;
    uint8_t configured_anchor_mask = 0;
    bool config_accepted = false;
    uint32_t last_update_ms = 0;
};

#endif // AP_BEACON_RTLSLINK_ENABLED
