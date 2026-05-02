#include "AP_Beacon_RTLSLink.h"

#if AP_BEACON_RTLSLINK_ENABLED

#include <AP_HAL/AP_HAL.h>

bool AP_Beacon_RTLSLink::healthy()
{
    return config_accepted && ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

void AP_Beacon_RTLSLink::update()
{
    if (uart == nullptr) {
        return;
    }

    uint32_t nbytes = MIN(uart->available(), 1024U);
    while (nbytes-- > 0) {
        const int16_t c = uart->read();
        if (c >= 0) {
            parse_byte((uint8_t)c);
        }
    }
}

void AP_Beacon_RTLSLink::reset_parser()
{
    parse_state = ParseState::MAGIC_1;
    msg_id = 0;
    payload_len = 0;
    seq = 0;
    payload_idx = 0;
    crc = 0xffff;
    frame_crc = 0;
}

void AP_Beacon_RTLSLink::parse_byte(uint8_t b)
{
    switch (parse_state) {
    case ParseState::MAGIC_1:
        if (b == FRAME_MAGIC_1) {
            crc = crc16_update(0xffff, b);
            parse_state = ParseState::MAGIC_2;
        }
        break;
    case ParseState::MAGIC_2:
        if (b != FRAME_MAGIC_2) {
            reset_parser();
            if (b == FRAME_MAGIC_1) {
                crc = crc16_update(0xffff, b);
                parse_state = ParseState::MAGIC_2;
            }
            break;
        }
        crc = crc16_update(crc, b);
        parse_state = ParseState::MSG_ID;
        break;
    case ParseState::MSG_ID:
        msg_id = b;
        crc = crc16_update(crc, b);
        parse_state = ParseState::LEN;
        break;
    case ParseState::LEN:
        payload_len = b;
        if (payload_len > PAYLOAD_LEN_MAX) {
            send_ack((MsgId)msg_id, AckStatus::BAD_FRAME);
            reset_parser();
            break;
        }
        crc = crc16_update(crc, b);
        parse_state = ParseState::SEQ;
        break;
    case ParseState::SEQ:
        seq = b;
        crc = crc16_update(crc, b);
        payload_idx = 0;
        parse_state = payload_len == 0 ? ParseState::CRC_LOW : ParseState::PAYLOAD;
        break;
    case ParseState::PAYLOAD:
        payload[payload_idx++] = b;
        crc = crc16_update(crc, b);
        if (payload_idx >= payload_len) {
            parse_state = ParseState::CRC_LOW;
        }
        break;
    case ParseState::CRC_LOW:
        frame_crc = b;
        parse_state = ParseState::CRC_HIGH;
        break;
    case ParseState::CRC_HIGH:
        frame_crc |= (uint16_t)b << 8;
        if (frame_crc == crc) {
            handle_frame();
        }
        reset_parser();
        break;
    }
}

void AP_Beacon_RTLSLink::handle_frame()
{
    switch ((MsgId)msg_id) {
    case MsgId::HELLO:
        handle_hello();
        break;
    case MsgId::ANCHOR:
        handle_anchor();
        break;
    case MsgId::POSITION:
        handle_position();
        break;
    case MsgId::TDOA:
        handle_tdoa();
        break;
    case MsgId::CONFIG_END:
        handle_config_end();
        break;
    case MsgId::ACK:
        break;
    }
}

void AP_Beacon_RTLSLink::handle_hello()
{
    if (payload_len < 3) {
        send_ack(MsgId::HELLO, AckStatus::BAD_FRAME);
        return;
    }
    if (payload[0] != PROTOCOL_VERSION) {
        send_ack(MsgId::HELLO, AckStatus::UNSUPPORTED_VERSION);
        return;
    }

    expected_anchor_count = MIN(payload[2], (uint8_t)AP_BEACON_MAX_BEACONS);
    configured_anchor_mask = 0;
    config_accepted = false;
    send_ack(MsgId::HELLO, AckStatus::OK);
}

void AP_Beacon_RTLSLink::handle_anchor()
{
    if (payload_len != 13) {
        return;
    }

    const uint8_t anchor_id = payload[0];
    if (anchor_id >= AP_BEACON_MAX_BEACONS ||
        (expected_anchor_count != 0 && anchor_id >= expected_anchor_count)) {
        return;
    }

    const Vector3f pos {
        read_i32_le(&payload[1]) * 0.001f,
        read_i32_le(&payload[5]) * 0.001f,
        read_i32_le(&payload[9]) * 0.001f
    };
    set_beacon_position(anchor_id, pos);
    configured_anchor_mask |= (1U << anchor_id);
}

void AP_Beacon_RTLSLink::handle_position()
{
    if (payload_len != 14 || !config_accepted) {
        return;
    }

    const Vector3f pos {
        read_i32_le(&payload[0]) * 0.001f,
        read_i32_le(&payload[4]) * 0.001f,
        read_i32_le(&payload[8]) * 0.001f
    };
    const float err_m = MAX(read_u16_le(&payload[12]) * 0.001f, 0.1f);
    set_vehicle_position(pos, err_m);
    last_update_ms = AP_HAL::millis();
}

void AP_Beacon_RTLSLink::handle_tdoa()
{
    if (payload_len != 8 || !config_accepted) {
        return;
    }

    const uint8_t anchor_id_a = payload[0];
    const uint8_t anchor_id_b = payload[1];
    if (anchor_id_a >= expected_anchor_count || anchor_id_b >= expected_anchor_count) {
        return;
    }

    const float distance_diff_m = read_i32_le(&payload[2]) * 0.001f;
    const float sigma_m = read_u16_le(&payload[6]) * 0.001f;
    set_tdoa_measurement(anchor_id_a, anchor_id_b, distance_diff_m, sigma_m);
    last_update_ms = AP_HAL::millis();
}

void AP_Beacon_RTLSLink::handle_config_end()
{
    if (payload_len < 1) {
        send_ack(MsgId::CONFIG_END, AckStatus::BAD_FRAME);
        return;
    }

    const uint8_t anchor_count = MIN(payload[0], (uint8_t)AP_BEACON_MAX_BEACONS);
    if (anchor_count == 0) {
        send_ack(MsgId::CONFIG_END, AckStatus::BAD_CONFIG);
        return;
    }

    const uint8_t required_mask = anchor_count >= 8 ? 0xff : ((1U << anchor_count) - 1U);
    if ((configured_anchor_mask & required_mask) != required_mask) {
        send_ack(MsgId::CONFIG_END, AckStatus::BAD_CONFIG);
        return;
    }

    expected_anchor_count = anchor_count;
    config_accepted = true;
    last_update_ms = AP_HAL::millis();
    send_ack(MsgId::CONFIG_END, AckStatus::OK);
}

void AP_Beacon_RTLSLink::send_ack(MsgId acked_msg_id, AckStatus status)
{
    if (uart == nullptr) {
        return;
    }

    uint8_t frame[12] {};
    uint8_t len = 0;
    frame[len++] = FRAME_MAGIC_1;
    frame[len++] = FRAME_MAGIC_2;
    frame[len++] = (uint8_t)MsgId::ACK;
    frame[len++] = 3;
    frame[len++] = seq;
    frame[len++] = (uint8_t)acked_msg_id;
    frame[len++] = (uint8_t)status;
    frame[len++] = PROTOCOL_VERSION;

    uint16_t out_crc = 0xffff;
    for (uint8_t i = 0; i < len; i++) {
        out_crc = crc16_update(out_crc, frame[i]);
    }
    write_u16_le(&frame[len], out_crc);
    len += 2;
    uart->write(frame, len);
}

uint16_t AP_Beacon_RTLSLink::crc16_update(uint16_t crc, uint8_t b)
{
    crc ^= (uint16_t)b << 8;
    for (uint8_t i = 0; i < 8; i++) {
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

int32_t AP_Beacon_RTLSLink::read_i32_le(const uint8_t *p)
{
    const uint32_t v = (uint32_t)p[0] |
                       ((uint32_t)p[1] << 8) |
                       ((uint32_t)p[2] << 16) |
                       ((uint32_t)p[3] << 24);
    return (int32_t)v;
}

uint16_t AP_Beacon_RTLSLink::read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

void AP_Beacon_RTLSLink::write_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = v & 0xff;
    p[1] = v >> 8;
}

#endif // AP_BEACON_RTLSLINK_ENABLED
