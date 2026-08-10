#include "AP_LearningSerial.h"

#include <AP_SerialManager/AP_SerialManager.h>

void AP_LearningSerial::init()
{
    auto &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Scripting, 0);
    if (_uart == nullptr) {
        return;
    }

    const uint32_t baud = serial_manager.find_baudrate(
        AP_SerialManager::SerialProtocol_Scripting, 0);
    if (baud == 0) {
        _uart = nullptr;
        return;
    }

    _uart->begin(baud, 256, 64);
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->discard_input();
}

void AP_LearningSerial::update()
{
    if (_uart == nullptr) {
        return;
    }

    uint8_t budget = READ_BUDGET_PER_UPDATE;
    while (budget-- > 0 && _uart->available() > 0) {
        const int16_t value = _uart->read();
        if (value < 0) {
            break;
        }
        process_byte(static_cast<uint8_t>(value));
    }
}

bool AP_LearningSerial::get_command(Command &command) const
{
    if (!_has_command) {
        return false;
    }
    command = _command;
    return true;
}

bool AP_LearningSerial::healthy(const uint32_t timeout_ms) const
{
    return _has_command && (AP_HAL::millis() - _command.received_ms <= timeout_ms);
}

void AP_LearningSerial::process_byte(const uint8_t byte)
{
    if (_frame_index == 0) {
        if (byte == HEADER_1) {
            _frame[_frame_index++] = byte;
        }
        return;
    }

    if (_frame_index == 1) {
        if (byte == HEADER_2) {
            _frame[_frame_index++] = byte;
        } else if (byte != HEADER_1) {
            _frame_index = 0;
        }
        return;
    }

    _frame[_frame_index++] = byte;
    if (_frame_index == FRAME_SIZE) {
        decode_complete_frame();
        _frame_index = 0;
    }
}

void AP_LearningSerial::decode_complete_frame()
{
    if (_frame[2] != PROTOCOL_VERSION ||
        _frame[3] != MESSAGE_COMMAND ||
        _frame[5] != PAYLOAD_LENGTH) {
        _bad_frames++;
        return;
    }

    // CRC covers VERSION through the last payload byte.  Header and the two
    // CRC bytes themselves are not included.
    const uint16_t received_crc = static_cast<uint16_t>(_frame[19]) |
                                  (static_cast<uint16_t>(_frame[20]) << 8);
    const uint16_t calculated_crc = crc16_ccitt(&_frame[2], 17);
    if (received_crc != calculated_crc) {
        _bad_frames++;
        return;
    }

    Command decoded;
    decoded.enabled = (_frame[6] & 0x01U) != 0;
    decoded.roll = read_int16_le(&_frame[7]);
    decoded.pitch = read_int16_le(&_frame[9]);
    decoded.yaw = read_int16_le(&_frame[11]);
    decoded.heave = read_int16_le(&_frame[13]);
    decoded.forward = read_int16_le(&_frame[15]);
    decoded.lateral = read_int16_le(&_frame[17]);

    if (!axis_is_valid(decoded.roll) ||
        !axis_is_valid(decoded.pitch) ||
        !axis_is_valid(decoded.yaw) ||
        !axis_is_valid(decoded.heave) ||
        !axis_is_valid(decoded.forward) ||
        !axis_is_valid(decoded.lateral)) {
        _bad_frames++;
        return;
    }

    decoded.sequence = _frame[4];
    decoded.received_ms = AP_HAL::millis();
    _command = decoded;
    _has_command = true;
    _good_frames++;
    send_ack(decoded.sequence);
}

void AP_LearningSerial::send_ack(const uint8_t sequence)
{
    const uint8_t ack[] = { 0xAC, sequence, 0x00 };
    _uart->write(ack, sizeof(ack));
}

uint16_t AP_LearningSerial::crc16_ccitt(const uint8_t *data, const uint8_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000U) != 0 ?
                  static_cast<uint16_t>((crc << 1) ^ 0x1021U) :
                  static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

int16_t AP_LearningSerial::read_int16_le(const uint8_t *data)
{
    const uint16_t raw = static_cast<uint16_t>(data[0]) |
                         (static_cast<uint16_t>(data[1]) << 8);
    return static_cast<int16_t>(raw);
}

bool AP_LearningSerial::axis_is_valid(const int16_t value)
{
    return value >= -1000 && value <= 1000;
}
