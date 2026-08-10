#pragma once

#include <AP_HAL/AP_HAL.h>

// Educational UART frontend.  It accepts bounded six-axis command samples,
// but it never sends them to AP_Motors; a consumer must make that policy
// decision explicitly.
class AP_LearningSerial
{
public:
    struct Command {
        int16_t roll = 0;
        int16_t pitch = 0;
        int16_t yaw = 0;
        int16_t heave = 0;
        int16_t forward = 0;
        int16_t lateral = 0;
        uint32_t received_ms = 0;
        uint8_t sequence = 0;
        bool enabled = false;
    };

    void init();
    void update();

    bool get_command(Command &command) const;
    bool healthy(uint32_t timeout_ms) const;

    uint32_t good_frame_count() const { return _good_frames; }
    uint32_t bad_frame_count() const { return _bad_frames; }

private:
    static constexpr uint8_t HEADER_1 = 0xAA;
    static constexpr uint8_t HEADER_2 = 0x55;
    static constexpr uint8_t PROTOCOL_VERSION = 1;
    static constexpr uint8_t MESSAGE_COMMAND = 0x31;
    static constexpr uint8_t PAYLOAD_LENGTH = 13;
    static constexpr uint8_t FRAME_SIZE = 21;
    static constexpr uint8_t READ_BUDGET_PER_UPDATE = 64;

    void process_byte(uint8_t byte);
    void decode_complete_frame();
    void send_ack(uint8_t sequence);
    static uint16_t crc16_ccitt(const uint8_t *data, uint8_t length);
    static int16_t read_int16_le(const uint8_t *data);
    static bool axis_is_valid(int16_t value);

    AP_HAL::UARTDriver *_uart = nullptr;
    uint8_t _frame[FRAME_SIZE] {};
    uint8_t _frame_index = 0;
    Command _command {};
    bool _has_command = false;
    uint32_t _good_frames = 0;
    uint32_t _bad_frames = 0;
};
