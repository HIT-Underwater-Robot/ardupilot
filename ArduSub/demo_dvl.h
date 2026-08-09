#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

#if AP_SUB_LEARNING_DEMOS_ENABLED

class DemoDVL
{
public:
    struct State {
        Vector3f velocity_body_mps;
        uint32_t sequence;
        uint32_t last_update_ms;
        uint32_t good_frames;
        uint32_t bad_frames;
        uint8_t quality;
    };

    DemoDVL();

    CLASS_NO_COPY(DemoDVL);

    void init();
    void update();

    bool enabled() const;
    bool healthy() const;
    bool telemetry_due();
    const State &state() const
    {
        return _state;
    }

    static const struct AP_Param::GroupInfo var_info[];

private:
    static constexpr uint8_t LINE_BUFFER_SIZE = 96;

    void decode_byte(char byte);
    bool parse_line(char *line);
    void send_status();

    static uint8_t checksum(const char *payload);
    static bool hex_to_nibble(char input, uint8_t &output);

    AP_Int8 _enable;
    AP_Int16 _timeout_ms;
    AP_Float _max_velocity_mps;
    AP_Int8 _tx_rate_hz;

    AP_HAL::UARTDriver *_uart;
    State _state;

    char _line_buffer[LINE_BUFFER_SIZE];
    uint8_t _line_length;
    bool _line_overflow;
    uint32_t _last_status_ms;
    uint32_t _last_telemetry_ms;
};

#endif // AP_SUB_LEARNING_DEMOS_ENABLED
