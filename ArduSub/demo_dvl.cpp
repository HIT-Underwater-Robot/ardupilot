#include "Sub.h"

#if AP_SUB_LEARNING_DEMOS_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include <cstdlib>
#include <cstring>

namespace {
bool parse_uint32(const char *text, uint32_t &value)
{
    char *end = nullptr;
    const unsigned long parsed = strtoul(text, &end, 10);
    if (end == text || *end != '\0' || parsed > UINT32_MAX) {
        return false;
    }
    value = static_cast<uint32_t>(parsed);
    return true;
}

bool parse_float(const char *text, float &value)
{
    char *end = nullptr;
    value = strtof(text, &end);
    return end != text && *end == '\0' && isfinite(value);
}
}

const AP_Param::GroupInfo DemoDVL::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Learning DVL enable
    // @Description: Enables the learning-only virtual DVL serial receiver. The data is logged and forwarded for observation but is not fused into navigation or used by control.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, DemoDVL, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: TIMEOUT
    // @DisplayName: Learning DVL timeout
    // @Description: Maximum age of the last valid virtual DVL frame before the receiver reports unhealthy
    // @Units: ms
    // @Range: 100 5000
    // @Increment: 50
    // @User: Advanced
    AP_GROUPINFO("TIMEOUT", 2, DemoDVL, _timeout_ms, 500),

    // @Param: MAX_VEL
    // @DisplayName: Learning DVL maximum velocity
    // @Description: Rejects virtual DVL body-frame velocity components with an absolute value above this limit
    // @Units: m/s
    // @Range: 0.1 20
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("MAX_VEL", 3, DemoDVL, _max_velocity_mps, 5.0f),

    // @Param: TX_HZ
    // @DisplayName: Learning DVL telemetry rate
    // @Description: Rate of serial status replies and MAVLink named-value forwarding. Zero disables both outputs while input and logging continue.
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TX_HZ", 4, DemoDVL, _tx_rate_hz, 2),

    AP_GROUPEND
};

DemoDVL::DemoDVL()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool DemoDVL::enabled() const
{
    return _enable.get() != 0;
}

void DemoDVL::init()
{
    if (!enabled()) {
        return;
    }

    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_LearningDVL, 0);
    if (_uart == nullptr) {
        return;
    }

    _uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_LearningDVL, 0));
}

bool DemoDVL::healthy() const
{
    if (!enabled() || _uart == nullptr || _state.last_update_ms == 0 || _state.quality == 0) {
        return false;
    }

    const uint32_t timeout_ms = constrain_int32(_timeout_ms.get(), 100, 5000);
    return AP_HAL::millis() - _state.last_update_ms <= timeout_ms;
}

void DemoDVL::update()
{
    if (!enabled() || _uart == nullptr) {
        return;
    }

    uint32_t bytes_available = MIN(_uart->available(), 256U);
    while (bytes_available-- > 0) {
        const int16_t value = _uart->read();
        if (value < 0) {
            break;
        }
        decode_byte(static_cast<char>(value));
    }

    send_status();
}

void DemoDVL::decode_byte(const char byte)
{
    if (byte == '$') {
        _line_length = 0;
        _line_overflow = false;
        _line_buffer[_line_length++] = byte;
        return;
    }

    if (_line_length == 0) {
        return;
    }

    if (byte == '\n') {
        if (!_line_overflow) {
            _line_buffer[_line_length] = '\0';
            if (!parse_line(_line_buffer)) {
                _state.bad_frames++;
            }
        } else {
            _state.bad_frames++;
        }
        _line_length = 0;
        _line_overflow = false;
        return;
    }

    if (byte == '\r') {
        return;
    }

    if (_line_length >= LINE_BUFFER_SIZE - 1) {
        _line_overflow = true;
        return;
    }

    _line_buffer[_line_length++] = byte;
}

bool DemoDVL::parse_line(char *line)
{
    char *checksum_separator = std::strchr(line, '*');
    if (line[0] != '$' || checksum_separator == nullptr || std::strlen(checksum_separator) != 3) {
        return false;
    }

    uint8_t checksum_high;
    uint8_t checksum_low;
    if (!hex_to_nibble(checksum_separator[1], checksum_high) ||
        !hex_to_nibble(checksum_separator[2], checksum_low)) {
        return false;
    }

    *checksum_separator = '\0';
    char *payload = &line[1];
    const uint8_t expected_checksum = (checksum_high << 4) | checksum_low;
    if (checksum(payload) != expected_checksum) {
        return false;
    }

    char *save = nullptr;
    const char *message_type = strtok_r(payload, ",", &save);
    const char *sequence_text = strtok_r(nullptr, ",", &save);
    const char *velocity_x_text = strtok_r(nullptr, ",", &save);
    const char *velocity_y_text = strtok_r(nullptr, ",", &save);
    const char *velocity_z_text = strtok_r(nullptr, ",", &save);
    const char *quality_text = strtok_r(nullptr, ",", &save);
    const char *extra_text = strtok_r(nullptr, ",", &save);
    if (message_type == nullptr || strcmp(message_type, "DVLD") != 0 ||
        sequence_text == nullptr || velocity_x_text == nullptr ||
        velocity_y_text == nullptr || velocity_z_text == nullptr ||
        quality_text == nullptr || extra_text != nullptr) {
        return false;
    }

    uint32_t sequence;
    uint32_t quality;
    float velocity_x;
    float velocity_y;
    float velocity_z;
    if (!parse_uint32(sequence_text, sequence) ||
        !parse_float(velocity_x_text, velocity_x) ||
        !parse_float(velocity_y_text, velocity_y) ||
        !parse_float(velocity_z_text, velocity_z) ||
        !parse_uint32(quality_text, quality) || quality > 100) {
        return false;
    }

    const float max_velocity_mps = constrain_float(_max_velocity_mps.get(), 0.1f, 20.0f);
    if (fabsf(velocity_x) > max_velocity_mps ||
        fabsf(velocity_y) > max_velocity_mps ||
        fabsf(velocity_z) > max_velocity_mps) {
        return false;
    }

    _state.velocity_body_mps = Vector3f(velocity_x, velocity_y, velocity_z);
    _state.sequence = sequence;
    _state.quality = static_cast<uint8_t>(quality);
    _state.last_update_ms = AP_HAL::millis();
    _state.good_frames++;

#if HAL_LOGGING_ENABLED
    AP::logger().Write("DDVL",
                       "TimeUS,Seq,VX,VY,VZ,Qual",
                       "s#nnn%",
                       "F-----",
                       "QIfffB",
                       AP_HAL::micros64(),
                       _state.sequence,
                       static_cast<double>(_state.velocity_body_mps.x),
                       static_cast<double>(_state.velocity_body_mps.y),
                       static_cast<double>(_state.velocity_body_mps.z),
                       _state.quality);
#endif

    return true;
}

bool DemoDVL::telemetry_due()
{
    const uint8_t tx_rate_hz = static_cast<uint8_t>(constrain_int16(_tx_rate_hz.get(), 0, 10));
    if (!enabled() || tx_rate_hz == 0) {
        return false;
    }

    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t interval_ms = 1000U / tx_rate_hz;
    if (now_ms - _last_telemetry_ms < interval_ms) {
        return false;
    }

    _last_telemetry_ms = now_ms;
    return true;
}

void DemoDVL::send_status()
{
    const uint8_t tx_rate_hz = static_cast<uint8_t>(constrain_int16(_tx_rate_hz.get(), 0, 10));
    if (tx_rate_hz == 0) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t interval_ms = 1000U / tx_rate_hz;
    if (now_ms - _last_status_ms < interval_ms) {
        return;
    }
    _last_status_ms = now_ms;

    char payload[64];
    const int payload_length = hal.util->snprintf(payload,
                               sizeof(payload),
                               "DVLA,%lu,%u,%lu,%lu",
                               static_cast<unsigned long>(_state.sequence),
                               healthy() ? 1U : 0U,
                               static_cast<unsigned long>(_state.good_frames),
                               static_cast<unsigned long>(_state.bad_frames));
    if (payload_length <= 0 || payload_length >= static_cast<int>(sizeof(payload))) {
        return;
    }

    char frame[72];
    const int frame_length = hal.util->snprintf(frame,
                             sizeof(frame),
                             "$%s*%02X\r\n",
                             payload,
                             checksum(payload));
    if (frame_length <= 0 || frame_length >= static_cast<int>(sizeof(frame)) ||
        _uart->txspace() < static_cast<uint32_t>(frame_length)) {
        return;
    }

    _uart->write(reinterpret_cast<const uint8_t *>(frame), frame_length);
}

uint8_t DemoDVL::checksum(const char *payload)
{
    uint8_t result = 0;
    while (*payload != '\0') {
        result ^= static_cast<uint8_t>(*payload++);
    }
    return result;
}

bool DemoDVL::hex_to_nibble(const char input, uint8_t &output)
{
    if (input >= '0' && input <= '9') {
        output = input - '0';
        return true;
    }
    if (input >= 'A' && input <= 'F') {
        output = input - 'A' + 10;
        return true;
    }
    if (input >= 'a' && input <= 'f') {
        output = input - 'a' + 10;
        return true;
    }
    return false;
}

#endif // AP_SUB_LEARNING_DEMOS_ENABLED
