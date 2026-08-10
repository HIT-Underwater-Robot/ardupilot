#include "learning_uart_ping.h"

#include <AP_SerialManager/AP_SerialManager.h>

#include <cstring>

void LearningUARTPing::init()
{
    // In this teaching branch Lua is disabled, so protocol 28 is used only as
    // a convenient SerialManager label for a lab-owned UART.  SERIAL2_PROTOCOL
    // must be set to 28 before rebooting the board.
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

    // SerialManager has already opened the port.  Calling begin() again here
    // documents the buffers required by this component and is safe for the
    // ChibiOS UART backend.
    _uart->begin(baud, 128, 64);
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->discard_input();
    write_text("UART-PING READY\n");
}

void LearningUARTPing::update()
{
    if (_uart == nullptr) {
        return;
    }

    // The byte budget prevents a noisy UART from monopolising the scheduler.
    uint8_t budget = READ_BUDGET_PER_UPDATE;
    while (budget-- > 0 && _uart->available() > 0) {
        const int16_t value = _uart->read();
        if (value < 0) {
            break;
        }

        const char ch = static_cast<char>(value);
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            _line[_line_length] = '\0';
            handle_complete_line();
            _line_length = 0;
            continue;
        }

        if (_line_length >= LINE_CAPACITY - 1) {
            _line_length = 0;
            write_text("OVF\n");
            continue;
        }
        _line[_line_length++] = ch;
    }
}

void LearningUARTPing::handle_complete_line()
{
    if (strcmp(_line, "PING") == 0) {
        write_text("PONG\n");
    } else if (_line_length > 0) {
        write_text("ERR\n");
    }
}

void LearningUARTPing::write_text(const char *text)
{
    if (_uart != nullptr) {
        _uart->write(text);
    }
}
