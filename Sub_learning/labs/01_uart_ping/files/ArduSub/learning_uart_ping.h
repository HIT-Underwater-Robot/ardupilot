#pragma once

#include <AP_HAL/AP_HAL.h>

// A deliberately small vehicle-level component used to teach UART ownership,
// initialisation and scheduler registration.  It does not know about motors.
class LearningUARTPing
{
public:
    void init();
    void update();

private:
    static constexpr uint8_t LINE_CAPACITY = 32;
    static constexpr uint8_t READ_BUDGET_PER_UPDATE = 32;

    void handle_complete_line();
    void write_text(const char *text);

    AP_HAL::UARTDriver *_uart = nullptr;
    char _line[LINE_CAPACITY] {};
    uint8_t _line_length = 0;
};
