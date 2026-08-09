#pragma once

#include "LogStructure.h"

class AP_DAL_Airspeed {
public:

    // Airspeed-like methods:

    // return health status of sensor
    bool healthy(uint8_t) const {
        return false;
    }
    // return health status of primary sensor
    bool healthy() const {
        return healthy(get_primary());
    }

    // return true if airspeed is enabled, and airspeed use is set
    bool use(uint8_t) const {
        return false;
    }
    bool use() const {
        return use(get_primary());
    }

    // return the current airspeed in m/s
    float get_airspeed(uint8_t) const {
        return 0.0f;
    }
    float get_airspeed() const {
        return get_airspeed(get_primary());
    }

    // return time in ms of last update
    uint32_t last_update_ms(uint8_t) const { return 0; }
    uint32_t last_update_ms() const { return last_update_ms(get_primary()); }

    // get number of sensors
    uint8_t get_num_sensors(void) const { return 0; }

    // get current primary sensor
    uint8_t get_primary(void) const { return 0; }

    // AP_DAL methods:
    AP_DAL_Airspeed() = default;

    void start_frame() {}

    void handle_message(const log_RASH &) {
    }
    void handle_message(const log_RASI &) {
    }
};
