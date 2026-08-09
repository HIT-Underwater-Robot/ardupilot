#include "Sub.h"

// return barometric altitude in centimeters
void Sub::read_barometer()
{
    barometer.update();
    // If we are reading a positive altitude, the sensor needs calibration
    // Even a few meters above the water we should have no significant depth reading
    if(barometer.get_altitude() > 0) {
        barometer.update_calibration();
    }

    if (ap.depth_sensor_present) {
        sensor_health.depth = barometer.healthy(depth_sensor_idx);
    }
}
