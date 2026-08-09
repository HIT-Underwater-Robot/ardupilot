#include "GCS_Sub.h"

#include "Sub.h"

MAV_TYPE GCS_Sub::frame_type() const
{
    return MAV_TYPE_SUBMARINE;
}

uint32_t GCS_Sub::custom_mode() const
{
    return (uint32_t)sub.control_mode;
}

bool GCS_Sub::vehicle_initialised() const
{
    return sub.ap.initialised;
}

void GCS_Sub::update_vehicle_sensor_status_flags()
{
    // mode-specific sensors:
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    if (sub.control_mode == Mode::Number::STABILIZE) {
        control_sensors_enabled |=
            MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
            MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
            MAV_SYS_STATUS_SENSOR_YAW_POSITION;
        control_sensors_health |=
            MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
            MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
            MAV_SYS_STATUS_SENSOR_YAW_POSITION;
    }

    // override the parent class's values for ABSOLUTE_PRESSURE to
    // only honour water-pressure sensors
    control_sensors_present &= ~MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    control_sensors_enabled &= ~MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    if (sub.ap.depth_sensor_present) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        if (sub.sensor_health.depth) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        }
    }

}

#if AP_LTM_TELEM_ENABLED
// avoid building/linking LTM:
void AP_LTM_Telem::init() {};
#endif
#if AP_DEVO_TELEM_ENABLED
// avoid building/linking Devo:
void AP_DEVO_Telem::init() {};
#endif
