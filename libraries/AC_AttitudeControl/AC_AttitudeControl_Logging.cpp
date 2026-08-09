#include <AP_Logger/AP_Logger_config.h>

#if HAL_LOGGING_ENABLED

#include "AC_AttitudeControl.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include "LogStructure.h"

// Write an ANG packet
void AC_AttitudeControl::Write_ANG() const
{
    Vector3f targets = get_att_target_euler_rad() * RAD_TO_DEG;

    const struct log_ANG pkt{
        LOG_PACKET_HEADER_INIT(LOG_ANG_MSG),
        time_us         : AP::scheduler().get_loop_start_time_us(),
        control_roll    : targets.x,
        roll            : degrees(_ahrs.roll),
        control_pitch   : targets.y,
        pitch           : degrees(_ahrs.pitch),
        control_yaw     : wrap_360(targets.z),
        yaw             : wrap_360(degrees(_ahrs.yaw)),
        sensor_dt       : AP::scheduler().get_last_loop_time_s()
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

#endif // HAL_LOGGING_ENABLED
