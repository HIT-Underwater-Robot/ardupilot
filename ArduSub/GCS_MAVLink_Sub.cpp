#include "Sub.h"

#include "GCS_MAVLink_Sub.h"

uint8_t GCS_MAVLINK_Sub::base_mode() const
{
    uint8_t base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    if (sub.control_mode == Mode::Number::STABILIZE) {
        base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }
    if (sub.motors.armed()) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }
    return base_mode;
}

MAV_STATE GCS_MAVLINK_Sub::vehicle_system_status() const
{
    if (sub.any_failsafe_triggered()) {
        return MAV_STATE_CRITICAL;
    }
    if (sub.motors.armed()) {
        return MAV_STATE_ACTIVE;
    }
    if (!sub.ap.initialised) {
        return MAV_STATE_BOOT;
    }
    return MAV_STATE_STANDBY;
}

void GCS_MAVLINK_Sub::send_banner()
{
    GCS_MAVLINK::send_banner();
    send_text(MAV_SEVERITY_INFO, "Minimal frame: %s", sub.motors.get_frame_string());
}

void GCS_MAVLINK_Sub::send_nav_controller_output() const
{
    const Vector3f &targets = sub.attitude_control.get_att_target_euler_cd();
    mavlink_msg_nav_controller_output_send(
        chan,
        targets.x * 1.0e-2f,
        targets.y * 1.0e-2f,
        targets.z * 1.0e-2f,
        0,
        0,
        0,
        0,
        0);
}

void GCS_MAVLINK_Sub::send_pid_tuning()
{
}

int16_t GCS_MAVLINK_Sub::vfr_hud_throttle() const
{
    return static_cast<int16_t>(sub.motors.get_throttle() * 100.0f);
}

void GCS_MAVLINK_Sub::handle_manual_control_axes(const mavlink_manual_control_t &packet, uint32_t tnow)
{
    sub.transform_manual_control_to_rc_override(
        packet.x,
        packet.y,
        packet.z,
        packet.r,
        packet.buttons,
        packet.buttons2,
        packet.enabled_extensions,
        packet.s,
        packet.t,
        packet.aux1,
        packet.aux2,
        packet.aux3,
        packet.aux4,
        packet.aux5,
        packet.aux6);
    sub.failsafe.last_pilot_input_ms = tnow;
}

void GCS_MAVLINK_Sub::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
        if (gcs().sysid_is_gcs(msg.sysid)) {
            sub.failsafe.last_pilot_input_ms = AP_HAL::millis();
            handle_rc_channels_override(msg);
        }
        break;

    case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t packet;
        mavlink_msg_sys_status_decode(&msg, &packet);
        if ((msg.sysid == gcs().sysid_this_mav()) &&
            (packet.onboard_control_sensors_present & MAV_SYS_STATUS_EXTENSION_USED) &&
            (packet.onboard_control_sensors_enabled_extended & MAV_SYS_STATUS_SENSOR_LEAK) &&
            !(packet.onboard_control_sensors_health_extended & MAV_SYS_STATUS_SENSOR_LEAK)) {
            sub.leak_detector.set_detect();
        }
        break;
    }

    default:
        GCS_MAVLINK::handle_message(msg);
        break;
    }
}

uint8_t GCS_MAVLINK_Sub::send_available_mode(uint8_t index) const
{
    const Mode *modes[] = {
        &sub.mode_manual,
        &sub.mode_stabilize,
    };
    const uint8_t mode_count = ARRAY_SIZE(modes);
    const uint8_t index_zero = index - 1;
    if (index_zero >= mode_count) {
        return mode_count;
    }

    const Mode *mode = modes[index_zero];
    mavlink_msg_available_modes_send(
        chan,
        mode_count,
        index,
        MAV_STANDARD_MODE::MAV_STANDARD_MODE_NON_STANDARD,
        0,
        static_cast<uint32_t>(mode->number()),
        mode->name());
    return mode_count;
}
