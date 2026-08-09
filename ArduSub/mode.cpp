#include "Sub.h"

Mode::Mode(void) :
    g(sub.g),
    ahrs(sub.ahrs),
    motors(sub.motors),
    channel_roll(sub.channel_roll),
    channel_pitch(sub.channel_pitch),
    channel_throttle(sub.channel_throttle),
    channel_yaw(sub.channel_yaw),
    channel_forward(sub.channel_forward),
    channel_lateral(sub.channel_lateral),
    attitude_control(&sub.attitude_control)
{ }

Mode *Sub::mode_from_mode_num(const Mode::Number mode)
{
    switch (mode) {
    case Mode::Number::MANUAL:
        return &mode_manual;
    case Mode::Number::STABILIZE:
        return &mode_stabilize;
    }

    return nullptr;
}

bool Sub::set_mode(Mode::Number mode, ModeReason reason)
{
    if (mode == control_mode) {
        control_mode_reason = reason;
        return true;
    }

    Mode *new_flightmode = mode_from_mode_num(mode);
    if (new_flightmode == nullptr) {
        notify_no_such_mode(static_cast<uint8_t>(mode));
        return false;
    }

    if (!new_flightmode->init(false)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Flight mode change failed %s", new_flightmode->name());
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    exit_mode(flightmode, new_flightmode);
    prev_control_mode = control_mode;
    flightmode = new_flightmode;
    control_mode = mode;
    control_mode_reason = reason;
#if HAL_LOGGING_ENABLED
    logger.Write_Mode(static_cast<uint8_t>(control_mode), reason);
#endif
    gcs().send_message(MSG_HEARTBEAT);
    notify_flight_mode();
    return true;
}

bool Sub::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    static_assert(sizeof(Mode::Number) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");
    return set_mode(static_cast<Mode::Number>(new_mode), reason);
}

void Sub::update_flight_mode()
{
    flightmode->run();
}

void Sub::exit_mode(Mode *&old_flightmode, Mode *&new_flightmode)
{
    motors.set_max_throttle(1.0f);
}

void Sub::notify_flight_mode()
{
    AP_Notify::flags.autopilot_mode = false;
    AP_Notify::flags.flight_mode = static_cast<uint8_t>(control_mode);
    notify.set_flight_mode_str(flightmode->name4());
}
