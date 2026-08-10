#include "Sub.h"

namespace
{
constexpr uint32_t COMMAND_TIMEOUT_MS = 250;
constexpr float MAX_OUTPUT_SCALE = 0.25f;
}

bool ModeSerialManual::init(bool ignore_checks)
{
    AP_LearningSerial::Command command;

    // Entry requires a live link sending disabled/neutral frames.  This avoids
    // changing mode while an old enabled packet is already requesting motion.
    if (!sub.learning_serial.get_command(command) ||
        !sub.learning_serial.healthy(COMMAND_TIMEOUT_MS) ||
        command.enabled) {
        return false;
    }

    _link_lost = false;
    sub.set_neutral_controls();
    return true;
}

void ModeSerialManual::run()
{
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE, true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        return;
    }

    AP_LearningSerial::Command command;
    if (!sub.learning_serial.get_command(command) ||
        !sub.learning_serial.healthy(COMMAND_TIMEOUT_MS)) {
        // This latch deliberately does not clear when packets resume.  The
        // operator must leave and re-enter the mode after investigating the
        // link loss, so a reconnect cannot restart thrusters automatically.
        _link_lost = true;
    }

    if (_link_lost || !command.enabled) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE, true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    const float unit_scale = MAX_OUTPUT_SCALE / 1000.0f;
    motors.set_roll(command.roll * unit_scale);
    motors.set_pitch(command.pitch * unit_scale);
    motors.set_yaw(command.yaw * unit_scale);
    motors.set_throttle(0.5f + 0.5f * command.heave * unit_scale);
    motors.set_forward(command.forward * unit_scale);
    motors.set_lateral(command.lateral * unit_scale);
}
