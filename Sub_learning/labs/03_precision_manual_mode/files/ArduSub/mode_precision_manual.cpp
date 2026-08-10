#include "Sub.h"

namespace
{
// This lab keeps direct-thrust semantics but reduces the maximum request.
// Values are constants on purpose; a later parameter lab can make them
// configurable without obscuring the mode-registration exercise.
constexpr float ANGULAR_SCALE = 0.35f;
constexpr float TRANSLATION_SCALE = 0.40f;
}

bool ModePrecisionManual::init(bool ignore_checks)
{
    // Avoid carrying a previous mode's non-neutral RC-derived targets into the
    // first Precision Manual iteration.
    sub.set_neutral_controls();
    return true;
}

void ModePrecisionManual::run()
{
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE, true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    motors.set_roll(channel_roll->norm_input() * ANGULAR_SCALE);
    motors.set_pitch(channel_pitch->norm_input() * ANGULAR_SCALE);
    motors.set_yaw(channel_yaw->norm_input() * g.acro_yaw_p / ACRO_YAW_P * ANGULAR_SCALE);

    // AP_Motors uses 0.5 as neutral heave.  Scaling norm_input() first keeps
    // the neutral point unchanged while reducing both upward and downward
    // authority symmetrically.
    const float heave = channel_throttle->norm_input() * TRANSLATION_SCALE;
    motors.set_throttle(0.5f + 0.5f * heave);
    motors.set_forward(channel_forward->norm_input() * TRANSLATION_SCALE);
    motors.set_lateral(channel_lateral->norm_input() * TRANSLATION_SCALE);
}
