#include "Sub.h"

#if AP_SUB_LEARNING_DEMOS_ENABLED

namespace
{
constexpr float PRECISION_ANGULAR_SCALE = 0.35f;
constexpr float PRECISION_TRANSLATION_SCALE = 0.40f;
}

bool ModePrecisionManual::init(bool ignore_checks)
{
    position_control->set_pos_desired_U_cm(0);
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

    motors.set_roll(channel_roll->norm_input() * PRECISION_ANGULAR_SCALE);
    motors.set_pitch(channel_pitch->norm_input() * PRECISION_ANGULAR_SCALE);
    motors.set_yaw(channel_yaw->norm_input() * g.acro_yaw_p / ACRO_YAW_P * PRECISION_ANGULAR_SCALE);

    const float heave = channel_throttle->norm_input() * PRECISION_TRANSLATION_SCALE;
    motors.set_throttle(0.5f + 0.5f * heave);
    motors.set_forward(channel_forward->norm_input() * PRECISION_TRANSLATION_SCALE);
    motors.set_lateral(channel_lateral->norm_input() * PRECISION_TRANSLATION_SCALE);
}

#endif // AP_SUB_LEARNING_DEMOS_ENABLED
