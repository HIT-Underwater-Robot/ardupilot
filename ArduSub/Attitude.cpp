#include "Sub.h"

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
void Sub::get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max)
{
    // sanity check angle max parameter
    const float angle_max_cd = attitude_control.lean_angle_max_cd();

    // limit max lean angle
    angle_max = constrain_float(angle_max, 1000, angle_max_cd);

    // scale roll_in, pitch_in to ATC_ANGLE_MAX parameter range
    float scaler = angle_max_cd/(float)ROLL_PITCH_INPUT_MAX;
    roll_in *= scaler;
    pitch_in *= scaler;

    // do circular limit
    float total_in = norm(pitch_in, roll_in);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // do lateral tilt to euler roll conversion
    roll_in = (18000/M_PI) * atanf(cosf(pitch_in*(M_PI/18000))*tanf(roll_in*(M_PI/18000)));

    // return
    roll_out = roll_in;
    pitch_out = pitch_in;
}

// get_pilot_desired_heading - transform pilot's yaw input into a
// desired yaw rate
// returns desired yaw rate in centi-degrees per second
float Sub::get_pilot_desired_yaw_rate(int16_t stick_angle) const
{
    // convert pilot input to the desired yaw rate
    return stick_angle * g.acro_yaw_p;
}

// check for ekf yaw reset and adjust target heading
void Sub::check_ekf_yaw_reset()
{
    float yaw_angle_change_rad;
    uint32_t new_ekfYawReset_ms = ahrs.getLastYawResetAngle(yaw_angle_change_rad);
    if (new_ekfYawReset_ms != ekfYawReset_ms) {
        attitude_control.inertial_frame_reset();
        ekfYawReset_ms = new_ekfYawReset_ms;
    }
}
