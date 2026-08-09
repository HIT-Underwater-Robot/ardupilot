#include "Sub.h"

// enable_motor_output() - enable and output lowest possible value to motors
void Sub::enable_motor_output()
{
    motors.output_min();
}
// motors_output - send output to motors library which will adjust and send to ESCs and servos
void Sub::motors_output()
{
    // Sub has no pre-takeoff RPM gate that asserts the spool-up block,
    // so clear it every cycle before motors.output() runs output_logic().
    // Without this the block set during ground-idle ramp would never clear
    // and the spool would stay in GROUND_IDLE.
    motors.set_spoolup_block(false);
    
    // check if we are performing the motor test
    if (ap.motor_test) {
        verify_motor_test();
    } else {
        motors.set_interlock(true);
        auto &srv = AP::srv();
        srv.cork();
        SRV_Channels::calc_pwm();
        SRV_Channels::output_ch_all();
        motors.output();
        srv.push();
    }
}

// Initialize new style motor test
// Perform checks to see if it is ok to begin the motor test
// Returns true if motor test has begun
bool Sub::init_motor_test()
{
    uint32_t tnow = AP_HAL::millis();

    // Ten second cooldown period required with no do_set_motor requests required
    // after failure.
    if (tnow < last_do_motor_test_fail_ms + 10000 && last_do_motor_test_fail_ms > 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "10 second cooldown required after motor test");
        return false;
    }

    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Disarm hardware safety switch before testing motors.");
        return false;
    }

    // Make sure we are on the ground
    if (!motors.armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Arm motors before testing motors.");
        return false;
    }

    enable_motor_output(); // set all motor outputs to zero
    ap.motor_test = true;

    return true;
}

// Verify new style motor test
// The motor test will fail if the interval between received
// MAV_CMD_DO_SET_MOTOR requests exceeds a timeout period
// Returns true if it is ok to proceed with new style motor test
bool Sub::verify_motor_test()
{
    bool pass = true;

    // Require at least 2 Hz incoming do_set_motor requests
    if (AP_HAL::millis() > last_do_motor_test_ms + 500) {
        gcs().send_text(MAV_SEVERITY_INFO, "Motor test timed out!");
        pass = false;
    }

    if (!pass) {
        ap.motor_test = false;
        AP::arming().disarm(AP_Arming::Method::MOTORTEST);
        last_do_motor_test_fail_ms = AP_HAL::millis();
        return false;
    }

    return true;
}

bool Sub::handle_do_motor_test(mavlink_command_int_t command) {
    last_do_motor_test_ms = AP_HAL::millis();

    // If we are not already testing motors, initialize test
    static uint32_t tLastInitializationFailed = 0;
    if(!ap.motor_test) {
        // Do not allow initializations attempt under 2 seconds
        // If one fails, we need to give the user time to fix the issue
        // instead of spamming error messages
        if (AP_HAL::millis() > (tLastInitializationFailed + 2000)) {
            if (!init_motor_test()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "motor test initialization failed!");
                tLastInitializationFailed = AP_HAL::millis();
                return false; // init fail
            }
        } else {
            return false;
        }
    }

    float motor_number = command.param1;
    float throttle_type = command.param2;
    float throttle = command.param3;
    // float timeout_s = command.param4; // not used
    // float motor_count = command.param5; // not used
    const uint32_t test_type = command.y;

    if (test_type != MOTOR_TEST_ORDER_BOARD) {
        gcs().send_text(MAV_SEVERITY_WARNING, "bad test type %0.2f", (double)test_type);
        return false; // test type not supported here
    }

    if (is_equal(throttle_type, (float)MOTOR_TEST_THROTTLE_PILOT)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "bad throttle type %0.2f", (double)throttle_type);

        return false; // throttle type not supported here
    }

    if (is_equal(throttle_type, (float)MOTOR_TEST_THROTTLE_PWM)) {
        return motors.output_test_num(motor_number, throttle); // true if motor output is set
    }

    if (is_equal(throttle_type, (float)MOTOR_TEST_THROTTLE_PERCENT)) {
        throttle = constrain_float(throttle, 0.0f, 100.0f);
        throttle = channel_throttle->get_radio_min() + throttle * 0.01f * (channel_throttle->get_radio_max() - channel_throttle->get_radio_min());
        return motors.output_test_num(motor_number, throttle); // true if motor output is set
    }

    return false;
}
