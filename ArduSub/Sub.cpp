/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Sub.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// 构造唯一的 Sub 车辆对象，并把控制器、导航器和推进器按依赖顺序连接起来。
Sub::Sub()
    :

#if !AP_SUB_RC_ENABLED
          control_mode(Mode::Number::MANUAL),
#endif
          motors(MAIN_LOOP_RATE),
          auto_yaw_mode(AUTO_YAW_LOOK_AT_NEXT_WP),
          ahrs_view(ahrs, ROTATION_NONE),
          attitude_control(ahrs_view, motors),
          pos_control(ahrs_view, motors, attitude_control),
          wp_nav(ahrs_view, pos_control, attitude_control),
          loiter_nav(ahrs_view, pos_control, attitude_control),
          circle_nav(ahrs_view, pos_control),
          param_loader(var_info),
          flightmode(&mode_manual),
          auto_mode(Auto_WP),
          guided_mode(Guided_WP)
{
    failsafe.pilot_input = true;
    if (_singleton != nullptr) {
        AP_HAL::panic("Can only be one Sub");
    }
    _singleton = this;
}

#define SCHED_TASK(func, rate_hz, max_time_micros, priority) SCHED_TASK_CLASS(Sub, &sub, func, rate_hz, max_time_micros, priority)
#define FAST_TASK(func) FAST_TASK_CLASS(Sub, &sub, func)

/*
  ArduSub 车辆任务表，所有车辆级周期任务都应从这里进入。

  本表会与 AP_Vehicle 的公共任务表合并，任务必须按优先级从高到低排列；
  priority 数字越小，优先级越高。

  FAST_TASK 每个主循环节拍都执行。SCHED_TASK/SCHED_TASK_CLASS 的参数为：

  - 要调用的函数，或“类、对象、成员函数”；
  - 目标执行频率 Hz；
  - 预计最大执行时间 us；
  - 任务优先级。

 */

const AP_Scheduler::Task Sub::scheduler_tasks[] = {
    // 首先取得本轮惯性数据。scheduler.loop() 已等待样本，这里完成数据更新。
    FAST_TASK_CLASS(AP_InertialSensor, &sub.ins, update),
    // 使用最新 IMU 数据运行底层角速度控制器。
    FAST_TASK(run_rate_controller),
    // 尽快把控制器结果送入推进器混控和硬件输出链。
    FAST_TASK(motors_output),
    // 更新 EKF/AHRS 状态估计；这是计算量较大的快速任务。
    FAST_TASK(read_AHRS),
    // 读取惯性导航位置和速度。
    FAST_TASK(read_inertia),
    // 检查 EKF 是否重置航向，并同步控制目标。
    FAST_TASK(check_ekf_yaw_reset),
    // 运行当前模式；模式把驾驶/导航目标转换为姿态、位置或推力目标。
    FAST_TASK(update_flight_mode),
    // 必要时使用 EKF 信息更新 Home。
    FAST_TASK(update_home_from_EKF),
    // 根据深度和运动状态判断是否到达水面或水底。
    FAST_TASK(update_surface_and_bottom_detector),
#if HAL_MOUNT_ENABLED
    // 云台需要快速更新的部分。
    FAST_TASK_CLASS(AP_Mount, &sub.camera_mount, update_fast),
#endif

    // 以下普通任务由“频率、预计耗时、优先级”决定何时运行。
    SCHED_TASK(fifty_hz_loop,         50,     75,   3),
#if AP_SUB_RC_ENABLED
    SCHED_TASK(rc_loop,              50,    130,  3),
#endif
    SCHED_TASK_CLASS(AP_GPS, &sub.gps, update, 50, 200,   6),
#if AP_OPTICALFLOW_ENABLED
    SCHED_TASK_CLASS(AP_OpticalFlow,          &sub.optflow,             update,         200, 160,   9),
#endif
    SCHED_TASK(update_batt_compass,   10,    120,  12),
    SCHED_TASK(read_rangefinder,      20,    100,  15),
    SCHED_TASK(update_altitude,       10,    100,  18),
#if AP_SUB_RC_ENABLED
    SCHED_TASK_CLASS(RC_Channels, (RC_Channels*)&sub.g2.rc_channels, read_aux_all, 10,  50,  18),
#endif
    SCHED_TASK(three_hz_loop,          3,     75,  21),
    SCHED_TASK(update_turn_counter,   10,     50,  24),
    SCHED_TASK(one_hz_loop,            1,    100,  33),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_receive,     400, 180,  36),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_send,        400, 550,  39),
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount,            &sub.camera_mount, update,              50,  75,  45),
#endif
#if AP_CAMERA_ENABLED
    SCHED_TASK_CLASS(AP_Camera,           &sub.camera,       update,              50,  75,  48),
#endif
#if HAL_LOGGING_ENABLED
    SCHED_TASK(ten_hz_logging_loop,   10,    350,  51),
    SCHED_TASK(twentyfive_hz_logging, 25,    110,  54),
    SCHED_TASK(loop_rate_logging, LOOP_RATE, 50,   55),
    SCHED_TASK_CLASS(AP_Logger,           &sub.logger,       periodic_tasks,     400, 300,  57),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,   &sub.ins,          periodic,           400,  50,  60),
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Scheduler,        &sub.scheduler,    update_logging,     0.1,  75,  63),
#endif
    SCHED_TASK(terrain_update,        10,    100,  72),
#if AP_STATS_ENABLED
    SCHED_TASK(stats_update,           1,    200,  76),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75,  78),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75,  81),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75,  84),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,     3.3,    75,  87),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,     75,  90),
#endif

};

void Sub::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                 uint8_t &task_count,
                                 uint32_t &log_bit)
{
    // AP_Vehicle::setup() 通过这个接口取得 ArduSub 任务表。
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t Sub::_failsafe_priorities[5];

void Sub::run_rate_controller()
{
    const float last_loop_time_s = AP::scheduler().get_last_loop_time_s();
    motors.set_dt_s(last_loop_time_s);
    attitude_control.set_dt_s(last_loop_time_s);
    pos_control.set_dt_s(last_loop_time_s);

    // MANUAL 直接控制推进器，MOTOR_DETECT 用于检测布局，两者不运行角速度闭环。
    if (control_mode != Mode::Number::MANUAL && control_mode != Mode::Number::MOTOR_DETECT) {
        // 使用本轮真实 dt 运行底层角速度控制器。
        attitude_control.rate_controller_run();
    }
}

// 50 Hz 汇总任务：驾驶输入和关键失控保护检查。
void Sub::fifty_hz_loop()
{
    // 检查是否持续收到有效驾驶输入。
    failsafe_pilot_input_check();

    failsafe_crash_check();

    failsafe_ekf_check();

    failsafe_sensors_check();
#if !AP_SUB_RC_ENABLED
    rc().read_input();
#endif
    g2.actuators.update_actuators();
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Sub::update_batt_compass()
{
    // read battery before compass because it may be used for motor interference compensation
    battery.read();

    if (AP::compass().available()) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors.get_throttle());
        compass.read();
    }
}

#if HAL_LOGGING_ENABLED
// ten_hz_logging_loop
// should be run at 10hz
void Sub::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        attitude_control.Write_ANG();
        attitude_control.Write_Rate(pos_control);
        if (should_log(MASK_LOG_PID)) {
            logger.Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            logger.Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            logger.Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            logger.Write_PID(LOG_PIDA_MSG, pos_control.D_get_accel_pid().get_pid_info());
        }
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        motors.Log_Write();
    }
    if (should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
    }
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (sub.flightmode->requires_GPS() || sub.flightmode->requires_altitude())) {
        pos_control.write_log();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        AP::ins().Write_Vibration();
    }
#if HAL_MOUNT_ENABLED
    if (should_log(MASK_LOG_CAMERA)) {
        camera_mount.write_log();
    }
#endif
}

// twentyfive_hz_logging_loop
// should be run at 25hz
void Sub::twentyfive_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        attitude_control.Write_ANG();
        attitude_control.Write_Rate(pos_control);
        if (should_log(MASK_LOG_PID)) {
            logger.Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            logger.Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            logger.Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            logger.Write_PID(LOG_PIDA_MSG, pos_control.D_get_accel_pid().get_pid_info());
        }
    }

    // log IMU data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_FAST)) {
        AP::ins().Write_IMU();
    }
}

// Full rate logging of IMU
void Sub::loop_rate_logging()
{
    if (should_log(MASK_LOG_IMU_FAST)) {
        AP::ins().Write_IMU();
    }
}
#endif  // HAL_LOGGING_ENABLED

// three_hz_loop - 3.3hz loop
void Sub::three_hz_loop()
{
    leak_detector.update();

    failsafe_leak_check();

    failsafe_internal_pressure_check();

    failsafe_internal_temperature_check();

    // check if we've lost contact with the ground station
    failsafe_gcs_check();

    // check if we've lost terrain data
    failsafe_terrain_check();

#if AP_SERVORELAYEVENTS_ENABLED
    ServoRelayEvents.update_events();
#endif
}

// one_hz_loop - runs at 1Hz
void Sub::one_hz_loop()
{
    bool arm_check = arming.pre_arm_checks(false);
    ap.pre_arm_check = arm_check;
    AP_Notify::flags.pre_arm_check = arm_check;
    AP_Notify::flags.pre_arm_gps_check = position_ok();
    AP_Notify::flags.flying = motors.armed();

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::AP_STATE, ap.value);
    }
#endif

    if (!motors.armed()) {
        motors.update_throttle_range();
    }

    // update assigned functions and enable auxiliary servos
    AP::srv().enable_aux_servos();

#if HAL_LOGGING_ENABLED
    // log terrain data
    terrain_logging();
#endif

    // need to set "likely flying" when armed to allow for compass
    // learning to run
    set_likely_flying(hal.util->get_soft_armed());

    attitude_control.set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
    pos_control.D_get_accel_pid().set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
}

void Sub::read_AHRS()
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
    // <true> tells AHRS to skip INS update as we have already done it in fast_loop()
    ahrs.update(true);
    ahrs_view.update();
}

// read baro and rangefinder altitude at 10hz
void Sub::update_altitude()
{
    // read in baro altitude
    read_barometer();

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
        AP::ins().write_notch_log_messages();
#endif
#if HAL_GYROFFT_ENABLED
        gyro_fft.write_log_messages();
#endif
    }
#endif  // HAL_LOGGING_ENABLED
}

bool Sub::control_check_barometer()
{
    if (!ap.depth_sensor_present) { // can't hold depth without a depth sensor
        gcs().send_text(MAV_SEVERITY_WARNING, "Depth sensor is not connected.");
        return false;
    } else if (failsafe.sensor_health) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Depth sensor error.");
        return false;
    }
    return true;
}

// vehicle specific waypoint info helpers
bool Sub::get_wp_distance_m(float &distance) const
{
    // see GCS_MAVLINK_Sub::send_nav_controller_output()
    distance = sub.wp_nav.get_wp_distance_to_destination_cm() * 0.01;
    return true;
}

// vehicle specific waypoint info helpers
bool Sub::get_wp_bearing_deg(float &bearing) const
{
    // see GCS_MAVLINK_Sub::send_nav_controller_output()
    bearing = sub.wp_nav.get_wp_bearing_to_destination_cd() * 0.01;
    return true;
}

// vehicle specific waypoint info helpers
bool Sub::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // no crosstrack error reported, see GCS_MAVLINK_Sub::send_nav_controller_output()
    xtrack_error = 0;
    return true;
}

#if AP_STATS_ENABLED
/*
  update AP_Stats
*/
void Sub::stats_update(void)
{
    AP::stats()->set_flying(motors.armed());
}
#endif

// get the altitude relative to the home position or the ekf origin
float Sub::get_alt_rel() const
{
    if (!ap.depth_sensor_present) {
        return 0;
    }

    // get relative position
    float posD;
    ahrs.get_relative_position_D_home(posD);
    return -posD;
}

// get the altitude above mean sea level
float Sub::get_alt_msl() const
{
    if (!ap.depth_sensor_present) {
        return 0;
    }

    Location origin;
    if (!ahrs.get_origin(origin)) {
        return 0;
    }

    // get relative position
    float posD;
    if (!ahrs.get_relative_position_D_origin_float(posD)) {
        // fall back to the barometer reading
        posD = -AP::baro().get_altitude();
    }

    // add in the ekf origin altitude
    posD -= static_cast<float>(origin.alt) * 0.01f;

    // convert down to up
    return -posD;
}

#if AP_SUB_RC_ENABLED
void Sub::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    rc().read_mode_switch();
}
#endif

// 建立唯一车辆对象，并同时暴露为 AP_Vehicle 引用。
Sub *Sub::_singleton = nullptr;

Sub sub;
AP_Vehicle& vehicle = sub;

// 生成平台 main()：HAL_ChibiOS::run() 会调用这个 sub 的 setup()/loop()。
AP_HAL_MAIN_CALLBACKS(&sub);
