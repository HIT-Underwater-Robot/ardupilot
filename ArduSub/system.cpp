#include "Sub.h"

/*****************************************************************************
* init_ardupilot() 是 AP_Vehicle 公共初始化之后的 ArduSub 专属初始化入口。
* 它依次建立通信、输入输出、传感器、任务、日志和失控保护，最后把
* ap.initialised 置位。初始化完成前不会向地面站报告完整可用状态。
*****************************************************************************/

static void failsafe_check_static()
{
    sub.mainloop_failsafe_check();
}

void Sub::init_ardupilot()
{
    // 1. 建立通知、供电和压力传感器等基础对象。
    notify.init();

    // 电池监视器负责电压、电流和电池失控保护来源。
    battery.init();

    barometer.init();

#if AP_FEATURE_BOARD_DETECT
    // BoardConfig 初始化之后才能识别具体硬件变体，并选择外部气压计总线。
    switch (AP_BoardConfig::get_board_type()) {
    case AP_BoardConfig::PX4_BOARD_PIXHAWK2:
        AP_Param::set_default_by_name("BARO_EXT_BUS", 0);
        break;
    case AP_BoardConfig::PX4_BOARD_PIXHAWK:
        AP_Param::set_by_name("BARO_EXT_BUS", 1);
        break;
    default:
        AP_Param::set_default_by_name("BARO_EXT_BUS", 1);
        break;
    }
#elif CONFIG_HAL_BOARD != HAL_BOARD_LINUX
    AP_Param::set_default_by_name("BARO_EXT_BUS", 1);
#endif

#if AP_TEMPERATURE_SENSOR_ENABLED
    // 保持 ArduSub 既有行为：温度传感器默认跟随外部气压计所在 I2C 总线。
    AP_Param::set_default_by_name("TEMP1_BUS", barometer.external_bus());
#endif

    // 2. 把 SERIALx 配置映射为遥测/MAVLink 端口。
    gcs().setup_uarts();

    // 初始化 RC/手柄通道和辅助功能映射。
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM);
    rc().init();


    init_rc_in();               // 建立驾驶输入通道
    init_rc_out();              // 建立推进器和 ESC 输出
    init_joystick();            // 初始化手柄按键映射

#if AP_RELAY_ENABLED
    relay.init();
#endif

#if OSD_ENABLED
    osd.init();
#endif

    /*
     * 注册独立于车辆主循环的定时失控保护。如果主循环没有按期运行，
     * 该回调仍能触发保护；注册前要求 RC 子系统已经初始化。
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // 3. 初始化导航相关传感器。即使当前系统主要使用外部里程计，公共
    // 车辆框架仍保留 GPS 对象和对应的编译功能。
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init();

    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

#if AP_AIRSPEED_ENABLED
    airspeed.set_log_bit(MASK_LOG_IMU);
#endif

#if AP_OPTICALFLOW_ENABLED
    // 可选光流传感器。
    optflow.init(MASK_LOG_OPTFLOW);
#endif

#if HAL_MOUNT_ENABLED
    // 初始化云台并先写入安全角度目标。
    camera_mount.init();
    // 必须先设置一次角度目标，才能正确初始化舵机输出。
    camera_mount.set_angle_target(0, 0, 0, false);
    // set_angle_target() 会改变云台模式，因此随后恢复为 RC 控制模式。
    camera_mount.set_mode(MAV_MOUNT_MODE_RC_TARGETING);
#endif

#if AP_CAMERA_ENABLED
    // 可选相机控制接口。
    camera.init();
#endif

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

    // 4. 校准压力传感器，并在 AP_Baro 实例中寻找水压深度传感器。
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate(false);
    barometer.update();

    for (uint8_t i = 0; i < barometer.num_instances(); i++) {
        if (barometer.get_type(i) == AP_Baro::BARO_TYPE_WATER) {
            barometer.set_primary_baro(i);
            depth_sensor_idx = i;
            ap.depth_sensor_present = true;
            sensor_health.depth = barometer.healthy(depth_sensor_idx); // 初始化深度计健康状态
            break; // 使用找到的第一个水压传感器
        }
    }

    if (!ap.depth_sensor_present) {
        // 未发现外接水压计时退回板载气压计；它不能提供可靠水下深度，
        // 因此显著增大高度量测噪声，降低其对状态估计的影响。
        barometer.set_primary_baro(0);
        ahrs.set_alt_measurement_noise(10.0f);
    } else {
        ahrs.set_alt_measurement_noise(0.1f);
    }

    leak_detector.init();

    last_pilot_heading_rad = ahrs.get_yaw_rad();

    // 可选测距仪可用于距底高度和 SurfTrak。
#if AP_RANGEFINDER_ENABLED
    init_rangefinder();
#endif

    // 5. 初始化任务和日志回调。
    mission.init();
#if HAL_LOGGING_ENABLED
    mission.set_log_start_mission_item_bit(MASK_LOG_CMD);
#endif

    // 日志开始时由 Sub 写入车辆专属启动消息。
#if HAL_LOGGING_ENABLED
    logger.setVehicle_Startup_Writer(FUNCTOR_BIND(&sub, &Sub::Log_Write_Vehicle_Startup_Messages, void));
#endif

    startup_INS_ground();

    // 6. INS 初始化完成后启用主循环/CPU 失控保护。
    mainloop_failsafe_enable();

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    // 参数迁移：首次运行新版本时把旧手柄配置转换为执行器/灯光配置。
    if (g2.param_conversion_increment < 1) {
        update_actuators_from_jsbuttons();
        update_lights_from_rcin();
        g2.param_conversion_increment.set_and_save(1);
    }

    g2.actuators.initialize_actuators();

#if LEAKDETECTOR_MAX_INSTANCES > 0
    update_leak_pins();
#endif
#if AP_RELAY_ENABLED
    update_relay_pins();
#endif
    // 所有 ArduSub 专属初始化完成；之后允许报告完整车辆状态。
    ap.initialised = true;
}


// 启动 AHRS/INS，并完成静止启动所需的陀螺仪校准。
void Sub::startup_INS_ground()
{
    // 初始化 AHRS，并声明当前车辆是水下机器人且不假设始终向前运动。
    ahrs.init();
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::SUBMARINE);
    ahrs.set_fly_forward(false);

    // 按主循环频率启动 INS，预热并校准陀螺仪零偏。
    ins.init(scheduler.get_loop_rate_hz());

    // 使用校准后的陀螺仪偏置重置 AHRS。
    ahrs.reset();
}

// calibrate gyros - returns true if successfully calibrated
// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Sub::position_ok()
{
    // return false if ekf failsafe has triggered
    if (failsafe.ekf) {
        return false;
    }

    // check ekf position estimate
    return (ekf_position_ok() || optflow_position_ok());
}

// ekf_position_ok - returns true if the ekf claims it's horizontal absolute position estimate is ok and home position is set
bool Sub::ekf_position_ok()
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // if disarmed we accept a predicted horizontal position
    if (!motors.armed()) {
        if (ahrs.has_status(AP_AHRS::Status::HORIZ_POS_ABS)) {
            return true;
        }
        if (ahrs.has_status(AP_AHRS::Status::PRED_HORIZ_POS_ABS)) {
            return true;
        }
        return false;
    }

    // once armed we require a good absolute position and EKF must not be in const_pos_mode
    if (ahrs.has_status(AP_AHRS::Status::CONST_POS_MODE)) {
        return false;
    }
    return ahrs.has_status(AP_AHRS::Status::HORIZ_POS_ABS);
}

// optflow_position_ok - returns true if optical flow based position estimate is ok
bool Sub::optflow_position_ok()
{
    // return immediately if EKF not used
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // return immediately if neither optflow nor visual odometry is enabled
    bool enabled = false;
#if AP_OPTICALFLOW_ENABLED
    if (optflow.enabled()) {
        enabled = true;
    }
#endif
#if HAL_VISUALODOM_ENABLED
    if (visual_odom.enabled()) {
        enabled = true;
    }
#endif
    if (!enabled) {
        return false;
    }

    // if disarmed we accept a predicted horizontal relative position
    if (!motors.armed()) {
        return ahrs.has_status(AP_AHRS::Status::PRED_HORIZ_POS_REL);
    }

    if (ahrs.has_status(AP_AHRS::Status::CONST_POS_MODE)) {
        return false;
    }

    return ahrs.has_status(AP_AHRS::Status::HORIZ_POS_REL);
}

#if HAL_LOGGING_ENABLED
/*
  should we log a message type now?
 */
bool Sub::should_log(uint32_t mask)
{
    ap.logging_started = logger.logging_started();
    return logger.should_log(mask);
}
#endif

#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>
#include <AP_Avoidance/AP_Avoidance.h>
#include <AP_ADSB/AP_ADSB.h>

// dummy method to avoid linking AFS
#if AP_ADVANCEDFAILSAFE_ENABLED
bool AP_AdvancedFailsafe::gcs_terminate(bool should_terminate, const char *reason) { return false; }
AP_AdvancedFailsafe *AP::advancedfailsafe() { return nullptr; }
#endif

#if AP_ADSB_AVOIDANCE_ENABLED
// dummy method to avoid linking AP_Avoidance
AP_Avoidance *AP::ap_avoidance() { return nullptr; }
#endif  // AP_ADSB_AVOIDANCE_ENABLED
