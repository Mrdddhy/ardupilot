#include "Copter.h"
#include <AP_BLHeli/AP_BLHeli.h>

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

static void mavlink_delay_cb_static()
{
    copter.mavlink_delay_cb();
}


static void failsafe_check_static()
{
    copter.failsafe_check();
}

void Copter::init_ardupilot()
{
    // initialise serial port
    serial_manager.init_console();/*初始化串口控制台端口*/

    hal.console->printf("\n\nInit %s"
                        "\n\nFree RAM: %u\n",
                        AP::fwversion().fw_string,              /*打印软件版本(git版本)*/
                        (unsigned)hal.util->available_memory());/*打印剩下可用内存*/

    //
    // Report firmware version code expect on console (check of actual EEPROM format version is done in load_parameters function)
    //
    report_version();/*报告版本格式代码*/

    // load parameters from EEPROM
    load_parameters();/*从EEPROM加载参数*/

    // time per loop - this gets updated in the main loop() based on
    // actual loop rate
    /*每一圈循环时间--这个得以更新是在main_loop()里面基于实际循环速率的*/
    G_Dt = 1.0 / scheduler.get_loop_rate_hz();/*获取运行周期时间*/

#if STATS_ENABLED == ENABLED
    // initialise stats module
    g2.stats.init();/*初始化统计模组*/
#endif

    // identify ourselves correctly with the ground station
    // g为Parameters对象，在Parameters.cpp中实现了sysid_this_mav的默认赋值
    // sysid_this_mav -> MAV_SYSTEM_ID = 1
    mavlink_system.sysid = g.sysid_this_mav;/*辨识我们与地面站正确连接*/
    
    // initialise serial ports
    serial_manager.init();/*串口管理初始化*/

    // setup first port early to allow BoardConfig to report errors
    gcs().setup_console();/*设置第一个端口允许波特率配置去报告错误*/

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);
    
    BoardConfig.init();/*板子配置初始化*/
#if HAL_WITH_UAVCAN
    BoardConfig_CAN.init();
#endif

    // init cargo gripper
#if GRIPPER_ENABLED == ENABLED
    g2.gripper.init();/*夹具初始化*/
#endif

#if AC_FENCE == ENABLED
    fence.init();/*地理围栏初始化*/
#endif
    
    // init winch and wheel encoder
    winch_init();/*轮式编码器初始化，飞机disabled*/

    // initialise notify system
    notify.init();/*通知对象初始化*/
    notify_flight_mode();/*飞行模式通知*/

    // initialise battery monitor
    battery.init();/*电池检测初始化*/

    // Init RSSI
    rssi.init();/*接收信号强度初始化*/
    
    barometer.init();/*气压计初始化*/

    // setup telem slots with serial ports
    gcs().setup_uarts();/*设置数传串口*/

#if OSD_ENABLED == ENABLED
    osd.init();/*图传初始化*/
#endif

#if LOGGING_ENABLED == ENABLED
    log_init();/*日志初始化*/
#endif

    // update motor interlock state
    update_using_interlock();/*更新电机内部锁状态*/

#if FRAME_CONFIG == HELI_FRAME
    // trad heli specific initialisation
    heli_init();
#endif
#if FRAME_CONFIG == HELI_FRAME
    input_manager.set_loop_rate(scheduler.get_loop_rate_hz());
#endif

   /*设置遥控器通道初始化*/
    init_rc_in();               // sets up rc channels from radio

    // allocate the motors class
    /*分配电机*/
    allocate_motors();

    // initialise rc channels including setting mode
    /*遥控器初始化？*/
    rc().init();

    // sets up motors and output to escs
    /*初始化电机和电调输出？*/
    init_rc_out();

    // check if we should enter esc calibration mode
    /*电调校准检测*/
    esc_calibration_startup_check();

    // motors initialised so parameters can be sent
    ap.initialised_params = true;

    /*继电器初始化*/
    relay.init();

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // Do GPS init
    /*GPS初始化*/
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

    /*罗盘初始化*/
    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

#if OPTFLOW == ENABLED
    // make optflow available to AHRS
    ahrs.set_optflow(&optflow); /*让AHRS可以使用光流*/
#endif

    // init Location class
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    Location::set_terrain(&terrain);
    wp_nav->set_terrain(&terrain);
#endif

#if AC_OAPATHPLANNER_ENABLED == ENABLED
    g2.oa.init();
#endif
    /*姿态控制检测参数*/
    attitude_control->parameter_sanity_check();
   
    /*位置控制设置时间周期*/
    pos_control->set_dt(scheduler.get_loop_period_s());

    // init the optical flow sensor
    /*初始化光流*/
    init_optflow();

#if MOUNT == ENABLED
    // initialise camera mount
    /*初始化相机*/
    camera_mount.init();
#endif

#if PRECISION_LANDING == ENABLED
    // initialise precision landing
    /*初始化精准降落*/
    init_precland();
#endif

    // initialise landing gear position
    /*初始化起落架位置*/
    landinggear.init();

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

#if HIL_MODE != HIL_MODE_DISABLED
    while (barometer.get_last_update() == 0) {
        // the barometer begins updating when we get the first
        // HIL_STATE message
        gcs().send_text(MAV_SEVERITY_WARNING, "Waiting for first HIL_STATE message");
        delay(1000);
    }

    // set INS to HIL mode
    ins.set_hil_mode();
#endif

    // read Baro pressure at ground
    /*读地面气压计并校准*/
    //-----------------------------
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();

    // initialise rangefinder
    /*初始化测距仪*/
    init_rangefinder();

    // init proximity sensor
    /*初始化近距离测距传感器*/
    init_proximity();

#if BEACON_ENABLED == ENABLED
    // init beacons used for non-gps position estimation
    /*初始化信标？当没有GPS的时候用其来进行位置估计*/
    g2.beacon.init();
#endif

    // init visual odometry
    /*初始化视觉里程计*/
    init_visual_odom();

#if RPM_ENABLED == ENABLED
    // initialise AP_RPM library
    rpm_sensor.init();
#endif

#if MODE_AUTO_ENABLED == ENABLED
    // initialise mission library
    /*初始化航线库*/
    mode_auto.mission.init();
#endif

#if MODE_SMARTRTL_ENABLED == ENABLED
    // initialize SmartRTL
    /*初始化智能返航*/
    g2.smart_rtl.init();
#endif

    // initialise AP_Logger library
    logger.setVehicle_Startup_Writer(FUNCTOR_BIND(&copter, &Copter::Log_Write_Vehicle_Startup_Messages, void));

    /*IMU传感器注册开始*/
    startup_INS_ground();/*--重点理解这个传感器的注册过程---*/

#ifdef ENABLE_SCRIPTING
    g2.scripting.init();
#endif // ENABLE_SCRIPTING

    // set landed flags
    /*设置着落标志*/
    set_land_complete(true);/*设置已经着陆标志位为true*/
    set_land_complete_maybe(true);/*设置可能着陆标志位为true*/

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    /*在飞行就绪时我们不想让串口发送东西过来中断我们的飞行*/
    serial_manager.set_blocking_writes_all(false);

    // enable CPU failsafe
    /*失控使能，用来设置对应的标志并记录失控的上一时刻的时间*/
    failsafe_enable();

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    // enable output to motors
    /*如果遥控校准检测通过，使能电机输出*/
    if (arming.rc_calibration_checks(true)) {
        enable_motor_output();
    }

    // disable safety if requested
    BoardConfig.init_safety();

    vehicle_setup();

    /*打印飞行就绪*/
    hal.console->printf("\nReady to FLY ");

    // flag that initialisation has completed
    /*设置初始化完成后的状态为true*/
    ap.initialised = true;

#if AP_PARAM_KEY_DUMP
    AP_Param::show_all(hal.console, true);
#endif
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//这个函数完成了地面启动时需要的所有校准
//******************************************************************************
void Copter::startup_INS_ground()
{
    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();/*初始化AHRS---设置安装方向*/
    ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);/*设置车辆为旋翼*/

    // Warm up and calibrate gyro offsets
    ins.init(scheduler.get_loop_rate_hz());/*预热和校准陀螺仪偏差*/

    // reset ahrs including gyro bias
    ahrs.reset();
}

// update the harmonic notch filter center frequency dynamically
void Copter::update_dynamic_notch()
{
    const float ref_freq = ins.get_gyro_harmonic_notch_center_freq_hz();
    const float ref = ins.get_gyro_harmonic_notch_reference();

    if (is_zero(ref)) {
        ins.update_harmonic_notch_freq_hz(ref_freq);
        return;
    }

    switch (ins.get_gyro_harmonic_notch_tracking_mode()) {
        case HarmonicNotch_UpdateThrottle: // throttle based tracking
            // set the harmonic notch filter frequency approximately scaled on motor rpm implied by throttle
            ins.update_harmonic_notch_freq_hz(ref_freq * MAX(1.0f, sqrtf(motors->get_throttle_out() / ref)));
            break;

#if RPM_ENABLED == ENABLED
        case HarmonicNotch_UpdateRPM: // rpm sensor based tracking
            if (rpm_sensor.healthy(0)) {
                // set the harmonic notch filter frequency from the main rotor rpm
                ins.update_harmonic_notch_freq_hz(MAX(ref_freq, rpm_sensor.get_rpm(0) * ref / 60.0f));
            } else {
                ins.update_harmonic_notch_freq_hz(ref_freq);
            }
            break;
#endif
#ifdef HAVE_AP_BLHELI_SUPPORT
        case HarmonicNotch_UpdateBLHeli: // BLHeli based tracking
            ins.update_harmonic_notch_freq_hz(MAX(ref_freq, AP_BLHeli::get_singleton()->get_average_motor_frequency_hz() * ref));
            break;
#endif
        case HarmonicNotch_Fixed: // static
        default:
            ins.update_harmonic_notch_freq_hz(ref_freq);
            break;
    }
}

// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Copter::position_ok() const
{
    // return false if ekf failsafe has triggered
    if (failsafe.ekf) {
        return false;
    }

    // check ekf position estimate
    return (ekf_position_ok() || optflow_position_ok());
}

// ekf_position_ok - returns true if the ekf claims it's horizontal absolute position estimate is ok and home position is set
bool Copter::ekf_position_ok() const
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // if disarmed we accept a predicted horizontal position
    if (!motors->armed()) {
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs));
    } else {
        // once armed we require a good absolute position and EKF must not be in const_pos_mode
        return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
    }
}

// optflow_position_ok - returns true if optical flow based position estimate is ok
bool Copter::optflow_position_ok() const
{
#if OPTFLOW != ENABLED && VISUAL_ODOMETRY_ENABLED != ENABLED
    return false;
#else
    // return immediately if EKF not used
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // return immediately if neither optflow nor visual odometry is enabled
    bool enabled = false;
#if OPTFLOW == ENABLED
    if (optflow.enabled()) {
        enabled = true;
    }
#endif
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    if (g2.visual_odom.enabled()) {
        enabled = true;
    }
#endif
    if (!enabled) {
        return false;
    }

    // get filter status from EKF
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // if disarmed we accept a predicted horizontal relative position
    if (!motors->armed()) {
        return (filt_status.flags.pred_horiz_pos_rel);
    } else {
        return (filt_status.flags.horiz_pos_rel && !filt_status.flags.const_pos_mode);
    }
#endif
}

// update_auto_armed - update status of auto_armed flag
void Copter::update_auto_armed()
{
    // disarm checks
    if(ap.auto_armed){
        // if motors are disarmed, auto_armed should also be false
        if(!motors->armed()) {
            set_auto_armed(false);
            return;
        }
        // if in stabilize or acro flight mode and throttle is zero, auto-armed should become false
        if(flightmode->has_manual_throttle() && ap.throttle_zero && !failsafe.radio) {
            set_auto_armed(false);
        }
        // if helicopters are on the ground, and the motor is switched off, auto-armed should be false
        // so that rotor runup is checked again before attempting to take-off
        if(ap.land_complete && motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED && ap.using_interlock) {
            set_auto_armed(false);
        }
    }else{
        // arm checks
        
        // for tradheli if motors are armed and throttle is above zero and the motor is started, auto_armed should be true
        if(motors->armed() && ap.using_interlock) {
            if(!ap.throttle_zero && motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
                set_auto_armed(true);
            }
        // if motors are armed and throttle is above zero auto_armed should be true
        // if motors are armed and we are in throw mode, then auto_armed should be true
        } else if (motors->armed() && !ap.using_interlock) {
            if(!ap.throttle_zero || control_mode == Mode::Number::THROW) {
                set_auto_armed(true);
            }
        }
    }
}

/*
  should we log a message type now?
 */
bool Copter::should_log(uint32_t mask)
{
#if LOGGING_ENABLED == ENABLED
    ap.logging_started = logger.logging_started();
    return logger.should_log(mask);
#else
    return false;
#endif
}

// return MAV_TYPE corresponding to frame class
MAV_TYPE Copter::get_frame_mav_type()
{
    switch ((AP_Motors::motor_frame_class)g2.frame_class.get()) {
        case AP_Motors::MOTOR_FRAME_QUAD:
        case AP_Motors::MOTOR_FRAME_UNDEFINED:
            return MAV_TYPE_QUADROTOR;
        case AP_Motors::MOTOR_FRAME_HEXA:
        case AP_Motors::MOTOR_FRAME_Y6:
            return MAV_TYPE_HEXAROTOR;
        case AP_Motors::MOTOR_FRAME_OCTA:
        case AP_Motors::MOTOR_FRAME_OCTAQUAD:
            return MAV_TYPE_OCTOROTOR;
        case AP_Motors::MOTOR_FRAME_HELI:
        case AP_Motors::MOTOR_FRAME_HELI_DUAL:
        case AP_Motors::MOTOR_FRAME_HELI_QUAD:
            return MAV_TYPE_HELICOPTER;
        case AP_Motors::MOTOR_FRAME_TRI:
            return MAV_TYPE_TRICOPTER;
        case AP_Motors::MOTOR_FRAME_SINGLE:
        case AP_Motors::MOTOR_FRAME_COAX:
        case AP_Motors::MOTOR_FRAME_TAILSITTER:
            return MAV_TYPE_COAXIAL;
        case AP_Motors::MOTOR_FRAME_DODECAHEXA:
            return MAV_TYPE_DODECAROTOR;
    }
    // unknown frame so return generic
    return MAV_TYPE_GENERIC;
}

// return string corresponding to frame_class
const char* Copter::get_frame_string()
{
    switch ((AP_Motors::motor_frame_class)g2.frame_class.get()) {
        case AP_Motors::MOTOR_FRAME_QUAD:
            return "QUAD";
        case AP_Motors::MOTOR_FRAME_HEXA:
            return "HEXA";
        case AP_Motors::MOTOR_FRAME_Y6:
            return "Y6";
        case AP_Motors::MOTOR_FRAME_OCTA:
            return "OCTA";
        case AP_Motors::MOTOR_FRAME_OCTAQUAD:
            return "OCTA_QUAD";
        case AP_Motors::MOTOR_FRAME_HELI:
            return "HELI";
        case AP_Motors::MOTOR_FRAME_HELI_DUAL:
            return "HELI_DUAL";
        case AP_Motors::MOTOR_FRAME_HELI_QUAD:
            return "HELI_QUAD";
        case AP_Motors::MOTOR_FRAME_TRI:
            return "TRI";
        case AP_Motors::MOTOR_FRAME_SINGLE:
            return "SINGLE";
        case AP_Motors::MOTOR_FRAME_COAX:
            return "COAX";
        case AP_Motors::MOTOR_FRAME_TAILSITTER:
            return "TAILSITTER";
        case AP_Motors::MOTOR_FRAME_DODECAHEXA:
            return "DODECA_HEXA";
        case AP_Motors::MOTOR_FRAME_UNDEFINED:
        default:
            return "UNKNOWN";
    }
}

/*
  allocate the motors class
 */
void Copter::allocate_motors(void)
{
    switch ((AP_Motors::motor_frame_class)g2.frame_class.get()) {
#if FRAME_CONFIG != HELI_FRAME
        case AP_Motors::MOTOR_FRAME_QUAD:
        case AP_Motors::MOTOR_FRAME_HEXA:
        case AP_Motors::MOTOR_FRAME_Y6:
        case AP_Motors::MOTOR_FRAME_OCTA:
        case AP_Motors::MOTOR_FRAME_OCTAQUAD:
        case AP_Motors::MOTOR_FRAME_DODECAHEXA:
        default:
            motors = new AP_MotorsMatrix(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsMatrix::var_info;
            break;
        case AP_Motors::MOTOR_FRAME_TRI:
            motors = new AP_MotorsTri(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsTri::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_TRICOPTER);
            break;
        case AP_Motors::MOTOR_FRAME_SINGLE:
            motors = new AP_MotorsSingle(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsSingle::var_info;
            break;
        case AP_Motors::MOTOR_FRAME_COAX:
            motors = new AP_MotorsCoax(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsCoax::var_info;
            break;
        case AP_Motors::MOTOR_FRAME_TAILSITTER:
            motors = new AP_MotorsTailsitter(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsTailsitter::var_info;
            break;
#else // FRAME_CONFIG == HELI_FRAME
        case AP_Motors::MOTOR_FRAME_HELI_DUAL:
            motors = new AP_MotorsHeli_Dual(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsHeli_Dual::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;

        case AP_Motors::MOTOR_FRAME_HELI_QUAD:
            motors = new AP_MotorsHeli_Quad(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsHeli_Quad::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;
            
        case AP_Motors::MOTOR_FRAME_HELI:
        default:
            motors = new AP_MotorsHeli_Single(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsHeli_Single::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;
#endif
    }
    if (motors == nullptr) {
        AP_HAL::panic("Unable to allocate FRAME_CLASS=%u", (unsigned)g2.frame_class.get());
    }
    AP_Param::load_object_from_eeprom(motors, motors_var_info);

    ahrs_view = ahrs.create_view(ROTATION_NONE);/*系统中默认为不旋转*/
    if (ahrs_view == nullptr) {
        AP_HAL::panic("Unable to allocate AP_AHRS_View");
    }

    const struct AP_Param::GroupInfo *ac_var_info;

#if FRAME_CONFIG != HELI_FRAME
    attitude_control = new AC_AttitudeControl_Multi(*ahrs_view, aparm, *motors, scheduler.get_loop_period_s());
    ac_var_info = AC_AttitudeControl_Multi::var_info;
#else
    attitude_control = new AC_AttitudeControl_Heli(*ahrs_view, aparm, *motors, scheduler.get_loop_period_s());
    ac_var_info = AC_AttitudeControl_Heli::var_info;
#endif
    if (attitude_control == nullptr) {
        AP_HAL::panic("Unable to allocate AttitudeControl");
    }
    AP_Param::load_object_from_eeprom(attitude_control, ac_var_info);
        
    pos_control = new AC_PosControl(*ahrs_view, inertial_nav, *motors, *attitude_control);
    if (pos_control == nullptr) {
        AP_HAL::panic("Unable to allocate PosControl");
    }
    AP_Param::load_object_from_eeprom(pos_control, pos_control->var_info);

#if AC_OAPATHPLANNER_ENABLED == ENABLED
    wp_nav = new AC_WPNav_OA(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
#else
    wp_nav = new AC_WPNav(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
#endif
    if (wp_nav == nullptr) {
        AP_HAL::panic("Unable to allocate WPNav");
    }
    AP_Param::load_object_from_eeprom(wp_nav, wp_nav->var_info);

    loiter_nav = new AC_Loiter(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    if (loiter_nav == nullptr) {
        AP_HAL::panic("Unable to allocate LoiterNav");
    }
    AP_Param::load_object_from_eeprom(loiter_nav, loiter_nav->var_info);

#if MODE_CIRCLE_ENABLED == ENABLED
    circle_nav = new AC_Circle(inertial_nav, *ahrs_view, *pos_control);
    if (circle_nav == nullptr) {
        AP_HAL::panic("Unable to allocate CircleNav");
    }
    AP_Param::load_object_from_eeprom(circle_nav, circle_nav->var_info);
#endif

    // reload lines from the defaults file that may now be accessible
    AP_Param::reload_defaults_file(true);
    
    // now setup some frame-class specific defaults
    switch ((AP_Motors::motor_frame_class)g2.frame_class.get()) {
    case AP_Motors::MOTOR_FRAME_Y6:
        attitude_control->get_rate_roll_pid().kP().set_default(0.1);
        attitude_control->get_rate_roll_pid().kD().set_default(0.006);
        attitude_control->get_rate_pitch_pid().kP().set_default(0.1);
        attitude_control->get_rate_pitch_pid().kD().set_default(0.006);
        attitude_control->get_rate_yaw_pid().kP().set_default(0.15);
        attitude_control->get_rate_yaw_pid().kI().set_default(0.015);
        break;
    case AP_Motors::MOTOR_FRAME_TRI:
        attitude_control->get_rate_yaw_pid().filt_D_hz().set_default(100);
        break;
    default:
        break;
    }

    // brushed 16kHz defaults to 16kHz pulses
    if (motors->get_pwm_type() == AP_Motors::PWM_TYPE_BRUSHED) {
        g.rc_speed.set_default(16000);
    }
    
    // upgrade parameters. This must be done after allocating the objects
    convert_pid_parameters();
#if FRAME_CONFIG == HELI_FRAME
    convert_tradheli_parameters();
#endif
}

bool Copter::is_tradheli() const
{
#if FRAME_CONFIG == HELI_FRAME
    return true;
#else
    return false;
#endif
}
