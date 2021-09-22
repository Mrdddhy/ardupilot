#include "AC_AttitudeControl.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
 // default gains for Plane
 # define AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT  0.2f    // Soft
#else
 // default gains for Copter and Sub
 # define AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT  0.15f   // Medium
#endif

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl::var_info[] = {

    // 0, 1 were RATE_RP_MAX, RATE_Y_MAX

    // @Param: SLEW_YAW
    // @DisplayName: Yaw target slew rate
    // @Description: Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
    // @Units: cdeg/s
    // @Range: 500 18000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("SLEW_YAW", 2, AC_AttitudeControl, _slew_yaw, AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT_CDS),

    // 3 was for ACCEL_RP_MAX

    // @Param: ACCEL_Y_MAX
    // @DisplayName: Acceleration Max for Yaw
    // @Description: Maximum acceleration in yaw axis
    // @Units: cdeg/s/s
    // @Range: 0 72000
    // @Values: 0:Disabled, 9000:VerySlow, 18000:Slow, 36000:Medium, 54000:Fast
    // @Increment: 1000
    // @User: Advanced
    AP_GROUPINFO("ACCEL_Y_MAX", 4, AC_AttitudeControl, _accel_yaw_max, AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_CDSS),

    // @Param: RATE_FF_ENAB
    // @DisplayName: Rate Feedforward Enable
    // @Description: Controls whether body-frame rate feedfoward is enabled or disabled
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("RATE_FF_ENAB", 5, AC_AttitudeControl, _rate_bf_ff_enabled, AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT),

    // @Param: ACCEL_R_MAX
    // @DisplayName: Acceleration Max for Roll
    // @Description: Maximum acceleration in roll axis
    // @Units: cdeg/s/s
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 30000:VerySlow, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("ACCEL_R_MAX", 6, AC_AttitudeControl, _accel_roll_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS),

    // @Param: ACCEL_P_MAX
    // @DisplayName: Acceleration Max for Pitch
    // @Description: Maximum acceleration in pitch axis
    // @Units: cdeg/s/s
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 30000:VerySlow, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("ACCEL_P_MAX", 7, AC_AttitudeControl, _accel_pitch_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS),

    // IDs 8,9,10,11 RESERVED (in use on Solo)

    // @Param: ANGLE_BOOST
    // @DisplayName: Angle Boost
    // @Description: Angle Boost increases output throttle as the vehicle leans to reduce loss of altitude
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ANGLE_BOOST", 12, AC_AttitudeControl, _angle_boost_enabled, 1),

    // @Param: ANG_RLL_P
    // @DisplayName: Roll axis angle controller P gain
    // @Description: Roll axis angle controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_roll, "ANG_RLL_", 13, AC_AttitudeControl, AC_P),

    // @Param: ANG_PIT_P
    // @DisplayName: Pitch axis angle controller P gain
    // @Description: Pitch axis angle controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_pitch, "ANG_PIT_", 14, AC_AttitudeControl, AC_P),

    // @Param: ANG_YAW_P
    // @DisplayName: Yaw axis angle controller P gain
    // @Description: Yaw axis angle controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 6.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_yaw, "ANG_YAW_", 15, AC_AttitudeControl, AC_P),

    // @Param: ANG_LIM_TC
    // @DisplayName: Angle Limit (to maintain altitude) Time Constant
    // @Description: Angle Limit (to maintain altitude) Time Constant
    // @Range: 0.5 10.0
    // @User: Advanced
    AP_GROUPINFO("ANG_LIM_TC", 16, AC_AttitudeControl, _angle_limit_tc, AC_ATTITUDE_CONTROL_ANGLE_LIMIT_TC_DEFAULT),

    // @Param: RATE_R_MAX
    // @DisplayName: Angular Velocity Max for Roll
    // @Description: Maximum angular velocity in roll axis
    // @Units: deg/s
    // @Range: 0 1080
    // @Increment: 1
    // @Values: 0:Disabled, 360:Slow, 720:Medium, 1080:Fast
    // @User: Advanced
    AP_GROUPINFO("RATE_R_MAX", 17, AC_AttitudeControl, _ang_vel_roll_max, 0.0f),

    // @Param: RATE_P_MAX
    // @DisplayName: Angular Velocity Max for Pitch
    // @Description: Maximum angular velocity in pitch axis
    // @Units: deg/s
    // @Range: 0 1080
    // @Increment: 1
    // @Values: 0:Disabled, 360:Slow, 720:Medium, 1080:Fast
    // @User: Advanced
    AP_GROUPINFO("RATE_P_MAX", 18, AC_AttitudeControl, _ang_vel_pitch_max, 0.0f),

    // @Param: RATE_Y_MAX
    // @DisplayName: Angular Velocity Max for Yaw
    // @Description: Maximum angular velocity in yaw axis
    // @Units: deg/s
    // @Range: 0 1080
    // @Increment: 1
    // @Values: 0:Disabled, 360:Slow, 720:Medium, 1080:Fast
    // @User: Advanced
    AP_GROUPINFO("RATE_Y_MAX", 19, AC_AttitudeControl, _ang_vel_yaw_max, 0.0f),

    // @Param: INPUT_TC
    // @DisplayName: Attitude control input time constant
    // @Description: Attitude control input time constant.  Low numbers lead to sharper response, higher numbers to softer response
    // @Units: s
    // @Range: 0 1
    // @Increment: 0.01
    // @Values: 0.5:Very Soft, 0.2:Soft, 0.15:Medium, 0.1:Crisp, 0.05:Very Crisp
    // @User: Standard
    AP_GROUPINFO("INPUT_TC", 20, AC_AttitudeControl, _input_tc, AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT),

    AP_GROUPEND
};

// Ensure attitude controller have zero errors to relax rate controller output
void AC_AttitudeControl::relax_attitude_controllers()
{
    // Initialize the attitude variables to the current attitude
    _ahrs.get_quat_body_to_ned(_attitude_target_quat);
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);
    _attitude_ang_error.initialise();

    // Initialize the angular rate variables to the current rate
    _attitude_target_ang_vel = _ahrs.get_gyro();
    ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);
    _rate_target_ang_vel = _ahrs.get_gyro();

    // Initialize remaining variables
    _thrust_error_angle = 0.0f;

    // Reset the PID filters
    get_rate_roll_pid().reset_filter();
    get_rate_pitch_pid().reset_filter();
    get_rate_yaw_pid().reset_filter();

    // Reset the I terms
    reset_rate_controller_I_terms();
}

void AC_AttitudeControl::reset_rate_controller_I_terms()
{
    get_rate_roll_pid().reset_I();
    get_rate_pitch_pid().reset_I();
    get_rate_yaw_pid().reset_I();
}

// reset rate controller I terms smoothly to zero in 0.5 seconds
void AC_AttitudeControl::reset_rate_controller_I_terms_smoothly()
{
    get_rate_roll_pid().reset_I_smoothly();
    get_rate_pitch_pid().reset_I_smoothly();
    get_rate_yaw_pid().reset_I_smoothly();
}

// The attitude controller works around the concept of the desired attitude, target attitude
// and measured attitude. The desired attitude is the attitude input into the attitude controller
// that expresses where the higher level code would like the aircraft to move to. The target attitude is moved
// to the desired attitude with jerk, acceleration, and velocity limits. The target angular velocities are fed
// directly into the rate controllers. The angular error between the measured attitude and the target attitude is
// fed into the angle controller and the output of the angle controller summed at the input of the rate controllers.
// By feeding the target angular velocity directly into the rate controllers the measured and target attitudes
// remain very close together.
//
// All input functions below follow the same procedure
// 1. define the desired attitude the aircraft should attempt to achieve using the input variables
// 2. using the desired attitude and input variables, define the target angular velocity so that it should
//    move the target attitude towards the desired attitude
// 3. if _rate_bf_ff_enabled is not being used then make the target attitude
//    and target angular velocities equal to the desired attitude and desired angular velocities.
// 4. ensure _attitude_target_quat, _attitude_target_euler_angle, _attitude_target_euler_rate and
//    _attitude_target_ang_vel have been defined. This ensures input modes can be changed without discontinuity.
// 5. attitude_controller_run_quat is then run to pass the target angular velocities to the rate controllers and
//    integrate them into the target attitude. Any errors between the target attitude and the measured attitude are
//    corrected by first correcting the thrust vector until the angle between the target thrust vector measured
//    trust vector drops below 2*AC_ATTITUDE_THRUST_ERROR_ANGLE. At this point the heading is also corrected.

// Command a Quaternion attitude with feedforward and smoothing
void AC_AttitudeControl::input_quaternion(Quaternion attitude_desired_quat)
{
    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

    Quaternion attitude_error_quat = _attitude_target_quat.inverse() * attitude_desired_quat;
    Vector3f attitude_error_angle;
    attitude_error_quat.to_axis_angle(attitude_error_angle);

    if (_rate_bf_ff_enabled) {
        // When acceleration limiting and feedforward are enabled, the sqrt controller is used to compute an euler
        // angular velocity that will cause the euler angle to smoothly stop at the input angle with limited deceleration
        // and an exponential decay specified by _input_tc at the end.
        _attitude_target_ang_vel.x = input_shaping_angle(wrap_PI(attitude_error_angle.x), _input_tc, get_accel_roll_max_radss(), _attitude_target_ang_vel.x, _dt);
        _attitude_target_ang_vel.y = input_shaping_angle(wrap_PI(attitude_error_angle.y), _input_tc, get_accel_pitch_max_radss(), _attitude_target_ang_vel.y, _dt);
        _attitude_target_ang_vel.z = input_shaping_angle(wrap_PI(attitude_error_angle.z), _input_tc, get_accel_yaw_max_radss(), _attitude_target_ang_vel.z, _dt);

        // Limit the angular velocity
        ang_vel_limit(_attitude_target_ang_vel, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));
        // Convert body-frame angular velocity into euler angle derivative of desired attitude
        ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);
    } else {
        _attitude_target_quat = attitude_desired_quat;

        // Set rate feedforward requests to zero
        _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
        _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
void AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds)
{
    // Convert from centidegrees on public interface to radians
    float euler_roll_angle = radians(euler_roll_angle_cd * 0.01f);
    float euler_pitch_angle = radians(euler_pitch_angle_cd * 0.01f);
    float euler_yaw_rate = radians(euler_yaw_rate_cds * 0.01f);

    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

    // Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
    euler_roll_angle += get_roll_trim_rad();

    if (_rate_bf_ff_enabled) {
        // translate the roll pitch and yaw acceleration limits to the euler axis
        Vector3f euler_accel = euler_accel_limit(_attitude_target_euler_angle, Vector3f(get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss()));

        // When acceleration limiting and feedforward are enabled, the sqrt controller is used to compute an euler
        // angular velocity that will cause the euler angle to smoothly stop at the input angle with limited deceleration
        // and an exponential decay specified by smoothing_gain at the end.
        _attitude_target_euler_rate.x = input_shaping_angle(wrap_PI(euler_roll_angle - _attitude_target_euler_angle.x), _input_tc, euler_accel.x, _attitude_target_euler_rate.x, _dt);
        _attitude_target_euler_rate.y = input_shaping_angle(wrap_PI(euler_pitch_angle - _attitude_target_euler_angle.y), _input_tc, euler_accel.y, _attitude_target_euler_rate.y, _dt);

        // When yaw acceleration limiting is enabled, the yaw input shaper constrains angular acceleration about the yaw axis, slewing
        // the output rate towards the input rate.
        _attitude_target_euler_rate.z = input_shaping_ang_vel(_attitude_target_euler_rate.z, euler_yaw_rate, euler_accel.z, _dt);

        // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        euler_rate_to_ang_vel(_attitude_target_euler_angle, _attitude_target_euler_rate, _attitude_target_ang_vel);
        // Limit the angular velocity
        ang_vel_limit(_attitude_target_ang_vel, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));
        // Convert body-frame angular velocity into euler angle derivative of desired attitude
        ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);
    } else {
        // When feedforward is not enabled, the target euler angle is input into the target and the feedforward rate is zeroed.
        _attitude_target_euler_angle.x = euler_roll_angle;
        _attitude_target_euler_angle.y = euler_pitch_angle;
        _attitude_target_euler_angle.z += euler_yaw_rate * _dt;
        // Compute quaternion target attitude
        _attitude_target_quat.from_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

        // Set rate feedforward requests to zero
        _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
        _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Command an euler roll, pitch and yaw angle with angular velocity feedforward and smoothing
// 通过roll，pitch和yaw角速度前馈和平滑控制欧拉角
// 前馈控制器控制量计算在此函数中完成
// 前三个输入参数：本次输入的期望姿态的roll,pitch,yaw角，可来自遥控器或地面站
void AC_AttitudeControl::input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw)
{
    // Convert from centidegrees on public interface to radians
    // 角度制转换成弧度制(乘以0.01是因为在输入这个函数之前乘以了100)
    float euler_roll_angle = radians(euler_roll_angle_cd * 0.01f);
    float euler_pitch_angle = radians(euler_pitch_angle_cd * 0.01f);
    float euler_yaw_angle = radians(euler_yaw_angle_cd * 0.01f);

    // calculate the attitude target euler angles
    // _attitude_target_quat表示为当前(未更新本次输入前)的期望姿态的四元数
    // 可以理解为操作员通过遥控器摇杆输入的期望姿态是实时变更的
    // 这个总函数(input_euler_xxx)的输入是本次输入的期望，而_attitude_target_quat中保存的是还未收到的本次输入进行更新前
    // 此次将其转化为欧拉角形式_attitude_target_euler_angle
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

    // Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
    // 添加侧倾调整以补充直升机尾部螺旋桨的推力(在多旋翼飞机上将返回0)
    euler_roll_angle += get_roll_trim_rad();

    if (_rate_bf_ff_enabled) {
        // translate the roll pitch and yaw acceleration limits to the euler axis
        // 如果开启了机体角速率前馈，注意下方将期望角度转换为期望角速率的操作表示的是FF前馈控制器的作用
        // FF前馈控制器根据期望姿态误差计算出期望姿态的期望角速率，其次是作为前馈控制量叠加到P控制器输出的当前姿态的期望角速率上

        // 将侧倾、俯仰和偏航的加速度限制转换到欧拉轴上
        // euler_accel_limit()接收当前期望姿态欧拉角和roll,pitch,yaw上的最大加速度
        // 以此计算出加速度限制，使增加加速度时不会超过任何一个轴的最大加速度
        Vector3f euler_accel = euler_accel_limit(_attitude_target_euler_angle, Vector3f(get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss()));

        // When acceleration limiting and feedforward are enabled, the sqrt controller is used to compute an euler
        // angular velocity that will cause the euler angle to smoothly stop at the input angle with limited deceleration
        // and an exponential decay specified by _input_tc at the end.
        // 根据本次输入的期望姿态和未更新输入的当前期望姿态的姿态角度误差计算期望欧拉角速率校正值_attitude_target_euler_rate
        // 以roll角为例
        // wrap_PI(euler_roll_angle - _attitude_target_euler_angle.x):计算本次输入期望roll角和当前期望姿态roll角之间的误差，并将其限制在[-PI,PI]
        // _input_tc:姿态控制输入时间常数，数字越小，响应越尖锐；数字越大，响应越柔和，默认设定为0.15
        // euler_accel.x:经限制后的欧拉角加速度
        // _attitude_target_euler_rate.x：当前期望姿态欧拉角速率
        // _dt:采样周期
        _attitude_target_euler_rate.x = input_shaping_angle(wrap_PI(euler_roll_angle - _attitude_target_euler_angle.x), _input_tc, euler_accel.x, _attitude_target_euler_rate.x, _dt);
        _attitude_target_euler_rate.y = input_shaping_angle(wrap_PI(euler_pitch_angle - _attitude_target_euler_angle.y), _input_tc, euler_accel.y, _attitude_target_euler_rate.y, _dt);
        _attitude_target_euler_rate.z = input_shaping_angle(wrap_PI(euler_yaw_angle - _attitude_target_euler_angle.z), _input_tc, euler_accel.z, _attitude_target_euler_rate.z, _dt);
        if (slew_yaw) {//如果开启了摆率限制，那么还要对期望欧拉角速率进行限制
            _attitude_target_euler_rate.z = constrain_float(_attitude_target_euler_rate.z, -get_slew_yaw_rads(), get_slew_yaw_rads());
        }

        // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        // 将期望姿态的欧拉角速率_attitude_target_euler_rate转换为前馈的期望机体角速度矢量_attitude_target_ang_vel
        // 欧拉姿态变化率(n系)转换为期望机体角速度(tb系)：euler_rate_to_ang_vel
        euler_rate_to_ang_vel(_attitude_target_euler_angle, _attitude_target_euler_rate, _attitude_target_ang_vel);
        // Limit the angular velocity
        // 角速度_attitude_target_ang_vel限幅
        ang_vel_limit(_attitude_target_ang_vel, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));
        // Convert body-frame angular velocity into euler angle derivative of desired attitude
        // 然后再将限幅后的期望机体角速度(tb系)重新转换为所需姿态的欧拉角速率(n系)
        ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);
        //最后得到的期望角速率_attitude_target_ang_vel和期望欧拉角速率_attitude_target_euler_rate是前馈速率，即前馈控制量
        //注意以上运算过程是在本次输入的期望姿态与当前期望姿态之间实现的，最后得到的期望角速率是针对于期望机体坐标系的
        //因此该期望角速率在使用前需要先转换到当前姿态下
        //_attitude_target_ang_vel:期望姿态下的期望角速率
    } else {
        // When feedforward is not enabled, the target euler angle is input into the target and the feedforward rate is zeroed.
        // 没有前馈控制时，则将本次目标欧拉角输入保存并将前馈速率归零
        _attitude_target_euler_angle.x = euler_roll_angle;
        _attitude_target_euler_angle.y = euler_pitch_angle;
        if (slew_yaw) {
            // Compute constrained angle error
            // 如果开启摆率控制，计算出本层次输入期望yaw角和当前期望yaw角之间的误差，并限幅
            float angle_error = constrain_float(wrap_PI(euler_yaw_angle - _attitude_target_euler_angle.z), -get_slew_yaw_rads() * _dt, get_slew_yaw_rads() * _dt);
            // Update attitude target from constrained angle error
            // 根据误差更新yaw角期望姿态
            _attitude_target_euler_angle.z = wrap_PI(angle_error + _attitude_target_euler_angle.z);
        } else {
            _attitude_target_euler_angle.z = euler_yaw_angle;//如果没有开启摆率控制，直接将本次输入保存为当前姿态yaw角
        }
        // Compute quaternion target attitude
        // 将期望姿态从欧拉角转换为四元数形式(n系下)
        // 在这里没有开启前馈，则会将本次输入的期望姿态更新到当前期望姿态，然后将前馈速率置0
        _attitude_target_quat.from_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

        // Set rate feedforward requests to zero
        // 将速率前馈设置0
        _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
        _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);
    }

    // Call quaternion attitude controller
    // 调用四元数姿态控制器(真正的P控制器在其中运行)
    // 根据姿态角误差计算出期望角速率
    attitude_controller_run_quat();
}

// Command euler pitch and yaw angles and roll rate (used only by tailsitter quadplanes)
// Multicopter style controls: roll stick is tailsitter bodyframe yaw in hover
void AC_AttitudeControl::input_euler_rate_yaw_euler_angle_pitch_bf_roll_m(float euler_yaw_rate_cds, float euler_pitch_cd, float body_roll_cd)
{
    // Convert from centidegrees on public interface to radians
    float euler_yaw_rate = radians(euler_yaw_rate_cds*0.01f);
    float euler_pitch = radians(constrain_float(euler_pitch_cd * 0.01f, -90.0f, 90.0f));
    float body_roll   = radians(constrain_float(body_roll_cd   * 0.01f, -90.0f, 90.0f));

    // Compute attitude error
    Quaternion attitude_vehicle_quat;
    Quaternion error_quat;
    attitude_vehicle_quat.from_rotation_matrix(_ahrs.get_rotation_body_to_ned());
    error_quat = attitude_vehicle_quat.inverse() * _attitude_target_quat;
    Vector3f att_error;
    error_quat.to_axis_angle(att_error);

    // limit yaw error
    if (fabsf(att_error.z) < AC_ATTITUDE_THRUST_ERROR_ANGLE) {
        // update heading
        _attitude_target_euler_angle.z = wrap_PI(_attitude_target_euler_angle.z + euler_yaw_rate * _dt);
    }

    // init attitude target to desired euler yaw and pitch with zero roll
    _attitude_target_quat.from_euler(0, euler_pitch, _attitude_target_euler_angle.z);

    const float cpitch = cosf(euler_pitch);
    const float spitch = fabsf(sinf(euler_pitch));

    // apply body-frame yaw/roll (this is roll/yaw for a tailsitter in forward flight)
    // rotate body_roll axis by |sin(pitch angle)|
    Quaternion bf_roll_Q;
    bf_roll_Q.from_axis_angle(Vector3f(0, 0, spitch * body_roll));

    // rotate body_yaw axis by cos(pitch angle)
    Quaternion bf_yaw_Q;
    bf_yaw_Q.from_axis_angle(Vector3f(-cpitch * body_roll, 0, 0));
    _attitude_target_quat = _attitude_target_quat * bf_roll_Q * bf_yaw_Q;

    // _attitude_target_euler_angle roll and pitch: Note: roll/yaw will be indeterminate when pitch is near +/-90
    // These should be used only for logging target eulers, with the caveat noted above.
    // Also note that _attitude_target_quat.from_euler() should only be used in special circumstances
    // such as when attitude is specified directly in terms of Euler angles.
    //    _attitude_target_euler_angle.x = _attitude_target_quat.get_euler_roll();
    //    _attitude_target_euler_angle.y = euler_pitch;

    // Set rate feedforward requests to zero
    _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
    _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);

    // Compute attitude error
    error_quat = attitude_vehicle_quat.inverse() * _attitude_target_quat;
    error_quat.to_axis_angle(att_error);

    // Compute the angular velocity target from the attitude error
    _rate_target_ang_vel = update_ang_vel_target_from_att_error(att_error);
}

// Command euler pitch and yaw angles and roll rate (used only by tailsitter quadplanes)
// Plane style controls: yaw stick is tailsitter bodyframe yaw in hover
void AC_AttitudeControl::input_euler_rate_yaw_euler_angle_pitch_bf_roll_p(float euler_yaw_rate_cds, float euler_pitch_cd, float body_roll_cd)
{
    // Convert from centidegrees on public interface to radians
    float euler_yaw_rate = radians(euler_yaw_rate_cds*0.01f);
    float euler_pitch = radians(constrain_float(euler_pitch_cd * 0.01f, -90.0f, 90.0f));
    float body_roll   = radians(constrain_float(body_roll_cd   * 0.01f, -90.0f, 90.0f));

    const float cpitch = cosf(euler_pitch);
    const float spitch = fabsf(sinf(euler_pitch));

    // Compute attitude error
    Quaternion attitude_vehicle_quat;
    Quaternion error_quat;
    attitude_vehicle_quat.from_rotation_matrix(_ahrs.get_rotation_body_to_ned());
    error_quat = attitude_vehicle_quat.inverse() * _attitude_target_quat;
    Vector3f att_error;
    error_quat.to_axis_angle(att_error);

    // limit yaw error
    if (fabsf(att_error.z) < AC_ATTITUDE_THRUST_ERROR_ANGLE) {
        // update heading
        float yaw_rate = euler_yaw_rate * spitch + body_roll * cpitch;
        _attitude_target_euler_angle.z = wrap_PI(_attitude_target_euler_angle.z + yaw_rate * _dt);
    }

    // init attitude target to desired euler yaw and pitch with zero roll
    _attitude_target_quat.from_euler(0, euler_pitch, _attitude_target_euler_angle.z);

    // apply body-frame yaw/roll (this is roll/yaw for a tailsitter in forward flight)
    // rotate body_roll axis by |sin(pitch angle)|
    Quaternion bf_roll_Q;
    bf_roll_Q.from_axis_angle(Vector3f(0, 0, spitch * body_roll));

    // rotate body_yaw axis by cos(pitch angle)
    Quaternion bf_yaw_Q;
    bf_yaw_Q.from_axis_angle(Vector3f(cpitch, 0, 0), euler_yaw_rate);
    _attitude_target_quat = _attitude_target_quat * bf_roll_Q * bf_yaw_Q;

    // _attitude_target_euler_angle roll and pitch: Note: roll/yaw will be indeterminate when pitch is near +/-90
    // These should be used only for logging target eulers, with the caveat noted above
    // Also note that _attitude_target_quat.from_euler() should only be used in special circumstances
    // such as when attitude is specified directly in terms of Euler angles.
    //    _attitude_target_euler_angle.x = _attitude_target_quat.get_euler_roll();
    //    _attitude_target_euler_angle.y = euler_pitch;

    // Set rate feedforward requests to zero
    _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
    _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);

    // Compute attitude error
    error_quat = attitude_vehicle_quat.inverse() * _attitude_target_quat;
    error_quat.to_axis_angle(att_error);

    // Compute the angular velocity target from the attitude error
    _rate_target_ang_vel = update_ang_vel_target_from_att_error(att_error);
}

// Command an euler roll, pitch, and yaw rate with angular velocity feedforward and smoothing
void AC_AttitudeControl::input_euler_rate_roll_pitch_yaw(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds)
{
    // Convert from centidegrees on public interface to radians
    float euler_roll_rate = radians(euler_roll_rate_cds * 0.01f);
    float euler_pitch_rate = radians(euler_pitch_rate_cds * 0.01f);
    float euler_yaw_rate = radians(euler_yaw_rate_cds * 0.01f);

    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

    if (_rate_bf_ff_enabled) {
        // translate the roll pitch and yaw acceleration limits to the euler axis
        Vector3f euler_accel = euler_accel_limit(_attitude_target_euler_angle, Vector3f(get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss()));

        // When acceleration limiting is enabled, the input shaper constrains angular acceleration, slewing
        // the output rate towards the input rate.
        _attitude_target_euler_rate.x = input_shaping_ang_vel(_attitude_target_euler_rate.x, euler_roll_rate, euler_accel.x, _dt);
        _attitude_target_euler_rate.y = input_shaping_ang_vel(_attitude_target_euler_rate.y, euler_pitch_rate, euler_accel.y, _dt);
        _attitude_target_euler_rate.z = input_shaping_ang_vel(_attitude_target_euler_rate.z, euler_yaw_rate, euler_accel.z, _dt);

        // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        euler_rate_to_ang_vel(_attitude_target_euler_angle, _attitude_target_euler_rate, _attitude_target_ang_vel);
    } else {
        // When feedforward is not enabled, the target euler angle is input into the target and the feedforward rate is zeroed.
        // Pitch angle is restricted to +- 85.0 degrees to avoid gimbal lock discontinuities.
        _attitude_target_euler_angle.x = wrap_PI(_attitude_target_euler_angle.x + euler_roll_rate * _dt);
        _attitude_target_euler_angle.y = constrain_float(_attitude_target_euler_angle.y + euler_pitch_rate * _dt, radians(-85.0f), radians(85.0f));
        _attitude_target_euler_angle.z = wrap_2PI(_attitude_target_euler_angle.z + euler_yaw_rate * _dt);

        // Set rate feedforward requests to zero
        _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
        _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);

        // Compute quaternion target attitude
        _attitude_target_quat.from_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Command an angular velocity with angular velocity feedforward and smoothing
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // Convert from centidegrees on public interface to radians
    float roll_rate_rads = radians(roll_rate_bf_cds * 0.01f);
    float pitch_rate_rads = radians(pitch_rate_bf_cds * 0.01f);
    float yaw_rate_rads = radians(yaw_rate_bf_cds * 0.01f);

    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

    if (_rate_bf_ff_enabled) {
        // Compute acceleration-limited body frame rates
        // When acceleration limiting is enabled, the input shaper constrains angular acceleration about the axis, slewing
        // the output rate towards the input rate.
        _attitude_target_ang_vel.x = input_shaping_ang_vel(_attitude_target_ang_vel.x, roll_rate_rads, get_accel_roll_max_radss(), _dt);
        _attitude_target_ang_vel.y = input_shaping_ang_vel(_attitude_target_ang_vel.y, pitch_rate_rads, get_accel_pitch_max_radss(), _dt);
        _attitude_target_ang_vel.z = input_shaping_ang_vel(_attitude_target_ang_vel.z, yaw_rate_rads, get_accel_yaw_max_radss(), _dt);

        // Convert body-frame angular velocity into euler angle derivative of desired attitude
        ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);
    } else {
        // When feedforward is not enabled, the quaternion is calculated and is input into the target and the feedforward rate is zeroed.
        Quaternion attitude_target_update_quat;
        attitude_target_update_quat.from_axis_angle(Vector3f(roll_rate_rads * _dt, pitch_rate_rads * _dt, yaw_rate_rads * _dt));
        _attitude_target_quat = _attitude_target_quat * attitude_target_update_quat;
        _attitude_target_quat.normalize();

        // Set rate feedforward requests to zero
        _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
        _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Command an angular velocity with angular velocity smoothing using rate loops only with no attitude loop stabilization
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_2(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // Convert from centidegrees on public interface to radians
    float roll_rate_rads = radians(roll_rate_bf_cds * 0.01f);
    float pitch_rate_rads = radians(pitch_rate_bf_cds * 0.01f);
    float yaw_rate_rads = radians(yaw_rate_bf_cds * 0.01f);

    // Compute acceleration-limited body frame rates
    // When acceleration limiting is enabled, the input shaper constrains angular acceleration about the axis, slewing
    // the output rate towards the input rate.
    _attitude_target_ang_vel.x = input_shaping_ang_vel(_attitude_target_ang_vel.x, roll_rate_rads, get_accel_roll_max_radss(), _dt);
    _attitude_target_ang_vel.y = input_shaping_ang_vel(_attitude_target_ang_vel.y, pitch_rate_rads, get_accel_pitch_max_radss(), _dt);
    _attitude_target_ang_vel.z = input_shaping_ang_vel(_attitude_target_ang_vel.z, yaw_rate_rads, get_accel_yaw_max_radss(), _dt);

    // Update the unused targets attitude based on current attitude to condition mode change
    _ahrs.get_quat_body_to_ned(_attitude_target_quat);
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);
    // Convert body-frame angular velocity into euler angle derivative of desired attitude
    ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);
    _rate_target_ang_vel = _attitude_target_ang_vel;
}

// Command an angular velocity with angular velocity smoothing using rate loops only with integrated rate error stabilization
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_3(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // Convert from centidegrees on public interface to radians
    float roll_rate_rads = radians(roll_rate_bf_cds * 0.01f);
    float pitch_rate_rads = radians(pitch_rate_bf_cds * 0.01f);
    float yaw_rate_rads = radians(yaw_rate_bf_cds * 0.01f);

    // Update attitude error
    Vector3f attitude_error_vector;
    _attitude_ang_error.to_axis_angle(attitude_error_vector);

    Quaternion attitude_ang_error_update_quat;
    // limit the integrated error angle
    float err_mag = attitude_error_vector.length();
    if (err_mag > AC_ATTITUDE_THRUST_ERROR_ANGLE) {
        attitude_error_vector *= AC_ATTITUDE_THRUST_ERROR_ANGLE / err_mag;
        _attitude_ang_error.from_axis_angle(attitude_error_vector);
    }

    Vector3f gyro_latest = _ahrs.get_gyro_latest();
    attitude_ang_error_update_quat.from_axis_angle(Vector3f((_attitude_target_ang_vel.x-gyro_latest.x) * _dt, (_attitude_target_ang_vel.y-gyro_latest.y) * _dt, (_attitude_target_ang_vel.z-gyro_latest.z) * _dt));
    _attitude_ang_error = attitude_ang_error_update_quat * _attitude_ang_error;

    // Compute acceleration-limited body frame rates
    // When acceleration limiting is enabled, the input shaper constrains angular acceleration about the axis, slewing
    // the output rate towards the input rate.
    _attitude_target_ang_vel.x = input_shaping_ang_vel(_attitude_target_ang_vel.x, roll_rate_rads, get_accel_roll_max_radss(), _dt);
    _attitude_target_ang_vel.y = input_shaping_ang_vel(_attitude_target_ang_vel.y, pitch_rate_rads, get_accel_pitch_max_radss(), _dt);
    _attitude_target_ang_vel.z = input_shaping_ang_vel(_attitude_target_ang_vel.z, yaw_rate_rads, get_accel_yaw_max_radss(), _dt);

    // Retrieve quaternion vehicle attitude
    Quaternion attitude_vehicle_quat;
    _ahrs.get_quat_body_to_ned(attitude_vehicle_quat);

    // Update the unused targets attitude based on current attitude to condition mode change
    _attitude_target_quat = attitude_vehicle_quat * _attitude_ang_error;

    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

    // Convert body-frame angular velocity into euler angle derivative of desired attitude
    ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);

    // Compute the angular velocity target from the integrated rate error
    _attitude_ang_error.to_axis_angle(attitude_error_vector);
    _rate_target_ang_vel = update_ang_vel_target_from_att_error(attitude_error_vector);
    _rate_target_ang_vel += _attitude_target_ang_vel;

    // ensure Quaternions stay normalized
    _attitude_ang_error.normalize();
}

// Command an angular step (i.e change) in body frame angle
// Used to command a step in angle without exciting the orthogonal axis during autotune
void AC_AttitudeControl::input_angle_step_bf_roll_pitch_yaw(float roll_angle_step_bf_cd, float pitch_angle_step_bf_cd, float yaw_angle_step_bf_cd)
{
    // Convert from centidegrees on public interface to radians
    float roll_step_rads = radians(roll_angle_step_bf_cd * 0.01f);
    float pitch_step_rads = radians(pitch_angle_step_bf_cd * 0.01f);
    float yaw_step_rads = radians(yaw_angle_step_bf_cd * 0.01f);

    // rotate attitude target by desired step
    Quaternion attitude_target_update_quat;
    attitude_target_update_quat.from_axis_angle(Vector3f(roll_step_rads, pitch_step_rads, yaw_step_rads));
    _attitude_target_quat = _attitude_target_quat * attitude_target_update_quat;
    _attitude_target_quat.normalize();

    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

    // Set rate feedforward requests to zero
    _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
    _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Calculates the body frame angular velocities to follow the target attitude
// 根据姿态误差计算得出期望角速率
void AC_AttitudeControl::attitude_controller_run_quat()
{
    // Retrieve quaternion vehicle attitude
    // 检索四元数车辆姿态
    // 获取到当前机体姿态的四元数并保存到attitude_vehicle_quat
    Quaternion attitude_vehicle_quat;
    _ahrs.get_quat_body_to_ned(attitude_vehicle_quat);

    // Compute attitude error
    // 计算姿态误差
    Vector3f attitude_error_vector;
    /*姿态误差的计算和倾转分离，该函数通过前两个输入的四元数参数计算得到各轴的姿态误差
      经过此函数后得到的参数均在b系下
      _thrust_error_angle：推力方向(z轴误差角)
     */
    thrust_heading_rotation_angles(_attitude_target_quat, attitude_vehicle_quat, attitude_error_vector, _thrust_error_angle);
    
    // Compute the angular velocity target from the attitude error
    // P控制器，输入为姿态误差，输出为期望角速率
    // 函数内部根据是否开启了sqrt_controller来区分计算各通道的期望角速率_rate_target_ang_vel
    // 是，则使用开方调整过的P控制器
    // 否，则使用常规的Kp*error
    _rate_target_ang_vel = update_ang_vel_target_from_att_error(attitude_error_vector);

    // Add feedforward term that attempts to ensure that roll and pitch errors rotate with the body frame rather than the reference frame.
    // todo: this should probably be a matrix that couples yaw as well.
    // 对P控制器输出叠加前馈项，尝试确保侧倾和俯仰误差随车身框架而不是参考框架旋转
    // x轴的前馈项 = y轴的姿态误差(限幅后的)*陀螺仪获取的z轴角速度
    // y轴的前馈项 = x轴的姿态误差(限幅后的)*陀螺仪获取的z轴角速度
    // 注意此处没有对Z轴的期望角速率进行处理
    _rate_target_ang_vel.x += constrain_float(attitude_error_vector.y, -M_PI / 4, M_PI / 4) * _ahrs.get_gyro().z;
    _rate_target_ang_vel.y += -constrain_float(attitude_error_vector.x, -M_PI / 4, M_PI / 4) * _ahrs.get_gyro().z;

    // 期望角速率限幅
    ang_vel_limit(_rate_target_ang_vel, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));

    // Add the angular velocity feedforward, rotated into vehicle frame
    // 角速度前馈旋转到机体坐标中
    // _attitude_target_ang_vel是一个全局变量，表示期望姿态的期望角速率，是由input_euler_angle_roll_pitch_yaw等类似函数获取到的前馈速率
    // attitude_target_ang_vel_quat在此处表明是一个向量[0,v]
    Quaternion attitude_target_ang_vel_quat = Quaternion(0.0f, _attitude_target_ang_vel.x, _attitude_target_ang_vel.y, _attitude_target_ang_vel.z);
    // 运算过程注意四元数和旋转矩阵一样是可以上下标相互约去的，此处以Q(a)(b)形式表明上标为a，下标为b，表示为b向a的旋转过程
    // attitude_vehicle_quat表示的是cb->n旋转的单位四元数，求逆后表示n->cb
    // to_to_from_quat = Q(cb)(n)*Q(n)(tb)=Q(cb)(tb)
    // 表明获取的四元数to_to_from_quat表示tb->cb的旋转，计算公式也可以视作期望姿态与当前姿态作差
    Quaternion to_to_from_quat = attitude_vehicle_quat.inverse() * _attitude_target_quat;

    //将tb下的期望角速率旋转至cb下获取当前姿态角速率期望前馈叠加量desired_ang_vel_quat
    Quaternion desired_ang_vel_quat = to_to_from_quat.inverse() * attitude_target_ang_vel_quat * to_to_from_quat;

    // Correct the thrust vector and smoothly add feedforward and yaw input
    // 校正推力矢量并平稳添加前馈和偏航输入
    // 下述过程根据推力角误差大小分配在Z轴YAW角上的数学计算结果和传感器测量值之间的确信度来确定Z轴的期望角速度
    // 同时将前馈控制量添加到各轴
    // 当推力角误差特别大的时候，表明计算出来的期望值不可信，此时Z轴期望角速度获取陀螺仪测量值
    _feedforward_scalar = 1.0f;
    if (_thrust_error_angle > AC_ATTITUDE_THRUST_ERROR_ANGLE * 2.0f) {
        _rate_target_ang_vel.z = _ahrs.get_gyro().z;
    } else if (_thrust_error_angle > AC_ATTITUDE_THRUST_ERROR_ANGLE) {
        _feedforward_scalar = (1.0f - (_thrust_error_angle - AC_ATTITUDE_THRUST_ERROR_ANGLE) / AC_ATTITUDE_THRUST_ERROR_ANGLE);
        _rate_target_ang_vel.x += desired_ang_vel_quat.q2 * _feedforward_scalar;
        _rate_target_ang_vel.y += desired_ang_vel_quat.q3 * _feedforward_scalar;
        _rate_target_ang_vel.z += desired_ang_vel_quat.q4;
        _rate_target_ang_vel.z = _ahrs.get_gyro().z * (1.0 - _feedforward_scalar) + _rate_target_ang_vel.z * _feedforward_scalar;
    } else {
        _rate_target_ang_vel.x += desired_ang_vel_quat.q2;
        _rate_target_ang_vel.y += desired_ang_vel_quat.q3;
        _rate_target_ang_vel.z += desired_ang_vel_quat.q4;
    }

    if (_rate_bf_ff_enabled) {
        // rotate target and normalize
        // 当开启机体速率前馈时，实时计算更新期望姿态四元数，每一次更新都需要将其单位化
        Quaternion attitude_target_update_quat;
        attitude_target_update_quat.from_axis_angle(Vector3f(_attitude_target_ang_vel.x * _dt, _attitude_target_ang_vel.y * _dt, _attitude_target_ang_vel.z * _dt));
        _attitude_target_quat = _attitude_target_quat * attitude_target_update_quat;//实时更新期望姿态四元数
        _attitude_target_quat.normalize();
    }

    // ensure Quaternions stay normalized
    // 确保四元数已经被单位化
    _attitude_target_quat.normalize();

    // Record error to handle EKF resets
    // 记录姿态误差以处理EKF重置
    // 等式相当于将_attitude_target_quat表示的期望姿态减去attitude_vehicle_quat表示的当前姿态
    _attitude_ang_error = attitude_vehicle_quat.inverse() * _attitude_target_quat;
}

// thrust_heading_rotation_angles - calculates two ordered rotations to move the att_from_quat quaternion to the att_to_quat quaternion.
// The first rotation corrects the thrust vector and the second rotation corrects the heading vector.
// 官方注释：计算两个有序旋转，以将att_from_quat四元数移动到att_to_quat四元数
// 第一个旋转校正推力矢量，第二个旋转校正航向矢量
/* att_to_quat：传入参数，期望姿态四元数，方向：tb->n
   att_from_quat:传入参数，当前姿态四元数，方向：cb->n，同时也表示从机体b系向n系的旋转
   att_diff_angle：传入传出参数，姿态误差
   thrust_vec_dot：传入传出参数，按轴角法进行Z轴对齐时的旋转角*/
void AC_AttitudeControl::thrust_heading_rotation_angles(Quaternion& att_to_quat, const Quaternion& att_from_quat, Vector3f& att_diff_angle, float& thrust_vec_dot)
{
    //获取姿态的期望Z轴和当前Z轴

    //从四元数att_to_quat计算出旋转矩阵att_to_rot_matrix
    //旋转矩阵att_to_rot_matrix用于将期望姿态机体系旋转到n系
    //并把期望姿态的Z轴转换到n系下的期望Z轴att_to_thrust_vec
    //旋转方向：tb->n
    Matrix3f att_to_rot_matrix; // rotation from the target body frame to the inertial frame.
    att_to_quat.rotation_matrix(att_to_rot_matrix);
    Vector3f att_to_thrust_vec = att_to_rot_matrix * Vector3f(0.0f, 0.0f, 1.0f);

    //从四元数att_from_quat计算出旋转矩阵att_from_rot_matrix
    //旋转矩阵att_from_rot_matrix用于将当前机体坐标系cb转换到n系
    //并将当前姿态的z轴转换到NED坐标系下的当前Z轴att_from_thrust_vec
    //旋转方向：cb->n
    Matrix3f att_from_rot_matrix; // rotation from the current body frame to the inertial frame.
    att_from_quat.rotation_matrix(att_from_rot_matrix);
    Vector3f att_from_thrust_vec = att_from_rot_matrix * Vector3f(0.0f, 0.0f, 1.0f);

    //下面通过轴角法和四元数配合，以Z轴误差计算tilt倾角误差
    //构建轴角(n系下)



    // the dot product is used to calculate the current lean angle for use of external functions
    _thrust_angle = acosf(constrain_float(Vector3f(0.0f,0.0f,1.0f) * att_from_thrust_vec,-1.0f,1.0f));

    // the cross product of the desired and target thrust vector defines the rotation vector
    //叉乘获取旋转向量(转轴)thrust_vec_cross(叉乘后不一定是单位向量)
    //转轴：thrust_vec_cross，旋转方向：att_from_thrust_vec -> att_to_thrust_vec
    Vector3f thrust_vec_cross = att_from_thrust_vec % att_to_thrust_vec;

    // the dot product is used to calculate the angle between the target and desired thrust vectors
    //点乘获取旋转角thrust_vec_dot
    //表示向量att_from_thrust_vec转向att_to_thrust_vec的夹角
    //由于均为单位向量因此直接求arcoss,之后限定为-1~1即可
    thrust_vec_dot = acosf(constrain_float(att_from_thrust_vec * att_to_thrust_vec, -1.0f, 1.0f));

    // Normalize the thrust rotation vector
    // 单位化旋转向量
    float thrust_vector_length = thrust_vec_cross.length();
    if (is_zero(thrust_vector_length) || is_zero(thrust_vec_dot)) {
        thrust_vec_cross = Vector3f(0, 0, 1);
        thrust_vec_dot = 0.0f;
    } else {
        thrust_vec_cross /= thrust_vector_length;
    }

    //轴角构造完毕，转轴为单位向量thrust_vec_cross，转角为thrust_vec_dot
    //旋转方向：n系下的向量att_from_thrust_vec(当前姿态Z轴)->att_to_thrust_vec(期望姿态Z轴)

    //下面将开始将Z轴对准，即通过Z轴误差获取倾角误差(不考虑Yaw转角)

    //轴角转换到四元数
    //四元数thrust_vec_correction_quat表示n系下以thrust_vec_cross为转轴，thrust_vec_dot为转角的旋转过程
    Quaternion thrust_vec_correction_quat;
    thrust_vec_correction_quat.from_axis_angle(thrust_vec_cross, thrust_vec_dot);

    // Rotate thrust_vec_correction_quat to the att_from frame
    //将倾角误差由n系转向机体坐标系b系
    //四元数att_from_quat表示的是由机体系b系向n系旋转的过程，n=q×b×inv(q)
    //b=inv(q)×n×q
    //此处q为单位四元数，因此其逆等于共轭
    //注意此处并非标准的四元数旋转描述，因为0不是0标量四元数，详见下方
    thrust_vec_correction_quat = att_from_quat.inverse() * thrust_vec_correction_quat * att_from_quat;//机体b系下的倾角误差获取完毕

    // calculate the remaining rotation required after thrust vector is rotated transformed to the att_from frame
    // 计算b系下的yaw转角误差
    // 注意：旋转矩阵B的逆*旋转矩阵A 等价于 旋转A与旋转B作差，由A减去B(但不是减法的意思，只是表明旋转姿态的差异)
    // 依次左乘：att_from_quat.inverse() * att_to_quat = 期望姿态 - 当前姿态 = 姿态总误差(total error)
    // thrust_vec_correction_quat.inverse()*total error = 姿态总误差 - 倾角误差(tilt error) = yaw 转角误差(torsion error)
    Quaternion yaw_vec_correction_quat = thrust_vec_correction_quat.inverse() * att_from_quat.inverse() * att_to_quat;

    // calculate the angle error in x and y.
    // 倾角误差四元数转换回轴角
    // 并通过att_diff_angle获取到roll和pitch上的误差角
    Vector3f rotation;
    thrust_vec_correction_quat.to_axis_angle(rotation);
    att_diff_angle.x = rotation.x;
    att_diff_angle.y = rotation.y;

    // calculate the angle error in z (x and y should be zero here).
    // 转角误差四元数转换回轴角
    // 通过att_diff_angle.z 获取到yaw上的转角(roll和pitch应该为0)
    yaw_vec_correction_quat.to_axis_angle(rotation);
    att_diff_angle.z = rotation.z;

    // 到此处获得了倾转的所有姿态误差，然而并没有对YAW角误差进行限制
    // Todo: Limit roll an pitch error based on output saturation and maximum error.

    // Limit Yaw Error based on maximum acceleration - Update to include output saturation and maximum error.
    // Currently the limit is based on the maximum acceleration using the linear part of the SQRT controller.
    // This should be updated to be based on an angle limit, saturation, or unlimited based on user defined parameters.
    // 基于最大加速度限制偏航误差-更新以包括输出饱和度和最大误差
    // 当前，该限制基于使用SQRT控制器的线性部分的最大加速度
    // 应该将其更新为基于角度限制，饱和度或基于用户定义的参数
    // 注：这里不明白
    if (!is_zero(_p_angle_yaw.kP()) && fabsf(att_diff_angle.z) > AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS / _p_angle_yaw.kP()) {
        att_diff_angle.z = constrain_float(wrap_PI(att_diff_angle.z), -AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS / _p_angle_yaw.kP(), AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS / _p_angle_yaw.kP());
        yaw_vec_correction_quat.from_axis_angle(Vector3f(0.0f, 0.0f, att_diff_angle.z));
        att_to_quat = att_from_quat * thrust_vec_correction_quat * yaw_vec_correction_quat;
    }
}

// calculates the velocity correction from an angle error. The angular velocity has acceleration and
// deceleration limits including basic jerk limiting using _input_tc
float AC_AttitudeControl::input_shaping_angle(float error_angle, float input_tc, float accel_max, float target_ang_vel, float dt)
{
    // Calculate the velocity as error approaches zero with acceleration limited by accel_max_radss
    // sqrt_controller作为调整后的P控制器，根据接收到的角度误差计算出期望角速率
    // Kp=1.0f / MAX(input_tc, 0.01f)
    float desired_ang_vel = sqrt_controller(error_angle, 1.0f / MAX(input_tc, 0.01f), accel_max, dt);

    // Acceleration is limited directly to smooth the beginning of the curve.
    // 直接限制加速度以平滑曲线的起点
    return input_shaping_ang_vel(target_ang_vel, desired_ang_vel, accel_max, dt);
}

// limits the acceleration and deceleration of a velocity request
// 限制速度请求的加速和减速  
float AC_AttitudeControl::input_shaping_ang_vel(float target_ang_vel, float desired_ang_vel, float accel_max, float dt)
{
    // Acceleration is limited directly to smooth the beginning of the curve.
    if (is_positive(accel_max)) {
        float delta_ang_vel = accel_max * dt;
        return constrain_float(desired_ang_vel, target_ang_vel - delta_ang_vel, target_ang_vel + delta_ang_vel);
    } else {
        return desired_ang_vel;
    }
}

// calculates the expected angular velocity correction from an angle error based on the AC_AttitudeControl settings.
// This function can be used to predict the delay associated with angle requests.
void AC_AttitudeControl::input_shaping_rate_predictor(const Vector2f &error_angle, Vector2f& target_ang_vel, float dt) const
{
    if (_rate_bf_ff_enabled) {
        // translate the roll pitch and yaw acceleration limits to the euler axis
        target_ang_vel.x = input_shaping_angle(wrap_PI(error_angle.x), _input_tc, get_accel_roll_max_radss(), target_ang_vel.x, dt);
        target_ang_vel.y = input_shaping_angle(wrap_PI(error_angle.y), _input_tc, get_accel_pitch_max_radss(), target_ang_vel.y, dt);
    } else {
        target_ang_vel.x = _p_angle_roll.get_p(wrap_PI(error_angle.x));
        target_ang_vel.y = _p_angle_pitch.get_p(wrap_PI(error_angle.y));
    }
    // Limit the angular velocity correction
    Vector3f ang_vel(target_ang_vel.x, target_ang_vel.y, 0.0f);
    ang_vel_limit(ang_vel, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), 0.0f);

    target_ang_vel.x = ang_vel.x;
    target_ang_vel.y = ang_vel.y;
}

// limits angular velocity
void AC_AttitudeControl::ang_vel_limit(Vector3f& euler_rad, float ang_vel_roll_max, float ang_vel_pitch_max, float ang_vel_yaw_max) const
{
    if (is_zero(ang_vel_roll_max) || is_zero(ang_vel_pitch_max)) {
        if (!is_zero(ang_vel_roll_max)) {
            euler_rad.x = constrain_float(euler_rad.x, -ang_vel_roll_max, ang_vel_roll_max);
        }
        if (!is_zero(ang_vel_pitch_max)) {
            euler_rad.y = constrain_float(euler_rad.y, -ang_vel_pitch_max, ang_vel_pitch_max);
        }
    } else {
        Vector2f thrust_vector_ang_vel(euler_rad.x / ang_vel_roll_max, euler_rad.y / ang_vel_pitch_max);
        float thrust_vector_length = thrust_vector_ang_vel.length();
        if (thrust_vector_length > 1.0f) {
            euler_rad.x = thrust_vector_ang_vel.x * ang_vel_roll_max / thrust_vector_length;
            euler_rad.y = thrust_vector_ang_vel.y * ang_vel_pitch_max / thrust_vector_length;
        }
    }
    if (!is_zero(ang_vel_yaw_max)) {
        euler_rad.z = constrain_float(euler_rad.z, -ang_vel_yaw_max, ang_vel_yaw_max);
    }
}

// translates body frame acceleration limits to the euler axis
Vector3f AC_AttitudeControl::euler_accel_limit(const Vector3f &euler_rad, const Vector3f &euler_accel)
{
    float sin_phi = constrain_float(fabsf(sinf(euler_rad.x)), 0.1f, 1.0f);
    float cos_phi = constrain_float(fabsf(cosf(euler_rad.x)), 0.1f, 1.0f);
    float sin_theta = constrain_float(fabsf(sinf(euler_rad.y)), 0.1f, 1.0f);

    Vector3f rot_accel;
    if (is_zero(euler_accel.x) || is_zero(euler_accel.y) || is_zero(euler_accel.z) || is_negative(euler_accel.x) || is_negative(euler_accel.y) || is_negative(euler_accel.z)) {
        rot_accel.x = euler_accel.x;
        rot_accel.y = euler_accel.y;
        rot_accel.z = euler_accel.z;
    } else {
        rot_accel.x = euler_accel.x;
        rot_accel.y = MIN(euler_accel.y / cos_phi, euler_accel.z / sin_phi);
        rot_accel.z = MIN(MIN(euler_accel.x / sin_theta, euler_accel.y / sin_phi), euler_accel.z / cos_phi);
    }
    return rot_accel;
}

// Shifts earth frame yaw target by yaw_shift_cd. yaw_shift_cd should be in centidegrees and is added to the current target heading
void AC_AttitudeControl::shift_ef_yaw_target(float yaw_shift_cd)
{
    float yaw_shift = radians(yaw_shift_cd * 0.01f);
    Quaternion _attitude_target_update_quat;
    _attitude_target_update_quat.from_axis_angle(Vector3f(0.0f, 0.0f, yaw_shift));
    _attitude_target_quat = _attitude_target_update_quat * _attitude_target_quat;
}

// Shifts earth frame yaw target by yaw_shift_cd. yaw_shift_cd should be in centidegrees and is added to the current target heading
void AC_AttitudeControl::inertial_frame_reset()
{
    // Retrieve quaternion vehicle attitude
    Quaternion attitude_vehicle_quat;
    _ahrs.get_quat_body_to_ned(attitude_vehicle_quat);

    // Recalculate the target quaternion
    _attitude_target_quat = attitude_vehicle_quat * _attitude_ang_error;

    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);
}

// Convert a 321-intrinsic euler angle derivative to an angular velocity vector
// euler_rad：为当前时刻的欧拉角，euler_rate_rads：为n系上的姿态欧拉角速率，ang_vel_rads：为b系上的角速度
/*说明：前面计算出来的前馈速率euler_rate_rads是基于期望姿态的，也就是根据本次输入期望姿态和当下期望姿态计算的，在使用前，要将其转化到当前姿态坐标系下
 *而attitude_controller_run_quat中的P控制器则是通过当下期望姿态_attitude_target_quat与当前姿态的误差计算得到期望角速度的
*/
void AC_AttitudeControl::euler_rate_to_ang_vel(const Vector3f& euler_rad, const Vector3f& euler_rate_rads, Vector3f& ang_vel_rads)
{
    float sin_theta = sinf(euler_rad.y);
    float cos_theta = cosf(euler_rad.y);
    float sin_phi = sinf(euler_rad.x);
    float cos_phi = cosf(euler_rad.x);

    ang_vel_rads.x = euler_rate_rads.x - sin_theta * euler_rate_rads.z;
    ang_vel_rads.y = cos_phi * euler_rate_rads.y + sin_phi * cos_theta * euler_rate_rads.z;
    ang_vel_rads.z = -sin_phi * euler_rate_rads.y + cos_theta * cos_phi * euler_rate_rads.z;
}

// Convert an angular velocity vector to a 321-intrinsic euler angle derivative
// Returns false if the vehicle is pitched 90 degrees up or down
bool AC_AttitudeControl::ang_vel_to_euler_rate(const Vector3f& euler_rad, const Vector3f& ang_vel_rads, Vector3f& euler_rate_rads)
{
    float sin_theta = sinf(euler_rad.y);
    float cos_theta = cosf(euler_rad.y);
    float sin_phi = sinf(euler_rad.x);
    float cos_phi = cosf(euler_rad.x);

    // When the vehicle pitches all the way up or all the way down, the euler angles become discontinuous. In this case, we just return false.
    if (is_zero(cos_theta)) {
        return false;
    }

    euler_rate_rads.x = ang_vel_rads.x + sin_phi * (sin_theta / cos_theta) * ang_vel_rads.y + cos_phi * (sin_theta / cos_theta) * ang_vel_rads.z;
    euler_rate_rads.y = cos_phi * ang_vel_rads.y - sin_phi * ang_vel_rads.z;
    euler_rate_rads.z = (sin_phi / cos_theta) * ang_vel_rads.y + (cos_phi / cos_theta) * ang_vel_rads.z;
    return true;
}

// Update rate_target_ang_vel using attitude_error_rot_vec_rad
// 使用姿态误差来更新角速率
Vector3f AC_AttitudeControl::update_ang_vel_target_from_att_error(const Vector3f &attitude_error_rot_vec_rad)
{
    Vector3f rate_target_ang_vel;
    // Compute the roll angular velocity demand from the roll angle error
    if (_use_sqrt_controller) {//如果使用了开方控制器
        rate_target_ang_vel.x = sqrt_controller(attitude_error_rot_vec_rad.x, _p_angle_roll.kP(), constrain_float(get_accel_roll_max_radss() / 2.0f, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS), _dt);
    } else {
        rate_target_ang_vel.x = _p_angle_roll.kP() * attitude_error_rot_vec_rad.x;
    }
    // todo: Add Angular Velocity Limit

    // Compute the pitch angular velocity demand from the pitch angle error
    if (_use_sqrt_controller) {
        rate_target_ang_vel.y = sqrt_controller(attitude_error_rot_vec_rad.y, _p_angle_pitch.kP(), constrain_float(get_accel_pitch_max_radss() / 2.0f, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS), _dt);
    } else {
        rate_target_ang_vel.y = _p_angle_pitch.kP() * attitude_error_rot_vec_rad.y;
    }
    // todo: Add Angular Velocity Limit

    // Compute the yaw angular velocity demand from the yaw angle error
    if (_use_sqrt_controller) {
        rate_target_ang_vel.z = sqrt_controller(attitude_error_rot_vec_rad.z, _p_angle_yaw.kP(), constrain_float(get_accel_yaw_max_radss() / 2.0f, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS), _dt);
    } else {
        rate_target_ang_vel.z = _p_angle_yaw.kP() * attitude_error_rot_vec_rad.z;
    }
    // todo: Add Angular Velocity Limit
    return rate_target_ang_vel;
}

// Enable or disable body-frame feed forward
void AC_AttitudeControl::accel_limiting(bool enable_limits)
{
    if (enable_limits) {
        // If enabling limits, reload from eeprom or set to defaults
        if (is_zero(_accel_roll_max)) {
            _accel_roll_max.load();
        }
        if (is_zero(_accel_pitch_max)) {
            _accel_pitch_max.load();
        }
        if (is_zero(_accel_yaw_max)) {
            _accel_yaw_max.load();
        }
    } else {
        _accel_roll_max = 0.0f;
        _accel_pitch_max = 0.0f;
        _accel_yaw_max = 0.0f;
    }
}

// Return tilt angle limit for pilot input that prioritises altitude hold over lean angle
float AC_AttitudeControl::get_althold_lean_angle_max() const
{
    // convert to centi-degrees for public interface
    return MAX(ToDeg(_althold_lean_angle_max), AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN) * 100.0f;
}

// Proportional controller with piecewise sqrt sections to constrain second derivative
/*用分段根号段约束二阶导数的比例控制器 
 *输入参数error         :误差量
 *输入参数p             :Kp值
 *输入参数second_ord_lim:二阶限制，acc_max
 *输入参数dt            ：时间
 *注：P开方控制器
*/
float AC_AttitudeControl::sqrt_controller(float error, float p, float second_ord_lim, float dt)
{
    float correction_rate;
    if (is_negative(second_ord_lim) || is_zero(second_ord_lim)) {
        // second order limit is zero or negative.
        correction_rate = error * p;/*二阶限制(加速度)是否为负或者为0，是的话开方控制器不起作用，常规方法算*/
    } else if (is_zero(p)) {
        // P term is zero but we have a second order limit./*Kp = 0,采用如下方法算二阶限制*/
        if (is_positive(error)) {/*如果误差为正*/
            correction_rate = safe_sqrt(2.0f * second_ord_lim * (error));
        } else if (is_negative(error)) {/*如果误差为负*/
            correction_rate = -safe_sqrt(2.0f * second_ord_lim * (-error));
        } else {/*如果误差为0*/
            correction_rate = 0.0f;
        }
    } else {
        // Both the P and second order limit have been defined./*同时定义了Kp和二阶限制*/
        float linear_dist = second_ord_lim / sq(p);/*分界点的计算*/
        if (error > linear_dist) {/*根据误差error与分界点的关系采用不同的公式*/
            correction_rate = safe_sqrt(2.0f * second_ord_lim * (error - (linear_dist / 2.0f)));
        } else if (error < -linear_dist) {
            correction_rate = -safe_sqrt(2.0f * second_ord_lim * (-error - (linear_dist / 2.0f)));
        } else {
            correction_rate = error * p;
        }
    }
    if (!is_zero(dt)) {
        // this ensures we do not get small oscillations by over shooting the error correction in the last time step.
        /*这确保了我们不会因为在最后一个时间步骤中超过了错误校正而得到小的振荡*/
        return constrain_float(correction_rate, -fabsf(error) / dt, fabsf(error) / dt);/*对速度进行限幅*/
    } else {
        return correction_rate;
    }
}

// Inverse proportional controller with piecewise sqrt sections to constrain second derivative
float AC_AttitudeControl::stopping_point(float first_ord_mag, float p, float second_ord_lim)
{
    if (is_positive(second_ord_lim) && !is_zero(second_ord_lim) && is_zero(p)) {
        return (first_ord_mag * first_ord_mag) / (2.0f * second_ord_lim);
    } else if ((is_negative(second_ord_lim) || is_zero(second_ord_lim)) && !is_zero(p)) {
        return first_ord_mag / p;
    } else if ((is_negative(second_ord_lim) || is_zero(second_ord_lim)) && is_zero(p)) {
        return 0.0f;
    }

    // calculate the velocity at which we switch from calculating the stopping point using a linear function to a sqrt function
    float linear_velocity = second_ord_lim / p;

    if (fabsf(first_ord_mag) < linear_velocity) {
        // if our current velocity is below the cross-over point we use a linear function
        return first_ord_mag / p;
    } else {
        float linear_dist = second_ord_lim / sq(p);
        float overshoot = (linear_dist * 0.5f) + sq(first_ord_mag) / (2.0f * second_ord_lim);
        if (is_positive(first_ord_mag)) {
            return overshoot;
        } else {
            return -overshoot;
        }
    }
}

// Return roll rate step size in centidegrees/s that results in maximum output after 4 time steps
float AC_AttitudeControl::max_rate_step_bf_roll()
{
    float alpha = MIN(get_rate_roll_pid().get_filt_E_alpha(), get_rate_roll_pid().get_filt_D_alpha());
    float alpha_remaining = 1 - alpha;
    // todo: When a thrust_max is available we should replace 0.5f with 0.5f * _motors.thrust_max
    float throttle_hover = constrain_float(_motors.get_throttle_hover(), 0.1f, 0.5f);
    return 2.0f * throttle_hover * AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX / ((alpha_remaining * alpha_remaining * alpha_remaining * alpha * get_rate_roll_pid().kD()) / _dt + get_rate_roll_pid().kP());
}

// Return pitch rate step size in centidegrees/s that results in maximum output after 4 time steps
float AC_AttitudeControl::max_rate_step_bf_pitch()
{
    float alpha = MIN(get_rate_pitch_pid().get_filt_E_alpha(), get_rate_pitch_pid().get_filt_D_alpha());
    float alpha_remaining = 1 - alpha;
    // todo: When a thrust_max is available we should replace 0.5f with 0.5f * _motors.thrust_max
    float throttle_hover = constrain_float(_motors.get_throttle_hover(), 0.1f, 0.5f);
    return 2.0f * throttle_hover * AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX / ((alpha_remaining * alpha_remaining * alpha_remaining * alpha * get_rate_pitch_pid().kD()) / _dt + get_rate_pitch_pid().kP());
}

// Return yaw rate step size in centidegrees/s that results in maximum output after 4 time steps
float AC_AttitudeControl::max_rate_step_bf_yaw()
{
    float alpha = MIN(get_rate_yaw_pid().get_filt_E_alpha(), get_rate_yaw_pid().get_filt_D_alpha());
    float alpha_remaining = 1 - alpha;
    // todo: When a thrust_max is available we should replace 0.5f with 0.5f * _motors.thrust_max
    float throttle_hover = constrain_float(_motors.get_throttle_hover(), 0.1f, 0.5f);
    return 2.0f * throttle_hover * AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX / ((alpha_remaining * alpha_remaining * alpha_remaining * alpha * get_rate_yaw_pid().kD()) / _dt + get_rate_yaw_pid().kP());
}

bool AC_AttitudeControl::pre_arm_checks(const char *param_prefix,
                                        char *failure_msg,
                                        const uint8_t failure_msg_len)
{
    // validate AC_P members:
    const struct {
        const char *pid_name;
        AC_P &p;
    } ps[] = {
        { "ANG_PIT", get_angle_pitch_p() },
        { "ANG_RLL", get_angle_roll_p() },
        { "ANG_YAW", get_angle_yaw_p() }
    };
    for (uint8_t i=0; i<ARRAY_SIZE(ps); i++) {
        // all AC_P's must have a positive P value:
        if (!is_positive(ps[i].p.kP())) {
            hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_P must be > 0", param_prefix, ps[i].pid_name);
            return false;
        }
    }

    // validate AC_PID members:
    const struct {
        const char *pid_name;
        AC_PID &pid;
    } pids[] = {
        { "RAT_RLL", get_rate_roll_pid() },
        { "RAT_PIT", get_rate_pitch_pid() },
        { "RAT_YAW", get_rate_yaw_pid() },
    };
    for (uint8_t i=0; i<ARRAY_SIZE(pids); i++) {
        // if the PID has a positive FF then we just ensure kP and
        // kI aren't negative
        AC_PID &pid = pids[i].pid;
        const char *pid_name = pids[i].pid_name;
        if (is_positive(pid.ff())) {
            // kP and kI must be non-negative:
            if (is_negative(pid.kP())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_P must be >= 0", param_prefix, pid_name);
                return false;
            }
            if (is_negative(pid.kI())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_I must be >= 0", param_prefix, pid_name);
                return false;
            }
        } else {
            // kP and kI must be positive:
            if (!is_positive(pid.kP())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_P must be > 0", param_prefix, pid_name);
                return false;
            }
            if (!is_positive(pid.kI())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_I must be > 0", param_prefix, pid_name);
                return false;
            }
        }
        // never allow a negative D term (but zero is allowed)
        if (is_negative(pid.kD())) {
            hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_D must be >= 0", param_prefix, pid_name);
            return false;
        }
    }
    return true;
}
