#include "Copter.h"

#if MODE_DRAWSTAR_ENABLED == ENABLED

/*五角星航线模式初始化，即通过地面站或遥控器设置切换到五角星航线模式后，会先进行对应初始化
  初始化航线对应的点，然后进入位置控制的初始化过程
*/
bool ModeDrawStar::init(bool ignore_checks)
{
    
    if(copter.position_ok() || ignore_checks){
     auto_yaw.set_mode_to_default(false);
      
      /*初始化路径点当前数目*/
      path_num = 0;
      /*生成路径点*/
      generate_path();
     
      // start in position control mode
      pos_control_start();
      return true;
    }else{
      return false;
    }
   
}

/*五角星航线模式生成路线*/
void ModeDrawStar::generate_path()
{
  /*五角星内切圆的半径值*/
  float radius_cm = g2.star_radius_cm;

  /*导航库获取第一个停止点，即0号点，这里是在模式发生切换前的该点*/
  wp_nav->get_wp_stopping_point(path[0]);  

  /*在0号点的基础上生成其他点的坐标*/
  path[1] = path[0] + Vector3f(1.0f,0,0)*radius_cm;
  path[2] = path[0] + Vector3f(-cosf(radians(36.0f)),-sinf((radians(36.0f))),0)*radius_cm;
  path[3] = path[0] + Vector3f(sinf(radians(18.0f)),cosf((radians(18.0f))),0)*radius_cm;
  path[4] = path[0] + Vector3f(sinf(radians(18.0f)),-cosf((radians(18.0f))),0)*radius_cm;
  path[5] = path[0] + Vector3f(-cosf(radians(36.0f)),sinf((radians(36.0f))),0)*radius_cm;
  path[6] = path[1];

}

// initialise drawstar mode's position controller
void ModeDrawStar::pos_control_start()
{
    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(path[0], false);

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

// guided_run - runs the guided controller
// should be called at 100hz or more
/*这里在fast_loop里被update_flight_mode 以400Hz的频率调用*/
void ModeDrawStar::run()
{
  /*判断是否到达对应的路径点，同时是否完成对应路径点飞行，如果完成，切换进入LOITER模式*/
  if (path_num < 6){
     /*判断是否到达对应的路径点，然后设置下一路径点*/
    if (wp_nav->reached_wp_destination()){
      path_num++;
      wp_nav->set_wp_destination(path[path_num], false);
    }
  } /*判断飞行是否完成，然后切换模式*/
  else if ((path_num == 6) && wp_nav->reached_wp_destination()){
    gcs().send_text(MAV_SEVERITY_CRITICAL,"Draw star finished,now go into Loiter Mode.");
    copter.set_mode(Mode::Number::LOITER,ModeReason::MODE_REASON_MISSION_END);
  }
   /*开启位置控制模式飞行*/
  pos_control_run();
}

// DrawStar_pos_control_run - runs the DrawStar position controller
// called from guided_run
void ModeDrawStar::pos_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}
#endif
