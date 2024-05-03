#include "Copter.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h> // 这里加入了头文件

/*
 * Init and run calls for hang flight mode
 * changed from althold, flight mode
 * writer lqd lz
*/

// althold_init - initialise althold controller
bool ModeHang::init(bool ignore_checks)
{
    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    return true;
}

// get the deg of sensor in centi-degs and add to target
void ModeHang::get_add_sensor_deg(float &roll_targ, float &pitch_targ)
{
    // Check for throttle failsafe or receiver present
    if (copter.failsafe.radio || !copter.ap.rc_receiver_present) {
        roll_targ = 0.0;
        pitch_targ = 0.0;
        return;
    }

    // Read sensor data (assuming you have initialized and read sensor data using AP_HAL)
    float roll_sensor_deg = AP_HAL::ToDeg(AP::ahal->sensors->get_roll());
    float pitch_sensor_deg = AP_HAL::ToDeg(AP::ahal->sensors->get_pitch());

    // Add sensor data to target roll and pitch 已经改成了串口读取的数据
    roll_targ += roll_sensor_deg;
    pitch_targ += pitch_sensor_deg;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeHang::run()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();
    // gcs().send_text(MAV_SEVERITY_CRITICAL,"mode hang fly");  //地面站消息发送
    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    //for sensor
    //in centi-degrees
    // target_roll +=5;
    // target_pitch +=5;

    //add the deg from sensor
    get_add_sensor_deg(target_roll,target_pitch);

    // target_roll, target_pitch 作为目标姿态输入，默认遥控中位就是飞机水平
    // call attitude controller 这是姿态，不是海拔！！！
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}
