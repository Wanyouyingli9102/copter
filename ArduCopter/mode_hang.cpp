#include "Copter.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

class ModeHang : public Mode {
public:
    ModeHang(Copter &copter) : Mode(copter) {}

    bool init(bool ignore_checks) override;
    void run() override;
    void get_add_sensor_deg(float &roll_targ, float &pitch_targ);

private:
    AP_HAL::UARTDriver *arduino_serial;
};

bool ModeHang::init(bool ignore_checks) {
    if (!copter.pos_control->is_active_z()) {
        copter.pos_control->init_z_controller();
    }
    copter.pos_control->set_max_speed_accel_z(-copter.get_pilot_speed_dn(), copter.g.pilot_speed_up, copter.g.pilot_accel_z);
    copter.pos_control->set_correction_speed_accel_z(-copter.get_pilot_speed_dn(), copter.g.pilot_speed_up, copter.g.pilot_accel_z);

    // 初始化串口
    arduino_serial = hal.serial(0);  // 使用第一个串口接口（例如UART E接口）
    arduino_serial->begin(57600);  // 设置波特率为57600
    
    return true;
}

void ModeHang::get_add_sensor_deg(float &roll_targ, float &pitch_targ) {
    if (copter.failsafe.radio || !copter.ap.rc_receiver_present) {
        roll_targ = 0.0;
        pitch_targ = 0.0;
        return;
    }

    // 使用AP_AHRS获取当前飞机的姿态角度
    const AP_AHRS &local_ahrs = AP::ahrs();
    float roll_sensor_deg = degrees(local_ahrs.roll);
    float pitch_sensor_deg = degrees(local_ahrs.pitch);

    // 增加从传感器读取的角度到目标角度
    roll_targ += roll_sensor_deg;
    pitch_targ += pitch_sensor_deg;
}

void ModeHang::run() {
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    update_simple_mode();

    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());

    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    switch (althold_state) {
        case AltHold_MotorStopped:
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->reset_yaw_target_and_rate(false);
            pos_control->relax_z_controller(0.0f);
            break;
        case AltHold_Landed_Ground_Idle:
            attitude_control->reset_yaw_target_and_rate();
            [[fallthrough]];
        case AltHold_Landed_Pre_Takeoff:
            attitude_control->reset_rate_controller_I_terms_smoothly();
            pos_control->relax_z_controller(0.0f);
            break;
        case AltHold_Takeoff:
            if (!takeoff.running()) {
                takeoff.start(constrain_float(g.pilot_takeoff_alt, 0.0f, 1000.0f));
            }
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
            takeoff.do_pilot_takeoff(target_climb_rate);
            break;
        case AltHold_Flying:
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
            #if AC_AVOID_ENABLED == ENABLED
            copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
            #endif
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
            copter.surface_tracking.update_surface_offset();
            pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
            break;
    }

    // 从Arduino串口读取传感器数据
    if (arduino_serial->available()) {
        char buffer[50];
        size_t len = arduino_serial->read(buffer, sizeof(buffer) - 1);
        buffer[len] = '\0'; // 确保字符串以null结尾
        String data = buffer;

        // 假设数据格式为 "X角度: <值> Y角度: <值>"
        float x_angle = data.substring(data.indexOf("X角度: ") + 6, data.indexOf(" Y角度: ")).toFloat();
        float y_angle = data.substring(data.indexOf("Y角度: ") + 6).toFloat();
        
        // 使用角度数据
        target_roll += x_angle;
        target_pitch += y_angle;
    }

    get_add_sensor_deg(target_roll, target_pitch);
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
    pos_control->update_z_controller();
}
