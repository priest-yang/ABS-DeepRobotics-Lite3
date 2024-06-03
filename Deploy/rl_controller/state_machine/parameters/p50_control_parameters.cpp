#include "control_parameters.h"

void ControlParameters::GenerateP50Parameters(){
    body_len_x_ = 0.2955;
    body_len_y_ = 0.088;
    hip_len_ = 0.142;
    thigh_len_ = 0.35;
    shank_len_ = 0.35;
    swing_leg_kp_ << 1000., 1000., 1000.;
    swing_leg_kd_ << 5., 5., 5.;

    fl_joint_lower_ << -0.330, -2.97, 0.349;
    fl_joint_upper_ << 0.590, 0.260, 2.60;
    joint_vel_limit_ << 17.5, 17.5, 16.1;
    torque_limit_ << 84, 84, 180.;

    common_policy_path_ = "/home/shaoze/Documents/DeepRobotics/rl_controller/state_machine/policy/policy_waq_v7.11-5history-----.pt";
    common_policy_p_gain_ << 350.0, 350.0, 480.0;
    common_policy_d_gain_ << 6., 6. , 8.;

    speed_policy_path_ = "/home/shaoze/Documents/DeepRobotics/rl_controller/state_machine/policy/policy_est_p50_v2.0.3.pt";
    speed_policy_p_gain_ << 400.0, 400.0, 500.0;
    speed_policy_d_gain_ << 3., 3. , 3.;
}