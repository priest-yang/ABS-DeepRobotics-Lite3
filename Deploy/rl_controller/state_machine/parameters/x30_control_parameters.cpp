#include "control_parameters.h"

void ControlParameters::GenerateX30Parameters(){
    body_len_x_ = 0.292;
    body_len_y_ = 0.08;
    hip_len_ = 0.11642;
    thigh_len_ = 0.3;
    shank_len_ = 0.31;
    swing_leg_kp_ << 500., 500., 500.;
    swing_leg_kd_ << 5., 5., 5.;

    fl_joint_lower_ << -0.330, -2.97, 0.349;
    fl_joint_upper_ << 0.590, 0.260, 2.60;
    joint_vel_limit_ << 17.5, 17.5, 16.1;
    torque_limit_ << 84, 84, 180.;

    common_policy_path_ = "/home/shaoze/Documents/DeepRobotics/rl_controller/state_machine/policy/x30_common.pt";
    common_policy_p_gain_ << 120., 120., 150.;
    common_policy_d_gain_ << 3., 3., 3.5;
}