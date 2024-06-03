#include "control_parameters.h"

void ControlParameters::GenerateLite3Parameters(){
    swing_leg_kp_ << 100., 100., 100.;
    swing_leg_kd_ << 2.5, 2.5, 2.5;

    fl_joint_lower_ << -0.530, -2.97, 0.349;
    fl_joint_upper_ << 0.530, 0.320, 2.80;
    joint_vel_limit_ << 30, 30, 20;
    torque_limit_ << 40, 40, 65;

    common_policy_path_ = "/home/ysc/wz/rl_controller/state_machine/policy/05_16_09-22-09_model_4000.pt";

    //wz
    // common_policy_path_ = "/home/ysc/wz/rl_controller/state_machine/policy/wz.pt";

    common_policy_p_gain_ << 30., 30., 30.;
    common_policy_d_gain_ << 0.8, 0.8, 0.8;
}
