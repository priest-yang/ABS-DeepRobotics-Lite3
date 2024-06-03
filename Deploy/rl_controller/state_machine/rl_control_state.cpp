#include "rl_control_state.h"

void RLControlState::CommonPolicyInitialize(){
    const std::string p = cp_ptr_->common_policy_path_;
    common_policy_ = std::make_shared<JueyingPolicyRunner>("common", p, 45, 5);
    common_policy_->SetPDGain(cp_ptr_->common_policy_p_gain_, cp_ptr_->common_policy_d_gain_);
    common_policy_->SetTorqueLimit(cp_ptr_->torque_limit_);
    common_policy_->DisplayPolicyInfo();

    common_policy_->UpdateObservation = [&](const RobotBasicState& ro){
        int obs_dim = common_policy_->obs_dim_;
        int obs_his_num = common_policy_->obs_history_num_;
        Vec3f cmd_vel = ro.cmd_vel_normlized.cwiseProduct(common_policy_->max_cmd_vel_);

        common_policy_->current_observation_.setZero(obs_dim);
        Vec3f project_gravity = ro.base_rot_mat.transpose() * Vec3f(0., 0., -1);
        common_policy_->current_observation_ << common_policy_->omega_scale_*ro.base_omega,
                                            project_gravity,
                                            cmd_vel.cwiseProduct(common_policy_->cmd_vel_scale_),
                                            ro.joint_pos - common_policy_->dof_pos_default_,
                                            common_policy_->dof_vel_scale_*ro.joint_vel,
                                            common_policy_->last_action_;

        VecXf obs_history_record = common_policy_->observation_history_.segment(obs_dim, (obs_his_num-1)*obs_dim).eval();
        common_policy_->observation_history_.segment(0, (obs_his_num-1)*obs_dim) = obs_history_record;
        common_policy_->observation_history_.segment((obs_his_num-1)*obs_dim, obs_dim) = common_policy_->current_observation_;

        common_policy_->observation_total_.segment(0, obs_dim) = common_policy_->current_observation_;
        common_policy_->observation_total_.segment(obs_dim, obs_dim*obs_his_num) = common_policy_->observation_history_;
    };
}

void RLControlState::AgilePolicyInitialize(){
    const std::string p = cp_ptr_->common_policy_path_;
    common_policy_ = std::make_shared<JueyingPolicyRunner>("common", p, 61, 0);
    common_policy_->SetPDGain(cp_ptr_->common_policy_p_gain_, cp_ptr_->common_policy_d_gain_);
    common_policy_->SetTorqueLimit(cp_ptr_->torque_limit_);
    common_policy_->DisplayPolicyInfo();

    common_policy_->UpdateObservation = [&](const RobotBasicState& ro){
        int obs_dim = common_policy_->obs_dim_;
        int obs_his_num = common_policy_->obs_history_num_;
        Vec3f cmd_vel = ro.cmd_vel_normlized.cwiseProduct(common_policy_->max_cmd_vel_);

        common_policy_->current_observation_.setZero(obs_dim);
        Vec3f project_gravity = ro.base_rot_mat.transpose() * Vec3f(0., 0., -1);

        // Fetch the latest sensor data
        Eigen::VectorXf sensor_data = depth_receiver_->get_latest_data(); // Ensure this returns a VecXf with 11 elements

        // Check that the sensor data has the correct size
        if (sensor_data.size() != 11) {
            std::cerr << "Sensor data size mismatch. Expected 11, got " << sensor_data.size() << std::endl;
            return;
        }

        common_policy_ -> current_observation_ << 1, 1, 1, 1, // contact
                                                0.25*ro.base_omega, // ang_vel
                                                project_gravity,
                                                cmd_vel.cwiseProduct(common_policy_->cmd_vel_scale_), // x,y,yaw
                                                1, // TODO: time left
                                                ro.joint_pos - common_policy_->dof_pos_default_, // joint_pos 12 
                                                common_policy_->dof_vel_scale_*ro.joint_vel, // joint_vel 12
                                                common_policy_->last_action_, // last_action 12
                                                6,6,6,6,6,6,6,6,6,6,6; // sensor data
        
        std::cout<< sensor_data <<std::endl;

        
        // std::cout << "current_observation_: " << common_policy_->current_observation_.transpose() << std::endl;

        // VecXf obs_history_record = common_policy_->observation_history_.segment(obs_dim, (obs_his_num-1)*obs_dim).eval();
        // common_policy_->observation_history_.segment(0, (obs_his_num-1)*obs_dim) = obs_history_record;
        // common_policy_->observation_history_.segment((obs_his_num-1)*obs_dim, obs_dim) = common_policy_->current_observation_;
        // common_policy_->observation_total_.segment(0, obs_dim) = common_policy_->current_observation_;
        // common_policy_->observation_total_.segment(obs_dim, obs_dim*obs_his_num) = common_policy_->observation_history_;

        common_policy_->observation_total_ = common_policy_->current_observation_;
    };
}

// void RLControlState::SpeedPolicyInitialize(){
//     const std::string p = cp_ptr_->speed_policy_path_;
//     speed_policy_ = std::make_shared<JueyingPolicyRunner>("speed", p, 141, 0);
//     speed_policy_->SetPDGain(cp_ptr_->speed_policy_p_gain_, cp_ptr_->speed_policy_d_gain_);
//     speed_policy_->SetCmdMaxVel(Vec3f(4.0, 0.5, 0.5));
//     speed_policy_->SetTorqueLimit(cp_ptr_->torque_limit_);
//     speed_policy_->DisplayPolicyInfo();
    
//     speed_policy_->UpdateObservation = [&](const RobotBasicState& ro){
//         int obs_dim = speed_policy_->obs_dim_;
//         int obs_his_num = speed_policy_->obs_history_num_;
//         int buffer_size = speed_policy_->buffer_size_;
//         VecXf dof_pos_default = speed_policy_->dof_pos_default_;
//         Vec3f cmd_vel = ro.cmd_vel_normlized.cwiseProduct(speed_policy_->max_cmd_vel_);

//         speed_policy_->current_observation_.setZero(obs_dim);
//         Vec3f project_gravity = ro.base_rot_mat.transpose() * Vec3f(0., 0., -1);

//         speed_policy_->current_observation_ << speed_policy_->omega_scale_*ro.base_omega,
//                                                 project_gravity,
//                                                 cmd_vel.cwiseProduct(speed_policy_->cmd_vel_scale_),
//                                                 ro.joint_pos - dof_pos_default,
//                                                 speed_policy_->dof_vel_scale_*ro.joint_vel,
//                                                 speed_policy_->last_action_,
//                                                 speed_policy_->action_buffer_[buffer_size-2],
//                                                 speed_policy_->dof_pos_buffer_[buffer_size-1] - dof_pos_default,
//                                                 speed_policy_->dof_pos_buffer_[buffer_size-2] - dof_pos_default,
//                                                 speed_policy_->dof_pos_buffer_[buffer_size-3] - dof_pos_default,
//                                                 speed_policy_->dof_vel_scale_*speed_policy_->dof_vel_buffer_[buffer_size-1],
//                                                 speed_policy_->dof_vel_scale_*speed_policy_->dof_vel_buffer_[buffer_size-2],
//                                                 speed_policy_->dof_vel_scale_*speed_policy_->dof_vel_buffer_[buffer_size-3],
//                                                 GetFootPos(ro.joint_pos);

//         speed_policy_->dof_pos_buffer_.push_back(ro.joint_pos);
//         speed_policy_->dof_vel_buffer_.push_back(ro.joint_vel);
//         speed_policy_->dof_vel_buffer_.pop_front();
//         speed_policy_->dof_pos_buffer_.pop_front();

//         speed_policy_->observation_total_ = speed_policy_->current_observation_;
//     };
// }


// void RLControlState::TumblerStandPolicyInitialize(){
//     const std::string p = "/home/shaoze/Documents/DeepRobotics/rl_controller/state_machine/policy/stand_student.pt";
    
//     tumbler_stand_policy_ = std::make_shared<JueyingPolicyRunner>("tumbler_stand", p, 45, 55, 12, 3);
//     printf("tumbler_stand_policy_ initialized\n");
    
//     tumbler_stand_policy_->SetPDGain(Vec3f(33., 25., 25), Vec3f(0.8, 0.8, 0.8));
//     tumbler_stand_policy_->SetDefaultJointPos(Vec3f(0.1, -1., 1.8));
//     tumbler_stand_policy_->SetTorqueLimit(cp_ptr_->torque_limit_);
//     tumbler_stand_policy_->DisplayPolicyInfo();
    
//     tumbler_stand_policy_->UpdateObservation = [&](const RobotBasicState& ro){
//         auto ptr = tumbler_stand_policy_;
//         int obs_dim = ptr->obs_dim_;
//         int obs_his_num = ptr->obs_history_num_;    
//         int cmd_dim = ptr->cmd_dim_;

//         Eigen::AngleAxisf pitchAngle(ro.base_rpy(1), Vec3f::UnitY());
//         Eigen::AngleAxisf rollAngle(ro.base_rpy(0), Vec3f::UnitX());
//         Eigen::Quaternion<float> q = pitchAngle*rollAngle;
//         Vec3f project_gravity = q.matrix() * Vec3f(0., 0., -1.);
//         // Vec3f project_gravity = ro.base_rot_mat.transpose() * Vec3f(0., 0., -1.);
//         VecXf obs_history_record = ptr->observation_history_.segment(obs_dim, (obs_his_num-1)*obs_dim).eval();
//         ptr->observation_history_.segment(0, (obs_his_num-1)*obs_dim) = obs_history_record;
//         ptr->observation_history_.segment((obs_his_num-1)*obs_dim, obs_dim) = ptr->current_observation_;

//         ptr->current_observation_.setZero(obs_dim);
//         ptr->current_observation_ << ptr->last_action_,
//                                     1./1.3*ro.base_omega,
//                                     ConventionShift(ro.joint_pos),
//                                     ConventionShift(ro.joint_vel),
//                                     1./9.815*ro.base_acc,
//                                     project_gravity;

//         ptr->observation_total_.segment(0, obs_dim*obs_his_num) = ptr->observation_history_;
//         ptr->observation_total_.segment(obs_dim*obs_his_num, obs_dim) = ptr->current_observation_;
        
//         ptr->observation_total_.segment(obs_dim*obs_his_num+obs_dim, cmd_dim) = Vec3f(0., cos(0.), sin(0.));
        
//     };

//     tumbler_stand_policy_->UpdateAction = [&](VecXf& action){
//         VecXf low_limit = Vec3f(-0.49, -2.7, 0.45).replicate(4, 1) - 0.1*VecXf::Ones(12);
//         VecXf high_limit = Vec3f(0.49, 0.33, 2.7).replicate(4, 1) + 0.1*VecXf::Ones(12);
//         float lpf = 0.766667;
//         VecXf output_action(12);
//         auto ptr = tumbler_stand_policy_;
        
//         if(ptr->run_cnt_ <= ptr->obs_history_num_+1){
//             output_action = ptr->dof_pos_default_;
//         }else{         
//             ptr->last_action_ = lpf*ptr->last_action_ + (1.-lpf)*action;
//             output_action = low_limit+0.5*(ptr->last_action_+VecXf::Ones(12)).cwiseProduct(high_limit-low_limit);  
//         }
//         output_action = output_action.cwiseMax(low_limit + 0.1*VecXf::Ones(12)).cwiseMin(high_limit - 0.1*VecXf::Ones(12));
//         ptr->last_action_ = 2.*(output_action - low_limit).cwiseQuotient(high_limit-low_limit)-VecXf::Ones(12);
//         ptr->last_action_ = ptr->last_action_.cwiseMax(-1*VecXf::Ones(12)).cwiseMin(VecXf::Ones(12));
        
//         output_action = ConventionShift(output_action);
        
//         RobotAction ra;
//         ra.goal_joint_pos = output_action;
//         ra.goal_joint_vel = VecXf::Zero(ptr->act_dim_);
//         ra.tau_ff = VecXf::Zero(ptr->act_dim_);
//         ra.kp = ptr->kp_;
//         ra.kd = ptr->kd_;
//         return ra;
//     };
// }

// void RLControlState::TumblerForwardPolicyInitialize(){
//     const std::string p = "/home/shaoze/Documents/DeepRobotics/rl_controller/state_machine/policy/forward_student.pt";
//     tumbler_stand_policy_ = std::make_shared<JueyingPolicyRunner>("tumbler_forward", p, 45, 55, 12, 3);
//     tumbler_stand_policy_->SetPDGain(Vec3f(33., 25., 25), Vec3f(0.7, 0.7, 0.7));
//     tumbler_stand_policy_->SetDefaultJointPos(Vec3f(0.1, -1., 1.8));
//     tumbler_stand_policy_->SetTorqueLimit(cp_ptr_->torque_limit_);
//     tumbler_stand_policy_->DisplayPolicyInfo();

//     tumbler_stand_policy_->UpdateObservation = [&](const RobotBasicState& ro){
//         auto ptr = tumbler_stand_policy_;
//         int obs_dim = ptr->obs_dim_;
//         int obs_his_num = ptr->obs_history_num_;    
//         int cmd_dim = ptr->cmd_dim_;

//         Eigen::AngleAxisf pitchAngle(ro.base_rpy(1), Vec3f::UnitY());
//         Eigen::AngleAxisf rollAngle(ro.base_rpy(0), Vec3f::UnitX());
//         Eigen::Quaternion<float> q = pitchAngle*rollAngle;
//         Vec3f project_gravity = q.matrix() * Vec3f(0., 0., -1.);
//         // Vec3f project_gravity = ro.base_rot_mat.transpose() * Vec3f(0., 0., -1.);
//         VecXf obs_history_record = ptr->observation_history_.segment(obs_dim, (obs_his_num-1)*obs_dim).eval();
//         ptr->observation_history_.segment(0, (obs_his_num-1)*obs_dim) = obs_history_record;
//         ptr->observation_history_.segment((obs_his_num-1)*obs_dim, obs_dim) = ptr->current_observation_;

//         ptr->current_observation_.setZero(obs_dim);
//         ptr->current_observation_ << ptr->last_action_,
//                                     1./1.3*ro.base_omega,
//                                     ConventionShift(ro.joint_pos),
//                                     ConventionShift(ro.joint_vel),
//                                     1./9.815*ro.base_acc,
//                                     project_gravity;

//         ptr->observation_total_.segment(0, obs_dim*obs_his_num) = ptr->observation_history_;
//         ptr->observation_total_.segment(obs_dim*obs_his_num, obs_dim) = ptr->current_observation_;
        
//         ptr->observation_total_.segment(obs_dim*obs_his_num+obs_dim, cmd_dim) = Vec3f(0., cos(0.), sin(0.));
//     };

//     tumbler_stand_policy_->UpdateAction = [&](VecXf& action){
//         VecXf low_limit = Vec3f(-0.49, -2.7, 0.45).replicate(4, 1) - 0.1*VecXf::Ones(12);
//         VecXf high_limit = Vec3f(0.49, 0.33, 2.7).replicate(4, 1) + 0.1*VecXf::Ones(12);
//         float lpf = 0.766667;
//         VecXf output_action(12);
//         auto ptr = tumbler_stand_policy_;
        
//         if(ptr->run_cnt_ <= ptr->obs_history_num_+1){
//             output_action = ptr->dof_pos_default_;
//         }else{         
//             ptr->last_action_ = lpf*ptr->last_action_ + (1.-lpf)*action;
//             output_action = low_limit+0.5*(ptr->last_action_+VecXf::Ones(12)).cwiseProduct(high_limit-low_limit);  
//         }
//         output_action = output_action.cwiseMax(low_limit + 0.1*VecXf::Ones(12)).cwiseMin(high_limit - 0.1*VecXf::Ones(12));
//         ptr->last_action_ = 2.*(output_action - low_limit).cwiseQuotient(high_limit-low_limit)-VecXf::Ones(12);
//         ptr->last_action_ = ptr->last_action_.cwiseMax(-1*VecXf::Ones(12)).cwiseMin(VecXf::Ones(12));
//         // std::cout << "last_action:  " << ptr->last_action_.transpose() << std::endl;
//         output_action = ConventionShift(output_action);
        
//         RobotAction ra;
//         ra.goal_joint_pos = output_action;
//         ra.goal_joint_vel = VecXf::Zero(ptr->act_dim_);
//         ra.tau_ff = VecXf::Zero(ptr->act_dim_);
//         ra.kp = ptr->kp_;
//         ra.kd = ptr->kd_;
//         return ra;
//     };   
// }


VecXf RLControlState::GetFootPos(const VecXf& joint_pos){
    float l0, l1, l2;
    float s1, s2, s3;
    float x, y, z, zt;
    VecXf foot_pos = VecXf::Zero(12);

    for (int i=0;i<4;i++){  
    // using the hip joint as the origin of coordinates, the axis direction is the same as the body coordination.
        l0 = cp_ptr_->hip_len_; l1 = cp_ptr_->thigh_len_; l2 = cp_ptr_->shank_len_;
        if(i==0||i==2) l0 = -cp_ptr_->hip_len_;
        s1 = joint_pos[3*i];
        s2 = joint_pos[3*i+1];
        s3 = joint_pos[3*i+2];

        x = l1 * sin(s2) + l2 * sin(s2 + s3);
        zt = -(l1 * cos(s2) + l2 * cos(s2 + s3));
        y = zt * sin(s1) - l0 * cos(s1);
        z = zt * cos(s1) + l0 * sin(s1);

        foot_pos[3*i] = x;
        foot_pos[3*i+1] = y;
        foot_pos[3*i+2] = z;
    }
    return foot_pos;
}

Mat3f RLControlState::RpyToRm(const Vec3f &rpy){
    Mat3f rm;
    Eigen::AngleAxisf yawAngle(rpy(2), Vec3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(rpy(1), Vec3f::UnitY());
    Eigen::AngleAxisf rollAngle(rpy(0), Vec3f::UnitX());
    Eigen::Quaternion<float> q = yawAngle*pitchAngle*rollAngle;
    return q.matrix();
}

VecXf RLControlState::ConventionShift(const VecXf& dr_convention){
    VecXf x(12);
    x.segment(0,3) = dr_convention.segment(3,3);
    x.segment(3,3) = dr_convention.segment(0,3);
    x.segment(6,3) = dr_convention.segment(9,3);
    x.segment(9,3) = dr_convention.segment(6,3);
    return x;
}
