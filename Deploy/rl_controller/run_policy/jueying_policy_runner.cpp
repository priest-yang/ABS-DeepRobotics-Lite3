
#include "jueying_policy_runner.h"

JueyingPolicyRunner::JueyingPolicyRunner(const std::string& name, const std::string& path, int obs_dim, int obs_his, int act_dim, int cmd_dim):
    PolicyRunner(name, path, obs_dim, obs_his, act_dim, cmd_dim){
    cmd_vel_scale_ << 2., 2., 0.25;


    // max_cmd_vel_ << 1.5, 0.6, 0.8;
    max_cmd_vel_ << 4,4,1.5; //wz commmon
    dof_pos_default_ = VecXf::Zero(act_dim);
    
    this->SetDecimation(20);

    this->SetDefaultJointPos(Vec3f(0.1, -1, 1.8)); //agile policy
    // this->SetDefaultJointPos(Vec3f(0, -1, 1.8)); //! wz policy

    this->SetPDGain(Vec3f(120, 120., 150.), Vec3f(3., 3. , 3.5));
    this->SetTorqueLimit(Vec3f(60., 60., 135.));
    this->SetCmdMaxVel(Vec3f(max_cmd_vel_(0), max_cmd_vel_(1), max_cmd_vel_(2)));

    dof_pos_buffer_.clear();
    dof_vel_buffer_.clear();
    action_buffer_.clear();
    for(int i=0;i<buffer_size_;++i){
        dof_pos_buffer_.push_back(VecXf::Zero(12));
        dof_vel_buffer_.push_back(VecXf::Zero(12));
        action_buffer_.push_back(VecXf::Zero(act_dim_));
    }

    this->UpdateAction = [this](VecXf& action){
        last_action_ = action_;
        RobotAction ra;
        ra.goal_joint_pos = action_scale_*action + dof_pos_default_;
        ra.goal_joint_vel = VecXf::Zero(act_dim_);
        ra.tau_ff = VecXf::Zero(act_dim_);
        ra.kp = kp_;
        ra.kd = kd_;
        return ra;
    };
}

JueyingPolicyRunner::~JueyingPolicyRunner(){
}

void JueyingPolicyRunner::SetDecimation(int decimation){
    decimation_ = decimation;
}

void JueyingPolicyRunner::SetDefaultJointPos(const Vec3f& pos){
    //输入判断？
    dof_pos_default_.setZero(12); 
    for(int i=0;i<12;++i) {
        dof_pos_default_(i) = pos(i%3);
        if(i%6==3) dof_pos_default_(i) = -pos(i%3);
    }
}

void JueyingPolicyRunner::SetPDGain(const Vec3f& p, const Vec3f& d){
    //输入判断？
    kp_ = p.replicate(4, 1);
    kd_ = d.replicate(4, 1);
}

void JueyingPolicyRunner::SetTorqueLimit(const Vec3f& torque_limit){
    for(int i=0;i<3;++i){
        if(torque_limit(i) < 10){
            std::cerr << policy_name_ << " torque_limit " << i << " set error" << std::endl;
        }
    }
    torque_limit_.setZero(12);
    torque_limit_ = torque_limit.replicate(4, 1);
}

void JueyingPolicyRunner::SetCmdMaxVel(const Vec3f& vel){
    for(int i=0;i<3;++i){
        if(vel(i) < 0){
            std::cerr << policy_name_ << " max_vel " << i << " set error" << std::endl;
        }
    }
    max_cmd_vel_ = vel;
}

void JueyingPolicyRunner::SetOmegaVelScale(float scale){
    omega_scale_ = scale;
}

void JueyingPolicyRunner::SetActionScale(float as){
    action_scale_ = as;
}

void JueyingPolicyRunner::OnEnter(){
    std::cout << "enter " << policy_name_ << std::endl;

    current_observation_.setZero(obs_dim_);
    observation_history_.setZero(obs_dim_*obs_history_num_);
    observation_total_.setZero(obs_total_dim_);
    action_.setZero(act_dim_);
    last_action_ = action_;

    dof_pos_buffer_.clear();
    dof_vel_buffer_.clear();
    action_buffer_.clear();
    for(int i=0;i<buffer_size_;++i){
        dof_pos_buffer_.push_back(VecXf::Zero(12));
        dof_vel_buffer_.push_back(VecXf::Zero(12));
        action_buffer_.push_back(VecXf::Zero(act_dim_));
    }

    action_tensor_ = torch::ones({1, act_dim_});
    obs_total_tensor_ = torch::ones({1, obs_total_dim_});
    
    run_cnt_ = 0;
}

void JueyingPolicyRunner::OnExit(){
    std::cout << "exit " << policy_name_ << std::endl;
}

RobotAction JueyingPolicyRunner::GetPolicyOutput(const RobotBasicState& rbs){
    VecXf action = this->PolicyForward(rbs);
    action_buffer_.push_back(action);
    action_buffer_.pop_front();

    RobotAction ra = this->UpdateAction(action);
    
    ++run_cnt_;
    return ra;
}

void JueyingPolicyRunner::DisplayPolicyInfo(){
    std::cout << "name : " << policy_name_ << std::endl; 
    std::cout << "path : " << policy_path_ << std::endl;
    std::cout << "dim  : " << obs_dim_ << " " << obs_history_num_ << " " << obs_total_dim_ << " " << act_dim_ << std::endl;
    std::cout << "dof  : " << dof_pos_default_.transpose() << std::endl;
    std::cout << "kp   : " << kp_.transpose() << std::endl;
    std::cout << "kd   : " << kd_.transpose() << std::endl;
    std::cout << "limit: " << torque_limit_.transpose() << std::endl;
    std::cout << "max_v: " << max_cmd_vel_.transpose() << std::endl;
    std::cout << "scale: " << action_scale_ << " " << omega_scale_ << " " << dof_vel_scale_ << " | " << cmd_vel_scale_.transpose() << std::endl;
}