#ifndef POLICY_RUNNER_H_
#define POLICY_RUNNER_H_

#include "policy_define.h"
#include "torch/torch.h"
#include "torch/script.h"

using namespace types;

class PolicyRunner{
public:
    int obs_dim_, obs_history_num_, obs_total_dim_, act_dim_, cmd_dim_;
    
    VecXf action_, last_action_;
    VecXf action_buffer_;
    VecXf current_observation_, observation_history_, observation_total_, command_input_;

    torch::jit::Module backbone_;
    torch::Tensor action_tensor_, obs_total_tensor_;
    std::vector<c10::IValue> obs_vector_{};
    const std::string policy_name_, policy_path_;

public:
    std::function<void(const RobotBasicState&)> UpdateObservation;
    std::function<RobotAction(VecXf&)> UpdateAction;

    PolicyRunner(const std::string& name, const std::string& path, int obs_dim, int history_dim, int act_dim, int cmd_dim=0):
        policy_name_(name), 
        policy_path_(path){
        try { 
            backbone_ = torch::jit::load(policy_path_); 
            backbone_.eval();
        }
        catch (const c10::Error &e) { std::cerr << "error loading policy at " << policy_path_ << "\n" << e.what(); }

        this->SetPolicyDim(obs_dim, history_dim, act_dim, cmd_dim);

        for(int i=0;i<10;++i){
            torch::Tensor reaction;
            obs_vector_.clear();
            obs_vector_.emplace_back(torch::ones({1, obs_total_dim_}));  
            reaction = backbone_.forward(obs_vector_).toTensor();
            if(i==9) std::cout << policy_name_ << " test network success" << std::endl;
        }
    }

    void SetPolicyDim(int obs_dim, int history_dim, int act_dim, int cmd_dim){
        obs_dim_ = obs_dim;
        obs_history_num_ = history_dim;
        act_dim_ = act_dim;
        cmd_dim_ = cmd_dim;
        if(obs_dim < 5){
            std::cerr << policy_name_ << "obs_dim error!" << std::endl;
        }
        if(act_dim < 0){
            std::cerr << policy_name_ << "act_dim error!" << std::endl;
        }
        obs_total_dim_ = obs_dim_ + obs_dim_*obs_history_num_ + cmd_dim_;

        action_.setZero(act_dim); 
        last_action_.setZero(act_dim); 

        current_observation_.setZero(obs_dim_);
        observation_history_.setZero(obs_dim_*obs_history_num_);
        if(cmd_dim_!=0) command_input_.setZero(cmd_dim_);
        observation_total_.setZero(obs_total_dim_);
    }

    VecXf PolicyForward(const RobotBasicState& ro){
        this->UpdateObservation(ro);
        Eigen::MatrixXf temp = observation_total_.transpose();
        torch::Tensor a = torch::from_blob(temp.data(), {temp.rows(), temp.cols()}, torch::kFloat);
        obs_total_tensor_ = a.clone();

        obs_vector_.clear();
        obs_vector_.emplace_back(obs_total_tensor_);
        action_tensor_ = backbone_.forward(obs_vector_).toTensor();

        // Eigen::Matrix<float, 12, 1> act(action_tensor_.data_ptr<float>());
        Eigen::Map<Eigen::MatrixXf> act(action_tensor_.data_ptr<float>(), act_dim_, 1);
        action_ = act.col(0);
        // VecXf action_exec = action_scale_*action_ + dof_pos_default_;
        return action_;
    }


};
#endif