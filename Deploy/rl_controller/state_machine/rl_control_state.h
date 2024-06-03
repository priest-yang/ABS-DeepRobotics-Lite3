#ifndef RL_CONTROL_STATE_H_
#define RL_CONTROL_STATE_H_

#include "state_base.h"
#include "jueying_policy_runner.h"

//! added by Shaoze
#include "sensor_receiver.hpp"

class RLControlState : public StateBase
{
private:
    RobotBasicState rbs_;
    int state_run_cnt_;
    std::shared_ptr<JueyingPolicyRunner> current_policy_; 
    std::shared_ptr<JueyingPolicyRunner> common_policy_, speed_policy_, tumbler_stand_policy_, tumbler_forward_policy_;
    std::thread run_policy_thread_;

    std::shared_ptr<DepthReceiver> depth_receiver_; //!Shaoze


    bool start_flag_ = true;

    // Added
    void AgilePolicyInitialize();

    void CommonPolicyInitialize();
    void SpeedPolicyInitialize();
    void TumblerStandPolicyInitialize();
    void TumblerForwardPolicyInitialize();               

    Mat3f RpyToRm(const Vec3f &rpy);
    VecXf GetFootPos(const VecXf& joint_pos);
    VecXf ConventionShift(const VecXf& x);

    void UpdateRobotObservation(){
        rbs_.base_rpy     = ri_ptr_->GetImuRpy();
        rbs_.base_rot_mat = RpyToRm(rbs_.base_rpy);
        rbs_.base_omega   = ri_ptr_->GetImuOmega();
        rbs_.base_acc     = ri_ptr_->GetImuAcc();
        rbs_.joint_pos    = ri_ptr_->GetJointPosition();
        rbs_.joint_vel    = ri_ptr_->GetJointVelocity();
        rbs_.joint_tau    = ri_ptr_->GetJointTorque();
        rbs_.cmd_vel_normlized      = Vec3f(uc_ptr_->GetUserCommand().forward_vel_scale, 
                                    uc_ptr_->GetUserCommand().side_vel_scale, 
                                    uc_ptr_->GetUserCommand().turnning_vel_scale);
        // rbs_.foot_pos     = GetFootPos(rbs_.joint_pos);

        // print observation
        // std::cout << "base_rpy: " << rbs_.base_rpy.transpose() << std::endl;
        // std::cout << "base_rot_mat: " << rbs_.base_rot_mat << std::endl;
        // std::cout << "base_omega: " << rbs_.base_omega.transpose() << std::endl;
        // std::cout << "base_acc: " << rbs_.base_acc.transpose() << std::endl;
        // std::cout << "joint_pos: " << rbs_.joint_pos.transpose() << std::endl;
        // std::cout << "joint_vel: " << rbs_.joint_vel.transpose() << std::endl;
        // std::cout << "joint_tau: " << rbs_.joint_tau.transpose() << std::endl;
        // std::cout << "cmd_vel_normlized: " << rbs_.cmd_vel_normlized.transpose() << std::endl;
    }

    void PolicyRunner(){
        int run_cnt_record = -1;
        while (start_flag_){
            
            if(state_run_cnt_%current_policy_->decimation_ == 0 && state_run_cnt_ != run_cnt_record){
                timespec start_timestamp, end_timestamp;
                clock_gettime(CLOCK_MONOTONIC,&start_timestamp);
                auto ra = current_policy_->GetPolicyOutput(rbs_);
                MatXf res = ra.ConvertToMat();

                // print action
                std::cout << "goal_joint_pos: " << ra.goal_joint_pos.transpose() << std::endl;
                std::cout << "goal_joint_vel: " << ra.goal_joint_vel.transpose() << std::endl;
                std::cout << "kp: " << ra.kp.transpose() << std::endl;
                std::cout << "kd: " << ra.kd.transpose() << std::endl;
                std::cout << "tau_ff: " << ra.tau_ff.transpose() << std::endl;
                
                ///////////////////////////////////////////
                ri_ptr_->SetJointCommand(res); //FOR DEBUG
                ///////////////////////////////////////////

                run_cnt_record = state_run_cnt_;
                clock_gettime(CLOCK_MONOTONIC,&end_timestamp);
                std::cout << "cost_time:  " << (end_timestamp.tv_sec-start_timestamp.tv_sec)*1e3 
                    + (end_timestamp.tv_nsec-start_timestamp.tv_nsec)/1e6 << " ms\n";
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

public:
    RLControlState(const RobotType& robot_type, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_type, state_name, data_ptr), 
        depth_receiver_(std::make_shared<DepthReceiver>()){
            // CommonPolicyInitialize();
            AgilePolicyInitialize();

            // if(robot_type==RobotType::P50){
            //     SpeedPolicyInitialize();
            // }else if(robot_type==RobotType::Lite3){
            //     TumblerStandPolicyInitialize();
            // }

            current_policy_ = common_policy_;
            std::memset(&rbs_, 0, sizeof(rbs_));
        }
    ~RLControlState(){}

    virtual void OnEnter() {
        state_run_cnt_ = -1;
        start_flag_ = true;
        run_policy_thread_ = std::thread(std::bind(&RLControlState::PolicyRunner, this));
        current_policy_->OnEnter();
        StateBase::msfb_.current_state = RobotMotionState::RLControlMode;
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);

        //! start receiver
        if (depth_receiver_) {
            depth_receiver_->start();
        }else{
            std::cerr << "DepthReceiver is not initialized!" << std::endl;
        }

    };

    virtual void OnExit() { 
        start_flag_ = false;
        run_policy_thread_.join();
        state_run_cnt_ = -1;
        depth_receiver_->stop(); //! stop receiver
    }

    virtual void Run() {
        UpdateRobotObservation();
        state_run_cnt_++;
    }

    virtual bool LoseControlJudge() {
        if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) return true;
        return false;
    }

    virtual StateName GetNextStateName() {
        return StateName::kRLControl;
    }
};


#endif