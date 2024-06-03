#ifndef JUEYING_POLICY_RUNNER_H_
#define JUEYING_POLICY_RUNNER_H_

#include "policy_runner.hpp"

class JueyingPolicyRunner : public PolicyRunner{
public:
    JueyingPolicyRunner(const std::string& name, const std::string& path, int obs_dim, int obs_his, int act_dim=12, int cmd_dim=0);
    ~JueyingPolicyRunner();

    void SetDecimation(int decimation);
    void SetOmegaVelScale(float scale);
    void SetActionScale(float as);
    void SetDefaultJointPos(const Vec3f& pos);
    void SetPDGain(const Vec3f& p, const Vec3f& d);
    void SetTorqueLimit(const Vec3f& torque_limit);
    void SetCmdMaxVel(const Vec3f& vel);

    void OnEnter();
    void OnExit();
    void DisplayPolicyInfo();
    RobotAction GetPolicyOutput(const RobotBasicState&);

    int run_cnt_ = 0;
    float action_scale_ = 0.25;
    float omega_scale_ = 0.25;
    float dof_vel_scale_ = 0.05;
    Vec3f cmd_vel_scale_;
    Vec3f max_cmd_vel_;
    VecXf dof_pos_default_, kp_, kd_, torque_limit_;
    int decimation_;

    int buffer_size_ = 10;
    std::deque<VecXf> dof_pos_buffer_;
    std::deque<VecXf> dof_vel_buffer_;
    std::deque<VecXf> action_buffer_;
    VecXf foot_pos_;
};

#endif