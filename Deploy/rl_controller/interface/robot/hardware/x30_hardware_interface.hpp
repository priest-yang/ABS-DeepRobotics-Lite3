#ifndef X30_HARDWARE_INTERFACE_HPP_
#define X30_HARDWARE_INTERFACE_HPP_

#include "robot_interface.h"
#include "x30_types.h"
#include "parse_cmd.h"
#include "send_to_robot.h"

using namespace types;
using namespace x30;

namespace interface{
class X30HardwareInterface : public RobotInterface
{
private:
    RobotCmdSDK robot_joint_cmd_{};
    RobotDataSDK* robot_data_ = nullptr;
    ParseCommand* data_recv_ = nullptr;
    SendToRobot* cmd_send_ = nullptr;
    Vec3f omega_body_, rpy_, acc_;
    VecXf joint_pos_, joint_vel_, joint_tau_;
    Eigen::Matrix<float, Eigen::Dynamic, 5> joint_cmd_;
    std::thread hw_thread_;

public:
    X30HardwareInterface(const std::string& robot_name, 
                    int local_port=43897, 
                    std::string robot_ip="192.168.1.103",
                    int robot_port=43893):RobotInterface(robot_name, 12){
        data_recv_ = new ParseCommand(local_port);   
      
        cmd_send_ = new SendToRobot(robot_ip, robot_port);
        cmd_send_->robot_state_init(); 
    }
    ~X30HardwareInterface(){
        delete data_recv_;
    }


    virtual float GetInterfaceTimeStamp(){
        return robot_data_->tick*0.001;
    }
    virtual VecXf GetJointPosition() {
        joint_pos_ = VecXf::Zero(dof_num_);
        for(int i=0;i<dof_num_;++i){
            joint_pos_(i) = robot_data_->joint_data[i].pos;
        }
        return joint_pos_;
    };
    virtual VecXf GetJointVelocity() {
        joint_vel_ = VecXf::Zero(dof_num_);
        for(int i=0;i<dof_num_;++i){
            joint_vel_(i) = robot_data_->joint_data[i].vel;
        }
        return joint_vel_;
    }
    virtual VecXf GetJointTorque() {
        joint_tau_ = VecXf::Zero(dof_num_);
        for(int i=0;i<dof_num_;++i){
            joint_tau_(i) = robot_data_->joint_data[i].tor;
        }
        return joint_tau_;
    }
    virtual Vec3f GetImuRpy() {
        rpy_ << robot_data_->imu.roll, robot_data_->imu.pitch, robot_data_->imu.yaw;
        return rpy_;
    }
    virtual Vec3f GetImuAcc() {
        acc_ << robot_data_->imu.acc_x, robot_data_->imu.acc_y, robot_data_->imu.acc_z;
        return acc_;
    }
    virtual Vec3f GetImuOmega() {
        omega_body_ << robot_data_->imu.omega_x, robot_data_->imu.omega_y, robot_data_->imu.omega_z;
        return omega_body_;
    }
    virtual VecXf GetContactForce() {
        return VecXf::Zero(4);
    }
    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input){
        for(int i=0;i<dof_num_;++i){
            robot_joint_cmd_.joint_cmd[i].kp = input(i, 0);
            robot_joint_cmd_.joint_cmd[i].pos = input(i, 1);
            robot_joint_cmd_.joint_cmd[i].kd = input(i, 2);
            robot_joint_cmd_.joint_cmd[i].vel = input(i, 3);
            robot_joint_cmd_.joint_cmd[i].tor = input(i, 4);
        }
        joint_cmd_ = input;
        cmd_send_->set_send(robot_joint_cmd_);
    }

    virtual void Start(){
        data_recv_->startWork();                         
        robot_data_ = &(data_recv_->getRecvState());
        if (cmd_send_ != nullptr)
            cmd_send_->control_get(ABLE);
    }

    virtual void Stop(){
        if (cmd_send_ != nullptr) cmd_send_->control_get(UNABLE);
    }
};


};

#endif