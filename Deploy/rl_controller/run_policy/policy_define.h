#ifndef POLICY_DEFINE_H_
#define POLICY_DEFINE_H_

#include "common_types.h"

#define OBS_BUFFER_SIZE 10

using namespace types;
struct RobotObservation{
    Vec3f omega_body, project_gravity, cmd_vel;
    VecXf dof_pos_now, dof_vel_now;
    Mat3f rot_mat_;
    
    VecXf raycast_data;
    std::deque<VecXf> dof_pos_buffer;
    std::deque<VecXf> dof_vel_buffer;
    VecXf foot_pos;

    RobotObservation(){
        dof_pos_now.setZero(12);
        dof_vel_now.setZero(12);

        dof_pos_buffer.clear();
        dof_vel_buffer.clear();
        for(int i=0;i<OBS_BUFFER_SIZE;++i){
            dof_pos_buffer.push_back(VecXf::Zero(12));
            dof_vel_buffer.push_back(VecXf::Zero(12));
        }
    }

    void UpdateBuffer(){
        dof_pos_buffer.push_back(dof_pos_now);
        while (dof_pos_buffer.size() > OBS_BUFFER_SIZE){
            dof_pos_buffer.pop_front();
        }
        dof_vel_buffer.push_back(dof_vel_now);
        while(dof_vel_buffer.size() > OBS_BUFFER_SIZE){
            dof_vel_buffer.pop_front();
        }
    }
};


enum ObservationType{
    kCommon = 0,
};

#endif