#include "state_base.h"
#include "idle_state.hpp"
#include "standup_state.hpp"
#include "joint_damping_state.hpp"
#include "rl_control_state.h"

#include "skydroid_gamepad_interface.hpp"
#include "retroid_gamepad_interface.hpp"
// #include "keyboard_interface.hpp"
#ifdef USE_RAISIM
    #include "simulation/jueying_raisim_simulation.hpp"
#endif
#include "hardware/x30_hardware_interface.hpp"
#include "hardware/lite3_hardware_interface.hpp"
#include "jueying_policy_runner.h"


class StateMachine{
private:
    std::shared_ptr<StateBase> current_controller_;
    std::shared_ptr<StateBase> idle_controller_;
    std::shared_ptr<StateBase> standup_controller_;
    std::shared_ptr<StateBase> rl_controller_;
    std::shared_ptr<StateBase> joint_damping_controller_;

    StateName current_state_name_, next_state_name_;

    std::shared_ptr<UserCommandInterface> uc_ptr_;
    std::shared_ptr<RobotInterface> ri_ptr_;
    std::shared_ptr<ControlParameters> cp_ptr_;

    std::shared_ptr<StateBase> GetNextStatePtr(StateName state_name){
        switch(state_name){
            case StateName::kInvalid:{
                return nullptr;
            }
            case StateName::kIdle:{
                return idle_controller_;
            }
            case StateName::kStandUp:{
                return standup_controller_;
            }
            case StateName::kRLControl:{
                return rl_controller_;
            }
            case StateName::kJointDamping:{
                return joint_damping_controller_;
            }
            default:{
                std::cerr << "error state name" << std::endl;
            }
        }
        return nullptr;
    }
public:
    StateMachine(RobotType robot_type){
        const std::string activation_key = "~/raisim/activation.raisim";
        std::string urdf_path = "";
        // uc_ptr_ = std::make_shared<SkydroidGamepadInterface>(12121);
        uc_ptr_ = std::make_shared<RetroidGamepadInterface>(12121);
        // uc_ptr_ = std::make_shared<KeyboardInterface>();

        if(robot_type == RobotType::Lite3){
            urdf_path = "/home/ysc/wz/rl_controller/third_party/URDF_model/lite3_urdf/Lite3/urdf/Lite3.urdf";
            #ifdef USE_RAISIM
                ri_ptr_ = std::make_shared<JueyingRaisimSimulation>(activation_key, urdf_path);
            #else 
                ri_ptr_ = std::make_shared<Lite3HardwareInterface>("Lite3");
            #endif
            cp_ptr_ = std::make_shared<ControlParameters>(robot_type);
        }else if(robot_type == RobotType::X30){
            const std::string urdf_path = "/home/ysc/wz/rl_controller/third_party/URDF_model/x30_urdf/X30/urdf/X30.urdf";
            #ifdef USE_RAISIM
                ri_ptr_ = std::make_shared<JueyingRaisimSimulation>(activation_key, urdf_path);
            #else
                ri_ptr_ = std::make_shared<X30HardwareInterface>("X30");
            #endif
            cp_ptr_ = std::make_shared<ControlParameters>(robot_type);       
        }else if(robot_type == RobotType::P50){
            const std::string urdf_path = "/home/ysc/wz/model/robot_model/P50/urdf/P50.urdf";
            #ifdef USE_RAISIM
                ri_ptr_ = std::make_shared<JueyingRaisimSimulation>(activation_key, urdf_path);
            #else
                ri_ptr_ = std::make_shared<X30HardwareInterface>("P50");
            #endif
            cp_ptr_ = std::make_shared<ControlParameters>(robot_type);  
        }

        uc_ptr_->Start();
        ri_ptr_->Start();
        std::shared_ptr<ControllerData> data_ptr = std::make_shared<ControllerData>();
        data_ptr->ri_ptr = ri_ptr_;
        data_ptr->uc_ptr = uc_ptr_;
        data_ptr->cp_ptr = cp_ptr_;

        idle_controller_ = std::make_shared<IdleState>(robot_type, "idle_state", data_ptr);
        // printf("idle_controller_ created\n");
        standup_controller_ = std::make_shared<StandUpState>(robot_type, "standup_state", data_ptr);
        // printf("standup_controller_ created\n");
        rl_controller_ = std::make_shared<RLControlState>(robot_type, "rl_control", data_ptr);
        // printf("rl_controller_ created\n");
        joint_damping_controller_ = std::make_shared<JointDampingState>(robot_type, "joint_damping", data_ptr);
        // printf("joint_damping_controller_ created\n");
        
        current_controller_ = idle_controller_;
        current_state_name_ = kIdle;
        next_state_name_ = kIdle;
        current_controller_->OnEnter();  
    }
    ~StateMachine(){}

    void Run(){
        int cnt = 0;
        static float time_record = 0;
        while(true){
            if(ri_ptr_->GetInterfaceTimeStamp()!= time_record){
                time_record = ri_ptr_->GetInterfaceTimeStamp();
                current_controller_ -> Run();
                if(current_controller_->LoseControlJudge()) {
                    printf("Lose Control\n");
                    next_state_name_ = StateName::kJointDamping;
                }
                else next_state_name_ = current_controller_ -> GetNextStateName();
                
                if(next_state_name_ != current_state_name_){
                    current_controller_ -> OnExit();
                    std::cout << current_controller_ -> state_name_ << " ------------> ";
                    current_controller_ = GetNextStatePtr(next_state_name_);
                    std::cout << current_controller_ -> state_name_ << std::endl;
                    current_controller_ ->OnEnter();
                    current_state_name_ = next_state_name_; 
                }
                ++cnt;

                if (cnt % 1000 == 0) {
                    std::cout << current_controller_ -> state_name_ << std::endl;    
                }

            }
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }

        // uc_ptr_->Stop();
        // ri_ptr_->Stop();
    }

};

