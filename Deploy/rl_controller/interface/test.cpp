#include "simulation/raisim_interface.h"
#include "simulation/jueying_raisim_simulation.h"
#include "user_command_interface.h"
#include "gamepad_interface.h"

using namespace interface;

int main(){
    const std::string activation_key = "~/raisim/activation.raisim";
    const std::string urdf_path = "/home/shaoze/Documents/DeepRobotics/model/robot_model/X30/urdf/X30.urdf";
    // JueyingRaisimSimulation sim(activation_key, urdf_path, "X30");
    // std::shared_ptr<RobotInterface> sim_ptr = std::make_shared<JueyingRaisimSimulation>(activation_key, urdf_path);
    // sim_ptr->Start();
    // std::this_thread::sleep_for(std::chrono::seconds(5));
    // sim_ptr->Stop();

    std::shared_ptr<UserCommandInterface> uc_ptr_ = std::make_shared<GamepadInterface>(12121);
    uc_ptr_->Start();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    uc_ptr_->Stop();
    return 0;
}