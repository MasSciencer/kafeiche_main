#ifndef DIFFDRIVER_SYSTEM_HPP
#define DIFFDRIVER_SYSTEM_HPP
#define P_p 6.0 //param p The proportional gain.
#define I_p 1.0 //param i The integral gain.
#define D_p 2.0 //param d The derivative gain.
#define i_max_p 0.3 //param i_max The max integral windup.
#define i_min_p -0.3 //param i_min The min integral windup.

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "diffdriver_system_wheel.hpp"
#include "control_toolbox/pid.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64.hpp"



namespace kafeiche_drivers
{
class DiffKfc : public hardware_interface::SystemInterface
{

struct Config
{
            std::string left_wheel_name = "";
            std::string right_wheel_name = "";
            int pid_p = 0;
            int pid_d = 0;
            int pid_i = 0;
            int pid_o = 0;
};


    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(DiffKfc)

            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo& info) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State& previous_state) override;
                
            hardware_interface::return_type read(
                const rclcpp::Time& time, const rclcpp::Duration& period) override;

            hardware_interface::return_type write(
                const rclcpp::Time& time, const rclcpp::Duration& period) override;


    private:

        Config cfg_;
        Wheel wheel_l_;
        Wheel wheel_r_;

        // Implementation PID-control
        control_toolbox::Pid pid_left_;  // call class Pid for left wheel
        control_toolbox::Pid pid_right_; // call class Pid for right wheel
};
}


#endif  // DIFFDRIVER_SYSTEM_H_
