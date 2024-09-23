#include "kafeiche_drivers/diffdriver_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "control_toolbox/pid.hpp"


namespace kafeiche_drivers
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Implementation sub and creation of pub
        auto node = rclcpp::Node::make_shared("diffdriver_node");

        left_velocity_sub_ = node->create_subscription<std_msgs::msg::Float64>(
            "/kfc/left_wheel/current_velocity", rclcpp::QoS(10),
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                wheel_l_.vel = msg->data;
            });

        left_position_sub_ = node->create_subscription<std_msgs::msg::Float64>(
            "/kfc/left_wheel/position", rclcpp::QoS(10),
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                wheel_l_.pos = msg->data;
            });
        right_velocity_sub_ = node->create_subscription<std_msgs::msg::Float64>(
            "/kfc/right_wheel/current_velocity", rclcpp::QoS(10),
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                wheel_r_.vel = msg->data;
            });

        right_position_sub_ = node->create_subscription<std_msgs::msg::Float64>(
            "/kfc/right_wheel/position", rclcpp::QoS(10),
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                wheel_r_.pos = msg->data;
            });

        left_rpm_pub_ = node->create_publisher<std_msgs::msg::Float64>(
            "/kfc/left_wheel/rpm", rclcpp::QoS(10));
        right_rpm_pub_ = node->create_publisher<std_msgs::msg::Float64>(
            "/kfc/right_wheel/rpm", rclcpp::QoS(10));



        cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
        cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];

        for (const hardware_interface::ComponentInfo& joint : info_.joints)
        {
            // DiffBotSystem has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffDriverKfcHardware"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffDriverKfcHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffDriverKfcHardware"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffDriverKfcHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffDriverKfcHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> DiffDriverKfcHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> DiffDriverKfcHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn DiffDriveKfcHardware::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {

        pid_left_.initPid(P_p, I_p, D_p, i_max_p, i_min_p);
        pid_right_.initPid(P_p, I_p, D_p, i_max_p, i_min_p); 
        last_time_ = this->now();
        last_period_ = rclcpp::Duration(0);

        return hardware_interface::CallbackReturn::SUCCESS;
    }


    hardware_interface::return_type DiffDriverKfcHardware::read(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
    {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type diffdriver::DiffDriverKfcHardware::write(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {

        static rclcpp::Time last_time = time;
        double dt = (time - last_time).nanoseconds() * 1e-9;  // Переводим в секунды
        last_time = time;

         // Publisher RPM
        std_msgs::msg::Float64 left_rpm_msg;
        left_rpm_msg.data = pid_left_.computeCommand(wheel_l_.cmd - wheel_l_.vel, dt); // Сглаживание значений через PID for left
        left_rpm_pub_->publish(left_rpm_msg);

        std_msgs::msg::Float64 right_rpm_msg;
        right_rpm_msg.data = pid_right_.computeCommand(wheel_r_.cmd - wheel_r_.vel, dt); // Сглаживание значений через PID for right
        right_rpm_pub_->publish(right_rpm_msg);

        return hardware_interface::return_type::OK;
};
}  // end namespace

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    diffdriver::DiffDriverKfcHardware, hardware_interface::SystemInterface)
