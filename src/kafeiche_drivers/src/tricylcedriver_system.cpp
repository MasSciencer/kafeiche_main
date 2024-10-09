#include "kafeiche_drivers/diffdriver_system.hpp"
#include "kafeiche_drivers/diffdriver_system_sub.hpp"
#include "kafeiche_drivers/diffdriver_system_pub.hpp"

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
hardware_interface::CallbackReturn TricyKfc::on_init(
  const hardware_interface::HardwareInfo & info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
        cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
        cfg_.front_wheel_name = info_.hardware_parameters["front_wheel_name"];

        wheel_l_.setup(cfg_.left_wheel_name);
        wheel_r_.setup(cfg_.right_wheel_name);
        wheel_f_.setup(cfg_.front_wheel_name);

        for (const hardware_interface::ComponentInfo& joint : info_.joints)
        {
            // tricycle_steering_controller has exactly two states and two command interface on each joint
            if (joint.command_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("TricyKfc"),
                    "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("TricyKfc"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("TricyKfc"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("TricyKfc"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("TricyKfc"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> TricyKfc::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_f_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> TricyKfc::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_f_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.cmd));

        return command_interfaces;
    }
    

    hardware_interface::CallbackReturn TricyKfc::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // Adjustment of pid
        // pid_left_.initPid(P_p, I_p, D_p, i_max_p, i_min_p);
        // pid_right_.initPid(P_p, I_p, D_p, i_max_p, i_min_p);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    
    hardware_interface::return_type TricyKfc::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // Экземпляр TricySubscriber and TricyPublisher
    static auto control_subscriber = std::make_shared<TricySubscriber>();

    // Читаем данные из подписчиков
    wheel_l_.vel = control_subscriber->get_left_wheel_velocity();
    wheel_r_.vel = control_subscriber->get_right_wheel_velocity();
    wheel_f_.pos = wheel_f_.cmd;

    return hardware_interface::return_type::OK;
}


hardware_interface::return_type TricyKfc::write(
    const rclcpp::Time& time, const rclcpp::Duration& period)
{ 
    static auto control_publisher = std::make_shared<TricyPublisher>();
    
    // Инициализация времени
    static rclcpp::Time last_time = time; 

    // Вычисляем временной интервал (dt)
    double dt = period.seconds();  // in second

    // Обновляем значения vel для левого и правого колес
    //double left_msg_data = pid_left_.computeCommand(wheel_l_.cmd - wheel_l_.vel, dt);
    //double right_msg_data = pid_right_.computeCommand(wheel_r_.cmd - wheel_r_.vel, dt);
    
    double left_msg_data = wheel_l_.cmd;
    double right_msg_data = wheel_r_.cmd;

    control_publisher->update_velocities(left_msg_data, right_msg_data);
    control_publisher->process();
    
    // Обновляем время
    last_time = time;  

    return hardware_interface::return_type::OK;
}

}  // end namespace

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    kafeiche_drivers::TricyKfc, hardware_interface::SystemInterface)
