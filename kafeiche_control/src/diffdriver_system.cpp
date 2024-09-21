#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
#include "diffdriver_system.h"

using time_source = std::chrono::steady_clock;

void controlLoop(KafeicheHardvareInterface& hardware, controller_manager::ControllerManager& cm, time_source::time_point& last_time) {
    time_source::time_point this_time = time_source::now();
    auto elapsed_duration = this_time - last_time;
    rclcpp::Duration elapsed(std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed_duration));
    last_time = this_time;

    hardware.updateJointsFromHardware(elapsed);
    cm.update(rclcpp::Time::now(), elapsed);
    hardware.writeCommandsToHardware();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("abot_base");
    auto private_node = std::make_shared<rclcpp::Node>("abot_base_private");

    int control_frequency;
    double max_wheel_angular_speed;

    private_node->declare_parameter<int>("control_frequency", 1);
    private_node->declare_parameter<double>("max_wheel_angular_speed", 1.0);
    private_node->get_parameter("control_frequency", control_frequency);
    private_node->get_parameter("max_wheel_angular_speed", max_wheel_angular_speed);

    KafeicheHardvareInterface hardware(node, max_wheel_angular_speed);
    controller_manager::ControllerManager cm(&hardware, node);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(private_node);

    time_source::time_point last_time = time_source::now();

    rclcpp::TimerOptions control_timer_options;
    control_timer_options.callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_timer_options.period = std::chrono::milliseconds(static_cast<int>(1000 / control_frequency));

    auto control_timer = node->create_wall_timer(control_timer_options.period,
        [&]() { controlLoop(hardware, cm, last_time); });

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
