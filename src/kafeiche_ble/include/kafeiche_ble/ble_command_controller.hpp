#ifndef BLE_COMMAND_CONTROLLER_HPP
#define BLE_COMMAND_CONTROLLER_HPP

#include <memory>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <chrono>
#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

//getter hcitool for RSSI through Bluetooth
int get_rssi(const std::string& hci_device);

class BleCommandController : public rclcpp::Node {
public:
    BleCommandController();

private:
    void timer_callback();
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // BLE_COMMAND_CONTROLLER_HPP
