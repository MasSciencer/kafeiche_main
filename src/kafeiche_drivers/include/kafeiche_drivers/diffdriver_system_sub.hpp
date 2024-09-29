#ifndef DIFFDRIVER_SYSTEM_SUB_HPP
#define DIFFDRIVER_SYSTEM_SUB_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#define RATE 10 //Gz

class DiffSubscriber : public rclcpp::Node {
public:
  DiffSubscriber() : Node("diff_sub") {
    // Подписка на топики для левого колеса
    left_wheel_velocity_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "/kfc/left_wheel/current_velocity", rclcpp::QoS(RATE),
      std::bind(&DiffSubscriber::left_wheel_velocity_callback, this, std::placeholders::_1));

    left_wheel_position_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "/kfc/left_wheel/position", rclcpp::QoS(RATE),
      std::bind(&DiffSubscriber::left_wheel_position_callback, this, std::placeholders::_1));

    // Подписка на топики для правого колеса
    right_wheel_velocity_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "/kfc/right_wheel/current_velocity", rclcpp::QoS(RATE),
      std::bind(&DiffSubscriber::right_wheel_velocity_callback, this, std::placeholders::_1));

    right_wheel_position_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "/kfc/right_wheel/position", rclcpp::QoS(RATE),
      std::bind(&DiffSubscriber::right_wheel_position_callback, this, std::placeholders::_1));
  }

  // Геттеры для получения значений извне
  double get_left_wheel_velocity() const { return wheel_l_.vel; }
  double get_left_wheel_position() const { return wheel_l_.pos; }
  double get_right_wheel_velocity() const { return wheel_r_.vel; }
  double get_right_wheel_position() const { return wheel_r_.pos; }

private:
  struct WheelData {
    double vel = 0.0;
    double pos = 0.0;
  };

  WheelData wheel_l_;
  WheelData wheel_r_;

  // Коллбеки для левого колеса
  void left_wheel_velocity_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    wheel_l_.vel = msg->data;
  }

  void left_wheel_position_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    wheel_l_.pos = msg->data;
  }

  // Коллбеки для правого колеса
  void right_wheel_velocity_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    wheel_r_.vel = msg->data;
  }

  void right_wheel_position_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    wheel_r_.pos = msg->data;
  }

  // Подписчики
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_wheel_velocity_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_wheel_position_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_wheel_velocity_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_wheel_position_subscriber_;
};

#endif  // DIFFDRIVER_SYSTEM_SUB_HPP
