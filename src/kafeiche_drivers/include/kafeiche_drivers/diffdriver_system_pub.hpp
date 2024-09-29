#ifndef DIFFDRIVER_SYSTEM_PUB_HPP
#define DIFFDRIVER_SYSTEM_PUB_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>


#define RATE 10 // Gz

class DiffPublisher : public rclcpp::Node {
public:
  DiffPublisher() : Node("diff_pub"), left_vel_(0.0), right_vel_(0.0) {
    // Создаем паблишеры для левого и правого колес
    left_vel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/kfc/left_wheel/target_velocity", rclcpp::QoS(RATE));
    right_vel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/kfc/right_wheel/target_velocity", rclcpp::QoS(RATE));

    // Создаем таймер для периодической публикации
    timer_ = this->create_wall_timer(
       std::chrono::milliseconds(100), std::bind(&DiffPublisher::timer_callback, this));
  }

  // Метод для обновления значений vel
  void update_velocities(double left_vel, double right_vel) {
    left_vel_ = left_vel;
    right_vel_ = right_vel;
  }

private:
  void timer_callback() {
    // Создаем сообщения
    std_msgs::msg::Float64 left_vel_msg;
    std_msgs::msg::Float64 right_vel_msg;

    // Заполняем сообщения
    left_vel_msg.data = left_vel_;
    right_vel_msg.data = right_vel_;

    // Публикуем сообщения
    left_vel_pub_->publish(left_vel_msg);
    right_vel_pub_->publish(right_vel_msg);
  }

  // Паблишеры
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_vel_pub_;

  // Значения vel
  double left_vel_;
  double right_vel_;

  // Таймер
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // DIFFDRIVER_SYSTEM_PUB_HPP

