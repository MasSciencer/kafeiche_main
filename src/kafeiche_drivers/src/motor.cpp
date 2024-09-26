#include "stepper_motor_wiring_pi.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class StepperMotorNode : public rclcpp::Node {
public:
    StepperMotorNode()
        : Node("motor_sub")
    {
        left_motor_ = std::make_shared<StepperMotorWiringPi>(MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_STEP_PIN, MOTOR_ENABLE_PIN);
        right_motor_ = std::make_shared<StepperMotorWiringPi>(MOTOR_RIGHT_DIR_PIN, MOTOR_RIGHT_STEP_PIN, MOTOR_ENABLE_PIN);

        // Подписчики для левого и правого двигателей
        left_motor_target_vel_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/kfc/left_wheel/rpm", rclcpp::QoS(10), std::bind(&StepperMotorNode::leftMotorCallback, this, std::placeholders::_1));

        right_motor_target_vel_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/kfc/right_wheel/rpm", rclcpp::QoS(10), std::bind(&StepperMotorNode::rightMotorCallback, this, std::placeholders::_1));
    }

private:
    void leftMotorCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        int16_t target_rpm = static_cast<int16_t>(msg->data);
        left_motor_->setSpeed(target_rpm);
    }

    void rightMotorCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        int16_t target_rpm = static_cast<int16_t>(msg->data);
        right_motor_->setSpeed(target_rpm);
    }

    std::shared_ptr<StepperMotorWiringPi> left_motor_;
    std::shared_ptr<StepperMotorWiringPi> right_motor_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_motor_target_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_motor_target_vel_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StepperMotorNode>());
    rclcpp::shutdown();
    return 0;
}
