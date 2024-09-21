#ifndef KAFEICHE_HARDWARE_INTERFACE_H_
#define KAFEICHE_HARDWARE_INTERFACE_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/joint_command_interface.hpp>
#include <hardware_interface/joint_state_interface.hpp>
#include <hardware_interface/robot_hw.hpp>

class KafeicheHardvareInterface : public hardware_interface::RobotHW {
public:
    KafeicheHardvareInterface(rclcpp::Node::SharedPtr node, double target_max_wheel_angular_speed);

    void updateJointsFromHardware(const rclcpp::Duration& period);
    void writeCommandsToHardware();

private:
    rclcpp::Node::SharedPtr _node;

    hardware_interface::JointStateInterface _joint_state_interface;
    hardware_interface::VelocityJointInterface _velocity_joint_interface;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _left_wheel_angle_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _right_wheel_angle_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _left_wheel_vel_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _right_wheel_vel_pub;

    struct Joint {
        double position;
        double position_offset;
        double velocity;
        double effort;
        double velocity_command;

        Joint() : position(0), velocity(0), effort(0), velocity_command(0) {}
    } _joints[2];

    double _left_wheel_angle;
    double _right_wheel_angle;
    double _max_wheel_angular_speed;

    void registerControlInterfaces();
    void leftWheelAngleCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void rightWheelAngleCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void limitDifferentialSpeed(double& diff_speed_left_side, double& diff_speed_right_side);
};

KafeicheHardvareInterface::KafeicheHardvareInterface(rclcpp::Node::SharedPtr node, double target_max_wheel_angular_speed)
    : _node(node), _max_wheel_angular_speed(target_max_wheel_angular_speed) {
    registerControlInterfaces();

    _left_wheel_vel_pub = _node->create_publisher<std_msgs::msg::Float64>("/kafeiche/left_wheel/target_velocity", 1);
    _right_wheel_vel_pub = _node->create_publisher<std_msgs::msg::Float64>("/kafeiche/right_wheel/target_velocity", 1);
    _left_wheel_angle_sub = _node->create_subscription<std_msgs::msg::Float64>("kafeiche/left_wheel/angle", 1,
        std::bind(&KafeicheHardvareInterface::leftWheelAngleCallback, this, std::placeholders::_1));
    _right_wheel_angle_sub = _node->create_subscription<std_msgs::msg::Float64>("kafeiche/right_wheel/angle", 1,
        std::bind(&KafeicheHardvareInterface::rightWheelAngleCallback, this, std::placeholders::_1));
}

void KafeicheHardvareInterface::writeCommandsToHardware() {
    double diff_angle_speed_left = _joints[0].velocity_command;
    double diff_angle_speed_right = _joints[1].velocity_command;

    limitDifferentialSpeed(diff_angle_speed_left, diff_angle_speed_right);

    auto left_wheel_vel_msg = std_msgs::msg::Float64();
    auto right_wheel_vel_msg = std_msgs::msg::Float64();

    left_wheel_vel_msg.data = diff_angle_speed_left;
    right_wheel_vel_msg.data = diff_angle_speed_right;

    _left_wheel_vel_pub->publish(left_wheel_vel_msg);
    _right_wheel_vel_pub->publish(right_wheel_vel_msg);
}

void KafeicheHardvareInterface::updateJointsFromHardware(const rclcpp::Duration& period) {
    double delta_left_wheel = _left_wheel_angle - _joints[0].position - _joints[0].position_offset;
    double delta_right_wheel = _right_wheel_angle - _joints[1].position - _joints[1].position_offset;

    if (std::abs(delta_left_wheel) < 1) {
        _joints[0].position += delta_left_wheel;
        _joints[0].velocity = delta_left_wheel / period.seconds();
    }
    else {
        _joints[0].position_offset += delta_left_wheel;
    }

    if (std::abs(delta_right_wheel) < 1) {
        _joints[1].position += delta_right_wheel;
        _joints[1].velocity = delta_right_wheel / period.seconds();
    }
    else {
        _joints[1].position_offset += delta_right_wheel;
    }
}

void KafeicheHardvareInterface::registerControlInterfaces() {
    std::vector<std::string> joint_names = { "left_wheel_joint", "right_wheel_joint" };

    for (size_t i = 0; i < joint_names.size(); ++i) {
        hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &_joints[i].position, &_joints[i].velocity, &_joints[i].effort);
        _joint_state_interface.registerHandle(joint_state_handle);

        hardware_interface::JointHandle joint_handle(joint_state_handle, &_joints[i].velocity_command);
        _velocity_joint_interface.registerHandle(joint_handle);
    }
    registerInterface(&_joint_state_interface);
    registerInterface(&_velocity_joint_interface);
}

void KafeicheHardvareInterface::leftWheelAngleCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    _left_wheel_angle = msg->data;
}

void KafeicheHardvareInterface::rightWheelAngleCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    _right_wheel_angle = msg->data;
}

void KafeicheHardvareInterface::limitDifferentialSpeed(double& diff_speed_left_side, double& diff_speed_right_side) {
    double large_speed = std::max(std::abs(diff_speed_left_side), std::abs(diff_speed_right_side));
    if (large_speed > _max_wheel_angular_speed) {
        diff_speed_left_side *= _max_wheel_angular_speed / large_speed;
        diff_speed_right_side *= _max_wheel_angular_speed / large_speed;
    }
}

#endif // KAFEICHE_HARDWARE_INTERFACE_H_
