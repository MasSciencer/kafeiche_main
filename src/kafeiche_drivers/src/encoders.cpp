f#include "encoder_wiring_pi.h"
#include "std_msgs/msg/float64.hpp"

typedef std::chrono::steady_clock time_source;

class EncodersPair : public rclcpp::Node {
public:
    EncodersPair(float update_rate);

private:
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _left_wheel_position_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _right_wheel_position_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _left_wheel_velocity_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _right_wheel_velocity_pub;

    // Timer
    rclcpp::TimerBase::SharedPtr _encoders_timer;

    // Messages
    std_msgs::msg::Float64 _left_wheel_position_msg;
    std_msgs::msg::Float64 _right_wheel_position_msg;
    std_msgs::msg::Float64 _left_wheel_velocity_msg;
    std_msgs::msg::Float64 _right_wheel_velocity_msg;

    // Encoder objects
    EncoderWiringPi _encoder_left;
    EncoderWiringPi _encoder_right;

    // Variables
    float _left_wheel_delta;
    float _right_wheel_delta;
    float _left_wheel_velocity;
    float _right_wheel_velocity;
    float _left_wheel_position;
    float _right_wheel_position;

    time_source::time_point _last_time;

    // Callback function
    void encodersCallback();
};

EncodersPair::EncodersPair(float update_rate) :
    Node("encoders_pair"),
    _encoder_left(),
    _encoder_right(),
    _left_wheel_position(0.0f),
    _right_wheel_position(0.0f)
{
    // Publishers
    _left_wheel_velocity_pub = this->create_publisher<std_msgs::msg::Float64>("/kfc/left_wheel/current_velocity", 10);
    _right_wheel_velocity_pub = this->create_publisher<std_msgs::msg::Float64>("/kfc/right_wheel/current_velocity", 10);
    _left_wheel_position_pub = this->create_publisher<std_msgs::msg::Float64>("/kfc/left_wheel/position", 10);
    _right_wheel_position_pub = this->create_publisher<std_msgs::msg::Float64>("/kfc/right_wheel/position", 10);

    // Timer with a defined update rate
    auto period = std::chrono::duration<float>(update_rate);
    _encoders_timer = this->create_wall_timer(period, std::bind(&EncodersPair::encodersCallback, this));

    _last_time = time_source::now();
}

void EncodersPair::encodersCallback() {
    time_source::time_point this_time = time_source::now();
    std::chrono::duration<double> elapsed_duration = this_time - _last_time;
    _last_time = this_time;

    // Get delta from encoders
    _left_wheel_delta = _encoder_left.get_del_left() / RATIO * RADIUS; //Ratio of stepper motor based on gearbox ration 
    _right_wheel_delta = _encoder_right.get_del_right() / RATIO * RADIUS;

    // Calculate position
    _left_wheel_position += _left_wheel_delta; //m
    _right_wheel_position += _right_wheel_delta; //m

    // Calculate velocity
    if (elapsed_duration.count() > 0) {
        _left_wheel_velocity = _left_wheel_delta / elapsed_duration.count() * RADIUS; //m/s
        _right_wheel_velocity = _right_wheel_delta / elapsed_duration.count() * RADIUS; //m/s
    } else {
        _left_wheel_velocity = 0.0f;
        _right_wheel_velocity = 0.0f;
    }

    // Publish velocity
    _left_wheel_velocity_msg.data = _left_wheel_velocity;
    _right_wheel_velocity_msg.data = _right_wheel_velocity;
    _left_wheel_velocity_pub->publish(_left_wheel_velocity_msg);
    _right_wheel_velocity_pub->publish(_right_wheel_velocity_msg);

    // Publish position
    _left_wheel_position_msg.data = _left_wheel_position;
    _right_wheel_position_msg.data = _right_wheel_position;
    _left_wheel_position_pub->publish(_left_wheel_position_msg);
    _right_wheel_position_pub->publish(_right_wheel_position_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EncodersPair>(0.02);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

