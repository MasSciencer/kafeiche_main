#ifndef STEPPER_MOTOR_WIRING_PI_H_
#define STEPPER_MOTOR_WIRING_PI_H_

#include <wiringPi.h>
#include <cmath>  // Для abs()
#include <thread>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"


constexpr uint8_t MOTOR_LEFT_DIR_PIN = 26;  // Pins
constexpr uint8_t MOTOR_LEFT_STEP_PIN = 13;
constexpr uint8_t MOTOR_ENABLE_PIN = 12; // Sharing enable

constexpr uint8_t MOTOR_RIGHT_DIR_PIN = 27;
constexpr uint8_t MOTOR_RIGHT_STEP_PIN = 4;

class StepperMotorWiringPi {
public:
    StepperMotorWiringPi(int8_t direction_pin, int8_t step_pin, int8_t enable_pin);
    void setSpeed(double target_vel);
    ~StepperMotorWiringPi();

private:
    void run();  // Функция, выполняющаяся в отдельном потоке
    uint16_t rpmToDelay(uint16_t current_rpm);  // Функция для преобразования скорости в задержку
    void stop();  // Объявление метода stop() (без тела)

    std::thread motor_thread_;  // Переменная для потока
    std::atomic<bool> running_;
    std::atomic<int16_t> current_rpm;
    int8_t _direction_pin;
    int8_t _step_pin;
    int8_t _enable_pin;
};


StepperMotorWiringPi::StepperMotorWiringPi(int8_t direction_pin, int8_t step_pin, int8_t enable_pin)
    : running_(true), current_rpm(0), _direction_pin(direction_pin), _step_pin(step_pin), _enable_pin(enable_pin) {
    
        if (wiringPiSetupGpio() == -1) {  // Используем GPIO номера пинов
        throw std::runtime_error("Failed to initialize wiringPi");
    }

    // Устанавливаем режимы пинов
    pinMode(_direction_pin, OUTPUT);
    pinMode(_step_pin, OUTPUT);
    pinMode(_enable_pin, OUTPUT);
    
    motor_thread_ = std::thread(&StepperMotorWiringPi::run, this);
}


StepperMotorWiringPi::~StepperMotorWiringPi() {
    running_ = false;
    if (motor_thread_.joinable()) {
        motor_thread_.join();
    }
}

// get in m/s and transform into rpm 
void StepperMotorWiringPi::setSpeed(double target_vel) { 
    current_rpm = target_vel / (2 * M_PI * 0.075) * 60 * 3.7; //0.075 - radius of wheel
    /*
    RCLCPP_INFO(rclcpp::get_logger("StepperMotorWiringPi"), 
                "Set speed: target velocity = %f, calculated RPM = %d", target_vel, current_rpm.load());
                */
}

void StepperMotorWiringPi::run() {
    while (running_) {
        if (std::abs(current_rpm) > 100) {
            // Ограничиваем current_rpm до 1200
            int16_t effective_rpm = std::min(std::abs(current_rpm), (int)1200);
            uint16_t step_delay = rpmToDelay(effective_rpm);
            
                        /*
                        RCLCPP_INFO(rclcpp::get_logger("StepperMotorWiringPi"),
                        "Motor running: current RPM = %d, effective RPM = %d, step delay = %d microseconds",
                        current_rpm.load(), effective_rpm, step_delay);
                        */
            
            digitalWrite(_enable_pin, LOW);
            digitalWrite(_direction_pin, current_rpm > 0 ? LOW : HIGH);

            digitalWrite(_step_pin, HIGH);
            delayMicroseconds(step_delay);
            digitalWrite(_step_pin, LOW);
            delayMicroseconds(step_delay);
        } else {
        
        /*
        RCLCPP_INFO(rclcpp::get_logger("StepperMotorWiringPi"), 
                        "Motor stopping: current RPM = %d", current_rpm.load());
        */
        
            stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }
}

void StepperMotorWiringPi::stop() {
    digitalWrite(_enable_pin, HIGH);
}

uint16_t StepperMotorWiringPi::rpmToDelay(uint16_t current_rpm) {
    uint16_t steps_per_revolution = 200;
    uint16_t steps_per_second = (current_rpm * steps_per_revolution) / 60;
    return 1000000 / steps_per_second;
}

#endif  // STEPPER_MOTOR_WIRING_PI_H_
