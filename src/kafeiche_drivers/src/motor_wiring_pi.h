#ifndef STEPPER_MOTOR_WIRING_PI_H_
#define STEPPER_MOTOR_WIRING_PI_H_

#include <wiringPi.h>
#include <cmath>  // Для abs()
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <thread>
#include <atomic>

constexpr uint8_t MOTOR_LEFT_DIR_PIN = 26;  // Pins
constexpr uint8_t MOTOR_LEFT_STEP_PIN = 13;
constexpr uint8_t MOTOR_ENABLE_PIN = 12; // Sharing enable

constexpr uint8_t MOTOR_RIGHT_DIR_PIN = 27;
constexpr uint8_t MOTOR_RIGHT_STEP_PIN = 4;

class StepperMotorWiringPi {
public:
    StepperMotorWiringPi(int8_t direction_pin, int8_t step_pin, int8_t enable_pin);
    void setSpeed(int16_t target_rpm);
    ~StepperMotorWiringPi();

private:
    void run();  // Функция, выполняющаяся в отдельном потоке
    uint16_t rpmToDelay(uint16_t rpm);  // Функция для преобразования скорости в задержку
    void stop();  // Объявление метода stop() (без тела)

    std::thread motor_thread_;  // Переменная для потока
    std::atomic<bool> running_;
    std::atomic<int16_t> current_rpm_;
    int8_t _direction_pin;
    int8_t _step_pin;
    int8_t _enable_pin;
};


StepperMotorWiringPi::StepperMotorWiringPi(int8_t direction_pin, int8_t step_pin, int8_t enable_pin)
    : running_(true), current_rpm_(0), _direction_pin(direction_pin), _step_pin(step_pin), _enable_pin(enable_pin) {
    
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

void StepperMotorWiringPi::setSpeed(int16_t target_rpm) {
    current_rpm_ = target_rpm * 100;
}

void StepperMotorWiringPi::run() {
    while (running_) {
        int16_t rpm = current_rpm_;

        //hardware evaluation of min velocity
        if (std::abs(rpm) > 150) {
            uint16_t step_delay = rpmToDelay(std::abs(rpm));
            digitalWrite(_enable_pin, LOW);
            digitalWrite(_direction_pin, rpm > 0 ? LOW : HIGH);

            digitalWrite(_step_pin, HIGH);
            delayMicroseconds(step_delay);
            digitalWrite(_step_pin, LOW);
            delayMicroseconds(step_delay);

        //hardware evaluation of max velocity
        if (std::abs(rpm) > 1200) {
            uint16_t step_delay = rpmToDelay(1200);
            digitalWrite(_enable_pin, LOW);
            digitalWrite(_direction_pin, rpm > 0 ? LOW : HIGH);

            digitalWrite(_step_pin, HIGH);
            delayMicroseconds(step_delay);
            digitalWrite(_step_pin, LOW);
            delayMicroseconds(step_delay);

        } else {
            stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void StepperMotorWiringPi::stop() {
    digitalWrite(_enable_pin, HIGH);
}

uint16_t StepperMotorWiringPi::rpmToDelay(uint16_t rpm) {
    uint16_t steps_per_revolution = 200;
    uint16_t steps_per_second = (rpm * steps_per_revolution) / 60;
    return 1000000 / steps_per_second;
}

#endif  // STEPPER_MOTOR_WIRING_PI_H_
