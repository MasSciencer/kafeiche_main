#ifndef STEPPER_MOTOR_WIRING_PI_H_
#define STEPPER_MOTOR_WIRING_PI_H_

#include <wiringPi.h>
#include <cmath>  // Для abs()
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

constexpr uint8_t MOTOR_LEFT_DIR_PIN = 26;  // Pins
constexpr uint8_t MOTOR_LEFT_STEP_PIN = 13;
constexpr uint8_t MOTOR_ENABLE_PIN = 12; // Sharing enable

constexpr uint8_t MOTOR_RIGHT_DIR_PIN = 27;
constexpr uint8_t MOTOR_RIGHT_STEP_PIN = 4;
constexpr uint16_t RPI_MAX_STEPS_PER_SEC = 1000; // Количество шагов для проверки данных. Меньше шагов - чаще проверяются

class StepperMotorWiringPi {
public:
    StepperMotorWiringPi(int8_t direction_pin, int8_t step_pin, int8_t enable_pin);
    void rotateCW(uint16_t rpm);
    void rotateCCW(uint16_t rpm);
    void stop();
    void setSpeed(int16_t target_rpm);
private:
    int8_t _direction_pin;
    int8_t _step_pin;
    int8_t _enable_pin;
    uint16_t rpmToDelay(uint16_t rpm);  // Перевод RPM в задержку между шагами
};

StepperMotorWiringPi::StepperMotorWiringPi(int8_t direction_pin, int8_t step_pin, int8_t enable_pin)
    : _direction_pin(direction_pin), _step_pin(step_pin), _enable_pin(enable_pin){
    if (wiringPiSetupGpio() < 0) {
        throw std::runtime_error("StepperMotor wiringPi error: GPIO setup error");
    }
    pinMode(_direction_pin, OUTPUT);
    pinMode(_step_pin, OUTPUT);
    pinMode(_enable_pin, OUTPUT);
    stop();  // Останавливаем двигатель при инициализации
}

void StepperMotorWiringPi::stop() {
    digitalWrite(_enable_pin, HIGH);  // Отключаем мотор
}

void StepperMotorWiringPi::rotateCW(uint16_t rpm) {
    digitalWrite(_direction_pin, LOW);
    uint16_t step_delay = rpmToDelay(rpm);
    digitalWrite(_enable_pin, LOW);  // Включаем мотор

    // Вращаемся пока не скажут остановиться
    for (int i = 0; i < RPI_MAX_STEPS_PER_SEC; i++) {
        digitalWrite(_step_pin, HIGH);
        delayMicroseconds(step_delay);
        digitalWrite(_step_pin, LOW);
        delayMicroseconds(step_delay);
    }
}

void StepperMotorWiringPi::rotateCCW(uint16_t rpm) {
    digitalWrite(_direction_pin, HIGH);
    uint16_t step_delay = rpmToDelay(rpm);
    digitalWrite(_enable_pin, LOW);  // Включаем мотор

    for (int i = 0; i < RPI_MAX_STEPS_PER_SEC; i++) {
        digitalWrite(_step_pin, HIGH);
        delayMicroseconds(step_delay);
        digitalWrite(_step_pin, LOW);
        delayMicroseconds(step_delay);
    }
}

void StepperMotorWiringPi::setSpeed(int16_t target_rpm) {
    target_rpm = target_rpm * 100; //Возвращаем нормализованную скорость вращения с PID из диапазона -12.0 -- 12 до соответсвия реальным физическим значениям -1200 -- 1200 RPM
    if (target_rpm > 0.2) {
        rotateCW(target_rpm);
    }
    else if (target_rpm < - 0.2) {
        rotateCCW(abs(target_rpm));
    }
    else {
        stop();
    }
}

//Основная идея регулировки с корости в ширине задержек между импульсами. Больше шаг - меньше скорость
uint16_t StepperMotorWiringPi::rpmToDelay(uint16_t rpm) {
    uint16_t steps_per_revolution = 200;  // Полный шаг: 200 шагов на оборот
    uint16_t steps_per_second = (rpm * steps_per_revolution) / 60;
    return 1000000 / steps_per_second;  // Возвращаем задержку в микросекундах
}

#endif  // STEPPER_MOTOR_WIRING_PI_H_
