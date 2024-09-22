#ifndef ENCODER_WIRING_PI_H_
#define ENCODER_WIRING_PI_H_

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <cstdint>
#include <chrono>

#define CHANNEL0 0
#define CHANNEL1 1
#define SPI_SPEED 200000   // Скорость SPI (0.2 MHz)
#define REGx03 0x83
#define REGx04 0x84
#define MT6816_CPR 16384
#define RATIO 3.7
#define RADIUS 0.075

class EncoderWiringPi {
public:
    EncoderWiringPi();

    // Публичная функция для получения угла
    float get_angle_left();
    float get_angle_right();
    float get_del_left();
    float get_del_right();

private:
    uint16_t get_0x03;
    uint16_t get_0x04;
    uint16_t raw_angle;
    float ex_angle_left;
    float ex_angle_right;
    unsigned char buffer0[2];
    float current_angle = 0;
    float del_angle = 0;

    // Приватная функция для получения "сырого" угла
    uint16_t get_raw_angle(int channel);
};

EncoderWiringPi::EncoderWiringPi() {
    wiringPiSPISetupMode(CHANNEL1, SPI_SPEED, 3);
    wiringPiSPISetupMode(CHANNEL0, SPI_SPEED, 3);

    ex_angle_left = get_angle_left();
    ex_angle_right = get_angle_right();
}

uint16_t EncoderWiringPi::get_raw_angle(int channel) {
    buffer0[0] = REGx03;
    buffer0[1] = 0x0;
    wiringPiSPIDataRW(channel, buffer0, 2);
    get_0x03 = buffer0[1];

    buffer0[0] = REGx04;
    buffer0[1] = 0x0;
    wiringPiSPIDataRW(channel, buffer0, 2);
    get_0x04 = buffer0[1];

    raw_angle = (get_0x03 << 6) | (get_0x04 >> 2);
    return raw_angle;
}

float EncoderWiringPi::get_angle_left() {
    float current_angle = get_raw_angle(CHANNEL1) * 2 * M_PI / MT6816_CPR;
    return current_angle;
}

float EncoderWiringPi::get_angle_right() {
    float current_angle = get_raw_angle(CHANNEL0) * 2 * M_PI / MT6816_CPR;
    return current_angle;
}

float EncoderWiringPi::get_del_left() {
    current_angle = get_angle_left();
    del_angle = current_angle - ex_angle_left;

    // Коррекция при полном обороте назад
    if (del_angle > M_PI) {
        del_angle = del_angle - (2 * M_PI);
    }
    // Коррекция при обороте вперед
    else if (del_angle < -M_PI) {
        del_angle = del_angle + (2 * M_PI);
    }
    ex_angle_left = current_angle;
    return del_angle;
}

float EncoderWiringPi::get_del_right() {
    current_angle = get_angle_right();
    del_angle = -1 * (current_angle - ex_angle_right);

    // Коррекция при полном обороте назад
    if (del_angle > M_PI) {
        del_angle = del_angle - (2 * M_PI);
    }
    // Коррекция при обороте вперед
    else if (del_angle < -M_PI) {
        del_angle = del_angle + (2 * M_PI);
    }
    
    ex_angle_right = current_angle;
    return del_angle;
}

#endif // ENCODER_WIRING_PI_H_

