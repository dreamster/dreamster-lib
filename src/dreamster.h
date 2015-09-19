#ifndef _DREAMSTER_H_
#define _DREAMSTER_H_

#include "Arduino.h"
#include <stdint.h>

class Dreamster
{
  public:
    Dreamster();

    void scan(uint16_t &a, uint16_t &b, uint16_t &c);
    void read(uint16_t &left, uint16_t &right);
    void move(int8_t left, int8_t right);
    void show(uint8_t red, uint8_t green, uint8_t blue);

  private:

    uint16_t scan_sensor(int trigger, int echo);
    void move_motor(int p, int n, int16_t speed);

    // Motor outputs
    const int motor_right_p_ = 9;
    const int motor_right_n_ = 10;
    const int motor_left_p_ = 5;
    const int motor_left_n_ = 6;
    // Ultrasound pingers
    const int us_trigger_a_ = A3;
    const int us_echo_a_ = 8;
    const int us_trigger_b_ = A4;
    const int us_echo_b_ = 2;
    const int us_trigger_c_ = A5;
    const int us_echo_c_ = 7;
    // Ir sensors
    const int ir_r_ = A1;
    const int ir_l_ = A0;
    // Leds
    const int led_r_ = 12;
    const int led_g_ = 11;
    const int led_b_ = 13;
};

#endif
