#ifndef _DREAMSTER_H_
#define _DREAMSTER_H_

#include "Arduino.h"
#include <stdint.h>

class Dreamster
{
  public:
    Dreamster();
    void scan(int &a, int &b, int &c);
    void scan_a(int &a);
    void scan_b(int &b);
    void scan_c(int &c);
    void read_ir(uint16_t &left, uint16_t &right);
    void move(int left, int right);
    void show(uint8_t red, uint8_t green, uint8_t blue);
    void update();
    void setup();
    void sleep(unsigned long msec);
    void calibrate_motors_zero(int left, int right);
    void set_sensor_a_active(bool active);
    void set_sensor_b_active(bool active);
    void set_sensor_c_active(bool active);

  private:
    int scan_sensor(int trigger, int echo);
    int scan_sensor_pulsein(int trigger, int echo);
    void move_motor(int p, int n, int16_t speed);
    // Ultrasound pingers
    const int us_trigger_a_ = A3;
    const int us_echo_a_ = 8;
    const int us_trigger_b_ = A4;
    const int us_echo_b_ = 2;
    const int us_trigger_c_ = A5;
    const int us_echo_c_ = 7;
    // Ir sensors
    const int ir_l_ = A1;
    const int ir_r_ = A0;
    // Leds
    const int led_r_ = 12;
    const int led_g_ = 11;
    const int led_b_ = 13;
    // motors
    const int left_motor_pin_ = 6;
    const int right_motor_pin_ = 9;
    int maximum_speed_ = 100; // 0 to 100
    const int value_from_center_ = 60; // pwm value
    const int initial_neutral_center_ = 184; // pwm value where servo is still
    int left_neutral_center_ = initial_neutral_center_; // pwm value
    int right_neutral_center_ = initial_neutral_center_; // pwm value
    
};

#endif
