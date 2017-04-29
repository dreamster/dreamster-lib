#ifndef _DREAMSTER_H_
#define _DREAMSTER_H_

#include "Arduino.h"
#include "Servo/Servo.h"
#include <stdint.h>

#define LEFT_MOTOR_PIN 6
#define RIGHT_MOTOR_PIN 9

class Dreamster
{
  public:
    class Motor {
      Servo servo;
      int currentSpeed;
      int targetSpeed;
      int dir;
      const int step = 1;
      const int clampSpeed = 80;

      public:
      Motor(int pin, int dir);
      void setSpeed(int speed);
      int getSpeed();
      void update();
    };

    Motor *rightMotor;
    Motor *leftMotor;

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

  private:

    int scan_sensor(int trigger, int echo);
    void move_motor(int p, int n, int16_t speed);
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
