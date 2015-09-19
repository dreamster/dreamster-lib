#include "dreamster.h"

Dreamster::Dreamster(void)
{
  pinMode(motor_left_n_, OUTPUT);
  pinMode(motor_left_p_, OUTPUT);
  pinMode(motor_right_n_, OUTPUT);
  pinMode(motor_right_p_, OUTPUT);
  pinMode(us_trigger_a_, OUTPUT);

  pinMode(us_echo_a_, INPUT);
  pinMode(us_trigger_b_, OUTPUT);
  pinMode(us_echo_b_, INPUT);
  pinMode(us_trigger_c_, OUTPUT);
  pinMode(us_echo_c_, INPUT);

  pinMode(ir_r_, INPUT);
  pinMode(ir_l_, INPUT);

  pinMode(led_r_, OUTPUT);
  pinMode(led_g_, OUTPUT);
  pinMode(led_b_, OUTPUT);

  Serial.begin(9600);
}

void Dreamster::scan(uint16_t &a, uint16_t &b, uint16_t &c)
{
  a = scan_sensor(us_trigger_b_, us_echo_b_);
  b = scan_sensor(us_trigger_a_, us_echo_a_);
  c = scan_sensor(us_trigger_c_, us_echo_c_);
}

void Dreamster::read(uint16_t &left, uint16_t &right)
{
  left = analogRead(ir_l_);
  right = analogRead(ir_r_);
}

void Dreamster::show(uint8_t red, uint8_t green, uint8_t blue)
{
  analogWrite(led_r_, red);
  analogWrite(led_g_, green);
  analogWrite(led_b_, blue);
}

void Dreamster::move(int8_t left, int8_t right)
{
  int8_t l = constrain(left, -100, 100);
  int8_t r = constrain(right, -100, 100);
  move_motor(motor_left_n_, motor_left_p_, l);
  move_motor(motor_right_n_, motor_right_p_, r);
}

void Dreamster::move_motor(int p, int n, int16_t speed)
{
  uint8_t s;
  if (speed > 0) {
    s = map(speed, 1, 100, 0, 255);
    analogWrite(n, (uint8_t) s);
    digitalWrite(p, LOW);
  } else {
    s = map(-speed, 0, 100, 0, 255);
    analogWrite(p, (uint8_t) s);
    digitalWrite(n, LOW);
  }
}

uint16_t Dreamster::scan_sensor(int trigger, int echo)
{
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  return (uint16_t) pulseIn(echo, HIGH) / 5.8;
}
