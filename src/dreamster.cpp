#include "dreamster.h"

Dreamster::Dreamster(void)
{
}

void Dreamster::scan(int &a, int &b, int &c)
{
  a = scan_sensor(us_trigger_a_, us_echo_a_);
  b = scan_sensor(us_trigger_b_, us_echo_b_);
  c = scan_sensor(us_trigger_c_, us_echo_c_);
}

void Dreamster::scan_a(int &a)
{
  a = scan_sensor(us_trigger_a_, us_echo_a_);
}

void Dreamster::scan_b(int &b)
{
  b = scan_sensor(us_trigger_b_, us_echo_b_);
}

void Dreamster::scan_c(int &c)
{
  c = scan_sensor(us_trigger_c_, us_echo_c_);
}

void Dreamster::read_ir(uint16_t &left, uint16_t &right)
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

void Dreamster::move(int left, int right)
{
  rightMotor->setSpeed(right);
  leftMotor->setSpeed(left);
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

int Dreamster::scan_sensor(int trigger, int echo)
{
  long duration;
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  duration = pulseIn(echo, HIGH) / 5.8;
  return duration;
}

void Dreamster::update()
{
  // Update motor speed ramps
  leftMotor->update();
  rightMotor->update();
}

void Dreamster::setup()
{
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

  leftMotor = new Motor(LEFT_MOTOR_PIN, 1);
  rightMotor = new Motor(RIGHT_MOTOR_PIN, -1);

  Serial.begin(9600);
}

/**
 * Sleep for msec milliseconds. The robot keeps updating it's status.
 * @param msec Time in milliseconds to sleep.
 */
void Dreamster::sleep(unsigned long msec)
{
  unsigned long start = millis();
  while ((millis() - start) < msec)
  {
    update();
  }
}

Dreamster::Motor::Motor(int pin, int dir)
{
  servo.attach(pin);
  servo.write(90);
  currentSpeed = 0;
  targetSpeed = 0;
  this->dir = dir;
}

void Dreamster::Motor::setSpeed(int speed)
{
  if (speed < (-clampSpeed)) speed = -clampSpeed;
  if (speed > clampSpeed) speed = clampSpeed;

  targetSpeed = speed;
}

int Dreamster::Motor::getSpeed()
{
  return currentSpeed;
}

void Dreamster::Motor::update()
{
  if (currentSpeed < targetSpeed)
    currentSpeed += step;
  else
    if (currentSpeed > targetSpeed)
      currentSpeed -= step;

  servo.write(90 + dir * currentSpeed);
}
