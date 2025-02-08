/**
   A robot that mantains a distance from a frontal obstacle.
   Author: Ernesto Corbellini, Misael.K
*/

#include <dreamster.h>

Dreamster robot;

const int MOTOR_SPEED = 100;

int dist_us_a;

uint16_t ir_left;
uint16_t ir_right;

int safe_floor_value;

void setup() {
  robot.setup();
  robot.calibrate_motors_zero(-5, -5);

  // only use frontal us sensor
  robot.set_sensor_a_active(true);
  robot.set_sensor_b_active(false);
  robot.set_sensor_c_active(false);

  delay(100);

  // set floor limit as initial position
  robot.read_ir(ir_left, ir_right);
  // if one of the sensors is dead, copy the value from the working one
  if (ir_right == 0) ir_right = ir_left;
  if (ir_left == 0) ir_left = ir_right;
  safe_floor_value = (ir_left + ir_right) / 2;
}

void loop() {
  int left_motor_speed = 0;
  int right_motor_speed = 0;

  robot.scan_a(dist_us_a);
  robot.read_ir(ir_left, ir_right);
  
  // if one of the sensors is dead, copy the value from the working one
  if (ir_right == 0) ir_right = ir_left;
  if (ir_left == 0) ir_left = ir_right;
  int avg_ir = (ir_left + ir_right) / 2;
  int abs_diff = abs(avg_ir - safe_floor_value);

  // print sensors for debugging
  Serial.print(dist_us_a); Serial.print(" ");
  Serial.print(ir_left);   Serial.print(" ");
  Serial.print(ir_right);  Serial.print(" ");
  Serial.print(safe_floor_value);  Serial.print(" ");
  Serial.print(abs_diff);  Serial.print(" ");
  Serial.println("");

  dist_us_a *= 10;

  // If the infrared sensors show the table continue, check for danger
  if (abs_diff < 200) {
    robot.show(0, 0, 0);
    // If too close, go backwards
    if (dist_us_a > 0 && dist_us_a < 80) {
      left_motor_speed = -MOTOR_SPEED;
      right_motor_speed = -MOTOR_SPEED;
      robot.show(255, 0, 0);
    }
    // If we are far enough then stop
    else if (dist_us_a > 120 && dist_us_a < 200) {
      left_motor_speed = MOTOR_SPEED;
      right_motor_speed = MOTOR_SPEED;
      robot.show(0, 255, 0);
    } else {
      left_motor_speed = 0;
      right_motor_speed = 0;
    }
  // If the sensors show the table is over, then stay there
  } else {
    robot.show(0, 0, 20);
    if (dist_us_a < 80) {
      left_motor_speed = -MOTOR_SPEED;
      right_motor_speed = -MOTOR_SPEED;
      robot.show(255, 0, 0);
    } else {
      left_motor_speed = 0;
      right_motor_speed = 0;
    }
  }
  robot.move(left_motor_speed, right_motor_speed);
  delay(1); // ensures cycles every 1 ms at least
}
