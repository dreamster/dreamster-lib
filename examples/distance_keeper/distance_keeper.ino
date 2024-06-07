/**
   A robot that mantains a distance from a frontal obstacle.
   Author: Ernesto Corbellini, Misael.K
*/

#include <dreamster.h>

Dreamster robot;

int dist_us_a;

uint16_t ir_left;
uint16_t ir_right;

const int floor_limit = 15;

// use this to calibrate the motors
const int left_motor_offset = -2;
const int right_motor_offset = 1;

void setup() {
  robot.setup();
}

void loop() {
  int left_motor_speed = 0;
  int right_motor_speed = 0;

  robot.scan_a(dist_us_a);
  robot.read_ir(ir_left, ir_right);
  
  // if one of the sensors is dead, copy the value from the working one
  if (ir_right == 0) ir_right = ir_left;
  if (ir_left == 0) ir_left = ir_right;

  // print sensors for debugging
  Serial.print(dist_us_a); Serial.print(" ");
  Serial.print(ir_left);   Serial.print(" ");
  Serial.print(ir_left);  Serial.print(" ");
  Serial.println("");

  // If the infrared sensors show the table continue, check for danger
  if (ir_left > floor_limit && ir_right > floor_limit) {
    // If too close, go backwards
    if (dist_us_a < 80) {
      left_motor_speed = -20;
      right_motor_speed = -20;
    }
    // If we are far enough then stop
    else if (dist_us_a > 120 && dist_us_a < 200) {
      left_motor_speed = 20;
      right_motor_speed = 20;
    } else {
      left_motor_speed = 0;
      right_motor_speed = 0;
    }
  // If the sensors show the table is over, then stay there
  } else {
    if (dist_us_a < 80) {
      left_motor_speed = -20;
      right_motor_speed = -20;
    } else {
      left_motor_speed = 0;
      right_motor_speed = 0;
    }
  }
  robot.move(left_motor_speed + left_motor_offset, right_motor_speed + right_motor_offset);
  robot.update();
  
  //delay(100); // to ease debugging
}
