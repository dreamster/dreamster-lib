/**
   A shy robot that approaches slowly, and retreats if grabbed from the sides.
   Author: Martín Mello Teggia, Misael.K
*/

#include <dreamster.h>

Dreamster robot;

int dist_us_b;
int dist_us_c;

uint16_t ir_left;
uint16_t ir_right;

const int floor_limit = 20; // assumes white floor with black border
const int distance_limit = 80;

// use this to calibrate the motors
const int left_motor_offset = -2;
const int right_motor_offset = 2;

void setup() {
  robot.setup();
}

void loop() {
  int left_motor_speed = 0;
  int right_motor_speed = 0;

  // read sensors
  robot.scan_b(dist_us_b);
  robot.scan_c(dist_us_c);
  robot.read_ir(ir_left, ir_right);

  // print sensors
  Serial.print(dist_us_b); Serial.print(" ");
  Serial.print(dist_us_c); Serial.print(" ");
  Serial.print(ir_left); Serial.print(" ");
  Serial.print(ir_right); Serial.print(" ");
  Serial.println("");

  // if too close from both sides, go backwards
  if (dist_us_b < distance_limit && dist_us_c < distance_limit) {
    left_motor_speed = -50;
    right_motor_speed = -50;
  } else {
    // if we are not feeling in danger, approach slowly
    if (ir_left > floor_limit && ir_right > floor_limit) {
      left_motor_speed = 4;
      right_motor_speed = 4;
    } else {
      // if the sensors show the table is over, then stay there
      left_motor_speed = 0;
      right_motor_speed = 0;
    }
  }
  // set the motors speed with the respective offsets
  robot.move(left_motor_speed + left_motor_offset, right_motor_speed + right_motor_offset);
  robot.update();
}
