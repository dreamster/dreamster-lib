/**
   A shy robot that approaches slowly, and retreats if grabbed from the sides.
   Author: Martín Mello Teggia, Misael.K
*/

#include <dreamster.h>

Dreamster robot;

const int MOTOR_SPEED = 50;

int dist_us_b;
int dist_us_c;

uint16_t ir_left;
uint16_t ir_right;

const int floor_limit = 200; // assumes white floor with black border
const int distance_limit = 8;
unsigned long tick = 0;
int safe_floor_value;

void setup() {
  robot.setup();
  robot.calibrate_motors_zero(-5, -10);

  // only use frontal us sensor
  robot.set_sensor_a_active(false);
  robot.set_sensor_b_active(true);
  robot.set_sensor_c_active(true);

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

  // read sensors
  robot.scan_b(dist_us_b);
  robot.scan_c(dist_us_c);
  robot.read_ir(ir_left, ir_right);

  // if one of the sensors is dead, copy the value from the working one
  if (ir_right == 0) ir_right = ir_left;
  if (ir_left == 0) ir_left = ir_right;
  int avg_ir = (ir_left + ir_right) / 2;
  int abs_diff = abs(avg_ir - safe_floor_value);

  // print sensors
  Serial.print(dist_us_b); Serial.print(" ");
  Serial.print(dist_us_c); Serial.print(" ");
  Serial.print(ir_left);   Serial.print(" ");
  Serial.print(ir_right);  Serial.print(" ");
  Serial.print(safe_floor_value);  Serial.print(" ");
  Serial.print(abs_diff);  Serial.print(" ");
  Serial.println("");

  // if too close from both sides, go backwards
  if (dist_us_b > 0 && dist_us_b < distance_limit && dist_us_c > 0 && dist_us_c < distance_limit) {
    left_motor_speed = -MOTOR_SPEED;
    right_motor_speed = -MOTOR_SPEED;
    tick = 0;
  } else {
    // if we are not feeling in danger, approach slowly
    if (abs_diff < floor_limit) {
      tick++;
      if (tick > 2000) {
        left_motor_speed = MOTOR_SPEED;
        right_motor_speed = MOTOR_SPEED;
      }
      if (tick > 2020) {
        tick = 0;
      }
    } else {
      // if the sensors show the table is over, then stay there
      left_motor_speed = 0;
      right_motor_speed = 0;
      tick = 0;
    }
  }
  // set the motors speed with the respective offsets
  robot.move(left_motor_speed, right_motor_speed);
  delay(1); // ensures cycles every 1 ms at least
}
