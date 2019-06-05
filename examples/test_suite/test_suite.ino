/**
   A suite for testing all sensors, motors, and leds.
   Author: Misael.K
*/

#include <dreamster.h>

Dreamster robot;

int dist_us_a;
int dist_us_b;
int dist_us_c;

uint16_t ir_left;
uint16_t ir_right;

int left_motor_speed = 0;
int right_motor_speed = 0;
int leds_state = 0;
int led_value = 0;
int led_value_increment = 0;

unsigned long int last_motor_time = 0;
unsigned long int last_led_time = 0;

char debugStringBuffer[60];
bool debugMode = true;
// sprintf + serial of 20 bytes takes ~200us
// sprintf + serial of 10 bytes takes ~144us
// sprintf + serial of  5 bytes takes ~108us
#define serialDebug(...) \
    if (debugMode) { \
        sprintf(debugStringBuffer, __VA_ARGS__); \
        Serial.print(debugStringBuffer); \
    }
  
void setup() {
  robot.setup();
  robot.calibrate_motors_zero(-4, -4);
}

void loop() {
  robot.scan(dist_us_a, dist_us_b, dist_us_c);
  robot.read_ir(ir_left, ir_right);

  if (millis() - last_motor_time > 5000) {
    last_motor_time = millis();
    left_motor_speed = left_motor_speed + 25;
    right_motor_speed = right_motor_speed + 25;
    if (left_motor_speed == 125) left_motor_speed = -100;
    if (right_motor_speed == 125) right_motor_speed = -100;
    leds_state = (leds_state + 1) % 3;
    led_value = 0;
    led_value_increment = 1;
  }
  if (millis() - last_led_time > 2) {
    last_led_time = millis();
    led_value = led_value + led_value_increment;
    if (led_value == 255) led_value_increment = -1;
    if (led_value == 0) led_value_increment = 1;
  }
  
  if (leds_state == 0) {
    robot.show(0, led_value, 0);
  } else if (leds_state == 1) {
    robot.show(0, 0, led_value);
  } else if (leds_state == 2) {
    robot.show(led_value, 0, 0);
  }
  
  serialDebug("ABC: %.4i %.4i %.4i LR: %.4u %.4u ID: % .3i % .3i\n", dist_us_a, dist_us_b, dist_us_c, ir_left, ir_right, left_motor_speed, right_motor_speed);

  robot.move(left_motor_speed, right_motor_speed);
  
  // delay(33); // to ease debugging
}
