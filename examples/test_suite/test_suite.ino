/**
   A suite for testing all sensors and motors.
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

unsigned long int last_time = 0;

char debugStringBuffer[50];
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
}

void loop() {

  robot.scan_a(dist_us_a);
  robot.scan_b(dist_us_b);
  robot.scan_c(dist_us_c);
  robot.read_ir(ir_left, ir_right);

  if (millis() - last_time > 5000) {
    Serial.println("CAMBIO VELOCIDAD MOTORES ");
    last_time = millis();
    left_motor_speed = left_motor_speed + 25;
    right_motor_speed = right_motor_speed + 25;
    if (left_motor_speed == 75) left_motor_speed = -50;
    if (right_motor_speed == 75) right_motor_speed = -50;
  }
  
  serialDebug("ABCLRID: %.4i %.4i %.4i %.4u %.4u % .3i % .3i\n", dist_us_a, dist_us_b, dist_us_c, ir_left, ir_right, left_motor_speed, right_motor_speed);

  robot.move(left_motor_speed, right_motor_speed);
  robot.update();
}
