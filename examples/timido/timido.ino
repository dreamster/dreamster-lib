/**
 * A robot that mantains a distance from a frontal obstacle.
 * Author: Ernesto Corbellini
 */

#include <dreamster.h>

Dreamster robot;

int distA;
int distB;
int distC;

int outA;
int outB;
int outC;

uint16_t irleft;
uint16_t irright;

const int MARGEN = 30;

void setup() {
  robot.setup();
}

void loop() {
  robot.scan_b(distB);
  robot.scan_c(distC);
  robot.read_ir(irleft,irright);

  // If the infrared sensors show the table continue, check for danger
  if(irleft > MARGEN && irright > MARGEN){
    // If too close from both sides, go backwards
    if((distB < 80) && (distC < 80)) {
      robot.move(-20, -15);
    }
    // If we are not feeling in danger, then stay there
    else{
      robot.move(0, 0);
    }
    
  }
  // If the sensors show the table is over, then stay there
  else{
    robot.move(0, 0);
  }
  robot.update();
}
