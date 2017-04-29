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

const int MARGEN = 15;

void setup() {
  robot.setup();
}

void loop() {
  robot.scan(distA, distB, distC);
  robot.read_ir(irleft,irright);


  // If the infrared sensors show the table continue, check for danger
  if(irleft > MARGEN && irright > MARGEN){

    // If too close, go backwards
    if(distA < 100) {
      robot.move(-20, -20);
    }
   // If we are far enough then stop
    else if(distA > 120 && distA < 300) {
      robot.move(20, 20);
    }
    else{
      robot.move(0, 0);
    }
  
  
  // If the sensors show the table is over, then stay there
  }else{
    if(distA < 100) {
      robot.move(-20, -20);
    }
    else{    
    robot.move(0, 0);
    }
  }


    
  robot.update();
}
