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

void setup() {
  robot.setup();
}

void loop() {
  robot.scan(distA, distB, distC);

  // If too close, go backwards
  if(distA < 100) {
    robot.move(-20, -20);
  }

  // If we are far enough then stop
  if(distA > 150) {
    robot.move(0, 0);
  }

  robot.update();
}
