#include <dreamster.h>

// Create our robot object.
Dreamster robot;

void setup() {
  // Setup our robot.
  robot.setup();
}

void loop() {
  // Move forward for 3 seconds.
  robot.move(20, 20);
  robot.sleep(3000);
  // Now move backwards for 3 seconds.
  robot.move(-20, -20);
  robot.sleep(3000);
}
