/*
########### dreamsterbot.org ###########

 This is an example of a basic Dreamster code.
 It avoids obstacles using 3 sensors.
 Also changes the led intensity according to the distance sensed.
*/

#include <dreamster.h>

Dreamster d;

int DistanceA_mm;
int DistanceB_mm;
int DistanceC_mm;

int outA;
int outB;
int outC;

void setup() {
  Serial.begin(9600);
}

void loop() {
  d.scan(&DistanceA_mm, &DistanceB_mm, &DistanceC_mm);

  outA = map(DistanceA_mm, 0, 1023, 120, 255);
  outB = map(DistanceB_mm, 0, 1023, 0, 255);
  outC = map(DistanceC_mm, 0, 1023, 0, 255);
  d.show(outA, outB, outC);

  Serial.print("DistanceA = ");
  Serial.print(DistanceA_mm);
  Serial.print (" mm // ");
  Serial.println (outA, DEC);

  Serial.print("DistanceB = ");
  Serial.print(DistanceB_mm);
  Serial.print (" mm // ");
  Serial.println (outB, DEC);

  Serial.print("DistanceC = ");
  Serial.print(DistanceC_mm);
  Serial.print (" mm // ");
  Serial.println (outB, DEC);

  if (DistanceA_mm < 200 && DistanceB_mm < 200) {
    while(DistanceA_mm < 200){
      d.move(100, 80);
      d.scan(&DistanceA_mm, &DistanceB_mm, &DistanceC_mm);
      delay(200);
    }
   }

  if (DistanceA_mm < 200 && DistanceC_mm < 200) {
    while(DistanceA_mm < 200){
      d.move(80, 100);
      d.scan(&DistanceA_mm, &DistanceB_mm, &DistanceC_mm);
      delay(200);
    }
  }

  while(DistanceA_mm < 200){
    d.move(80, 80);
    d.scan(&DistanceA_mm, &DistanceB_mm, &DistanceC_mm);
    delay(200);
  }

  d.move(100, 100);
  delay(200);
}
