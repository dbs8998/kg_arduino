#include "LedClass.h"


LedClass l1(5);
LedClass l2(6);
LedClass l3(13);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  l1.begin();
  l2.begin();
  l3.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  l1.turn_on();
  l2.turn_on();
  l3.turn_on();
  delay(1000);

  l1.turn_off();
  l2.turn_off();
  l3.turn_off();
  delay(1000);
}
