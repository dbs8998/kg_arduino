#include <Servo.h>

const int SERVO = 10;
Servo servo;

void setup() {
  // put your setup code here, to run once:
  servo.attach(SERVO);
  servo.write(0);

  delay(1000);

  for(int i=0;i<=10;i++){
    servo.write(0);
    delay(100);
    servo.write(30);
    delay(100);
  }

  servo.detach();
}

void loop() {
  // put your main code here, to run repeatedly:

}
