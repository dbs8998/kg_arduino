#include "Arduino.h"
#include "MotorClass.h"

MotorClass::MotorClass(int pin1, int pin2, int pin3, int pin4, int spd_l, int spd_r){
  pin_l_num1 = pin1;
  pin_l_num2 = pin2;

  pin_r_num1 = pin3;
  pin_r_num2 = pin4;
  speed_l = spd_l;
  speed_r = spd_r;
}

void MotorClass::begin(){
  pinMode(pin_l_num1, OUTPUT);
  pinMode(pin_l_num2, OUTPUT);

  pinMode(pin_r_num1, OUTPUT);
  pinMode(pin_r_num2, OUTPUT);
}


void MotorClass::move(){
  analogWrite(pin_l_num1, map(speed_l, 0, 100, 0, 255));
  analogWrite(pin_l_num2, LOW);

  analogWrite(pin_r_num1, map(speed_r, 0, 100, 0, 255));
  analogWrite(pin_r_num2, LOW);
}


void MotorClass::turn_left(){
  analogWrite(pin_l_num1, LOW);
  analogWrite(pin_l_num2, map(speed_l+20, 0, 100, 0, 255));

  // analogWrite(pin_r_num1, (speed_l+80));
  analogWrite(pin_r_num1, map(speed_r, 0, 100, 0, 255));
  analogWrite(pin_r_num2, LOW);
}

void MotorClass::turn_right(){
  // analogWrite(pin_l_num1, (speed_r+80));
  analogWrite(pin_l_num1, map(speed_l, 0, 100, 0, 255));
  analogWrite(pin_l_num2, LOW);

  analogWrite(pin_r_num1, LOW);
  analogWrite(pin_r_num2, map(speed_r+20, 0, 100, 0, 255));
}

void MotorClass::curveLeft(){
  analogWrite(pin_l_num1, map(speed_l * 0.4, 0, 100, 0, 255));  // 왼쪽은 느리게
  analogWrite(pin_l_num2, LOW);

  analogWrite(pin_r_num1, map(speed_r, 0, 100, 0, 255));        // 오른쪽은 그대로
  analogWrite(pin_r_num2, LOW);
}

void MotorClass::curveRight(){
  analogWrite(pin_l_num1, map(speed_l, 0, 100, 0, 255));        // 왼쪽은 그대로
  analogWrite(pin_l_num2, LOW);

  analogWrite(pin_r_num1, map(speed_r * 0.4, 0, 100, 0, 255));  // 오른쪽은 느리게
  analogWrite(pin_r_num2, LOW);
}

void MotorClass::back(){
  analogWrite(pin_l_num1, LOW);
  analogWrite(pin_l_num2, map(speed_l, 0, 100, 0, 255));

  analogWrite(pin_r_num1, LOW);
  analogWrite(pin_r_num2, map(speed_r, 0, 100, 0, 255));
}

void MotorClass::setSpeed(int left, int right){
  speed_l = constrain(left, 0, 100);
  speed_r = constrain(right, 0, 100);
}

void MotorClass::stop(){
  analogWrite(pin_l_num1, LOW);
  analogWrite(pin_l_num2, LOW);

  analogWrite(pin_r_num1, LOW);
  analogWrite(pin_r_num2, LOW);
}