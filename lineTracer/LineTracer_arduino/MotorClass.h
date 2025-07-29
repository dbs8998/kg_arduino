#ifndef MOTOR_CLASS_H
#define MOTOR_CLASS_H

#include "Arduino.h"

class MotorClass{
  public:
    MotorClass(int pin1, int pin2, int pin3, int pin4, int speedL, int speedR);
    void begin();
    void move();
    void turn_left();
    void turn_right();
    void curveLeft();
    void curveRight();
    void back();
    void stop();
    void setSpeed(int left, int right);   
    
  private:
    int pin_l_num1;
    int pin_l_num2;
    int pin_r_num1;
    int pin_r_num2;
    int speed_l; // 기본 최대 속도
    int speed_r;

};
#endif