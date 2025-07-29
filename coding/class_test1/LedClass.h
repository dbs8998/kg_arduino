#ifndef LED_CLASS_H
#define LED_CLASS_H

#include "Arduino.h"

class LedClass{
  public:
    LedClass(int pin);
    void begin();
    void turn_on();
    void turn_off();

  private:
    int pin_num;
};

#endif