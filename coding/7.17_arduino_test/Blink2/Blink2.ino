/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://docs.arduino.cc/hardware/

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://docs.arduino.cc/built-in-examples/basics/Blink/
*/

const unsigned int LED_BUILTIN_10 = 10;
const unsigned int LED_BUILTIN_9 = 9;
const unsigned int LED_BUILTIN_8 = 8;
const unsigned int LED_BUILTIN_7 = 7;
const unsigned int LED_BUILTIN_6 = 6;
const unsigned int LED_BUILTIN_5 = 5;
const unsigned int LED_BUILTIN_4 = 4;
const unsigned int LED_BUILTIN_3 = 3;


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN_10, OUTPUT);
  pinMode(LED_BUILTIN_9, OUTPUT);
  pinMode(LED_BUILTIN_8, OUTPUT);
  pinMode(LED_BUILTIN_7, OUTPUT);
  pinMode(LED_BUILTIN_6, OUTPUT);
  pinMode(LED_BUILTIN_5, OUTPUT);
  pinMode(LED_BUILTIN_4, OUTPUT);
  pinMode(LED_BUILTIN_3, OUTPUT);

  digitalWrite(LED_BUILTIN_10, HIGH);  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN_9, HIGH); 
  digitalWrite(LED_BUILTIN_8, HIGH); 
  digitalWrite(LED_BUILTIN_7, HIGH); 
  digitalWrite(LED_BUILTIN_6, HIGH); 
  digitalWrite(LED_BUILTIN_5, HIGH); 
  digitalWrite(LED_BUILTIN_4, HIGH); 
  digitalWrite(LED_BUILTIN_3, HIGH); 
}

// the loop function runs over and over again forever
void loop() {



}
