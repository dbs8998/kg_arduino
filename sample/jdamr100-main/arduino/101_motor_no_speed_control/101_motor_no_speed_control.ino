/*
MIT License

Copyright (c) 2024 JD edu. http://jdedu.kr author: conner.jeong@gmail.com
     
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
     
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
     
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN TH
SOFTWARE.
*/

#define motor_A_enable 12
#define motor_B_enable 13
#define motor_A 10
#define motor_B 11

void setup() {
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  pinMode(motor_A_enable, OUTPUT);
  pinMode(motor_B_enable, OUTPUT);

}
void loop() {
  //for (int x = 50; x < 200; x++) {
    digitalWrite(motor_A_enable, HIGH);
    digitalWrite(motor_B_enable, HIGH);
    digitalWrite(motor_A, HIGH);
    digitalWrite(motor_B, HIGH);
    //analogWrite(motor_A, x);
    //analogWrite(motor_B, x);
    delay(1000);
  //}
  //for (int y = 200; y > 50; y--) {
    digitalWrite(motor_A_enable, LOW);
    digitalWrite(motor_B_enable, LOW);
    digitalWrite(motor_A, HIGH);
    digitalWrite(motor_B, HIGH);
    //analogWrite(motor_A, y);
    //analogWrite(motor_B, y);
    //delay(1);
  //}
  delay(1000);
}
