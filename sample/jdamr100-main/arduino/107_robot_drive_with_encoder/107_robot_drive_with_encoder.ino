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

#define encoder_R 2
#define encoder_L 3

volatile int encoder_R_cnt = 0;
volatile int encoder_L_cnt = 0;

unsigned long time_prev = 0;
unsigned long time_curr = 0;
unsigned long encoder_update_interval = 50;

int moving_direction = 0;
int moving_speed = 0;
int diff_weight = 4;
int delta_R = 0;
int delta_L = 0;
int speed_R = 0;
int speed_L = 0;
int speed_R_turn = 0;
int speed_L_turn = 0;
int speed_adjust_val = 0;
int delay_time =100;
int turn_speed = 20;
String inString;

void forward(int R, int L) {
  analogWrite(motor_A_enable, L);
  analogWrite(motor_B_enable, R);
  digitalWrite(motor_A, HIGH);
  digitalWrite(motor_B, HIGH);
}

void backward(int R, int L) {
  analogWrite(motor_A_enable, L);
  analogWrite(motor_B_enable, R);
  digitalWrite(motor_A, LOW);
  digitalWrite(motor_B, LOW);
}

void turnLeft(int R, int L) {
  analogWrite(motor_A_enable, L);
  analogWrite(motor_B_enable, R);
  digitalWrite(motor_A, LOW);
  digitalWrite(motor_B, HIGH);
}

void turnRight(int R, int L) {
  analogWrite(motor_A_enable, L);
  analogWrite(motor_B_enable, R);
  digitalWrite(motor_A, HIGH);
  digitalWrite(motor_B, LOW);
}

void stopAll() {
  analogWrite(motor_A_enable, 0);
  analogWrite(motor_B_enable, 0);
  digitalWrite(motor_A, HIGH);
  digitalWrite(motor_B, HIGH);
}

void encoder_R_ISR(){
  encoder_R_cnt++;
}

void encoder_L_ISR(){
  encoder_L_cnt++;
}

void setup() {
  Serial.begin(115200);

  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);

  pinMode(encoder_R, INPUT_PULLUP);
  pinMode(encoder_L, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder_R), encoder_R_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoder_L), encoder_L_ISR, FALLING);
}

void adjustSpeed() {
  static int last_encoder_R_cnt = 0;
  static int last_encoder_L_cnt = 0;
  
  delta_R = encoder_R_cnt - last_encoder_R_cnt;
  delta_L = encoder_L_cnt - last_encoder_L_cnt;

  last_encoder_R_cnt = encoder_R_cnt;
  last_encoder_L_cnt = encoder_L_cnt;

  int diff = delta_R - delta_L;

  speed_adjust_val = diff * diff_weight ;

}

void loop() {
  time_curr = millis();
  
  if (Serial.available() > 0) {
    inString = Serial.readStringUntil('\n');
    char cmd = inString[0];
    if (cmd == '1') {
      moving_direction = inString.substring(inString.indexOf('a') + 1, inString.indexOf('b')).toInt();
      moving_speed = inString.substring(inString.indexOf('b') + 1, inString.indexOf('c')).toInt();

      speed_R = moving_speed + speed_adjust_val;
      speed_L = moving_speed + speed_adjust_val;

      speed_R_turn = speed_R - turn_speed;
      speed_L_turn = speed_R - turn_speed;

      speed_R = constrain(speed_R,0,255);
      speed_L = constrain(speed_L,0,255);
      speed_R_turn = constrain(speed_R_turn,0,255);
      speed_L_turn = constrain(speed_L_turn,0,255);


      if (moving_direction == 1) {
        forward(speed_R, speed_L);
      } else if (moving_direction == 2) {
        backward(speed_R, speed_L);
      } else if (moving_direction == 3) {
        turnLeft(speed_R_turn, speed_L_turn);
      } else if (moving_direction == 4) {
        turnRight(speed_R_turn, speed_L_turn);
      }
    } else if (cmd == "0") {
      diff_weight = inString.substring(inString.indexOf('a') + 1, inString.indexOf('b')).toInt();
      encoder_update_interval = inString.substring(inString.indexOf('b') + 1, inString.indexOf('c')).toInt();
      delay_time = inString.substring(inString.indexOf('c') + 1, inString.indexOf('d')).toInt();
      turn_speed = inString.substring(inString.indexOf('d') + 1, inString.indexOf('e')).toInt();
    }
  } else {
    stopAll();
  }

  if (time_curr - time_prev >= encoder_update_interval) {
    time_prev = time_curr;
    adjustSpeed();
  }
  delay(delay_time);
}
