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

// 모터 핀 정의
#define motor_A_enable 12
#define motor_B_enable 13
#define motor_A 10
#define motor_B 11

// 엔코더 핀 정의
#define encoder_A 2
#define encoder_B 3  // left from view of robot 

volatile int encoder_A_count = 0;  // 엔코더 카운트 변수
volatile int encoder_B_count = 0;  // 엔코더 카운트 변수

void setup() {
  // 모터 핀 출력 설정
  pinMode(motor_A_enable, OUTPUT);
  pinMode(motor_B_enable, OUTPUT);
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);

  // 엔코더 핀 입력 설정 및 인터럽트 연결
  pinMode(encoder_A, INPUT);
  pinMode(encoder_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_A), encoder_A_ISR, RISING); // A채널 상승 시 인터럽트 발생
  attachInterrupt(digitalPinToInterrupt(encoder_B), encoder_B_ISR, RISING); // A채널 상승 시 인터럽트 발생

  Serial.begin(115200);  // 시리얼 모니터 시작
}

void loop() {
  // 모터 회전 예제
  motorForward(255); // 모터 전진, 속도 255 (최대)
  delay(2000);       // 2초간 전진
  motorStop();       // 모터 정지
  delay(1000);       // 1초 대기
  
  // 엔코더 값 출력
  Serial.print("Encoder A Count: ");
  Serial.print(encoder_B_count);
  Serial.print("   Encoder B Count: ");
  Serial.println(encoder_A_count);
  delay(500);
}

// 모터 전진 함수
void motorForward(int speed) {
  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, HIGH);
  analogWrite(motor_A, speed);
  analogWrite(motor_B, speed);
}

// 모터 후진 함수
void motorBackward(int speed) {
  digitalWrite(motor_A_enable, LOW);
  digitalWrite(motor_B_enable, LOW);
  analogWrite(motor_A, speed);
  analogWrite(motor_B, speed);
}

// 모터 정지 함수
void motorStop() {
  digitalWrite(motor_A_enable, LOW);
  digitalWrite(motor_B_enable, LOW);
  analogWrite(motor_A, 0);
  analogWrite(motor_B, 0);
}

// 엔코더 인터럽트 서비스 루틴 (ISR)
void encoder_A_ISR() {
  encoder_A_count++;
}

void encoder_B_ISR() {
  encoder_B_count++;
}
