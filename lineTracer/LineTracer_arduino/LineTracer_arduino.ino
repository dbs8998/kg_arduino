#include "MotorClass.h"
#include <SoftwareSerial.h>

SoftwareSerial mySerial(12, 13); // RX, TX


// IR센서 설정
#define LEFT_IR_PIN  8
#define RIGHT_IR_PIN 7

//엔코더
int A_ENCODER = 2;
int B_ENCODER = 3;


// 모터 핀 설정
const int baseSpeed = 40;
int leftSpeed = baseSpeed + 8;
int rightSpeed = baseSpeed;

MotorClass motor(6, 5, 11, 10, leftSpeed, rightSpeed);

// PID 변수
float Kp = 0.6, Ki = 0.01, Kd = 0.10;
float error = 0, previous_error = 0, integral = 0;

// 
volatile int encoder_left = 0;
volatile int encoder_right = 0;

unsigned long currentTime = 0;
unsigned long lastAdjustTime = 0;

bool moving = false;
String cmd = "";


void setup() {
  Serial.begin(9600);
  delay(2000);
  mySerial.begin(9600);

  motor.begin();
  pinMode(LEFT_IR_PIN, INPUT_PULLUP);
  pinMode(RIGHT_IR_PIN, INPUT_PULLUP);

  pinMode(A_ENCODER, INPUT_PULLUP);
  pinMode(B_ENCODER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(3), countRight, RISING);
}

void loop() {

  // 명령 수신
  if (mySerial.available()) {
    cmd = mySerial.readStringUntil('\n');
    cmd.trim();
    Serial.print("📥 수신 명령: ");
    Serial.println(cmd);

    motor.setSpeed(leftSpeed, rightSpeed);  // 초기 속도
    
    // 무빙
    if(cmd == "m"){
      moving = true;   // 전진 시작
    } else if(cmd == "s"){
      motor.stop();
      moving = false;
    } else if(cmd == "b"){
      motor.back();
      moving = false;
    }

  }
  
  if (moving) {
    int left = digitalRead(LEFT_IR_PIN);
    int right = digitalRead(RIGHT_IR_PIN);

    if (left == LOW && right == LOW) {
      motor.move();  // 둘 다 선 위 → 직진
    } else if (left == LOW && right == HIGH) {
      motor.turn_right();  // 오른쪽 벗어남 → 오른쪽 회전
    } else if (left == HIGH && right == LOW) {
      motor.turn_left();   // 왼쪽 벗어남 → 왼쪽 회전
    }  else if (left == HIGH && right == HIGH) {
      motor.stop();   // 골인
      moving = false;
    } 
  }

  // PID 보정
  if (moving && (currentTime - lastAdjustTime >= 50)) {
    lastAdjustTime = currentTime;

    error = encoder_left - encoder_right;
    integral += error;
    float derivative = error - previous_error;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    output = constrain(output, -20, 20);

    int adjusted_left = constrain(leftSpeed - output, 0, 100);
    int adjusted_right = constrain(rightSpeed + output, 0, 100);
    motor.setSpeed(adjusted_left, adjusted_right);

    Serial.print("EncL/R: ");
    Serial.print(encoder_left);
    Serial.print("/");
    Serial.print(encoder_right);
    Serial.print(" | Out: ");
    Serial.print(output);
    Serial.print(" | L/R: ");
    Serial.print(adjusted_left);
    Serial.print("/");
    Serial.println(adjusted_right);
  }
}

// 인터럽트
void countLeft() {
  encoder_left++;
}
void countRight() {
  encoder_right++;
}
