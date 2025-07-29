#include "MotorClass.h"
#include <SoftwareSerial.h>

SoftwareSerial mySerial(12, 13); // RX, TX

const int baseSpeed = 30;
int leftSpeed = baseSpeed + 8;
int rightSpeed = baseSpeed;

MotorClass motor(6, 5, 11, 10, leftSpeed, rightSpeed);

// 엔코더
volatile int encoder_left = 0;
volatile int encoder_right = 0;

unsigned long lastSendTime = 0;
unsigned long lastAdjustTime = 0;

// 누적 주행 거리 (단위: cm)
float total_dist_cm = 0;
const float TICK_TO_CM = 0.66;

bool moving = false;
String cmd = "";

// PID 변수
float Kp = 0.6, Ki = 0.01, Kd = 0.10;
float error = 0, previous_error = 0, integral = 0;

void setup() {
  Serial.begin(9600);
  delay(2000);
  mySerial.begin(9600);

  motor.begin();

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(3), countRight, RISING);
}

void loop() {
  unsigned long currentTime = millis();

  // 명령 수신
  if (mySerial.available()) {
    cmd = mySerial.readStringUntil('\n');
    cmd.trim();
    Serial.print("📥 수신 명령: ");
    Serial.println(cmd);

    motor.setSpeed(leftSpeed, rightSpeed);  // 초기 속도

    if (cmd == "m") {
      encoder_left = 0;
      encoder_right = 0;
      total_dist_cm = 0;
      moving = true;
      motor.move();
      mySerial.println("ACK:m");   // ✅ 응답
      delay(300);  // ✅ 최초 거리 전송 너무 빠르지 않도록
    } else if (cmd == "b") {
      moving = false;
      motor.back();
    } else if (cmd == "L") {
      moving = false;
      encoder_left = 0;
      encoder_right = 0;
      total_dist_cm = 0;  // ✅ 회전 시 거리 초기화
      motor.turn_left();
    } else if (cmd == "R") {
      moving = false;
      encoder_left = 0;
      encoder_right = 0;
      total_dist_cm = 0;  // ✅ 회전 시 거리 초기화
      motor.turn_right();
    } else if (cmd == "s") {
      moving = false;
      motor.stop();
    } else if (cmd == "i") {
      encoder_left = 0;
      encoder_right = 0;
      total_dist_cm = 0;
    }
  }

  // 거리 정보 전송
  if (moving && (currentTime - lastSendTime >= 200)) {
    lastSendTime = currentTime;

    float dist_l = encoder_left * TICK_TO_CM;
    float dist_r = encoder_right * TICK_TO_CM;
    float avg_dist = (dist_l + dist_r) / 2;

    total_dist_cm += avg_dist;

    encoder_left = 0;
    encoder_right = 0;

    mySerial.print("dist:");
    mySerial.println(total_dist_cm, 1);  // 소수점 1자리
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
