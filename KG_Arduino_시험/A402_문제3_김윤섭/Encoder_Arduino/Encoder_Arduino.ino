#include <SoftwareSerial.h>

SoftwareSerial mySerial(12, 13); // RX, TX

// 모터 핀 설정
const int pin_l_num1 = 6;   // 왼쪽 모터 입력 1
const int pin_l_num2 = 5;  // 왼쪽 모터 입력 2
const int pin_r_num1 = 11;  // 오른쪽 모터 입력 1
const int pin_r_num2 = 10;  // 오른쪽 모터 입력 2

// 모터 핀 설정(속도 보정)
const int baseSpeed = 30;
int speed_l = baseSpeed + 8;
int speed_r = baseSpeed;


//엔코더 설정
int A_ENCODER = 2;
int B_ENCODER = 3;

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

  // 모터 핀을 출력으로 설정
  pinMode(pin_l_num1, OUTPUT);
  pinMode(pin_l_num2, OUTPUT);

  pinMode(pin_r_num1, OUTPUT);
  pinMode(pin_r_num2, OUTPUT);


  //엔코더 핀 설
  pinMode(A_ENCODER, INPUT_PULLUP);
  pinMode(B_ENCODER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(3), countRight, RISING);
}

void loop() {
  currentTime = millis();

  // 명령 수신
  if (mySerial.available()) {
    cmd = mySerial.readStringUntil('\n');
    cmd.trim();
    Serial.print("📥 수신 명령: ");
    Serial.println(cmd);


    //파이썬으로 부터 수신명령 처리
    if(cmd == "m"){
      analogWrite(pin_l_num1, map(speed_l, 0, 100, 0, 255));
      analogWrite(pin_l_num2, LOW);

      analogWrite(pin_r_num1, map(speed_r, 0, 100, 0, 255));
      analogWrite(pin_r_num2, LOW);
      moving = true;
    }else if(cmd == "s"){
      analogWrite(pin_l_num1, LOW);
      analogWrite(pin_l_num2, LOW);

      analogWrite(pin_r_num1, LOW);
      analogWrite(pin_r_num2, LOW);
      moving = false;
    }else if(cmd == "b"){
      analogWrite(pin_l_num1, LOW);
      analogWrite(pin_l_num2, map(speed_l, 0, 100, 0, 255));

      analogWrite(pin_r_num1, LOW);
      analogWrite(pin_r_num2, map(speed_r, 0, 100, 0, 255));
      moving = true;
    }else if(cmd == "l"){
      analogWrite(pin_l_num1, LOW);
      analogWrite(pin_l_num2, map(speed_l, 0, 100, 0, 255));

      analogWrite(pin_r_num1, LOW);
      analogWrite(pin_r_num2, LOW);
      moving = true;
    }else if(cmd == "r"){
      analogWrite(pin_l_num1, LOW);
      analogWrite(pin_l_num2, LOW);

      analogWrite(pin_r_num1, LOW);
      analogWrite(pin_r_num2, map(speed_r, 0, 100, 0, 255));
      moving = true;
    }

  }

  
  // 엔코더 처리(이동 중일 때만 적용)
  if (moving && (currentTime - lastAdjustTime >= 50)) {
    lastAdjustTime = currentTime;

    mySerial.print("EncL/R: ");
    mySerial.print(encoder_left);
    mySerial.print("/");
    mySerial.println(encoder_right);
  }
  delay(1);
}

// 인터럽트
void countLeft() {
  encoder_left++;
}
void countRight() {
  encoder_right++;
}


