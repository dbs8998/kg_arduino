const unsigned int led[8] = {3, 4, 5, 6, 7, 8, 9, 10};  // 8개의 LED 핀
int inputValue = 0;  // 시리얼 입력 값을 저장할 변수

void setup() {
  Serial.begin(9600);  // 시리얼 통신 시작
  for (int i = 0; i < 8; i++) {
    pinMode(led[i], OUTPUT);  // LED 핀을 출력으로 설정
  }

  Serial.println("Enter a number between 0 and 255 to control LEDs:");
}

void loop() {
  // 시리얼에서 값을 입력 받으면 실행
  if (Serial.available() > 0) {
    inputValue = Serial.parseInt();  // 시리얼 입력 값 받기

    // 입력 값이 0~255 범위 내에 있는지 확인
    if (inputValue >= 0 && inputValue <= 255) {
      controlLEDs(inputValue);  // 입력 값에 맞춰 LED 제어
    } else {
      Serial.println("Please enter a number between 0 and 255.");
    }
  }
}

void controlLEDs(int value) {
  // 2진수 값에 따라 LED 켜기/끄기
  for (int i = 0; i < 8; i++) {
    if (bitRead(value, i) == 1) {
      digitalWrite(led[i], HIGH);  // 해당 비트가 1이면 LED 켬
    } else {
      digitalWrite(led[i], LOW);   // 해당 비트가 0이면 LED 끔
    }
  }

  // 현재 2진수 값을 시리얼 모니터에 출력
  Serial.print("LEDs for binary value: ");
  Serial.println(value, BIN);  // 2진수로 출력
}
