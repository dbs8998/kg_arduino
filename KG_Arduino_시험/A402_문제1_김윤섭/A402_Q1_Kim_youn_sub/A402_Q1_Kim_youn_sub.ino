const int leds[8] = {3, 4, 5, 6, 7, 8, 9, 10}; // LED 핀
const int buttonPin = 2; // INT0 핀 (버튼)

volatile bool started = false;
unsigned int count = 0;
volatile unsigned long lastInterruptTime = 0;

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 8; i++) {
    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);
  }

  pinMode(buttonPin, INPUT_PULLUP);  // 버튼은 GND에 연결
  attachInterrupt(digitalPinToInterrupt(buttonPin), toggleStartStop, FALLING);

  Serial.println("대기");
}

void loop() {
  if (started) {
    displayBinary(count);
    count = (count + 1) % 256;
    delay(1000);
  }
}

// LED, 숫자/2진 문자열 출력
void displayBinary(unsigned int num) {
  for (int i = 0; i < 8; i++) {
    digitalWrite(leds[i], bitRead(num, i));
  }

  // 2진수 문자열 만들기
  String binaryStr = "";
  for (int i = 7; i >= 0; i--) {
    binaryStr += bitRead(num, i);
  }

  Serial.print("출력: ");
  Serial.print(num);
  Serial.print(" (");
  Serial.print(binaryStr);
  Serial.println(")");
}

// 버튼 누르면 실행/정지 토글 (디바운싱 포함)
void toggleStartStop() {
  unsigned long now = millis();
  if (now - lastInterruptTime > 300) {
    started = !started;
    Serial.println(started ? "시작" : "정지");
    lastInterruptTime = now;
  }
}
