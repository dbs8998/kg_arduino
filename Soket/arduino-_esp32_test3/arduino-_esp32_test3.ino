void setup() {
  Serial.begin(9600);  // ESP32와 UART 통신
  delay(1000);
}

void loop() {
  // 센서 값 전송 (개행문자 \n 반드시 포함)
  Serial.println("💧 센서값: 327");

  // 명령 수신 처리
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');  // \n 기준 수신
    Serial.print("📥 수신 명령: ");
    Serial.println(cmd);
  }

  delay(1000);
}
