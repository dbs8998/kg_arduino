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

#include <SoftwareSerial.h>

// 소프트웨어 시리얼 핀 설정 (7번: RX, 8번: TX)
SoftwareSerial mySerial(6, 9); // RX, TX

void setup() {
  // 소프트웨어 시리얼 시작
  mySerial.begin(9600);
  
  // 기본 시리얼 모니터 시작
  Serial.begin(115200);
  
  Serial.println("SoftwareSerial 데이터 수신 준비 완료");
}

void loop() {
  // 소프트웨어 시리얼로 데이터가 수신되었는지 확인
  if (mySerial.available()) {
    String incomingStr = mySerial.readStringUntil('\n');  // 수신한 데이터 읽기
    Serial.print("수신한 데이터: ");
    Serial.println(incomingStr);
    mySerial.println(incomingStr);
  }

  // 100ms 지연
  delay(100);
}

