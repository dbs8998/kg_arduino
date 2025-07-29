#include <WiFi.h>

#define ssid "ConnectValue_A402_2G"
#define password "CVA402!@#$"

WiFiServer server(1234);  // 사용자 정의 포트

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.print("WiFi 연결 중");
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.println(".");
  }

  Serial.println("\n✅ WiFi 연결 완료!");
  Serial.print("할당된 IP 주소: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("📡 TCP 서버 시작됨 (포트 1234)");
}


void loop() {
  WiFiClient client = server.available();

  if (client) {
    Serial.println("🔗 클라이언트가 연결되었습니다!");

    while (client.connected()) {
      if (client.available()) {
        String msg = client.readStringUntil('\n');
        Serial.print("📥 수신: ");
        Serial.println(msg);

        client.println("ESP32 응답입니다!\n");
      }
    }

    client.stop();
    Serial.println("❌ 클라이언트 연결 종료");
  }
}