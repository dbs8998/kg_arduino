#include <WiFi.h>

const char* ssid = "ConnectValue_A402_2G";
const char* password = "CVA402!@#$";

WiFiServer server(1234);

// 안전한 UART1 핀으로 변경 (GPIO16, GPIO17 권장)
#define UART_RX 25
#define UART_TX 26

HardwareSerial ArduinoSerial(1);

void setup() {
  Serial.begin(115200);  // 디버깅용
  ArduinoSerial.begin(9600, SERIAL_8N1, UART_RX, UART_TX);
  
  WiFi.setSleep(false);  // 절전모드로 인한 통신 끊김 방지
  WiFi.begin(ssid, password);
  Serial.print("WiFi 연결 중");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi 연결 완료");
  Serial.print("📡 IP: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("🔌 TCP 서버 시작됨 (포트 1234)");
}

void loop() {
  WiFiClient client = server.available();

  if (client) {
    Serial.println("📥 Python 클라이언트 연결됨");

    while (client.connected()) {
      // ✅ Arduino → Python
      if (ArduinoSerial.available()) {
        char buffer[100];
        int len = ArduinoSerial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
        buffer[len] = '\0';  // null-terminate
        client.println(buffer);
        Serial.print("🔁 전달(Arduino→PC): ");
        Serial.println(buffer);
      }

      // ✅ Python → Arduino
      if (client.available()) {
        String fromPython = client.readStringUntil('\n');
        ArduinoSerial.println(fromPython);
        Serial.print("🔁 전달(PC→Arduino): ");
        Serial.println(fromPython);
      }

      delay(1);  // WDT 방지
    }

    Serial.println("❌ 클라이언트 연결 종료");
    ArduinoSerial.println("s");
    client.stop();
  }

  delay(1);  // WDT 방지
}
