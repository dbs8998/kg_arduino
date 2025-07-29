#include <Wire.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
void setup() {
    Serial.begin(115200);
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  delay(2000);
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 10);
  oled.println("Hello");
  oled.display();
}


void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // 줄바꿈('\n')까지 읽기
    Serial.print("받은 데이터: ");
    Serial.println(input);

    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 10);
    oled.println(input);
    oled.display();
  }
}
