#include <DHT.h>

#define DHTPIN 2
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  dht.begin();
  Serial.println("DHT11 테스트 시작");
}

void loop() {
  // put your main code here, to run repeatedly:
   delay(2000);

   float h = dht.readHumidity();
   float t = dht.readTemperature();

   if(isnan(h) || isnan(t)){
    Serial.println("센서 읽기 실패!");
    return;
   }

  Serial.print("습도 : ");
  Serial.print(h);
  Serial.print("%  온도 : ");

  Serial.print(t);
  Serial.println(" ℃");


}
