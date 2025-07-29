const int LED = 13;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // digitalWrite(LED, HIGH);
  // delay(1000);
  // digitalWrite(LED, LOW);
  // delay(1000);

  // digitalWrite(LED, HIGH);
  // delay(100);
  // digitalWrite(LED, LOW);
  // delay(100);
 
  // digitalWrite(LED, HIGH);
  // delay(10);
  // digitalWrite(LED, LOW);
  // delay(10);

  // digitalWrite(LED, HIGH);
  // delay(100);
  // digitalWrite(LED, LOW);
  // delay(900);

  // digitalWrite(LED, HIGH);
  // delay(900);
  // digitalWrite(LED, LOW);
  // delay(100);

  analogWrite(LED, HIGH);  // 10% 밝기
  delay(1);

  analogWrite(LED, LOW); // 90% 밝기
  delay(9);
}
