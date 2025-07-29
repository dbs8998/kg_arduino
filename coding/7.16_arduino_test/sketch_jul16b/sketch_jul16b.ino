void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Serial.println(78, DEC);
  Serial.println(78, HEX);
  Serial.println(78,BIN);

  Serial.println(1.23456,0);
  Serial.println(1.23456,2);
  Serial.println(1.23456,4);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("int의 크기: ");
  Serial.println(sizeof(int));
  Serial.print("float의 크기: ");
  Serial.println(sizeof(float));
  
  Serial.print("long long의 크기: ");
  Serial.println(sizeof(long long));
  Serial.print("double의 크기: ");
  Serial.println(sizeof(double));
  while(1);  
}
