void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

String inString = "";
void loop() {
  // put your main code here, to run repeatedly:
  inString += 'a';
  inString += 34;
  inString += 'b';
  inString += 65;
  inString += 'c';
  inString += 39;
  inString += 'd';
  inString += 75;
  inString += 'e';
 
  Serial.println(inString);
  inString = "";
  delay(500);
}
