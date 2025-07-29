int red = 11;
int green = 10;
int blue  = 9;

void setup() {
  // put your setup code here, to run once:
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(red, 55);
  analogWrite(green, 0);
  analogWrite(blue, 55);
  delay(1000);
  analogWrite(red, 55);
  analogWrite(green, 55);
  analogWrite(blue, 0);
  delay(1000);
  analogWrite(red, 0);
  analogWrite(green, 55);
  analogWrite(blue, 55);
  delay(1000);
}
