const int trin_pin = 13;
const int echo_pin = 12;


void setup() {
  // put your setup code here, to run once:
  pinMode(trin_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(trin_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trin_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trin_pin, LOW);

  long duration = pulseIn(echo_pin, HIGH);
  long distance = (duration/2) / 29.1;

  Serial.print(distance);
  Serial.println(" cm");

}
