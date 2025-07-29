int ENCODER = 2;

volatile int count = 0;
unsigned long oldTime = 0;
unsigned long newTime = 0;


void ISRencoder() {
  count++;
}
void setup()
{
  Serial.begin(115200);
  pinMode(ENCODER, INPUT_PULLUP);
  attachInterrupt(INT0, ISRencoder, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
newTime = millis();
if(newTime-oldTime > 1000){
  oldTime = newTime;
  noInterrupts();
  Serial.println(count);
  interrupts();
}
}
