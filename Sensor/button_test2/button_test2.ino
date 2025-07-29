const int buttonPin = 2;
const int ledPin = 10;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  int buttonInput = digitalRead(buttonPin);



    if(buttonInput == LOW){
      for(int i=0;i<=255;i++){
        analogWrite(ledPin, i);
        delay(4);
      }
    }else{
      analogWrite(ledPin, 0);
    }
 


}
