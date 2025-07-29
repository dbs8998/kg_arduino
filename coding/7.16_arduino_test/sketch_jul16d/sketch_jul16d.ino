const int LED = 13;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 0;i<=10;i++){
    int cnt =0;
    while(true){
      digitalWrite(LED, HIGH);
      delay(i);
      digitalWrite(LED, LOW);
      delay(10-i);

      cnt++;
      if(cnt==10) {
        break;
      }
    }
  }

  for(int i = 0;i<=10;i++){
    int cnt =0;
    while(true){
      digitalWrite(LED, HIGH);
      delay(10-i);
      digitalWrite(LED, LOW);
      delay(i);

      cnt++;
      if(cnt==10) {
        break;
      }
    }
  }


}
