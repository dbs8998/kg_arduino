const int led = 10;

void setup() {
  // put your setup code here, to run once:
  // analogWrite(led, 25);
  pinMode(led, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //led 11단계로 밝기 표시
  // for(int t_high=0;t_high<=10;t_high++){
  //   analogWrite(led, t_high*25);
  //   delay(100);
  // }

  //led 점점 밝게 하기
  for(int t_high=0;t_high<=255;t_high++){
    analogWrite(led, t_high);
    delay(4);
  }
  for(int j=255;j>=0;j--){
    analogWrite(led, j);
    delay(4);
  }
}
