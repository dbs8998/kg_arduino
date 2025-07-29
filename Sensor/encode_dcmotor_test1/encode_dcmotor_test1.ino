// 모터 핀 설정
const int A1A = 6;   // 왼쪽 모터 입력 1
const int A1B = 5;  // 왼쪽 모터 입력 2
const int B1A = 11;  // 오른쪽 모터 입력 1
const int B1B = 10;  // 오른쪽 모터 입력 2

int A_ENCODER = 2;
int B_ENCODER = 3;


volatile int count = 0;
unsigned long oldTime = 0;
unsigned long newTime = 0;



void setup() {

  Serial.begin(115200);
  pinMode(A_ENCODER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), ISRencoder, FALLING);
  
  // 모터 핀을 출력으로 설정
  pinMode(B1A, OUTPUT);
  pinMode(B1B, OUTPUT);
  pinMode(A1A, OUTPUT);
  pinMode(A1B, OUTPUT);

}


void ISRencoder() {
  count++;
}

void loop() {
  // loop에서는 아무 작업 안 함
  newTime = millis();
  if(newTime-oldTime > 1000){
    oldTime = newTime;
    noInterrupts();
    Serial.println(count);
    interrupts();
  }
  // 앞으로 가기 (양쪽 바퀴 모두 정방향)
  analogWrite(B1A, 0);
  analogWrite(B1B, 255);  // 정방향 회전
  analogWrite(A1A, 0);
  analogWrite(A1B, 255);

}