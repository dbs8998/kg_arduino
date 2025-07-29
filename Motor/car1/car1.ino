// 모터 핀 설정
const int A1A = 6;   // 왼쪽 모터 입력 1
const int A1B = 5;  // 왼쪽 모터 입력 2
const int B1A = 11;  // 오른쪽 모터 입력 1
const int B1B = 10;  // 오른쪽 모터 입력 2


int B_ENCODER = 13;
int A_ENCODER = 12;

volatile int count = 0;
unsigned long oldTime = 0;
unsigned long newTime = 0;



void setup() {

  Serial.begin(115200);
  pinMode(A_ENCODER, INPUT_PULLUP);
  attachInterrupt(INT0, ISRencoder, FALLING);
  
  // 모터 핀을 출력으로 설정
  pinMode(B1A, OUTPUT);
  pinMode(B1B, OUTPUT);
  pinMode(A1A, OUTPUT);
  pinMode(A1B, OUTPUT);
  // 앞으로 가기 (양쪽 바퀴 모두 정방향)
  digitalWrite(B1A, LOW);
  digitalWrite(B1B, HIGH);
  digitalWrite(A1A, LOW);
  digitalWrite(A1B, HIGH);
  // 1초(1000ms) 동안 전진
  delay(1000);

  Serial.println(Serial.read());
  // 정지
  digitalWrite(B1A, LOW);
  digitalWrite(B1B, LOW);
  digitalWrite(A1A, LOW);
  digitalWrite(A1B, LOW);
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

}