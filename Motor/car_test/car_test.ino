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


void ISRencoder() {
  count++;
}

void setup() {

  Serial.begin(115200);
  pinMode(A_ENCODER, INPUT_PULLUP);
  attachInterrupt(INT0, ISRencoder, FALLING);

  pinMode(B_ENCODER, INPUT_PULLUP);
  attachInterrupt(INT1, ISRencoder, FALLING);
  
  
  // 모터 핀을 출력으로 설정
  // pinMode(B1A, OUTPUT);
  // pinMode(B1B, OUTPUT);
  pinMode(A1A, OUTPUT);
  pinMode(A1B, OUTPUT);
  
}



void loop() {

  // 앞으로 가기 (양쪽 바퀴 모두 정방향)
  // digitalWrite(B1A, LOW);
  // digitalWrite(B1B, HIGH);
 

  char c = Serial.read();
  if(c=='f'){
    analogWrite(A1A, 128);
    analogWrite(A1B, 0);
    
  }else if(c=='s'){
    analogWrite(A1A, 0);
    analogWrite(A1B, 0);
  }
  if(c=='z'){
    analogWrite(B1A, 128);
    analogWrite(B1B, 0);
  }else if(c=='d'){
    analogWrite(B1A, 0);
    analogWrite(B1B, 0);
  }


  newTime = millis();
  if(newTime-oldTime > 1000){
    oldTime = newTime;
    noInterrupts();
    Serial.print(c+" 입력 cnt : ");
    Serial.println(count);
    count = 0;
    interrupts();
  }

}