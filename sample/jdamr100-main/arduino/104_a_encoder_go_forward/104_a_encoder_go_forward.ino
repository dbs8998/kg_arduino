
// 모터 핀 정의
#define motor_A_enable 12
#define motor_B_enable 13
#define motor_A 10
#define motor_B 11

// 엔코더 핀 정의
#define encoder_A 2
#define encoder_B 3  // left from view of robot 

unsigned long time_prev = 0;
unsigned long time_curr = 0;
unsigned long start_time = 0;

volatile int encoder_A_count = 0;  // 엔코더 카운트 변수
volatile int encoder_B_count = 0;  // 엔코더 카운트 변수

int moving_direction = 0;
int moving_speed = 0;
int delta_R = 0;
int delta_L = 0;
int speed_A = 200;
int speed_B = 200;

bool stop_flag = false;

String inString;

void motorForward(int speed_A, int speed_B) {
  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, HIGH);
  analogWrite(motor_A, speed_A);
  analogWrite(motor_B, speed_B);
}

// 모터 후진 함수
void motorBackward(int speed_A, int speed_B) {
  digitalWrite(motor_A_enable, LOW);
  digitalWrite(motor_B_enable, LOW);
  analogWrite(motor_A, speed_A);
  analogWrite(motor_B, speed_B);
}

// 모터 정지 함수
void motorStop() {
  digitalWrite(motor_A_enable, LOW);
  digitalWrite(motor_B_enable, LOW);
  analogWrite(motor_A, 0);
  analogWrite(motor_B, 0);
}

// 엔코더 인터럽트 서비스 루틴 (ISR)
void encoder_A_ISR() {
  encoder_A_count++;
}

void encoder_B_ISR() {
  encoder_B_count++;
}


void speed_correct() {
  static int encoder_A_cnt_last = 0;
  static int encoder_B_cnt_last = 0;

  delta_R = encoder_A_count - encoder_A_cnt_last;
  delta_L = encoder_B_count - encoder_B_cnt_last;

  encoder_A_cnt_last = encoder_A_count;
  encoder_B_cnt_last = encoder_B_count;

  int diff = delta_R - delta_L;

  double correction_factor = 5;

  if (diff < 0) {
    speed_A -= int(diff * correction_factor);
    speed_B += int(diff * correction_factor);
  } else if (diff > 0) {
    speed_A += int(-diff * correction_factor);
    speed_B -= int(-diff * correction_factor);
  }
  speed_A = map(speed_A,0,255,130,254);
  speed_B = map(speed_B,0,255,130,254);
}


void setup() {
  pinMode(motor_A_enable, OUTPUT);
  pinMode(motor_B_enable, OUTPUT);
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);

  // 엔코더 핀 입력 설정 및 인터럽트 연결
  pinMode(encoder_A, INPUT);
  pinMode(encoder_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_A), encoder_A_ISR, RISING); // A채널 상승 시 인터럽트 발생
  attachInterrupt(digitalPinToInterrupt(encoder_B), encoder_B_ISR, RISING); // A채널 상승 시 인터럽트 발생

  Serial.begin(115200);  // 시리얼 모니터 시작

  start_time = millis();
  stop_flag = false;
}

void loop() {
  if (!stop_flag) {
    motorForward(speed_A, speed_B);
    Serial.print("speed_A: ");
    Serial.println(speed_A);
    Serial.print("speed_B: ");
    Serial.println(speed_B);
    Serial.println("");
  } else {
    motorStop();
  }

  time_curr = millis();
  if (time_curr - time_prev >= 10) {
    time_prev = time_curr;
    speed_correct();
  } 

  if (millis() - start_time >= 15000) {
    stop_flag = true;
  }

  delay(50);
}