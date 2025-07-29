# JDAMR100 아두이노 코드

[101_motor_no_speed_control 모터 정/역회전 테스트(아두이노)](#101_motor_no_speed_control)   
[101_motor_variable_speed_control 모터 속도 조절 테스트(아두이노)](#101_motor_variable_speed_control)   
[102_ir_test IR 센서 테스트(아두이노)](#102_ir_test)   
[103_ultrasonic_test 초음파 센서 테스트(아두이노)](#103_ultrasonic_test)   
[104_encoder_go_forward 양쪽 바퀴 엔코더 동작 테스트(아두이노)](#104_encoder_go_forward)   
[105_arduino_serial_comm_with_esp32 아두이노 - ESP32 시리얼 통신(아두이노 수신 코드)](#105_arduino_serial_comm_with_esp32)   
[105_esp32_serial_comm_with_arduino 아두이노 - ESP32 시리얼 통신(esp32 출력 코드)](#105_esp32_serial_comm_with_arduino)   
[106_esp32_i2c_test 소프트 시리얼을 통해 ESP32에서 전송한 정보를 받는 코드(아두이노)](#106_esp32_i2c_test)   
[107_robot_drive_with_encoder ESP32 없이 엔코더를 사용하여 직진하는 코드(아두이노)](#107_robot_drive_with_encoder)   
[108_arduino_serial_esp32_comm 아두이노 - ESP32 블루투스 통신(아두이노)](#108_arduino_serial_esp32_comm)   
[109_arduino_motor_remote_control ESP32에서 수신한 W,A,S,D,SPACE로 모터 제어(아두이노)](#109_arduino_motor_remote_control)   
[110_esp32_socket_comm ESP32와 소켓(TCP) 통신하는 파이썬 클라이언트](#110_esp32_socket_comm)   
[110_esp32_socket_serial_bridge 클라이언트(소켓) - ESP32 - 아두이노(시리얼) 통신하는 코드(ESP32)](#110_esp32_socket_serial_bridge)   
[111_esp32_socket_robot_remote ESP32와 소켓(TCP) 통신하여 로봇을 제어하는 파이썬 클라이언트](#111_esp32_socket_robot_remote)   
[112_arduino_ros_motor_remote_ctrl ESP32에 W,A,S,D,SPACE를 전송하는 파이썬 클라이언트](#112_arduino_ros_motor_remote_ctrl)   
[balancing_l298p MPU6050 센서와 L298P 모터드라이버를 이용한 밸런싱 로봇 구현](#balancing_l298p)   
[line_tracer 적외선(IR) 센서를 이용한 라인 트레이싱(Line Tracing)](#line_tracer)


## 101_motor_no_speed_control

### 📌 코드의 핵심 기능 요약
- 모터를 **1초 동안 회전** 후 **1초 동안 정지**하는 테스트 코드.
- 속도 조절 없이 **단순 ON/OFF 제어**만 수행.
- `digitalWrite()`를 사용하여 모터의 **회전 방향을 HIGH(한쪽 방향)**로 설정.
- 향후 속도 조절(`analogWrite()`) 기능을 추가하면 **PWM 기반 속도 제어**도 가능.

### 1. 핀 정의
```cpp
#define motor_A_enable 12
#define motor_B_enable 13
#define motor_A 10
#define motor_B 11
```
- `motor_A_enable` (핀 12) → 모터 A의 **전원 ON/OFF** 제어
- `motor_B_enable` (핀 13) → 모터 B의 **전원 ON/OFF** 제어
- `motor_A` (핀 10) → 모터 A의 **회전 방향 제어**
- `motor_B` (핀 11) → 모터 B의 **회전 방향 제어**

👉 **L298N 모터 드라이버** 같은 모듈을 사용하여, 모터를 제어하는 방식입니다.

---

### 2. `setup()` 함수
```cpp
void setup() {
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  pinMode(motor_A_enable, OUTPUT);
  pinMode(motor_B_enable, OUTPUT);
}
```
- **모든 핀을 출력 모드(`OUTPUT`)로 설정**하여 전압을 제어할 수 있도록 합니다.

---

### 3. `loop()` 함수
```cpp
void loop() {
  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, HIGH);
  digitalWrite(motor_A, HIGH);
  digitalWrite(motor_B, HIGH);
  delay(1000);
  
  digitalWrite(motor_A_enable, LOW);
  digitalWrite(motor_B_enable, LOW);
  digitalWrite(motor_A, HIGH);
  digitalWrite(motor_B, HIGH);
  delay(1000);
}
```
📌 **이 코드의 동작 방식**
1. **모터 ON**  
   - `motor_A_enable`과 `motor_B_enable`을 `HIGH`로 설정 → 전원 공급  
   - `motor_A`와 `motor_B`를 `HIGH`로 설정 → **모터 회전**  
   - 1초(`delay(1000)`) 동안 회전

2. **모터 OFF**  
   - `motor_A_enable`과 `motor_B_enable`을 `LOW`로 설정 → **전원 차단**  
   - `motor_A`, `motor_B`는 그대로 HIGH  
   - 1초(`delay(1000)`) 동안 멈춤  

📌 **주석 처리된 부분 (`analogWrite`)**
```cpp
//analogWrite(motor_A, x);
//analogWrite(motor_B, x);
```
- 속도 조절(`PWM`)을 위한 코드가 주석 처리되어 있음 → **단순 ON/OFF 방식으로 동작**.



## 101_motor_variable_speed_control

### **📌 코드의 핵심 기능 요약**
- `analogWrite()`를 이용하여 **모터 속도를 조절**하는 코드.
- 1초 동안 **최대 속도 → 중간 속도 → 정지** 순서로 실행.
- **PWM 신호(0~255 값)를 사용하여 속도를 부드럽게 조절**할 수 있음.

### 1. 핀 정의
```cpp
#define motor_A_enable 12
#define motor_B_enable 13
#define motor_A 10
#define motor_B 11
```
- `motor_A_enable` (핀 12) → 모터 A의 **전원 ON/OFF** 제어
- `motor_B_enable` (핀 13) → 모터 B의 **전원 ON/OFF** 제어
- `motor_A` (핀 10) → 모터 A의 **속도 및 방향 제어 (PWM 출력)**
- `motor_B` (핀 11) → 모터 B의 **속도 및 방향 제어 (PWM 출력)**

👉 **L298N 모터 드라이버** 같은 모듈을 사용하여 모터 속도를 조절합니다.

---

### 2. `setup()` 함수
```cpp
void setup() {
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  pinMode(motor_A_enable, OUTPUT);
  pinMode(motor_B_enable, OUTPUT);
}
```
- **모든 핀을 출력 모드(`OUTPUT`)로 설정**하여 전압을 제어할 수 있도록 합니다.

---

### 3. `loop()` 함수
```cpp
void loop() {

  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, HIGH);
  analogWrite(motor_A, 255);
  analogWrite(motor_B, 255);
  delay(1000);
 
  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, HIGH);
  analogWrite(motor_A, 127);
  analogWrite(motor_B, 127);
  delay(1000);

  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, HIGH);
  analogWrite(motor_A, 0);
  analogWrite(motor_B, 0);
  delay(1000);
}
```
📌 **이 코드의 동작 방식**
1. **최대 속도로 모터 회전**
   - `motor_A_enable`과 `motor_B_enable`을 `HIGH`로 설정 → 전원 공급
   - `analogWrite(motor_A, 255)`, `analogWrite(motor_B, 255)` → **PWM 신호 255(최대 속도)**
   - 1초(`delay(1000)`) 동안 회전

2. **절반 속도로 모터 회전**
   - `analogWrite(motor_A, 127)`, `analogWrite(motor_B, 127)` → **PWM 신호 127(중간 속도)**
   - 1초(`delay(1000)`) 동안 회전

3. **모터 정지**
   - `analogWrite(motor_A, 0)`, `analogWrite(motor_B, 0)` → **PWM 신호 0 (정지)**
   - 1초(`delay(1000)`) 동안 멈춤

📌 **PWM을 활용한 속도 제어**
- `analogWrite(pin, value)`를 사용하여 **0~255 범위**의 값을 출력
- `255`는 **최대 속도**, `127`은 **절반 속도**, `0`은 **정지**



## 102_IR_test

### **📌 코드의 핵심 기능 요약**
- **IR 센서 2개의 신호를 읽어 시리얼 모니터에 출력하는 코드**.
- `digitalRead()`를 이용하여 센서의 **HIGH(1)/LOW(0) 값을 판별**.
- 시리얼 모니터를 통해 **장애물 감지 여부를 실시간으로 확인 가능**.

### 1. 핀 정의
```cpp
int ir1 = 7;
int ir2 = 8;
```
- `ir1` (핀 7) → **첫 번째 IR 센서 입력 핀**
- `ir2` (핀 8) → **두 번째 IR 센서 입력 핀**

👉 **IR 센서**는 장애물을 감지하면 **HIGH(1) 또는 LOW(0)** 값을 출력하는 방식으로 동작합니다.

---

### 2. `setup()` 함수
```cpp
void setup() {
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  Serial.begin(115200);
}
```
- **IR 센서 핀(`ir1`, `ir2`)을 입력(`INPUT`) 모드로 설정**하여 값을 읽을 수 있도록 설정.
- **시리얼 통신(`Serial.begin(115200)`)을 설정**하여 센서 값을 출력할 수 있도록 설정.

---

### 3. `loop()` 함수
```cpp
void loop() {
 
  int state = digitalRead(ir1);
  Serial.print("ir1 = ");
  Serial.println(state);

  state = digitalRead(ir2);
  Serial.print("ir2 = ");
  Serial.println(state);

  delay(100);
}
```
📌 **이 코드의 동작 방식**
1. **첫 번째 IR 센서(`ir1`)의 상태를 읽음**
   - `digitalRead(ir1)`을 사용하여 HIGH(1) 또는 LOW(0) 값을 가져옴.
   - `Serial.print("ir1 = "); Serial.println(state);`로 시리얼 모니터에 출력.

2. **두 번째 IR 센서(`ir2`)의 상태를 읽음**
   - `digitalRead(ir2)`을 사용하여 HIGH(1) 또는 LOW(0) 값을 가져옴.
   - `Serial.print("ir2 = "); Serial.println(state);`로 시리얼 모니터에 출력.

3. **0.1초(100ms) 대기**
   - `delay(100);`로 센서값을 읽는 주기를 조정.



## 103_ultrasonic_test

### 📌 코드의 핵심 기능 요약
- **초음파 센서를 사용하여 거리 컴퓨팅**을 수행하는 코드.
- `TRIG` 핀을 이용해 **초음파 시험을 전송**하고, `ECHO` 핀을 통해 **반사된 시험을 결과로 수신**.
- `pulseIn()`을 사용하여 시간을 컴퓨팅하고, **거리(제곱은 cm)로 변환**.
- 결과를 **시리얼 모니터에 출력**하여 검사 가능.

---

### 1. 핀 정의
```cpp
#define TRIG A4
#define ECHO A5
```
- `TRIG` (A4 핀) → **초음파 시험 전송 핀**
- `ECHO` (A5 핀) → **반사된 초음파 시험 수신 핀**

👉 **TRIG 핀에서 시험을 지정하고, ECHO 핀에서 수신 후 거리 계산**을 수행.

---

### 2. `setup()` 함수
```cpp
void setup() {
  Serial.begin(115200);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}
```
- **시리얼 통신을 115200 baud rate로 설정**하여 거리 값을 검사할 수 있도록 함.
- `TRIG` 핀을 `OUTPUT`으로 설정하여 **초음파 시험을 전송 가능**하게 함.
- `ECHO` 핀을 `INPUT`으로 설정하여 **반사된 시험을 수신 가능**하게 함.

---

### 3. `loop()` 함수
```cpp
void loop()
{
  long duration, distance;
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = duration * 17 / 1000;
  Serial.println(duration);
  Serial.print("DIstance : ");
  Serial.print(distance);
  Serial.println(" Cm");
  delay(1000);
}
```

📌 **이 코드의 동작 방식**
1. **초음파 시험 전송**
   - `TRIG` 핀을 LOW → HIGH (10µs) → LOW 로 전환하여 **초음파 시험을 전송**.

2. **반사된 시험 수신 및 거리 계산**
   - `pulseIn(ECHO, HIGH)`를 사용하여 **반사된 시험과의 시간 계산**.
   - `distance = duration * 17 / 1000;`
     → **음소가 (343m/s)를 이용하여 거리(제곱은 cm)로 변환**.

3. **시리얼 모니터 출력**
   - **`duration` 값을 출력** (반사된 시간).
   - **`distance` 값을 출력** (거리).
   - 1초 (`delay(1000)`) 후 복돌.

## 104_encoder_go_forward

### 📌 코드의 핵심 기능 요약
- **엔코더를 사용하여 양쪽 바퀴의 회전 수를 측정하고 균형을 조정하며 직진하는 코드**.
- `encoder_R` 및 `encoder_L` 인터럽트를 사용하여 **각 바퀴의 회전 속도를 측정**.
- `speed_correct()` 함수를 통해 **두 바퀴의 속도를 자동으로 보정**.
- 3초간 직진한 후 정지하며, **시리얼 모니터를 통해 속도를 출력**.

---

### 1. 핀 정의
```cpp
#define motor_A_1A 5
#define motor_A_1B 6
#define motor_B_1A 10
#define motor_B_1B 9

#define encoder_R 2
#define encoder_L 3
```
- `motor_A_1A`, `motor_A_1B` → **왼쪽 바퀴 모터 제어 핀**
- `motor_B_1A`, `motor_B_1B` → **오른쪽 바퀴 모터 제어 핀**
- `encoder_R` (핀 2) → **오른쪽 바퀴 엔코더 입력 핀**
- `encoder_L` (핀 3) → **왼쪽 바퀴 엔코더 입력 핀**

👉 **모터 드라이버와 엔코더를 이용해 바퀴의 회전 속도를 측정하고 조정**

---

### 2. 모터 및 속도 조정 함수
```cpp
void forward(int speed_R, int speed_L) {
  analogWrite(motor_A_1A, 0);
  analogWrite(motor_A_1B, speed_R);
  analogWrite(motor_B_1A, speed_L);
  analogWrite(motor_B_1B, 0);
}

void stopAll() {
  analogWrite(motor_A_1A, 0);
  analogWrite(motor_A_1B, 0);
  analogWrite(motor_B_1A, 0);
  analogWrite(motor_B_1B, 0);
}
```
- `forward(speed_R, speed_L)`: **모터를 정방향으로 회전**시켜 이동.
- `stopAll()`: **모든 모터를 정지**시킴.

---

### 3. 엔코더 인터럽트 및 속도 보정
```cpp
void encoder_R_ISR() {
  encoder_R_cnt++;
}

void encoder_L_ISR() {
  encoder_L_cnt++;
}
```
- 엔코더 값이 변할 때마다 인터럽트를 통해 **회전 수를 증가**시킴.

```cpp
void speed_correct() {
  static int encoder_R_cnt_last = 0;
  static int encoder_L_cnt_last = 0;

  delta_R = encoder_R_cnt - encoder_R_cnt_last;
  delta_L = encoder_L_cnt - encoder_L_cnt_last;

  encoder_R_cnt_last = encoder_R_cnt;
  encoder_L_cnt_last = encoder_L_cnt;

  int diff = delta_R - delta_L;
  int correction_factor = 1;

  if (diff > 0) {
    speed_R -= diff * correction_factor;
    speed_L += diff * correction_factor;
    if (speed_R < 0) speed_R = 0;
    if (speed_L > 255) speed_L = 255;
  } else if (diff < 0) {
    speed_R += -diff * correction_factor;
    speed_L -= -diff * correction_factor;
    if (speed_R > 255) speed_R = 255;
    if (speed_L < 0) speed_L = 0;
  }
}
```
- **양쪽 바퀴의 회전 속도를 비교하여 보정**.
- `delta_R`과 `delta_L`을 비교하여 속도를 조절함.
- `speed_R`과 `speed_L`을 조정하여 **두 바퀴의 속도를 균형 맞춤**.

---

### 4. `setup()` 함수
```cpp
void setup() {
  Serial.begin(115200);
  pinMode(motor_A_1A, OUTPUT);
  pinMode(motor_A_1B, OUTPUT);
  pinMode(motor_B_1A, OUTPUT);
  pinMode(motor_B_1B, OUTPUT);

  pinMode(encoder_R, INPUT_PULLUP);
  pinMode(encoder_L, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder_R), encoder_R_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoder_L), encoder_L_ISR, FALLING);

  start_time = millis();
  stop_flag = false;
}
```
- **엔코더 핀을 `INPUT_PULLUP`으로 설정**하여 풀업 저항 사용.
- `attachInterrupt()`를 사용하여 **엔코더 값 변화를 감지**.
- `start_time`을 기록하여 3초 후 정지하도록 설정.

---

### 5. `loop()` 함수
```cpp
void loop() {
  if (!stop_flag) {
    forward(speed_R, speed_L);
    Serial.print("speed_R: ");
    Serial.println(speed_R);
    Serial.print("speed_L: ");
    Serial.println(speed_L);
    Serial.println("");
  } else {
    stopAll();
  }

  time_curr = millis();
  if (time_curr - time_prev >= 100) {
    time_prev = time_curr;
    speed_correct();
  }

  if (millis() - start_time >= 3000) {
    stop_flag = true;
  }

  delay(50);
}
```
- **모터를 작동시키며 3초 동안 직진**.
- 100ms마다 `speed_correct()`를 호출하여 속도를 보정.
- `millis()`를 사용하여 **3초 후 정지**하도록 설정.

---

## 105_arduino_serial_comm_with_esp32

### 📌 코드의 핵심 기능 요약
- **아두이노와 ESP32 간의 소프트웨어 시리얼 통신을 수행하는 코드**.
- `SoftwareSerial`을 사용하여 **7번(RX), 8번(TX) 핀을 통해 시리얼 데이터 수신**.
- `mySerial.available()`을 이용해 데이터가 수신되었는지 확인하고, **수신된 데이터를 시리얼 모니터에 출력**.

---

### 1. 소프트웨어 시리얼 설정
```cpp
#include <SoftwareSerial.h>

// 소프트웨어 시리얼 핀 설정 (7번: RX, 8번: TX)
SoftwareSerial mySerial(7, 8); // RX, TX
```
- `SoftwareSerial` 라이브러리를 포함하여 **소프트웨어 기반 시리얼 통신을 사용**.
- `mySerial(7, 8)` →  **7번 핀을 RX, 8번 핀을 TX로 설정**하여 ESP32와 통신 준비.

---

### 2. `setup()` 함수
```cpp
void setup() {
  // 소프트웨어 시리얼 시작
  mySerial.begin(9600);
  
  // 기본 시리얼 모니터 시작
  Serial.begin(115200);
  
  Serial.println("SoftwareSerial 데이터 수신 준비 완료");
}
```
- `mySerial.begin(9600);` → **소프트웨어 시리얼 통신 속도를 9600bps로 설정**.
- `Serial.begin(115200);` → **기본 시리얼 모니터 속도를 115200bps로 설정**.
- 프로그램 시작 시 **"SoftwareSerial 데이터 수신 준비 완료" 메시지를 출력**.

---

### 3. `loop()` 함수
```cpp
void loop() {
  // 소프트웨어 시리얼로 데이터가 수신되었는지 확인
  if (mySerial.available()) {
    char incomingByte = mySerial.read();  // 수신한 데이터 읽기
    Serial.print("수신한 데이터: ");
    Serial.println(incomingByte);
  }

  // 100ms 지연
  delay(100);
}
```
- `mySerial.available()`을 사용하여 **데이터가 수신되었는지 확인**.
- 데이터가 수신되면 `mySerial.read()`를 통해 **1바이트(char)씩 읽어들임**.
- 읽은 데이터를 `Serial.print()`를 이용하여 **시리얼 모니터에 출력**.
- 100ms마다 루프를 반복하여 **연속적인 데이터 수신을 처리**.

---

## 105_esp32_serial_comm_with_arduino

### 📌 코드의 핵심 기능 요약
- **ESP32가 하드웨어 시리얼2(Serial2)를 이용하여 아두이노와 통신하는 코드**.
- `Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2)`를 사용하여 **ESP32의 RXD2(16번), TXD2(17번) 핀을 통해 시리얼 통신을 설정**.
- 1초마다 **"hello world"를 기본 시리얼(USB) 출력, "hello 2 world"를 Serial2(UART2)로 출력**.

---

### 1. 핀 정의
```cpp
#define RXD2 16
#define TXD2 17
```
- `RXD2` (16번 핀) → **ESP32의 하드웨어 UART2 RX(수신) 핀**
- `TXD2` (17번 핀) → **ESP32의 하드웨어 UART2 TX(송신) 핀**

👉 **ESP32의 Serial2(UART2) 핀을 사용하여 아두이노와 직접 통신 가능**.

---

### 2. `setup()` 함수
```cpp
void setup() {
  Serial.begin(115200);  // 시리얼 모니터 시작
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(1000);
}
```
- `Serial.begin(115200);` → **ESP32의 기본 시리얼(USB) 출력 속도를 115200bps로 설정**.
- `Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);` → **ESP32의 UART2(Serial2)를 9600bps로 설정하여 아두이노와 통신 준비**.
- `delay(1000);` → 초기화 지연을 줘서 안정적인 통신이 가능하도록 설정.

---

### 3. `loop()` 함수
```cpp
void loop() {
  Serial.println("hello world");
  Serial2.println("hello 2 world");
  delay(1000);
}
```
- `Serial.println("hello world");` → **ESP32의 기본 시리얼(USB) 모니터에 "hello world" 출력**.
- `Serial2.println("hello 2 world");` → **ESP32의 UART2(Serial2)를 통해 "hello 2 world"를 아두이노에 전송**.
- `delay(1000);` → **1초마다 데이터를 반복 전송**.

---

## 106_esp32_i2c_test

### 📌 코드의 핵심 기능 요약
- **ESP32가 I2C 통신을 이용하여 OLED 디스플레이(SSD1306)를 제어하는 코드**.
- `U8x8lib` 라이브러리를 사용하여 **텍스트를 화면에 출력**.
- `"Hello World!"`와 `"012345678901234567890123456789"` 문자열을 표시하고, **일부 텍스트를 반전(Inverse) 효과로 출력**.

---

### 1. 라이브러리 포함 및 디스플레이 객체 생성
```cpp
#include <Arduino.h>
#include <U8x8lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);
```
- `#include <U8x8lib.h>` → **SSD1306 OLED 디스플레이를 제어하기 위한 라이브러리 포함**.
- `#include <SPI.h>` → **U8x8lib가 SPI 모드도 지원하기 때문에 조건부 포함**.
- `U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);` → **I2C 방식의 128x64 해상도 SSD1306 디스플레이 객체 생성**.

---

### 2. `setup()` 함수
```cpp
void setup(void) {
  u8x8.begin();
  u8x8.setPowerSave(0);
}
```
- `u8x8.begin();` → **디스플레이 초기화 및 I2C 통신 시작**.
- `u8x8.setPowerSave(0);` → **디스플레이 절전 모드 해제**.

---

### 3. `loop()` 함수
```cpp
void loop(void) {
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0,1,"Hello World!");
  u8x8.setInverseFont(1);
  u8x8.drawString(0,0,"012345678901234567890123456789");
  u8x8.setInverseFont(0);
  u8x8.refreshDisplay();
  delay(2000);
}
```
- `u8x8.setFont(u8x8_font_chroma48medium8_r);` → **텍스트 출력에 사용할 폰트 설정**.
- `u8x8.drawString(0,1,"Hello World!");` → **좌표 (0,1)에 "Hello World!" 출력**.
- `u8x8.setInverseFont(1);` → **이후 출력될 텍스트를 반전(Inverse) 모드로 설정**.
- `u8x8.drawString(0,0,"012345678901234567890123456789");` → **좌표 (0,0)에 긴 문자열 출력**.
- `u8x8.setInverseFont(0);` → **텍스트 반전 모드를 해제**.
- `u8x8.refreshDisplay();` → **SSD1606/7 같은 일부 디스플레이에서 필요**.
- `delay(2000);` → **2초 동안 화면을 유지한 후 반복**.

---

## 107_robot_drive_with_encoder

### 📌 코드의 핵심 기능 요약
- **ESP32 없이 엔코더를 사용하여 로봇을 직진시키는 코드**.
- `encoder_R` 및 `encoder_L` 인터럽트를 사용하여 **각 바퀴의 회전 수를 측정**.
- `adjustSpeed()` 함수를 통해 **두 바퀴의 속도를 자동으로 보정**.
- 시리얼 입력을 받아 **이동 방향과 속도를 설정**하며, 기본적으로 직진, 후진, 좌회전, 우회전을 수행.

---

### 1. 핀 정의
```cpp
#define motor_A_enable 12
#define motor_B_enable 13
#define motor_A 10
#define motor_B 11

#define encoder_R 2
#define encoder_L 3
```
- `motor_A_enable`, `motor_B_enable` → **모터 속도 조절 핀**
- `motor_A`, `motor_B` → **모터 회전 방향 제어 핀**
- `encoder_R` (핀 2) → **오른쪽 바퀴 엔코더 입력 핀**
- `encoder_L` (핀 3) → **왼쪽 바퀴 엔코더 입력 핀**

👉 **엔코더 값을 기반으로 바퀴 속도를 조정하여 균형 잡힌 직진 주행 가능**.

---

### 2. 모터 제어 함수
```cpp
void forward(int R, int L) {
  analogWrite(motor_A_enable, L);
  analogWrite(motor_B_enable, R);
  digitalWrite(motor_A, HIGH);
  digitalWrite(motor_B, HIGH);
}
```
- `forward(R, L)`: **지정한 속도로 직진**.
- `backward(R, L)`, `turnLeft(R, L)`, `turnRight(R, L)`, `stopAll()`도 유사한 방식으로 구현.

---

### 3. 엔코더 인터럽트 및 속도 보정
```cpp
void encoder_R_ISR(){
  encoder_R_cnt++;
}

void encoder_L_ISR(){
  encoder_L_cnt++;
}
```
- 엔코더 값이 변할 때마다 **회전 수를 증가**.

```cpp
void adjustSpeed() {
  static int last_encoder_R_cnt = 0;
  static int last_encoder_L_cnt = 0;
  
  delta_R = encoder_R_cnt - last_encoder_R_cnt;
  delta_L = encoder_L_cnt - last_encoder_L_cnt;

  last_encoder_R_cnt = encoder_R_cnt;
  last_encoder_L_cnt = encoder_L_cnt;

  int diff = delta_R - delta_L;
  speed_adjust_val = diff * diff_weight;
}
```
- **양쪽 바퀴의 속도를 비교하여 차이를 보정**.
- `diff_weight` 값을 이용하여 **속도 차이에 따른 보정값을 계산**.

---

### 4. `setup()` 함수
```cpp
void setup() {
  Serial.begin(115200);

  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);

  pinMode(encoder_R, INPUT_PULLUP);
  pinMode(encoder_L, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder_R), encoder_R_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoder_L), encoder_L_ISR, FALLING);
}
```
- **엔코더 핀을 `INPUT_PULLUP`으로 설정**하여 풀업 저항 사용.
- `attachInterrupt()`를 사용하여 **엔코더 값 변화를 감지**.

---

### 5. `loop()` 함수
```cpp
void loop() {
  time_curr = millis();
  
  if (Serial.available() > 0) {
    inString = Serial.readStringUntil('\n');
    char cmd = inString[0];
    if (cmd == '1') {
      moving_direction = inString.substring(inString.indexOf('a') + 1, inString.indexOf('b')).toInt();
      moving_speed = inString.substring(inString.indexOf('b') + 1, inString.indexOf('c')).toInt();

      speed_R = moving_speed + speed_adjust_val;
      speed_L = moving_speed + speed_adjust_val;

      speed_R_turn = speed_R - turn_speed;
      speed_L_turn = speed_R - turn_speed;

      speed_R = constrain(speed_R,0,255);
      speed_L = constrain(speed_L,0,255);
      speed_R_turn = constrain(speed_R_turn,0,255);
      speed_L_turn = constrain(speed_L_turn,0,255);

      if (moving_direction == 1) {
        forward(speed_R, speed_L);
      } else if (moving_direction == 2) {
        backward(speed_R, speed_L);
      } else if (moving_direction == 3) {
        turnLeft(speed_R_turn, speed_L_turn);
      } else if (moving_direction == 4) {
        turnRight(speed_R_turn, speed_L_turn);
      }
    } else if (cmd == '0') {
      diff_weight = inString.substring(inString.indexOf('a') + 1, inString.indexOf('b')).toInt();
      encoder_update_interval = inString.substring(inString.indexOf('b') + 1, inString.indexOf('c')).toInt();
      delay_time = inString.substring(inString.indexOf('c') + 1, inString.indexOf('d')).toInt();
      turn_speed = inString.substring(inString.indexOf('d') + 1, inString.indexOf('e')).toInt();
    }
  } else {
    stopAll();
  }

  if (time_curr - time_prev >= encoder_update_interval) {
    time_prev = time_curr;
    adjustSpeed();
  }
  delay(delay_time);
}
```
- **시리얼 입력을 통해 이동 방향과 속도를 설정**.
- `1`을 입력받으면 **방향(moving_direction)과 속도(moving_speed) 값을 분석하여 이동**.
- `0`을 입력받으면 **속도 보정 파라미터(diff_weight, delay_time 등)를 설정**.
- `encoder_update_interval` 마다 `adjustSpeed()`를 호출하여 **속도 차이를 조정**.
- 기본적으로 입력이 없을 경우 `stopAll()`을 실행하여 정지.

---

## 108_arduino_serial_esp32_comm

### 📌 코드의 핵심 기능 요약
- **아두이노와 ESP32 간의 소프트웨어 시리얼 통신을 수행하는 코드**.
- `SoftwareSerial`을 사용하여 **7번(RX), 8번(TX) 핀을 통해 시리얼 데이터 송수신**.
- 수신된 데이터를 **시리얼 모니터에 출력하고, 다시 ESP32로 전송(Echo)**.

---

### 1. 소프트웨어 시리얼 설정
```cpp
#include <SoftwareSerial.h>

// 소프트웨어 시리얼 핀 설정 (7번: RX, 8번: TX)
SoftwareSerial mySerial(7, 8); // RX, TX
```
- `SoftwareSerial` 라이브러리를 포함하여 **소프트웨어 기반 시리얼 통신을 사용**.
- `mySerial(7, 8)` → **7번 핀을 RX, 8번 핀을 TX로 설정**하여 ESP32와 통신 준비.

---

### 2. `setup()` 함수
```cpp
void setup() {
  // 소프트웨어 시리얼 시작
  mySerial.begin(9600);
  
  // 기본 시리얼 모니터 시작
  Serial.begin(115200);
  
  Serial.println("SoftwareSerial 데이터 수신 준비 완료");
}
```
- `mySerial.begin(9600);` → **소프트웨어 시리얼 통신 속도를 9600bps로 설정**.
- `Serial.begin(115200);` → **기본 시리얼 모니터 속도를 115200bps로 설정**.
- 프로그램 시작 시 **"SoftwareSerial 데이터 수신 준비 완료" 메시지를 출력**.

---

### 3. `loop()` 함수
```cpp
void loop() {
  // 소프트웨어 시리얼로 데이터가 수신되었는지 확인
  if (mySerial.available()) {
    String incomingStr = mySerial.readStringUntil('\n');  // 수신한 데이터 읽기
    Serial.print("수신한 데이터: ");
    Serial.println(incomingStr);
    mySerial.println(incomingStr);
  }

  // 100ms 지연
  delay(100);
}
```
- `mySerial.available()`을 사용하여 **데이터가 수신되었는지 확인**.
- 데이터가 수신되면 `mySerial.readStringUntil('\n')`을 통해 **문자열을 읽어옴**.
- 읽은 데이터를 `Serial.print()`를 이용하여 **시리얼 모니터에 출력**.
- 받은 데이터를 다시 `mySerial.println()`을 사용하여 **ESP32로 되돌려 전송(Echo)**.
- 100ms마다 루프를 반복하여 **연속적인 데이터 수신을 처리**.

---

## 109_arduino_motor_remote_control

### 📌 코드의 핵심 기능 요약
- **ESP32에서 수신한 `w, a, s, d` 명령을 기반으로 아두이노에서 모터를 제어하는 코드**.
- `SoftwareSerial`을 이용해 **ESP32로부터 명령을 수신하고, 시리얼 모니터에서도 입력을 받을 수 있음**.
- 전진(`w`), 후진(`s`), 좌회전(`a`), 우회전(`d`), 정지(`space`) 명령을 인식하여 **모터를 제어**.

---

### 1. 핀 정의 및 소프트웨어 시리얼 설정
```cpp
#include <SoftwareSerial.h>

#define motor_A_enable 12
#define motor_B_enable 13
#define motor_A 10
#define motor_B 11

// 소프트웨어 시리얼 핀 설정 (7번: RX, 8번: TX)
SoftwareSerial mySerial(7, 8); // RX, TX
```
- `motor_A_enable`, `motor_B_enable` → **모터 속도 조절 핀**
- `motor_A`, `motor_B` → **모터 회전 방향 제어 핀**
- `mySerial(7, 8)` → **7번 핀을 RX, 8번 핀을 TX로 설정하여 ESP32와 시리얼 통신**

---

### 2. `setup()` 함수
```cpp
void setup() {
  // 소프트웨어 시리얼 시작
  mySerial.begin(9600);
  
  // 기본 시리얼 모니터 시작
  Serial.begin(115200);
  
  Serial.println("SoftwareSerial 데이터 수신 준비 완료");
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  pinMode(motor_A_enable, OUTPUT);
  pinMode(motor_B_enable, OUTPUT);
}
```
- `mySerial.begin(9600);` → **ESP32와의 시리얼 통신 속도를 9600bps로 설정**.
- `Serial.begin(115200);` → **기본 시리얼 모니터 속도를 115200bps로 설정**.
- 모터 제어를 위해 **모든 관련 핀을 출력 모드로 설정**.

---

### 3. 모터 제어 함수
```cpp
void forward(int R, int L) {
  digitalWrite(motor_A_enable, LOW);
  digitalWrite(motor_B_enable, LOW);
  analogWrite(motor_A, L);
  analogWrite(motor_B, R);
}
```
- `forward(R, L)`: **모터를 정방향으로 회전**하여 전진.
- `backward(R, L)`, `turnLeft(R, L)`, `turnRight(R, L)`, `stopAll()`도 유사한 방식으로 구현됨.

---

### 4. `loop()` 함수 - 명령 수신 및 모터 제어
```cpp
void loop() {
  // 소프트웨어 시리얼로 데이터가 수신되었는지 확인
  if (mySerial.available()) {
    String incomingStr2 = mySerial.readStringUntil('\n');  // 수신한 데이터 읽기
    if(incomingStr2[0] == 'w'){
      Serial.println("forward");
      forward(255, 255);
    }else if(incomingStr2[0] == 's'){
      Serial.println("backward");
      backward(255, 255);
    }else if(incomingStr2[0] == ' '){
      Serial.println("stop");
      stopAll();
    }else if(incomingStr2[0] == 'a'){
      Serial.println("turnleft");
      turnLeft(255, 255);
    }else if(incomingStr2[0] == 'd'){
      Serial.println("turnRight");
      turnRight(255, 255);
    }
  }
```
- **ESP32에서 받은 데이터(`w, a, s, d, space`)를 분석하여 모터 동작 수행**.
- 입력에 따라 **전진, 후진, 좌회전, 우회전, 정지 명령 실행**.

```cpp
  if(Serial.available()){
    String incomingStr = Serial.readStringUntil('\n');
    if(incomingStr[0] == 'w'){
      Serial.println("forward");
      forward(255, 255);
    }else if(incomingStr[0] == 's'){
      Serial.println("backward");
      backward(255, 255);
    }else if(incomingStr[0] == ' '){
      Serial.println("stop");
      stopAll();
    }else if(incomingStr[0] == 'a'){
      Serial.println("turnleft");
      turnLeft(255, 255);
    }else if(incomingStr[0] == 'd'){
      Serial.println("turnRight");
      turnRight(255, 255);
    }
  }
```
- **PC 시리얼 모니터에서 직접 명령 입력 가능**.
- 동일한 명령어(`w, a, s, d, space`)를 사용하여 **모터 제어 가능**.

```cpp
  // 100ms 지연
  delay(100);
}
```
- **명령을 처리한 후 100ms 대기 후 반복 실행**.

---
## 110_esp32_socket_comm

### 📌 코드의 핵심 기능 요약
- **ESP32와 소켓(TCP) 통신을 수행하는 클라이언트 코드**.
- ESP32 서버(IP, 포트)에 접속하여 **'w', 's', ' '`(정지) 명령을 전송**.
- ESP32로부터 **응답을 수신하고 출력**.
- 4회 반복하여 명령을 전송한 후, **소켓을 종료**.

---

### 1. 소켓 설정 및 서버 연결
```python
import socket
import time

# ESP32 서버의 IP 주소 및 포트 번호 설정
server_ip = '172.30.1.40'  # ESP32의 IP 주소 (Wi-Fi 연결 시 확인)
port = 8080                  # ESP32에서 설정한 포트 번호

# 소켓 생성 (IPv4, TCP)
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 서버에 연결
client_socket.connect((server_ip, port))
print(f"서버 {server_ip}:{port}에 연결 성공")
```
- `socket.socket(socket.AF_INET, socket.SOCK_STREAM)` → **IPv4, TCP 방식의 소켓 생성**.
- `client_socket.connect((server_ip, port))` → **ESP32 서버에 연결**.
- ESP32가 Wi-Fi에 연결되어 있어야 하며, 서버 IP 및 포트 번호를 확인해야 함.

---

### 2. 서버에 명령 전송 및 응답 수신
```python
for i in range(4):
    # 메시지 전송
    message = 'w'
    client_socket.send((message + "\n").encode())  # 줄바꿈 문자 추가
    # 서버로부터의 응답 수신
    response = client_socket.recv(1024)
    print(f"서버 응답: {response.decode()}")
    time.sleep(1)

    message = 's'
    client_socket.send((message + "\n").encode())  # 줄바꿈 문자 추가
    # 서버로부터의 응답 수신
    response = client_socket.recv(1024)
    print(f"서버 응답: {response.decode()}")
    time.sleep(1)
```
- `'w'`(전진), `'s'`(후진) 명령을 **ESP32 서버로 4회 반복하여 전송**.
- `client_socket.send((message + "\n").encode())` → **메시지를 전송** (`\n` 줄바꿈 추가).
- `client_socket.recv(1024)` → **ESP32로부터 응답을 수신**하고 출력.
- 1초 대기 후 다음 명령을 전송.

---

### 3. 정지 명령 전송 및 소켓 종료
```python
message = ' '
client_socket.send((message + "\n").encode())  # 줄바꿈 문자 추가
# 서버로부터의 응답 수신
response = client_socket.recv(1024)
print(f"서버 응답: {response.decode()}")
time.sleep(1)

# 소켓 종료
client_socket.close()
```
- `' '`(공백) 메시지를 전송하여 **로봇 정지 명령을 실행**.
- ESP32의 응답을 확인한 후 `client_socket.close()`를 호출하여 **TCP 연결 종료**.

---

## 110_esp32_socket_serial_bridge

### 📌 코드의 핵심 기능 요약
- **ESP32가 Wi-Fi를 통해 소켓 서버를 생성하고, 아두이노(또는 다른 MCU)와 시리얼 통신을 수행하는 코드**.
- `WiFiServer`를 이용해 **포트 8080에서 TCP 서버를 실행**.
- 클라이언트(예: Python 소켓 코드)에서 명령을 보내면 **ESP32가 해당 명령을 Serial2(UART2)로 전달**.
- 클라이언트에게 **"데이터 수신 완료" 응답을 전송**.
- **LED(GPIO 2번)를 사용하여 클라이언트 연결 상태를 시각적으로 표시**.

---

### 1. Wi-Fi 및 핀 설정
```cpp
#include <WiFi.h>

// Wi-Fi 설정
const char* ssid = "ConnectValue_A403_2G";       // Wi-Fi 이름
const char* password = "CVA403!@#$";  // Wi-Fi 비밀번호

#define RXD2 16   // 기본 RX2 핀 (GPIO16)
#define TXD2 17   // 기본 TX2 핀 (GPIO17)

// 서버 포트 설정
WiFiServer server(8080);
```
- `ssid`, `password` → **ESP32가 연결할 Wi-Fi 네트워크 정보**.
- `RXD2(16)`, `TXD2(17)` → **ESP32의 UART2 핀 설정 (시리얼 통신용)**.
- `WiFiServer server(8080);` → **ESP32가 포트 8080에서 TCP 서버를 실행하도록 설정**.

---

### 2. `setup()` 함수
```cpp
void setup() {
  Serial.begin(115200);  // 시리얼 모니터 시작
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(1000);
  
  // Wi-Fi 연결
  Serial.println("Wi-Fi 연결 중...");
  WiFi.begin(ssid, password);

  // Wi-Fi 연결이 완료될 때까지 대기
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("연결 중...");
  }

  Serial.println("Wi-Fi 연결 완료!");
  Serial.print("ESP32 IP 주소: ");
  Serial.println(WiFi.localIP());

  // TCP 서버 시작
  server.begin();
  Serial.println("서버 시작됨, 클라이언트 대기 중...");

  pinMode(2, OUTPUT);  // LED 핀 설정
  digitalWrite(2, LOW); // 초기 상태에서 LED 끄기
}
```
- `Serial.begin(115200);` → **ESP32 기본 시리얼 통신 초기화**.
- `Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);` → **UART2(Serial2) 초기화 (아두이노와 시리얼 통신용)**.
- `WiFi.begin(ssid, password);` → **Wi-Fi 연결 시도**.
- `while (WiFi.status() != WL_CONNECTED)` → **Wi-Fi가 연결될 때까지 대기**.
- `WiFi.localIP();` → **ESP32의 IP 주소를 시리얼 모니터에 출력**.
- `server.begin();` → **TCP 서버 시작 (포트 8080 대기)**.
- `pinMode(2, OUTPUT);` → **GPIO 2번 핀을 출력 모드로 설정(LED용)**.
- `digitalWrite(2, LOW);` → **초기 상태에서 LED를 꺼 둠**.

---

### 3. `loop()` 함수 - 클라이언트 요청 처리 및 시리얼 통신
```cpp
void loop() {
  // 클라이언트가 연결되면 처리
  WiFiClient client = server.available();

  if (client) {
    Serial.println("클라이언트가 연결되었습니다.");
    digitalWrite(2, HIGH);  // 클라이언트 연결 시 LED 켜기

    while (client.connected()) {
      if (client.available()) {
        // 클라이언트로부터 데이터 수신
        String message = client.readStringUntil('\n');
        Serial.println(message);
        Serial2.println(message);

        // 클라이언트에 응답 보내기
        client.println("데이터 수신 완료");
      }
    }

    digitalWrite(2, LOW);  // 클라이언트 연결 해제 시 LED 끄기
    client.stop();
    Serial.println("클라이언트 연결 종료.");
  }
}
```
- `WiFiClient client = server.available();` → **클라이언트가 연결되었는지 확인**.
- `digitalWrite(2, HIGH);` → **클라이언트가 연결되면 LED를 켜서 연결 상태를 표시**.
- `client.readStringUntil('\n');` → **클라이언트가 보낸 메시지를 수신**.
- `Serial.println(message);` → **수신된 메시지를 시리얼 모니터에 출력**.
- `Serial2.println(message);` → **UART2(Serial2)를 통해 아두이노로 명령 전송**.
- `client.println("데이터 수신 완료");` → **클라이언트에게 응답 전송**.
- `digitalWrite(2, LOW);` → **클라이언트 연결이 끊어지면 LED를 꺼서 상태를 표시**.
- `client.stop();` → **클라이언트 연결 종료**.

---

## 111_esp32_socket_robot_remote

### 📌 코드의 핵심 기능 요약
- **ESP32와 소켓(TCP) 통신을 수행하는 로봇 원격 제어 클라이언트 코드**.
- `pygame`을 사용하여 **키보드 입력(`w, a, s, d, space`)을 감지**하고 이를 ESP32로 전송.
- ESP32에서 수신된 데이터를 기반으로 **아두이노의 모터를 제어**하여 로봇을 움직임.
- 이 코드가 동작하려면 **다음 두 개의 코드가 함께 실행되어야 함**:
  1. `109_arduino_motor_remote_control.ino` → **아두이노에서 모터를 제어하는 코드**
  2. `110_esp32_socket_serial_bridge.ino` → **ESP32가 Wi-Fi를 통해 명령을 받고, 이를 시리얼로 아두이노에 전달하는 브릿지 역할 수행**

**⚙️ 전체적인 동작 방식**
1. `111_esp32_socket_robot_remote.py` (현재 코드)에서 키 입력을 감지하고 ESP32로 명령 전송.
2. `110_esp32_socket_serial_bridge.ino`에서 **Wi-Fi를 통해 수신한 명령을 아두이노로 전달**.
3. `109_arduino_motor_remote_control.ino`에서 **아두이노가 ESP32에서 받은 명령을 기반으로 모터를 제어**.

---

### 1. 소켓 설정 및 ESP32 서버 연결
```python
import socket
import pygame

# ESP32 서버의 IP 주소 및 포트 번호 설정
server_ip = '172.30.1.40'  # ESP32의 IP 주소 (Wi-Fi 연결 시 확인)
port = 8080                  # ESP32에서 설정한 포트 번호

# 소켓 생성 (IPv4, TCP)
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 서버에 연결
try:
    client_socket.connect((server_ip, port))
    print(f"서버 {server_ip}:{port}에 연결 성공")
except Exception as e:
    print(f"서버에 연결 실패: {e}")
    exit()
```
- `server_ip`, `port` → **ESP32의 IP 주소 및 TCP 포트 설정**.
- `client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)` → **TCP 소켓 생성**.
- `client_socket.connect((server_ip, port))` → **ESP32 서버에 연결 시도**.
- 연결 실패 시 프로그램 종료.

---

### 2. `pygame`을 이용한 키 입력 감지
```python
# Pygame 초기화
pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("ESP32 통신")
```
- `pygame.init()` → **Pygame 초기화**.
- `screen = pygame.display.set_mode((400, 300))` → **GUI 창 생성 (실제 기능은 키 입력 감지)**.

```python
def handle_keypress():
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        print("forward")
        return "w\n"
    elif keys[pygame.K_s]:
        print("backward")
        return "s\n"
    elif keys[pygame.K_a]:
        print("left")
        return "a\n"
    elif keys[pygame.K_d]:
        print("right")
        return "d\n"
    elif keys[pygame.K_SPACE]:
        print("stop")
        return " \n"
    return None
```
- `pygame.key.get_pressed()` → **현재 눌린 키를 감지**.
- `w, a, s, d, space` 키 입력 시 **해당 명령을 문자열로 반환**.

---

### 3. 키 입력 처리 및 ESP32로 명령 전송
```python
try:
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
       
        # 키 상태 확인 및 처리
        message = handle_keypress()
        if message:
            # 메시지 전송
            client_socket.send(message.encode())
            print(f"전송: {message.strip()}")

            # 서버로부터의 응답 수신
            response = client_socket.recv(1024)
            print(f"서버 응답: {response.decode()}")
```
- `pygame.event.get()` → **Pygame 창이 종료되었는지 확인**.
- `handle_keypress()` → **키 입력을 감지하고 명령 생성**.
- `client_socket.send(message.encode())` → **ESP32로 명령 전송**.
- `client_socket.recv(1024)` → **ESP32의 응답을 수신 후 출력**.

---

### 4. 프로그램 종료 시 소켓 닫기
```python
finally:
    # 소켓 종료 및 pygame 종료
    client_socket.close()
    pygame.quit()
```
- `client_socket.close()` → **TCP 소켓 종료**.
- `pygame.quit()` → **Pygame 종료**.

---

## 112_arduino_ROS_motor_remote_ctrl

### 📌 코드의 핵심 기능 요약
이 코드는 **Arduino Uno와 ESP32** 간의 **소프트웨어 시리얼 통신**을 통해 **모터를 원격 제어**하는 기능을 수행합니다. 수신된 데이터에 따라 모터의 방향과 속도를 제어할 수 있습니다.

---

### 1. 하드웨어 연결

| Arduino Uno | ESP32 Devkit V1 |
|-------------|---------------|
| GND         | GND           |
| 7           | TX2(17)       |
| 8           | RX2(16)       |

---

### 2. 코드 설명

#### 📌 라이브러리 포함 및 소프트웨어 시리얼 설정
```cpp
#include <SoftwareSerial.h>
SoftwareSerial mySerial(7, 8); // RX, TX
```
ESP32와의 시리얼 통신을 위해 **소프트웨어 시리얼**을 설정합니다.

#### 📌 모터 핀 설정
```cpp
#define motor_A_enable 12
#define motor_B_enable 13
#define motor_A 10
#define motor_B 11
```
모터 드라이버 제어를 위한 핀을 설정합니다.

#### 📌 모터 제어 함수 정의
각 함수는 모터의 회전 방향과 속도를 조절합니다.
```cpp
void backward(int R, int L);
void forward(int R, int L);
void turnLeft(int R, int L);
void turnRight(int R, int L);
void stopAll();
```
- `forward(R, L)`: 전진
- `backward(R, L)`: 후진
- `turnLeft(R, L)`: 좌회전
- `turnRight(R, L)`: 우회전
- `stopAll()`: 정지

각 함수에서 **PWM(Pulse Width Modulation)**을 사용하여 모터의 속도를 조절합니다.

#### 📌 `setup()` 함수: 초기 설정
```cpp
void setup() {
  Serial.begin(115200);
  mySerial.begin(9600);
  Serial.println("Software Serial에서 데이터 수신 준비 완료");

  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  pinMode(motor_A_enable, OUTPUT);
  pinMode(motor_B_enable, OUTPUT);
}
```
- **소프트웨어 시리얼 통신 속도**: `9600 baud` 설정
- **모터 드라이버 핀 모드**: 출력(OUTPUT)으로 설정

#### 📌 `loop()` 함수: 수신 데이터 처리 및 모터 제어
```cpp
void loop() {
  if (mySerial.available()) {
    String inString = mySerial.readStringUntil('\n');
    motor_r = inString.substring(inString.indexOf('a') + 1, inString.indexOf('b')).toInt();
    motor_l = inString.substring(inString.indexOf('b') + 1, inString.indexOf('c')).toInt();

    Serial.print(motor_r);
    Serial.print("  ");
    Serial.println(motor_l);
    forward(motor_r, motor_l);
  }
}
```
- **소프트웨어 시리얼로 데이터 수신**
- **문자열 파싱하여 모터 속도 값 추출**
- **추출한 속도로 모터 전진 실행**

---

### 3. 실행 방식
1. **ESP32에서 데이터를 전송** (예: "a100b100c\n")
2. **Arduino가 데이터를 수신하여 파싱**
3. **모터를 해당 속도로 전진**
4. **시리얼 모니터에서 속도 값 확인 가능**

---

### 4. 참고 사항
- 모터 드라이버가 필요하며, **L298N 모터 드라이버**와 함께 사용할 수 있습니다.
- `analogWrite()`를 사용하여 **모터 속도를 PWM 신호로 조절**합니다.
- 데이터 포맷은 **"a[속도값]b[속도값]c"** 형태로 전송해야 합니다.

---

## balancing_L298P

### 📌 코드의 핵심 기능 요약
이 코드는 **MPU6050 센서를 사용하여** 로봇의 기울기를 측정하고, **PID 제어 알고리즘**을 통해 **자동으로 균형을 유지하도록** 모터를 제어하는 프로그램입니다. L298P 모터 드라이버를 사용하여 전진, 후진, 정지 동작을 수행합니다.

---

### 1. 하드웨어 연결

| 모듈 | 연결 핀 |
|------|--------|
| MPU6050 | I2C (SDA, SCL) |
| 모터 A 제어 핀 | 10, 12 |
| 모터 B 제어 핀 | 11, 13 |

---

### 2. 코드 설명

#### 📌 라이브러리 포함 및 센서 초기화
```cpp
#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
```
- **MPU6050** 센서를 사용하기 위한 라이브러리 포함
- **PID_v1.h** 라이브러리를 사용하여 PID 제어 적용

#### 📌 MPU6050 센서 및 PID 설정
```cpp
MPU6050 mpu;
double setpoint = 180;
double Kp = 10, Ki = 95, Kd = 0.99;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
```
- **MPU6050 센서 객체** 생성
- **PID 제어 변수** 및 초기 파라미터 설정

#### 📌 모터 핀 설정
```cpp
#define motor_A_enable 10
#define motor_B_enable 11
#define motor_A 12
#define motor_B 13
```
- 모터 드라이버의 핀 번호를 설정합니다.

#### 📌 `setup()` 함수: 초기 설정
```cpp
void setup() {
  Serial.begin(115200);
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  
  if (devStatus == 0) {
      mpu.setDMPEnabled(true);
      attachInterrupt(0, dmpDataReady, RISING);
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
      
      pid.SetMode(AUTOMATIC);
      pid.SetSampleTime(10);
      pid.SetOutputLimits(-255, 255);
  }

  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
}
```
- MPU6050을 **초기화 및 DMP 활성화**
- **PID 컨트롤러 설정** 및 **출력 한계 (-255 ~ 255) 설정**
- 모터 핀을 출력 모드로 설정

#### 📌 `loop()` 함수: 균형 유지 및 모터 제어
```cpp
void loop() {
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {
        pid.Compute();   
        Serial.print(input); Serial.print(" =>"); Serial.println(output);
                
        if (input > 150 && input < 200) {
            if (output > 0)
                Forward(output, output);
            else if (output < 0)
                Reverse(-output, -output);
        } else {
            Stop();
        }
    }
    
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        input = ypr[2] * 180/M_PI + 180;
    }
}
```
- **DMP 데이터를 읽고 FIFO 버퍼 관리**
- **로봇의 기울기에 따라 모터 제어**
  - 전진(`Forward`)
  - 후진(`Reverse`)
  - 정지(`Stop`)
- **FIFO 오버플로우 방지**

#### 📌 모터 제어 함수
```cpp
void Forward(int R, int L) {
  analogWrite(motor_A_enable, L);
  analogWrite(motor_B_enable, R);
  digitalWrite(motor_A, HIGH);
  digitalWrite(motor_B, HIGH);
  Serial.print("F");
}

void Reverse(int R, int L) {
  analogWrite(motor_A_enable, L);
  analogWrite(motor_B_enable, R);
  digitalWrite(motor_A, LOW);
  digitalWrite(motor_B, LOW);
  Serial.print("R");
}

void Stop() {
  analogWrite(motor_A_enable, 0);
  analogWrite(motor_B_enable, 0);
  digitalWrite(motor_A, HIGH);
  digitalWrite(motor_B, HIGH);
  Serial.print("S");
}
```
- **전진(`Forward`)**: 모터를 전진 방향으로 회전
- **후진(`Reverse`)**: 모터를 후진 방향으로 회전
- **정지(`Stop`)**: 모터 정지

---

### 3. 실행 방식
1. **MPU6050 센서가 로봇의 기울기를 측정**
2. **PID 컨트롤러가 기울기에 따라 출력값 계산**
3. **로봇이 기울어지면 모터가 움직여 균형 유지**
4. **기울기 범위를 벗어나면 모터가 정지**
5. **시리얼 모니터에서 현재 기울기 및 출력값 확인 가능**

---

### 4. 참고 사항
- `setpoint` 값(180도)은 **로봇이 수직 상태를 유지하도록 설정된 기준점**입니다.
- `Kp`, `Ki`, `Kd` 값은 **PID 튜닝을 통해 조정 가능**합니다.
- FIFO 버퍼가 오버플로우하지 않도록 **데이터를 지속적으로 읽어야 합니다.**


---

## line_tracer

### 📌 코드의 핵심 기능 요약
이 코드는 **적외선 센서(IR 센서)를 이용하여** 선을 감지하고, 이를 기반으로 **라인 트레이싱(Line Tracing) 로봇**을 제어하는 프로그램입니다. 두 개의 IR 센서를 사용하여 로봇이 검은 선을 따라 이동할 수 있도록 모터를 조작합니다.

---

### 1. 하드웨어 연결

| 모듈 | 연결 핀 |
|------|--------|
| IR 센서 (오른쪽) | D6 |
| IR 센서 (왼쪽) | D5 |
| 모터 A 제어 핀 | D10, D12 |
| 모터 B 제어 핀 | D11, D13 |

---

### 2. 코드 설명

#### 📌 핀 설정
```cpp
#define ir_R 6
#define ir_L 5

#define motor_A 10
#define motor_B 11

#define motor_A_enable 12
#define motor_B_enable 13
```
- **IR 센서**는 **D6, D5**에 연결
- **모터 A, B**는 각각 D10, D11에서 PWM 신호로 제어됨
- **모터 A, B의 Enable 핀**은 D12, D13에서 ON/OFF 제어됨

#### 📌 모터 제어 함수 정의
```cpp
void forward(int R, int L);
void backward(int R, int L);
void turnLeft(int R, int L);
void turnRight(int R, int L);
void stopAll();
```
- **`forward(R, L)`**: 전진
- **`backward(R, L)`**: 후진
- **`turnLeft(R, L)`**: 좌회전
- **`turnRight(R, L)`**: 우회전
- **`stopAll()`**: 정지

각 함수에서 **PWM 신호를 사용하여 속도를 조절**합니다.

#### 📌 `setup()` 함수: 초기 설정
```cpp
void setup() {
  Serial.begin(115200);
  
  pinMode(ir_R, INPUT);
  pinMode(ir_L, INPUT);

  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  pinMode(motor_A_enable, OUTPUT);
  pinMode(motor_B_enable, OUTPUT);
}
```
- **IR 센서를 입력 모드로 설정**
- **모터 핀을 출력 모드로 설정**
- **시리얼 통신 속도를 115200 bps로 설정**

#### 📌 `loop()` 함수: 라인 트레이싱 로직
```cpp
void loop() {
  int val_R = digitalRead(ir_R);
  int val_L = digitalRead(ir_L);
```
- **IR 센서 값 읽기**
- `LOW` = 검은 선 감지, `HIGH` = 흰색 바닥 감지

```cpp
  if (val_R == LOW && val_L == LOW) {
    forward(200, 200);
    Serial.println("F");
  }
```
- **양쪽 센서가 검은 선을 감지하면 전진**

```cpp
  else if (val_R == HIGH && val_L == HIGH) {
    stopAll();
  }
```
- **양쪽 센서가 흰색을 감지하면 정지**

```cpp
  else if (val_L == HIGH) {
    turnLeft(150, 150);
    Serial.println("L");
    delay(170);
  }
```
- **왼쪽 센서만 흰색을 감지하면 좌회전**

```cpp
  else if (val_R == HIGH) {
    turnRight(150, 150);
    Serial.println("R");
    delay(170);
  }
```
- **오른쪽 센서만 흰색을 감지하면 우회전**

---

### 3. 실행 방식
1. **IR 센서가 검은 선을 감지**하면 **로봇이 전진**
2. **한쪽 센서가 흰색을 감지**하면 **반대쪽 방향으로 회전**
3. **양쪽 센서가 흰색을 감지**하면 **정지**
4. **시리얼 모니터에서 현재 이동 방향 확인 가능**

---

### 4. 참고 사항
- IR 센서의 감도는 **환경에 따라 다를 수 있으므로 조정 필요**
- `delay(170)` 값은 **로봇의 반응 속도에 따라 변경 가능**
- `analogWrite()`를 통해 **속도를 세밀하게 조정 가능**


