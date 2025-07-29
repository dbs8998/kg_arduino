#define ir_R 7
#define ir_L 8

#define motor_A 10
#define motor_B 11

#define motor_A_enable 12
#define motor_B_enable 13

int moving_direction = 0;
int moving_speed = 0;
int delta_R = 0;
int delta_L = 0;
int speed_R = 100;
int speed_L = 100;

void backward(int R, int L) {
  analogWrite(motor_A, L);
  analogWrite(motor_B, R);
  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, HIGH);
}

void forward(int R, int L) {
  analogWrite(motor_A, L);
  analogWrite(motor_B, R);
  digitalWrite(motor_A_enable, LOW);
  digitalWrite(motor_B_enable, LOW);
}

void turnLeft(int R, int L) {
  analogWrite(motor_A, L);
  analogWrite(motor_B, R);
  digitalWrite(motor_A_enable, LOW);
  digitalWrite(motor_B_enable, HIGH);
}

void turnRight(int R, int L) {
  analogWrite(motor_A, L);
  analogWrite(motor_B, R);
  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, LOW);
}

void stopAll() {
  analogWrite(motor_A, 0);
  analogWrite(motor_B, 0);
  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, HIGH);
}

void setup() {
  Serial.begin(115200);
  
  pinMode(ir_R,INPUT);
  pinMode(ir_L,INPUT);

  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  pinMode(motor_A_enable, OUTPUT);
  pinMode(motor_B_enable, OUTPUT);
}

void loop() {
  int val_R = digitalRead(ir_R);
  int val_L = digitalRead(ir_L);

  if (val_R == LOW && val_L == LOW) {
    forward(200,200);
    Serial.println("F");

  } else if (val_R == HIGH && val_L == HIGH) {
    stopAll();

  } else if (val_L == HIGH) {
    turnLeft(150,150);
    Serial.println("L");
    delay(170);

  }  else if (val_R == HIGH) {
    turnRight(150,150);
    Serial.println("R");
    delay(170);
  }
}
