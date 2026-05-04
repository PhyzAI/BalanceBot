#include <Adafruit_TinyUSB.h> // <--- ADD THIS LINE TO FIX COMPILER ERROR

// Motor 1 Pins
const int AIN1 = D8, AIN2 = D9, PWMA = D7, C1_A = D3, C2_A = D2;
// Motor 2 Pins
// const int BIN1 = A0, BIN2 = A1, PWMB = D10, C1_B = D4, C2_B = D5;
const int BIN1 = A1, BIN2 = A0, PWMB = D10, C1_B = D4, C2_B = D5;
const int STBY = D6;

volatile long posA = 0, posB = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);

  pinMode(C1_A, INPUT_PULLUP); pinMode(C2_A, INPUT_PULLUP);
  pinMode(C1_B, INPUT_PULLUP); pinMode(C2_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(C1_A), []{ posA += (digitalRead(C1_A) == digitalRead(C2_A)) ? 1 : -1; }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C1_B), []{ posB += (digitalRead(C1_B) == digitalRead(C2_B)) ? 1 : -1; }, CHANGE);
}

void setMotor(int speed, int in1, int in2, int pwmPin) {
  analogWrite(pwmPin, abs(speed));
  digitalWrite(in1, speed > 0);
  digitalWrite(in2, speed < 0);
}

void loop() {
  setMotor(150, AIN1, AIN2, PWMA);
  setMotor(150, BIN1, BIN2, PWMB);
  
  unsigned long start = millis();
  while(millis() - start < 2000) {
    Serial.print("M1: "); Serial.print(posA);
    Serial.print(" | M2: "); Serial.println(posB);
    delay(100);
  }

  setMotor(0, AIN1, AIN2, PWMA);
  setMotor(0, BIN1, BIN2, PWMB);
  delay(1000);
}
