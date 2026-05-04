#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <Wire.h>

bool DEBUG = false;

// IMU Initialization for Seeed XIAO Sense
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// --- LIGHTWEIGHT KALMAN FILTER ---
// This replaces the external Kalman.h library to avoid "BasicLinearAlgebra" errors.
struct KalmanFilter {
    float Q_angle = 0.001f;
    float Q_bias = 0.003f;
    float R_measure = 0.03f;
    float angle = 0.0f;
    float bias = 0.0f;
    float P[2][2] = {{0,0},{0,0}};

    float getAngle(float newAngle, float newRate, float dt) {
        float rate = newRate - bias;
        angle += dt * rate;
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;
        float S = P[0][0] + R_measure;
        float K[2] = {P[0][0] / S, P[1][0] / S};
        float y = newAngle - angle;
        angle += K[0] * y;
        bias  += K[1] * y;
        float P00_temp = P[0][0];
        float P01_temp = P[0][1];
        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;
        return angle;
    };
    void setAngle(float a) { angle = a; };
};

KalmanFilter kalman; 

// --- PIN DEFINITIONS ---
const int mAIN1 = D8, mAIN2 = D9, PWMA = D7;
const int mBIN1 = A1, mBIN2 = A0, PWMB = D10;
const int STBY = D6;

// --- BLE UUIDs ---
BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic rxChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite | BLEWriteWithoutResponse, 20);
BLEStringCharacteristic txChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 20);

// --- PID & CONTROL VARIABLES ---
float Kp = 0.0, Kd = 0.0, Ki = 0.0;
float angle = 0.0, pidError, lastError, integral;
unsigned long lastTime;
float targetAngle = 2.2; // Your physical balance point

// --- MOTOR DEADZONE SETTINGS ---
// Update these after running the calibration sketch!
int deadzoneA = 40; 
int deadzoneB = 40;

void setup() {
  Serial.begin(115200);
  
  pinMode(LED_RED, OUTPUT); pinMode(LED_GREEN, OUTPUT); pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, HIGH); digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_BLUE, HIGH);

  delay(1000); // Stabilization delay

  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(mAIN1, OUTPUT); pinMode(mAIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(mBIN1, OUTPUT); pinMode(mBIN2, OUTPUT); pinMode(PWMB, OUTPUT);

  if (myIMU.begin() != 0) {
    digitalWrite(LED_RED, LOW); // Red LED on error
  }

  if (BLE.begin()) {
    BLE.setLocalName("BalanceBot_XIAO");
    BLE.setAdvertisedService(uartService);
    uartService.addCharacteristic(rxChar);
    uartService.addCharacteristic(txChar);
    BLE.addService(uartService);
    BLE.advertise();
    digitalWrite(LED_BLUE, LOW); 
  }

  // Seed Kalman with starting position
  float ax = myIMU.readFloatAccelX();
  float az = myIMU.readFloatAccelZ();
  float initialAngle = atan2(ax, az) * 180 / PI;
  kalman.setAngle(initialAngle);

  lastTime = micros();
}

void driveMotors(int speed) {
  if (speed == 0) {
    analogWrite(PWMA, 0); analogWrite(PWMB, 0);
    return;
  }

  int absSpeed = abs(speed);
  
  // Apply Deadzone Compensation
  int speedA = map(absSpeed, 0, 255, deadzoneA, 255);
  int speedB = map(absSpeed, 0, 255, deadzoneB, 255);

  analogWrite(PWMA, speedA);
  analogWrite(PWMB, speedB);

  // Direction logic
  digitalWrite(mAIN1, speed < 0);
  digitalWrite(mAIN2, speed > 0);
  digitalWrite(mBIN1, speed < 0);
  digitalWrite(mBIN2, speed > 0);
}

void loop() {
  BLEDevice central = BLE.central();
  
  if (central.connected()) {
    digitalWrite(LED_BLUE, HIGH); digitalWrite(LED_GREEN, LOW);
  } else {
    digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_BLUE, LOW);
  }

  // Timing
  unsigned long now = micros();
  float dt = (float)(now - lastTime) / 1000000.0;
  if (dt <= 0 || dt > 0.1) dt = 0.01;
  lastTime = now;

  // Sensor Data
  float ax = myIMU.readFloatAccelX();
  float az = myIMU.readFloatAccelZ();
  float gy = myIMU.readFloatGyroY();
  
  // Kalman Filter Sensor Fusion
  float accAngle = atan2(ax, az) * 180 / PI;
  angle = kalman.getAngle(accAngle, gy, dt);

  // PID Math
  pidError = targetAngle - angle;
  integral = constrain(integral + (pidError * dt), -100, 100);
  float derivative = (pidError - lastError) / dt;
  lastError = pidError;
  
  float output = (Kp * pidError) + (Ki * integral) + (Kd * derivative);
  int motorSpeed = constrain((int)output, -255, 255);
  
  // Safety Reset
  if (abs(angle) > 45.0) {
    motorSpeed = 0;
    integral = 0;
  }

  driveMotors(motorSpeed);

  // Bluetooth & Serial Diagnostics
  static unsigned long lastPrint = 0;
  static bool flip = false;
  if (millis() - lastPrint > 250) {
    lastPrint = millis();
    char buffer[40];
    if (flip) {
      if (DEBUG) {snprintf(buffer, sizeof(buffer), "A:%.1f TRG:%.1f",      angle,     
         targetAngle);}
    } else {
      if (DEBUG) {snprintf(buffer, sizeof(buffer), "P:%.1f I:%.1f D:%.2f", Kp, 
         Ki, Kd);}
    }
    if (central.connected()) txChar.writeValue(buffer);
    Serial.println(buffer);
    flip = !flip;
  }

  // BLE Tuning Commands
  if (rxChar.written()) {
    const uint8_t* data = rxChar.value();
    char cmd = (char)data[0];

    if (cmd == 'p') Kp += 0.25; if (cmd == 'P') Kp += 1.0;
    if (cmd == 'q') Kp -= 0.25; if (cmd == 'Q') Kp -= 1.0;
    if (cmd == 'd') Kd += 0.05; if (cmd == 'D') Kd += 0.2;
    if (cmd == 'e') Kd -= 0.05; if (cmd == 'E') Kd -= 0.2;
    if (cmd == 'i') Ki += 0.1;  if (cmd == 'I') Ki += 1.0;
    if (cmd == 'j') Ki -= 0.1;  if (cmd == 'J') Ki -= 1.0;
    if (cmd == 'a' || cmd == '[') targetAngle -= 0.1;
    if (cmd == 'A' || cmd == ']') targetAngle += 0.1;
    if (cmd == 's') { Kp = 0; Kd = 0; Ki = 0; integral = 0; }

    if (Kp < 0) Kp = 0; if (Kd < 0) Kd = 0; if (Ki < 0) Ki = 0;
  }
}