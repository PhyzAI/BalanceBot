#include <Arduino.h>
#include <LSM6DS3.h>
#include <Wire.h>

// IMU Initialization
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// Pin Definitions
const int mAIN1 = D8, mAIN2 = D9, PWMA = D7;
const int mBIN1 = A1, mBIN2 = A0, PWMB = D10;
const int STBY = D6;

// --- CALIBRATION & CORRECTION ---
float motorA_Scale = 0.94; // Your discovered magic number
float yawCorrection = 0.0;
float Kp_yaw = 2.5;       // Sensitivity of the "straight line" correction
float currentHeading = 0.0;

// Timing
unsigned long lastUpdate = 0;

void driveMotors(int baseSpeed, float correction) {
  if (baseSpeed == 0) {
    analogWrite(PWMA, 0); analogWrite(PWMB, 0);
    return;
  }

  // Adjust left/right speeds based on yaw error
  // If twisting right, Speed A (Left) decreases, Speed B (Right) increases
  int speedA = abs(baseSpeed) * motorA_Scale - correction;
  int speedB = abs(baseSpeed) + correction;

  analogWrite(PWMA, constrain(speedA, 0, 255));
  analogWrite(PWMB, constrain(speedB, 0, 255));
  
  digitalWrite(mAIN1, baseSpeed > 0);
  digitalWrite(mAIN2, baseSpeed < 0);
  digitalWrite(mBIN1, baseSpeed > 0);
  digitalWrite(mBIN2, baseSpeed < 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(mAIN1, OUTPUT); pinMode(mAIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(mBIN1, OUTPUT); pinMode(mBIN2, OUTPUT); pinMode(PWMB, OUTPUT);

  if (myIMU.begin() != 0) Serial.println("IMU Error");
  
  Serial.println("Straight-Line IMU Test Starting in 3s...");
  delay(3000);
  lastUpdate = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastUpdate) / 1000000.0;
  lastUpdate = now;

  // 1. Read the Gyro Z-axis (Yaw rate in degrees/sec)
  float gyroZ = myIMU.readFloatGyroZ();
  
  // 2. Integrate gyroZ to find the current heading
  // We want currentHeading to stay at 0.0
  currentHeading += gyroZ * dt;

  // 3. Calculate correction
  // If Heading > 0, we've turned one way; if < 0, the other.
  yawCorrection = currentHeading * Kp_yaw;

  // 4. Run test cycle (Forward for 2s, Stop for 2s)
  static unsigned long cycleTimer = millis();
  if (millis() - cycleTimer < 2000) {
    driveMotors(100, yawCorrection);
  } else if (millis() - cycleTimer < 4000) {
    driveMotors(0, 0);
    currentHeading = 0; // Reset heading while stopped
  } else {
    cycleTimer = millis();
  }

  // Debugging
  static unsigned long printTimer = 0;
  if (millis() - printTimer > 100) {
    Serial.print("Heading: "); Serial.print(currentHeading);
    Serial.print(" | Correction: "); Serial.println(yawCorrection);
    printTimer = millis();
  }
}