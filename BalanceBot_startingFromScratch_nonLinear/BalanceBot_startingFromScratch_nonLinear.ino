#include <Arduino.h>
#include <LSM6DS3.h>
#include <Wire.h>

// IMU Initialization
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// --- TUNING PARAMETERS ---
float Kp = 12.5;           
float targetAngle = -5.9;   
int maxMotorSpeed = 250;  
int minMotorSpeed = 40;    // Minimum PWM to overcome static friction
float maxLinearError = 5.0; 

float alpha = 0.95;        
float motorA_Scale = 0.94; 
// -------------------------

// Pin Definitions
const int mAIN1 = D8, mAIN2 = D9, PWMA = D7;
const int mBIN1 = A1, mBIN2 = A0, PWMB = D10;
const int STBY = D6;

// Logic Variables
float angle = 0.0;
float lastError = 0.0;
unsigned long nextLoopTime = 0;
const unsigned long LOOP_PERIOD = 10000; // 10ms (100Hz)



void driveMotors(int speed) {
  if (speed == 0) {
    analogWrite(PWMA, 0); 
    analogWrite(PWMB, 0);
    return;
  }

  int limitedSpeed = constrain(speed, -maxMotorSpeed, maxMotorSpeed);
  int absSpeed = abs(limitedSpeed);

  analogWrite(PWMA, (int)(absSpeed * motorA_Scale));
  analogWrite(PWMB, absSpeed);
  
  digitalWrite(mAIN1, limitedSpeed < 0);
  digitalWrite(mAIN2, limitedSpeed > 0);
  digitalWrite(mBIN1, limitedSpeed < 0);
  digitalWrite(mBIN2, limitedSpeed > 0);
}


void driveMotors_old(int speed) {
  if (abs(speed) < 5) { 
    analogWrite(PWMA, 0); 
    analogWrite(PWMB, 0);
    return;
  }

  int finalOutput;
  int absSpeed = abs(speed);
  
  // 1. Apply the "Kick" (Deadzone compensation)
  // Maps linear PID output to the motor's actual working range
  finalOutput = map(absSpeed, 0, 255, minMotorSpeed, maxMotorSpeed);
  
  // 2. The "Slam" Logic (Non-linear recovery)
  // If error is > 8 degrees, jump to near-maximum power
  if (absSpeed > (Kp * 8.0)) { 
    finalOutput = 230; 
  }

  finalOutput = constrain(finalOutput, 0, 255);

  // Apply scaling and write PWM
  analogWrite(PWMA, (int)(finalOutput * motorA_Scale));
  analogWrite(PWMB, finalOutput);
  
  // Direction logic
  digitalWrite(mAIN1, speed < 0);
  digitalWrite(mAIN2, speed > 0);
  digitalWrite(mBIN1, speed < 0);
  digitalWrite(mBIN2, speed > 0);
}

void setup() {
  Serial.begin(115200);
  
  pinMode(STBY, OUTPUT); 
  digitalWrite(STBY, HIGH);
  
  pinMode(mAIN1, OUTPUT); pinMode(mAIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(mBIN1, OUTPUT); pinMode(mBIN2, OUTPUT); pinMode(PWMB, OUTPUT);

  if (myIMU.begin() != 0) {
    Serial.println("IMU Error");
  }

  // Initial angle setup
  float accX = myIMU.readFloatAccelX();
  float accY = myIMU.readFloatAccelY();
  float accZ = myIMU.readFloatAccelZ();
  angle = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  
  nextLoopTime = micros();
}

void loop() {
  if (micros() >= nextLoopTime) {
    nextLoopTime += LOOP_PERIOD;

    float accX = myIMU.readFloatAccelX();
    float accY = myIMU.readFloatAccelY();
    float accZ = myIMU.readFloatAccelZ();
    float gyroY = myIMU.readFloatGyroY();

    const float dt = 0.01; 
    float pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

    // Complementary Filter
    angle = (alpha * (angle + (gyroY * dt))) + ((1.0 - alpha) * pitch);

    // PID Calculation
    float error = targetAngle - angle;
    error = -error; // Logic flip for motor orientation
    
    float derivative = (error - lastError) / dt;
    lastError = error;

    // Proportional + Derivative Control (Simplified for this test)
    float output = (Kp * error) + (0.5 * derivative);

    if (abs(angle) > 45.0) {
      driveMotors(0);
    } else if (error > maxLinearError) {
      driveMotors((int)maxMotorSpeed);
    } else if (error < -maxLinearError) {
      driveMotors(-(int)maxMotorSpeed);
    } else {
      driveMotors((int)output);
    }
  }
}