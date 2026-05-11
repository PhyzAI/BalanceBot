#include <Arduino.h>
#include <LSM6DS3.h>
#include <Wire.h>

LSM6DS3 myIMU(I2C_MODE, 0x6A);

// --- TUNING PARAMETERS ---
float Kp = 16.0;           // Bumped up slightly for a firmer response
float Kd = 1.0;            // Added a bit of D to fight the "runaway" momentum
float Ki = 0.05;
float targetAngle = -6.8;  // CORRECTED SIGN
int maxMotorSpeed = 255;  
int minMotorSpeed = 45;    // Increased slightly to ensure immediate movement

float alpha = 0.95;        
float motorA_Scale = 0.9; 
// -------------------------

const int mAIN1 = D8, mAIN2 = D9, PWMA = D7;
const int mBIN1 = A1, mBIN2 = A0, PWMB = D10;
const int STBY = D6;

float angle = 0.0;
float lastError = 0.0;
float integral = 0.0;
unsigned long nextLoopTime = 0;
const unsigned long LOOP_PERIOD = 10000; 

void driveMotors(int speed) {
  if (abs(speed) < 2) { 
    analogWrite(PWMA, 0); analogWrite(PWMB, 0);
    return;
  }

  int finalOutput;
  int absSpeed = abs(speed);
  
  // 1. Minimum Kick: Ensure we move immediately
  finalOutput = map(absSpeed, 0, 255, minMotorSpeed, maxMotorSpeed);
  
  // 2. Panic Threshold: If tilt error > 6 degrees, go to 90% power
  if (absSpeed > (Kp * 6.0)) { 
    finalOutput = 235; 
  }

  finalOutput = constrain(finalOutput, 0, 255);

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
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(mAIN1, OUTPUT); pinMode(mAIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(mBIN1, OUTPUT); pinMode(mBIN2, OUTPUT); pinMode(PWMB, OUTPUT);

  myIMU.begin();
  
  // Initial angle catch
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

    angle = (alpha * (angle + (gyroY * dt))) + ((1.0 - alpha) * pitch);

    // PID Calculation
    float error = targetAngle - angle;
    error = -error; // Logic flip
    
    integral = constrain(integral + (error * dt), -20, 20);

    float derivative = (error - lastError) / dt;
    lastError = error;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);


    if (abs(angle) > 45.0) {
      driveMotors(0);
    } else {
      driveMotors((int)output);
    }
    
    // Debugging output
    static int pCount = 0;
    if (pCount++ > 20) {
       Serial.print("Ang:"); Serial.print(angle);
       Serial.print(" Err:"); Serial.println(error);
       pCount = 0;
    }
  }
}