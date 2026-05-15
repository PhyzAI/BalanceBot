#include <Arduino.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <ArduinoBLE.h> 

LSM6DS3 myIMU(I2C_MODE, 0x6A);

// --- TUNING PARAMETERS ---
float Kp = 8.0;
float Kd = 0.0;
float Ki = 0.0;
float targetAngle = -4.5;
int maxMotorSpeed = 255;  
int minMotorSpeed = 35;

float alpha = 0.95;        
float motorA_Scale = 1.0;
float motorB_Scale = 0.94;

// --- BLE GLOBALS ---
BLEService tuningService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEStringCharacteristic tuningChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite, 20);

// --- PIN DEFINITIONS ---
const int mAIN1 = D8, mAIN2 = D9, PWMA = D7;
const int mBIN1 = A1, mBIN2 = A0, PWMB = D10;
const int STBY = D6;

float angle = 0.0;
float lastError = 0.0;
float integral = 0.0;
unsigned long nextLoopTime = 0;
const unsigned long LOOP_PERIOD = 10000;

const bool ENABLE_NONLINEAR = false;
const float nonlinear_threshold = 6.0;
const int max_motor_nonlinear = 235;

// --- FORWARD DECLARATIONS ---
void setupBLE();
void handleBLE();

void driveMotors(int speed) {
  if (abs(speed) < 2) {
    analogWrite(PWMA, 0); 
    analogWrite(PWMB, 0);
    return;
  }

  int finalOutput;
  int absSpeed = abs(speed);
  
  finalOutput = map(absSpeed, 0, 255, minMotorSpeed, maxMotorSpeed);
  
  if ( ENABLE_NONLINEAR && (absSpeed > (Kp * nonlinear_threshold)) ) {
    finalOutput = max_motor_nonlinear;
  }

  finalOutput = constrain(finalOutput, 0, 255);

  // Motor A and B logic with scaling for the L/R swap
  analogWrite(PWMA, finalOutput);
  analogWrite(PWMB, (int)(finalOutput * motorB_Scale));
  
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
  
  float accX = myIMU.readFloatAccelX();
  float accY = myIMU.readFloatAccelY();
  float accZ = myIMU.readFloatAccelZ();
  angle = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  
  setupBLE(); 
  nextLoopTime = micros();
}

void loop() {
  handleBLE(); 

  if (micros() >= nextLoopTime) {
    nextLoopTime += LOOP_PERIOD;

    float accX = myIMU.readFloatAccelX();
    float accY = myIMU.readFloatAccelY();
    float accZ = myIMU.readFloatAccelZ();
    float gyroY = myIMU.readFloatGyroY();

    const float dt = 0.01;
    float pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    angle = (alpha * (angle + (gyroY * dt))) + ((1.0 - alpha) * pitch);

    float error = targetAngle - angle;
    
    integral = constrain(integral + (error * dt), -20, 20);
    float derivative = (error - lastError) / dt;
    lastError = error;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    if (abs(angle) > 45.0) {
      driveMotors(0);
    } else {
      driveMotors((int)output);
    }
    
    static int pCount = 0;
    if (pCount++ > 20) {
       Serial.print("Ang:"); Serial.print(angle);
       Serial.print(" Err:"); Serial.print(error);
       Serial.print(" Kp:"); Serial.print(Kp);
       Serial.print(" Ki:"); Serial.print(Ki);
       Serial.print(" Kd:"); Serial.println(Kd);
       pCount = 0;
    }
  }
}

// --- SEPARATE BLE FUNCTIONS ---

void setupBLE() {
  if (!BLE.begin()) {
    Serial.println("BLE Failed!");
    return;
  }
  BLE.setLocalName("BalanceBot_Tuner");
  BLE.setAdvertisedService(tuningService);
  tuningService.addCharacteristic(tuningChar);
  BLE.addService(tuningService);
  tuningChar.writeValue("p8.0 i0.0 d0.0");
  BLE.advertise();
}

void handleBLE() {
  BLEDevice central = BLE.central();
  if (central && tuningChar.written()) {
    String val = tuningChar.value();
    char type = val.charAt(0);
    float num = val.substring(1).toFloat();
    
    if (type == 'p') Kp = num;
    else if (type == 'i') Ki = num;
    else if (type == 'd') Kd = num;
    
    Serial.print("Updated: "); Serial.print(type); Serial.println(num);
  }
}