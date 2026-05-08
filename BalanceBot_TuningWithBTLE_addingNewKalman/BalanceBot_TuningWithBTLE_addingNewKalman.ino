#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <Kalman.h>

// IMU Initialization
LSM6DS3 myIMU(I2C_MODE, 0x6A);
Kalman kalmanX;
Kalman kalmanY;


float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
uint32_t timer;

// Pin Definitions
const int mAIN1 = D8, mAIN2 = D9, PWMA = D7;
const int mBIN1 = A1, mBIN2 = A0, PWMB = D10;
const int STBY = D6;

// Encoder Pins
const int enc1_A = D2, enc1_B = D3;
const int enc2_A = D4, enc2_B = D5;

// BLE Setup
BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic rxChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite | BLEWriteWithoutResponse, 20);
BLEStringCharacteristic txChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 128); // was 20

// Control Variables
float Kp = 0.0, Kd = 0.0, Ki = 0.0;  // 50, 1.2, 0  (closest I've gotten)
float angle = 0.0, pidError, lastError, integral;
float targetAngle = 5.9; 
float driftCorrection = 0.0; 
const float MAX_DRIFT_ADJUST_P = 0.0; //0.5;
const float MAX_DRIFT_ADJUST_M = 0.0; //-0.5;

// Encoder Variables
volatile long leftEncoderPos = 0; 
volatile long rightEncoderPos = 0;

// ISR for Left Encoder
void readLeftEncoder() {
  if (digitalRead(enc1_A) == digitalRead(enc1_B)) leftEncoderPos++;
  else leftEncoderPos--;
}

// ISR for Right Encoder
void readRightEncoder() {
  if (digitalRead(enc2_A) == digitalRead(enc2_B)) rightEncoderPos++;
  else rightEncoderPos--;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_RED, OUTPUT); pinMode(LED_GREEN, OUTPUT); pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, HIGH); digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_BLUE, HIGH);
  delay(1000);

  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(mAIN1, OUTPUT); pinMode(mAIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(mBIN1, OUTPUT); pinMode(mBIN2, OUTPUT); pinMode(PWMB, OUTPUT);

  // Setup Encoder Interrupts
  pinMode(enc1_A, INPUT_PULLUP); pinMode(enc1_B, INPUT_PULLUP);
  pinMode(enc2_A, INPUT_PULLUP); pinMode(enc2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc1_A), readLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2_A), readRightEncoder, CHANGE);

  if (myIMU.begin() != 0) digitalWrite(LED_RED, LOW);

  if (BLE.begin()) {
    BLE.setLocalName("BalanceBot_XIAO");
    BLE.setAdvertisedService(uartService);
    uartService.addCharacteristic(rxChar);
    uartService.addCharacteristic(txChar);
    BLE.addService(uartService);
    BLE.advertise();
    digitalWrite(LED_BLUE, LOW);
  }


  kalmanX.setQangle(0.002);
  kalmanX.setQbias(0.003);
  kalmanX.setRmeasure(0.01);

  kalmanY.setQangle(0.001);
  kalmanY.setQbias(0.003);
  kalmanY.setRmeasure(0.05);




  accX = myIMU.readFloatAccelX(); accY = myIMU.readFloatAccelY(); accZ = myIMU.readFloatAccelZ();
  double roll = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  kalmanX.setAngle(roll); kalmanY.setAngle(pitch);
  timer = micros();
}

void driveMotors(int speed) {
  if (speed == 0) {
    analogWrite(PWMA, 0); analogWrite(PWMB, 0);
    return;
  }

  int deadzoneA = 39; 
  int deadzoneB = 28;
  
  // Fixed scale factor for Motor A
  int absSpeedA = 0.85 * map(abs(speed), 0, 255, deadzoneA, 255);
  int absSpeedB = map(abs(speed), 0, 255, deadzoneB, 255);

  analogWrite(PWMA, absSpeedA);
  analogWrite(PWMB, absSpeedB);
  
  digitalWrite(mAIN1, speed < 0);
  digitalWrite(mAIN2, speed > 0);
  digitalWrite(mBIN1, speed < 0);
  digitalWrite(mBIN2, speed > 0);
}


bool toggle = false; // Toggle between 2 outputs on BLT monitor
double lastLoopTime; 

float activeTarget;

unsigned long nextLoopTime = 0;
unsigned long lastMicros = 0;
unsigned long LOOP_PERIOD = 10000; // 10ms in microseconds
int loopCounter = 0;

unsigned long lastExecutionTime = 0;
unsigned long actualInterval = 0;

void loop() {
  BLEDevice central = BLE.central();
  if (central.connected()) { digitalWrite(LED_BLUE, HIGH); digitalWrite(LED_GREEN, LOW); }
  else { digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_BLUE, LOW); }


  if (micros() >= nextLoopTime) {

    actualInterval = micros() - lastExecutionTime; // Time since last PID run
    lastExecutionTime = micros();

    nextLoopTime += LOOP_PERIOD;

    // Read IMU
    accX = myIMU.readFloatAccelX(); accY = myIMU.readFloatAccelY(); accZ = myIMU.readFloatAccelZ();
    gyroX = myIMU.readFloatGyroX(); gyroY = myIMU.readFloatGyroY(); gyroZ = myIMU.readFloatGyroZ();

    // Update Kalman
    //double dt = (double)(micros() - timer) / 1000000;
    const double dt = 0.01; // Exactly 10ms


    timer = micros();
    double roll = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    angle = -kalmanY.getAngle(pitch, gyroY, dt);

    // 1. Automatic Target Angle Tuning
    // Finds the center of gravity by monitoring encoder drift
    // if (abs(angle) < 10.0) {
    //   long avgPos = (leftEncoderPos + rightEncoderPos) / 2;
    //   // Adjust multiplier (0.000005) if the tuning is too fast or slow
    //   driftCorrection -= (float)avgPos * 0.000001; // was 0.000005
    //   driftCorrection = constrain(driftCorrection, MAX_DRIFT_ADJUST_M, MAX_DRIFT_ADJUST_P);
    // }
    driftCorrection = 0;

    // Update PID
    activeTarget = targetAngle + driftCorrection;
    pidError = activeTarget - angle;
    integral = constrain(integral + (pidError * dt), -100, 100);
    float derivative = (pidError - lastError) / dt;
    lastError = pidError;
    float output = (Kp * pidError) + (Ki * integral) + (Kd * derivative);

    // Drive Motors
    int motorSpeed = constrain((int)output, -255, 255);
    if (abs(angle) > 45.0) { 
      motorSpeed = 0; 
      integral = 0; 
    }
    driveMotors(motorSpeed);

  }


  // BLE & Serial Diagnostics
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    char buffer[128];

    if (toggle) {
      snprintf(buffer, sizeof(buffer), "T:%.1f A:%.1f", activeTarget, angle);
      // Serial.println(buffer);
    } else {
      snprintf(buffer, sizeof(buffer), "P%.0f D%.1f I%.1f ", Kp, Kd, Ki);
      //Serial.println(buffer);
    }

    if (central.connected()) txChar.writeValue(buffer);
    toggle = !toggle;
  }

  if (rxChar.written()) {
    const uint8_t* data = rxChar.value();
    char cmd = (char)data[0];
    if (cmd == 'p') Kp += 0.25; if (cmd == 'P') Kp += 5.0;
    if (cmd == 'q') Kp -= 0.25; if (cmd == 'Q') Kp -= 5.0;
    if (cmd == 'd') Kd += 0.1; if (cmd == 'D') Kd += 1.0;
    if (cmd == 'e') Kd -= 0.1; if (cmd == 'E') Kd -= 1.0;
    if (cmd == 'i') Ki += 0.1; if (cmd == 'I') Ki += 1.0;
    if (cmd == 'j') Ki -= 0.1; if (cmd == 'J') Ki -= 1.0;
    if (cmd == 'a') targetAngle -= 0.1; if (cmd == 'A') targetAngle += 0.1;
    if (cmd == 'r') { driftCorrection = 0; }
    if (cmd == 's') { Kp = 0; Kd = 0; Ki = 0; integral = 0; }
    if (Kp < 0) Kp = 0; if (Kd < 0) Kd = 0; if (Ki < 0) Ki = 0;
  }


  // unsigned long currentMicros = micros();
  // unsigned long loopTime = currentMicros - lastMicros;
  // lastMicros = currentMicros;

  // loopCounter++;

  // // Print the loop time every 500 cycles (roughly once or twice a second)
  // if (loopCounter >= 500) {
  //     Serial.print("Actual PID Interval (us): ");
  //     Serial.println(actualInterval); // This should stay near 10000
  //     loopCounter = 0;
  // }


}