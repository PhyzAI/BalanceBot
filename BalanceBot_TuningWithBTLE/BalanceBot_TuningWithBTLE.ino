#include <Arduino
BLE.h>
#include <LSM6DS3.h>
#include <Wire.h>
NEW SKETCH




// Create the IMU object for the Seeed library
// I2C_MODE and 0x6A are the internal settings for the XIAO Sense
LSM6DS3 myIMU(I2C_MODE, 0x6A);



// --- RENAMED PINS ---
const int mAIN1 = D8, mAIN2 = D9, PWMA = D7;
const int mBIN1 = A1, mBIN2 = A0, PWMB = D10;
const int STBY = D6;

BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic rxChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite | BLEWriteWithoutResponse, 20);
BLEStringCharacteristic txChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 20);

float Kp = 0.0, Kd = 0.0, Ki = 0.0;
float angle = 0.0, pidError, lastError, integral;
unsigned long lastTime;

float targetAngle = 2.2; // Your physical balance point


void setup() {
  Serial.begin(115200);
  
  pinMode(LED_RED, OUTPUT); pinMode(LED_GREEN, OUTPUT); pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, HIGH); digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_BLUE, HIGH); 

  // --- NEW: POWER STABILIZATION DELAY ---
  delay(1000); // Wait 1 second for the LiPo/Regulator to settle

  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(mAIN1, OUTPUT); pinMode(mAIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(mBIN1, OUTPUT); pinMode(mBIN2, OUTPUT); pinMode(PWMB, OUTPUT);


  // Seeed's begin() returns 0 on success, unlike the Arduino library
  if (myIMU.begin() != 0) {
    digitalWrite(LED_RED, LOW); // Error: Red ON
  } else {
    digitalWrite(LED_RED, HIGH); // Success: Red OFF
  }

  // BLE Setup (unchanged)
  if (BLE.begin()) {
    BLE.setLocalName("BalanceBot_XIAO");
    BLE.setAdvertisedService(uartService);
    uartService.addCharacteristic(rxChar);
    uartService.addCharacteristic(txChar);
    BLE.addService(uartService);
    BLE.advertise();
    digitalWrite(LED_BLUE, LOW); 
  }

  lastTime = micros();
}

void driveMotors(int speed) {
  if (speed == 0) {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    return;
  }

  // Apply the deadzone offset found in calibration
  int deadzoneA = 39; // Replace with your result
  int deadzoneB = 28; // Replace with your result
  
  int absSpeedA = map(abs(speed), 0, 255, deadzoneA, 255);
  int absSpeedB = map(abs(speed), 0, 255, deadzoneB, 255);

  analogWrite(PWMA, absSpeedA);
  analogWrite(PWMB, absSpeedB);
  
  digitalWrite(mAIN1, speed < 0);
  digitalWrite(mAIN2, speed > 0);
  digitalWrite(mBIN1, speed < 0);
  digitalWrite(mBIN2, speed > 0);
}


void loop() {
  BLEDevice central = BLE.central();
  
  // Status LED: Green when phone is connected, otherwise Blue
  if (central.connected()) {
    digitalWrite(LED_BLUE, HIGH);
    digitalWrite(LED_GREEN, LOW);
  } else {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, LOW);
  }

  // Timing
  unsigned long now = micros();
  float dt = (float)(now - lastTime) / 1000000.0;
  if (dt <= 0 || dt > 0.1) dt = 0.01; // Prevent calculation explosions
  lastTime = now;

  // IMU Read (Seeed Library)
  float ax = myIMU.readFloatAccelX();
  float az = myIMU.readFloatAccelZ();
  float gy = myIMU.readFloatGyroY();
  
  // Complimentary Filter
  float accAngle = atan2(ax, az) * 180 / PI;
  angle = 0.98 * (angle + gy * dt) + 0.02 * accAngle;



  // PID math
  pidError = targetAngle - angle;
  integral = constrain(integral + (pidError * dt), -100, 100);
  float derivative = (pidError - lastError) / dt;
  lastError = pidError;
  float output = (Kp * pidError) + (Ki * integral) + (Kd * derivative);
  
  int motorSpeed = constrain((int)output, -255, 255);
  if (abs(angle) > 45.0) motorSpeed = 0; // Tip-over safety
  

  // Prevent Integral windup
  if (abs(angle) > 45.0) {
    motorSpeed = 0;
    integral = 0; // Reset so it doesn't "explode" when you pick it up
  }


  driveMotors(motorSpeed);

  // --- NEW: BLE & SERIAL DIAGNOSTICS ---
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) { // Every 200ms
    lastPrint = millis();
    
    char buffer[40];
    // Format: Angle, Kp, Ki, Kd
    snprintf(buffer, sizeof(buffer), "A:%.0f P:%.1f D:%.1f I:%.1f T:%.1f", angle, Kp, Kd, Ki, targetAngle);
    // Print to PC
    Serial.println(buffer);
    
    // Push to Phone
    if (central.connected()) {
      txChar.writeValue(buffer);
    }
  }

  // Listen for Phone commands
if (rxChar.written()) {
    const uint8_t* data = rxChar.value();
    char cmd = (char)data[0];

    // --- Proportional (Kp) ---
    if (cmd == 'p') Kp += 0.25;
    if (cmd == 'P') Kp += 1.0;
    if (cmd == 'q') Kp -= 0.25;
    if (cmd == 'Q') Kp -= 1.0;

    // --- Derivative (Kd) ---
    if (cmd == 'd') Kd += 0.05;
    if (cmd == 'D') Kd += 0.2;
    if (cmd == 'e') Kd -= 0.05;
    if (cmd == 'E') Kd -= 0.2;

    // --- Integral (Ki) ---
    if (cmd == 'i') Ki += 0.1;
    if (cmd == 'I') Ki += 1.0;
    if (cmd == 'j') Ki -= 0.1;
    if (cmd == 'J') Ki -= 1.0;

    // --- Target Angle (Offset) ---
    if (cmd == 'a') targetAngle -= 0.1;
    if (cmd == 'A') targetAngle += 0.1;

    // --- Emergency Stop / Reset ---
    if (cmd == 's') { Kp = 0; Kd = 0; Ki = 0; integral = 0; }

    // Constrain values so they don't go negative
    if (Kp < 0) Kp = 0;
    if (Kd < 0) Kd = 0;
    if (Ki < 0) Ki = 0;
  }

}


