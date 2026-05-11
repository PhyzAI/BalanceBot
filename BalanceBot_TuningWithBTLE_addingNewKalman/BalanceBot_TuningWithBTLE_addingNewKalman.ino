#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <Kalman.h>

// --- FILTER TOGGLE ---
bool useKalman = false; // Set to false to use Complementary Filter
float alpha = 0.98;    // Complementary Filter constant (Higher = smoother/more lag).  Too jittery, 0.95; too lazy: 0.99;
// -------------------------

// IMU Initialization
LSM6DS3 myIMU(I2C_MODE, 0x6A);
Kalman kalmanY; 

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

// Pin Definitions
const int mAIN1 = D8, mAIN2 = D9, PWMA = D7;
const int mBIN1 = A1, mBIN2 = A0, PWMB = D10;
const int STBY = D6;

// BLE Setup
BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic rxChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite | BLEWriteWithoutResponse, 20);
BLEStringCharacteristic txChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 128);

// Control Variables
float Kp = 0.0, Kd = 0.0, Ki = 0.0;
float angle = 0.0, pidError, lastError, integral;
float targetAngle = -7.7; // no coins on top, 4.9.
float activeTarget;

// Timing Variables
unsigned long nextLoopTime = 0;
const unsigned long LOOP_PERIOD = 10000; // 10ms (100Hz)
int loopCounter = 0;

void driveMotors(int speed) {
  if (speed == 0) {
    analogWrite(PWMA, 0); 
    analogWrite(PWMB, 0);
    return;
  }

  int deadzoneA = 39; 
  int deadzoneB = 28;

  // Motor A Scale factor 0.85 to account for physical asymmetry
  int absSpeedA = 0.85 * map(abs(speed), 0, 255, deadzoneA, 255);
  int absSpeedB = map(abs(speed), 0, 255, deadzoneB, 255);

  analogWrite(PWMA, absSpeedA);
  analogWrite(PWMB, absSpeedB);
  
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

  if (myIMU.begin() != 0) {
    Serial.println("IMU Initialization Failed");
  }

  if (BLE.begin()) {
    BLE.setLocalName("BalanceBot_XIAO");
    BLE.setAdvertisedService(uartService);
    uartService.addCharacteristic(rxChar);
    uartService.addCharacteristic(txChar);
    BLE.addService(uartService);
    BLE.advertise();
  }

  // Kalman Defaults
  kalmanY.setQangle(0.001);
  kalmanY.setQbias(0.003);
  kalmanY.setRmeasure(0.05);

  // Initial Angle Calculation
  accX = myIMU.readFloatAccelX(); 
  accY = myIMU.readFloatAccelY(); 
  accZ = myIMU.readFloatAccelZ();
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  kalmanY.setAngle(pitch);
  angle = pitch;
  
  nextLoopTime = micros();
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

  // --- CRITICAL TIMING BLOCK (100Hz) ---
  if (micros() >= nextLoopTime) {
    nextLoopTime += LOOP_PERIOD;
    
    // Read Sensors
    accX = myIMU.readFloatAccelX(); 
    accY = myIMU.readFloatAccelY(); 
    accZ = myIMU.readFloatAccelZ();
    gyroY = myIMU.readFloatGyroY();

    const double dt = 0.01; // Matches 10ms LOOP_PERIOD
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

    // Update Filter Choice
    if (useKalman) {
      angle = -kalmanY.getAngle(pitch, gyroY, dt);
    } else {
      // Complementary Filter
      angle = (alpha * (angle + (gyroY * dt))) + ((1.0 - alpha) * pitch);
    }

    // PID Calculation
    activeTarget = targetAngle;
    pidError = activeTarget - angle;
    integral = constrain(integral + (pidError * dt), -100, 100);
    float derivative = (pidError - lastError) / dt;
    lastError = pidError;

    float output = (Kp * pidError) + (Ki * integral) + (Kd * derivative);

    // Drive Motors
    int motorSpeed = constrain((int)output, -255, 255);
    
    // Safety Cutoff (45 degrees)
    if (abs(angle) > 45.0) { 
      motorSpeed = 0; 
      integral = 0; 
    }
    
    driveMotors(motorSpeed);
    loopCounter++;
  }

  // --- NON-CRITICAL UPDATES ---
  static unsigned long lastBleUpdate = 0;
  if (millis() - lastBleUpdate > 500) {
    lastBleUpdate = millis();
    static bool toggleDiagnostic = false;
    char buffer[128];

    if (toggleDiagnostic) {
      snprintf(buffer, sizeof(buffer), "T:%.1f A:%.1f", activeTarget, angle);
    } else {
      snprintf(buffer, sizeof(buffer), "P%.1f D%.1f I%.1f", Kp, Kd, Ki);
    }
    
    if (central.connected()) txChar.writeValue(buffer);
    toggleDiagnostic = !toggleDiagnostic;
  }

  // Read BLE Tuning Commands
  if (rxChar.written()) {
    const uint8_t* data = rxChar.value();
    char cmd = (char)data[0];
    
    // Tune Kp
    if (cmd == 'p') Kp += 0.25; if (cmd == 'P') Kp += 5.0;
    if (cmd == 'q') Kp -= 0.25; if (cmd == 'Q') Kp -= 5.0;
    
    // Tune Kd
    if (cmd == 'd') Kd += 0.1;  if (cmd == 'D') Kd += 1.0;
    if (cmd == 'e') Kd -= 0.1;  if (cmd == 'E') Kd -= 1.0;
    
    // Tune Ki
    if (cmd == 'i') Ki += 0.1;  if (cmd == 'I') Ki += 1.0;
    if (cmd == 'j') Ki -= 0.1;  if (cmd == 'J') Ki -= 1.0;

    // Tune Center of Gravity
    if (cmd == 'a') targetAngle -= 0.1; if (cmd == 'A') targetAngle += 0.1;
    
    // Stop/Reset
    if (cmd == 's') { Kp = 0; Kd = 0; Ki = 0; integral = 0; }
    
    // Safety Bounds
    Kp = max(0.0f, Kp); Kd = max(0.0f, Kd); Ki = max(0.0f, Ki);
  }
}