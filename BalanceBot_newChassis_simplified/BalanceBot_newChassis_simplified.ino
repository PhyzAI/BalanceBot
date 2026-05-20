#include <Arduino.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <ArduinoBLE.h> 


const bool ENABLE_MEASURE_ANGLE = false; // turn off control and just display angle



// The encoder pins are defined: Encoder 1 is D2 and D3, Encoder 2 is D4 and D5.


LSM6DS3 myIMU(I2C_MODE, 0x6A);

// --- TUNING PARAMETERS ---
float Kp = 6.0; // 
float Kd = 0.1;  // 0.2
float Ki = 0.0;  // 0.3
// 6, 0.1, 0.01 works for the V3 chassi

float targetAngle = -3.7;
int maxMotorSpeed = 255;  
int minMotorSpeed = 30; // was 35



float angle = 0.0;
float lastError = 0.0;
float integral = 0.0;
unsigned long nextLoopTime = 0;
const unsigned long LOOP_PERIOD = 10000; // was 10000

float alpha = 0.95;        
float motorA_Scale = 1.0;
float motorB_Scale = 0.9;


// --- ENCODERS ---
const int ENC_L_A = D2;
const int ENC_L_B = D3;

const int ENC_R_A = D4;
const int ENC_R_B = D5;

volatile long encoderLeft = 0;
volatile long encoderRight = 0;

// Velocity damping gain
float Kv = 0.35;






// BLE Setup
BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic rxChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite | BLEWriteWithoutResponse, 20);
BLEStringCharacteristic txChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 128);

// --- PIN DEFINITIONS ---
const int mAIN1 = D8, mAIN2 = D9, PWMA = D7;
const int mBIN1 = A1, mBIN2 = A0, PWMB = D10;
const int STBY = D6;




// --- FORWARD DECLARATIONS ---
void setupBLE();
void handleBLE();



void driveMotors(int speed) {

  int absSpeed = abs(speed);

  // Small quiet zone
  if (absSpeed < 8) {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    return;
  }

  // Deadband compensation
  int finalOutput =
      minMotorSpeed +
      ((absSpeed - 8) *
      (maxMotorSpeed - minMotorSpeed))
      / (255 - 8);

  finalOutput = constrain(finalOutput, 0, 255);

  analogWrite(PWMA, finalOutput);
  analogWrite(PWMB, (int)(finalOutput * motorB_Scale));

  // Direction
  digitalWrite(mAIN1, speed < 0);
  digitalWrite(mAIN2, speed > 0);

  digitalWrite(mBIN1, speed < 0);
  digitalWrite(mBIN2, speed > 0);
}




void ISR_leftEncoder() {
  bool A = digitalRead(ENC_L_A);
  bool B = digitalRead(ENC_L_B);

  if (A == B)
    encoderLeft++;
  else
    encoderLeft--;
}

void ISR_rightEncoder() {
  bool A = digitalRead(ENC_R_A);
  bool B = digitalRead(ENC_R_B);

  if (A == B)
    encoderRight++;
  else
    encoderRight--;
}





void setup() {
  Serial.begin(115200);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(mAIN1, OUTPUT); pinMode(mAIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(mBIN1, OUTPUT); pinMode(mBIN2, OUTPUT); pinMode(PWMB, OUTPUT);


  // Encoder pins
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);

  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  // Encoder interrupts
  attachInterrupt(
    digitalPinToInterrupt(ENC_L_A),
    ISR_leftEncoder,
    CHANGE);

  attachInterrupt(
    digitalPinToInterrupt(ENC_R_A),
    ISR_rightEncoder,
    CHANGE);




  myIMU.begin();
  

  if (ENABLE_MEASURE_ANGLE) {
    Kp = 0; Kd = 0; Ki = 0;
  }


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

    //const float dt = 0.01;

    static unsigned long lastMicros = micros();
    unsigned long now = micros();
    float dt = (now - lastMicros) / 1000000.0;
    lastMicros = now;


    float pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    angle = (alpha * (angle + (gyroY * dt))) + ((1.0 - alpha) * pitch);

    float error = targetAngle - angle;
    
    integral = constrain(integral + (error * dt), -20, 20);
    //float derivative = (error - lastError) / dt;
    float derivative = -gyroY;

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
  BLE.setLocalName("BalanceBot_XIAO");
  BLE.setAdvertisedService(uartService);
  uartService.addCharacteristic(rxChar);
  uartService.addCharacteristic(txChar);
  BLE.addService(uartService);
  BLE.advertise();
  
}

void handleBLE() {
  BLEDevice central = BLE.central();

  // Status LED: Green when phone is connected, otherwise Blue
  if (central.connected()) {
    digitalWrite(LED_BLUE, HIGH);
    digitalWrite(LED_GREEN, LOW);
  } else {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, LOW);
  }


  static unsigned long lastBleUpdate = 0;
  if (millis() - lastBleUpdate > 500) {
    lastBleUpdate = millis();
    static bool toggleDiagnostic = false;
    char buffer[128];

    
    if (ENABLE_MEASURE_ANGLE)
      snprintf(buffer, sizeof(buffer), "A%.1f ", angle); 
    else {
      snprintf(buffer, sizeof(buffer), "A%.1f P%.0f D%.1f I%.2f ", targetAngle,Kp, Kd, Ki);
    }
    
    if (central.connected()) txChar.writeValue(buffer);
    toggleDiagnostic = !toggleDiagnostic;
  }


  // Read BLE Tuning Commands
  if (rxChar.written()) {
    const uint8_t* data = rxChar.value();
    char cmd = (char)data[0];
    
    // Tune Kp
    if (cmd == 'p') Kp += 0.5; if (cmd == 'P') Kp += 1.0;
    if (cmd == 'q') Kp -= 0.5; if (cmd == 'Q') Kp -= 1.0;
    
    // Tune Kd
    if (cmd == 'd') Kd += 0.1;  if (cmd == 'D') Kd += 1.0;
    if (cmd == 'e') Kd -= 0.1;  if (cmd == 'E') Kd -= 1.0;
    
    // Tune Ki
    if (cmd == 'i') Ki += 0.01;  if (cmd == 'I') Ki += 0.1;
    if (cmd == 'j') Ki -= 0.01;  if (cmd == 'J') Ki -= 0.1;

    // Tune Center of Gravity
    if (cmd == 'a') targetAngle -= 0.1; if (cmd == 'A') targetAngle += 0.1;
    
    // Stop/Reset
    if (cmd == 's') { Kp = 0; Kd = 0; Ki = 0; integral = 0; }
    
    // Safety Bounds
    Kp = max(0.0f, Kp); Kd = max(0.0f, Kd); Ki = max(0.0f, Ki);
  }
  
}