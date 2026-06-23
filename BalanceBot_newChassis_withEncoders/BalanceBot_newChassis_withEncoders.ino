#include <Arduino.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <ArduinoBLE.h> 


const bool ENABLE_MEASURE_ANGLE = false; // turn off control and just display angle



// The encoder pins are defined: Encoder 1 is D2 and D3, Encoder 2 is D4 and D5.


LSM6DS3 myIMU(I2C_MODE, 0x6A);

// --- TUNING PARAMETERS ---
float Kp = 20.0; // 
float Kd = 0.4;  // 0.2
float Ki = 0.4;  // 0.3
// Velocity damping gain
float Kv = 0.0;
// 6, 0.1, 0.01 works for the V3 chassis
// 20, 0.4, 0.4 works for the V4 chassis

// Encoder based position and velocity, control loop
float K_pos = 3.0; // Outer Loop: Returning to home spot
float K_vel = 0.0; // Outer loop: velocity gain
const float max_encoder_angle = 0.5;  // maximum amount this loop will adjust the angle.  Was 2.0.
// 5.0, 0.0, 1.0 works for V4 chassis
    

float targetAngle = -3.4;
int maxMotorSpeed = 255;  
int minMotorSpeed = 20; // was 35.  At 15, the shaking mostly goes away, but the motors become very noisy (high freq noise)
const int speed_deadzone = 3; 



float angle = 0.0;
float lastError = 0.0;
float integral = 0.0;
unsigned long nextLoopTime = 0;
const unsigned long LOOP_PERIOD = 5000; // was 10000

float alpha = 0.95;        // was 0.95
float motorA_Scale = 1.0;
float motorB_Scale = 0.9;


// --- ENCODER CONFIGURATION ---
const int encLeftA = A2;  // Adjust pins to match your V3 wiring
const int encLeftB = D3;
const int encRightA = A4;
const int encRightB = D5;

volatile long leftEncCount = 0;
volatile long rightEncCount = 0;

long lastLeftCount = 0;
long lastRightCount = 0;
float wheelVelocity = 0.0;
float wheelPosition = 0.0;





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
  if (absSpeed < speed_deadzone) { // was 8.  didn't balance
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    return;
  }

  // Deadband compensation
  int finalOutput;
  //int absSpeed = abs(speed);
  
  finalOutput = map(absSpeed, 0, 255, minMotorSpeed, maxMotorSpeed);

  finalOutput = constrain(finalOutput, 0, 255);

  analogWrite(PWMA, finalOutput);
  analogWrite(PWMB, (int)(finalOutput * motorB_Scale));

  // Direction
  digitalWrite(mAIN1, speed < 0);
  digitalWrite(mAIN2, speed > 0);

  digitalWrite(mBIN1, speed < 0);
  digitalWrite(mBIN2, speed > 0);
}




void isrLeft() {
  if (digitalRead(encLeftB) == HIGH) leftEncCount++;
  else leftEncCount--;
}

void isrRight() {
  if (digitalRead(encRightB) == HIGH) rightEncCount++;
  else rightEncCount++;
}



void setup() {
  Serial.begin(115200);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(mAIN1, OUTPUT); pinMode(mAIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(mBIN1, OUTPUT); pinMode(mBIN2, OUTPUT); pinMode(PWMB, OUTPUT);

  // --- ENCODER PIN INITIALIZATION ---
  pinMode(encLeftA, INPUT_PULLUP);  // D2 [cite: 38]
  pinMode(encLeftB, INPUT_PULLUP);  // D3 - CRITICAL FIX FOR INTERRUPT READING [cite: 38]
  pinMode(encRightA, INPUT_PULLUP); // D4 [cite: 39]
  pinMode(encRightB, INPUT_PULLUP); // D5 - CRITICAL FIX FOR INTERRUPT READING [cite: 39]

  // Attach hardware interrupts to the primary channels 
  attachInterrupt(digitalPinToInterrupt(encLeftA), isrLeft, RISING); //[cite: 51]
  attachInterrupt(digitalPinToInterrupt(encRightA), isrRight, RISING); // [cite: 51]



  myIMU.begin();
  
  Serial.print("***Starting***");

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


    float dynamicTarget = targetAngle + (wheelPosition * K_pos) + (wheelVelocity * K_vel);
    
    // Don't let the encoder tilt the bot more than 2 degrees off-center
    dynamicTarget = constrain(dynamicTarget, targetAngle - max_encoder_angle, targetAngle + max_encoder_angle);

    // Calculate PID using the dynamic target
    float error = dynamicTarget - angle;
    //float error = targetAngle - angle;
    
    integral = constrain(integral + (error * dt), -20, 20);
    //float derivative = (error - lastError) / dt;
    float derivative = -gyroY;



    // 1. Read current counts safely (turn off interrupts briefly to prevent data tearing)
    noInterrupts();
    long currentLeft = leftEncCount;
    long currentRight = rightEncCount;
    interrupts();

    // 2. Calculate change since last loop
    long dLeft = currentLeft - lastLeftCount;
    long dRight = currentRight - lastRightCount;
    
    lastLeftCount = currentLeft;
    lastRightCount = currentRight;

    // 3. Average wheel speed (Velocity) and distance traveled (Position)
    float currentVelocity = (dLeft + dRight) / 2.0; 
    
    // Smooth out velocity noise with a quick low-pass filter
    wheelVelocity = (0.9 * wheelVelocity) + (0.1 * currentVelocity);
    
    // Accumulate total position drift
    wheelPosition += wheelVelocity;
    // Constrain position memory so it doesn't build up infinitely
    wheelPosition = constrain(wheelPosition, -1000, 1000);




    lastError = error;

    float output =
      (Kp * error)
    + (Ki * integral)
    + (Kd * derivative)
    - (Kv * wheelVelocity); 







    if (abs(angle) > 35.0) { // was 45
      driveMotors(0);
    } else {
      driveMotors((int)output);
    }
    
    static int pCount = 0;
    //if (abs(wheelSpeed) > 0.0) {
    if (pCount++ > 10) {
       Serial.print("Ang:"); Serial.print(angle);
       Serial.print(" Err:"); Serial.print(error);
       Serial.print(" Vel:");
       Serial.print(wheelVelocity);

       Serial.print(" Out:");
       Serial.print(output);

       Serial.print(" L:");
       Serial.print(leftEncCount);
       Serial.print(" R:");
       Serial.println(rightEncCount);
       //Serial.print(" Kp:"); Serial.print(Kp);
       //Serial.print(" Ki:"); Serial.print(Ki);
       //Serial.print(" Kd:"); Serial.println(Kd);
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
      snprintf(buffer, sizeof(buffer), "A%.1f P%.0f V:%.1f D%.1f I%.1f ", targetAngle, Kp, Kv, Kd, Ki);
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


    // Tune Kv
    if (cmd == 'v') Kv += 0.1;  if (cmd == 'V') Kv += 1.0;
    if (cmd == 'w') Kv -= 0.1;  if (cmd == 'W') Kv -= 1.0;
  
    // Tune K_pos (location)
    if (cmd == 'l') K_pos += 0.01;  if (cmd == 'L') K_pos += 0.1;
    if (cmd == 'm') K_pos -= 0.01;  if (cmd == 'm') K_pos -= 0.1;


    // Tune Center of Gravity
    if (cmd == 'a') targetAngle -= 0.1; if (cmd == 'A') targetAngle += 0.1;
    
    // Stop/Reset
    if (cmd == 's') { Kp = 0; Kd = 0; Ki = 0; integral = 0; }
    
    // Safety Bounds
    Kp = max(0.0f, Kp); Kd = max(0.0f, Kd); Ki = max(0.0f, Ki);
  }
  
}