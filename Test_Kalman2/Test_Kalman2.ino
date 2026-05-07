#include <LSM6DS3.h>
#include <Kalman.h> 
#include <Wire.h>

// 5/5/26 - KD - This works.  Lookin at wires: fwd, pitch is negative; back: pitch is positive



// Use the I2C address 0x6A for the onboard IMU
LSM6DS3 myIMU(I2C_MODE, 0x6A); 
Kalman kalmanX;
Kalman kalmanY;

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
uint32_t timer;

void setup() {
    Serial.begin(115200);
    // Initialize the IMU
    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    }

    // 1. Initial readings to set starting angle
    accX = myIMU.readFloatAccelX();
    accY = myIMU.readFloatAccelY();
    accZ = myIMU.readFloatAccelZ();

    // Calculate initial roll and pitch
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

    kalmanX.setAngle(roll);
    kalmanY.setAngle(pitch);
    timer = micros();
}

void loop() {
    // 2. Read Accelerometer axes individually
    accX = myIMU.readFloatAccelX();
    accY = myIMU.readFloatAccelY();
    accZ = myIMU.readFloatAccelZ();
    
    // 3. Read Gyroscope axes individually
    gyroX = myIMU.readFloatGyroX();
    gyroY = myIMU.readFloatGyroY();
    gyroZ = myIMU.readFloatGyroZ();

    // 4. Calculate delta time (dt)
    double dt = (double)(micros() - timer) / 1000000;
    timer = micros();

    // 5. Calculate source angles for the filter
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

    // 6. Apply Kalman Filter
    double kalAngleX = kalmanX.getAngle(roll, gyroX, dt);
    double kalAngleY = kalmanY.getAngle(pitch, gyroY, dt);

    // Output results
    Serial.print("Roll:"); Serial.print(kalAngleX); Serial.print("\t");
    Serial.print("Pitch:"); Serial.println(kalAngleY);
}
