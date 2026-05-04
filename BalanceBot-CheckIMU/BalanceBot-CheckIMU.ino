#include <LSM6DS3.h>
#include <Wire.h>

// Create an instance of the LSM6DS3 sensor
LSM6DS3 myIMU(I2C_MODE, 0x6A);    // I2C device address 0x6A

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open

  if (myIMU.begin() != 0) {
    Serial.println("IMU error! Check if you have the 'Sense' version.");
    while (1);
  } else {
    Serial.println("IMU initialized successfully!");
  }
}

void loop() {
  // Accelerometer data (G's)
  Serial.print("Accel X: "); Serial.print(myIMU.readFloatAccelX(), 4);
  Serial.print(" Y: "); Serial.print(myIMU.readFloatAccelY(), 4);
  Serial.print(" Z: "); Serial.print(myIMU.readFloatAccelZ(), 4);

  // Gyroscope data (Degrees per second)
  Serial.print(" | Gyro X: "); Serial.print(myIMU.readFloatGyroX(), 4);
  Serial.print(" Y: "); Serial.print(myIMU.readFloatGyroY(), 4);
  Serial.print(" Z: "); Serial.println(myIMU.readFloatGyroZ(), 4);

  delay(200);
}
