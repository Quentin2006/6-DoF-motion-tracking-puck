#include <Arduino.h>
#include "mpu6050.h"

Mpu6050 mpu;

void setup() {
  Serial.begin(115200);
  Serial.flush();
  delay(3000);
  mpu.getAccelBias(10);
  mpu.getGyroBias(10);
}

void loop() {
  // vec3 gyro;
  // mpu.readGyro(gyro);
  // Serial.printf("gyro: %f, %f, %f\n", gyro.x, gyro.y, gyro.z);
}
