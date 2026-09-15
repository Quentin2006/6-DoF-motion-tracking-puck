#include "config.h"
#include "mpu6050.h"
#include <Arduino.h>

Mpu6050 mpu;
void setup() {
  Serial.begin(115200);
  delay(500);
  if (!mpu.begin()) {
    Serial.println("MPU6050 init failed");
    while (true) {
      delay(100);
    }
  }
  mpu.getAccelBias(GYRO_CALIB_SAMPLES);
  mpu.getGyroBias(GYRO_CALIB_SAMPLES);
}
void loop() {
  vec3 accel, gyro;
  bool okA = mpu.readAccel(accel);
  bool okG = mpu.readGyro(gyro);
  if (!okA || !okG) {
    Serial.println("MPU read failed");
    delay(100);
    return;
  }
  // rates here are instantaneous dps / m/s²; integrate/Δt belongs to the future filter, not the driver
  delay(100);
}
