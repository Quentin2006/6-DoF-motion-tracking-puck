#pragma once

#include <Arduino.h>
#include <Wire.h>

struct vec3 {
  float x;
  float y;
  float z;
};

class Mpu6050 {
public:
  Mpu6050();

  // these will auto normalize to the delta time
  bool readGyro(vec3 &sample);
  bool readAccel(vec3 &sample);

private:
  float read_data(size_t registerAddress, float scale);

  uint8_t addr;
  int sda, scl;
  uint8_t mpu_gyro_base, mpu_accel_base;

  unsigned long now_gyro_us, last_gyro_us;
  unsigned long now_accel_us, last_accel_us;
};
