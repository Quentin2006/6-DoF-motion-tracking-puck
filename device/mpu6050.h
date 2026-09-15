#pragma once

#include <Arduino.h>
#include <Wire.h>

struct vec3 {
  float x = 0;
  float y = 0;
  float z = 0;

  vec3 &operator+=(const vec3 &rhs) {
    this->x += rhs.x;
    this->y += rhs.y;
    this->z += rhs.z;
    return *this;
  }

  vec3 &operator-=(const vec3 &rhs) {
    this->x -= rhs.x;
    this->y -= rhs.y;
    this->z -= rhs.z;
    return *this;
  }

  vec3 &operator/=(const float rhs) {
    this->x /= rhs;
    this->y /= rhs;
    this->z /= rhs;
    return *this;
  }
};

class Mpu6050 {
public:
  Mpu6050(); // trivial: init addr/sda/scl/bases, zero biases; NO I2C

  bool begin(); // Wire.begin(sda,scl), wake, verify, configure; false on failure

  bool readGyro(vec3 &out);  // dps, bias-corrected, NO dt scaling
  bool readAccel(vec3 &out); // m/s², cal-scale + bias-corrected, NO dt scaling

  void getGyroBias(int samples);
  void getAccelBias(int samples);

  void setAccelCal(const float off[3], const float scale[3]); // persist to NVS
  void getAccelCal(float off[3], float scale[3]) const;
  void nudgeGyroBias(float mx, float my, float mz, float alpha); // still-detect re-bias, no I2C

private:
  bool writeReg(uint8_t reg, uint8_t val);
  bool readBurst6(uint8_t baseReg, float scale, vec3 &out);
  void loadAccelCal();      // NVS namespace `mpu`, identity defaults
  void saveAccelCal() const;

  uint8_t addr;
  int sda, scl;
  uint8_t mpu_gyro_base, mpu_accel_base;
  vec3 gyro_bias, accel_bias;
  float accOff_[3] = {0, 0, 0};     // m/s² per-axis offset (6-point cal)
  float accScale_[3] = {1, 1, 1};   // dimensionless per-axis scale
};
