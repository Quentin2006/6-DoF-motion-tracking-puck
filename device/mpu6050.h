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

  vec3 operator*(const float rhs) const { return {x * rhs, y * rhs, z * rhs}; }
};

class Mpu6050 {
public:
  bool begin(); // Wire.begin(sda,scl), wake, verify, configure; false on failure

  bool readGyro(vec3 &out);     // dps, bias-corrected, NO dt scaling
  bool readAccel(vec3 &out);    // m/s², factory-cal + bias-corrected, NO dt scaling
  bool readRawAccel(vec3 &out); // m/s², all cal bypassed; CAL6-only

  bool getGyroBias();  // GYRO_CALIB_SAMPLES-burst mean; false if no sample
  bool getAccelBias(); // ditto, gravity removed along the measured direction

  void setAccelCal(const float off[3], const float scale[3]); // persist to NVS
  void getAccelCal(float off[3], float scale[3]) const;
  void nudgeGyroBias(const vec3 &residualDps, float alpha); // still-detect re-bias, no I2C

private:
  bool writeReg(uint8_t reg, uint8_t val);
  bool readBurst6(uint8_t baseReg, float scale, vec3 &out);
  bool meanBurst(uint8_t baseReg, float scale, vec3 &out); // mean over GYRO_CALIB_SAMPLES bursts
  vec3 applyAccelCal(const vec3 &raw) const;               // 6-point factory cal
  void loadAccelCal();                                     // NVS namespace `mpu`, identity defaults
  void saveAccelCal() const;

  vec3 gyro_bias, accel_bias;
  float accOff_[3] = {0, 0, 0};   // m/s² per-axis offset (6-point cal)
  float accScale_[3] = {1, 1, 1}; // dimensionless per-axis scale
};
