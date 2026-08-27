#include "HardwareSerial.h"
#include "madgwickFilter.h"
#include <Arduino.h>
#include <Wire.h>

struct vec3 {
  float x;
  float y;
  float z;

  // Overload the += operator
  vec3 &operator+=(const vec3 &rhs) {
    this->x += rhs.x;
    this->y += rhs.y;
    this->z += rhs.z;
    return *this;
  }
};

constexpr int SDA_PIN = 5;
constexpr int SCL_PIN = 4;

const int MPU_ADDR = 0x68;
const int MPU_PWR_MGMT_1_REG = 0x6B;

constexpr vec3 MPU_ACCEL_REGS{0x3B, 0x3D, 0x3F};
constexpr vec3 MPU_GYRO_REGS{0x43, 0x45, 0x47};

constexpr float ACCEL_LSB_O_MPS2 = 9.80665f / 16384.0f;
constexpr float GYRO_LSB_TO_DPS = 1.0f / 131.0f;
constexpr float GRAVITY_MPS2 = 9.80665f;

void wake_mpu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT_1_REG);
  Wire.write(0x00); // Wake the MPU6050.
  Wire.endTransmission(true);
  Wire.endTransmission(true);
}

float read_data(size_t registerAddress, const float scale) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(registerAddress);
  if (Wire.endTransmission(false) != 0) {
    return NAN;
  }

  if (Wire.requestFrom(MPU_ADDR, 2) != 2) {
    return NAN;
  }

  const uint8_t high = Wire.read();
  const uint8_t low = Wire.read();

  const int16_t data =
      static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
  return static_cast<float>(data) * scale;
}

bool read_gyro_sample(vec3 &sample) {
  sample = {read_data(MPU_GYRO_REGS.x, GYRO_LSB_TO_DPS),
            read_data(MPU_GYRO_REGS.y, GYRO_LSB_TO_DPS),
            read_data(MPU_GYRO_REGS.z, GYRO_LSB_TO_DPS)};

  return !isnan(sample.x) && !isnan(sample.y) && !isnan(sample.z);
}

bool read_accel_sample(vec3 &sample) {
  sample = {read_data(MPU_ACCEL_REGS.x, ACCEL_LSB_O_MPS2),
            read_data(MPU_ACCEL_REGS.y, ACCEL_LSB_O_MPS2),
            read_data(MPU_ACCEL_REGS.z, ACCEL_LSB_O_MPS2)};

  return !isnan(sample.x) && !isnan(sample.y) && !isnan(sample.z);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);
  wake_mpu();
  Serial.println("Device awake");
}

void loop() {}
