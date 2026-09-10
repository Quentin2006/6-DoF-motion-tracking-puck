#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "imu.h"

static void wake_mpu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT_1_REG);
  Wire.write(0x00);  // Wake the MPU6050.
  Wire.endTransmission(true);
  Wire.endTransmission(true);
}

static float read_data(size_t registerAddress, const float scale) {
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
  sample = { read_data(MPU_GYRO_REGS.x, GYRO_LSB_TO_DPS),
             read_data(MPU_GYRO_REGS.y, GYRO_LSB_TO_DPS),
             read_data(MPU_GYRO_REGS.z, GYRO_LSB_TO_DPS) };

  return !isnan(sample.x) && !isnan(sample.y) && !isnan(sample.z);
}

bool read_accel_sample(vec3 &sample) {
  sample = { read_data(MPU_ACCEL_REGS.x, ACCEL_LSB_O_MPS2),
             read_data(MPU_ACCEL_REGS.y, ACCEL_LSB_O_MPS2),
             read_data(MPU_ACCEL_REGS.z, ACCEL_LSB_O_MPS2) };

  return !isnan(sample.x) && !isnan(sample.y) && !isnan(sample.z);
}

void imu_init() {
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);
  wake_mpu();
}
