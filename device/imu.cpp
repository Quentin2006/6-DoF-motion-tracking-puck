#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "imu.h"

static constexpr uint8_t SMPRT_DIV_REG = 0x19;
static constexpr uint8_t CONFIG_REG = 0x1A;

static void write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

static void wake_mpu() {
  write_reg(MPU_PWR_MGMT_1_REG, 0x00);  // Wake the MPU6050.
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

static bool read_triplet(vec3 &sample, vec3 regs, float scale) {
  sample = { read_data(regs.x, scale),
             read_data(regs.y, scale),
             read_data(regs.z, scale) };
  return !isnan(sample.x) && !isnan(sample.y) && !isnan(sample.z);
}

bool read_gyro_sample(vec3 &sample) {
  return read_triplet(sample, MPU_GYRO_REGS, GYRO_LSB_TO_DPS);
}

bool read_accel_sample(vec3 &sample) {
  return read_triplet(sample, MPU_ACCEL_REGS, ACCEL_LSB_O_MPS2);
}

void imu_init() {
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);
  wake_mpu();
  // ~100 Hz internal rate (1 kHz / (1 + 9)) with DLPF ~42 Hz gyro / 44 Hz accel.
  write_reg(SMPRT_DIV_REG, 9);
  write_reg(CONFIG_REG, 0x03);
}
