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
  write_reg(MPU_PWR_MGMT_1_REG, 0x00); // Wake the MPU6050.
}

static bool read_burst(uint8_t base, int16_t out[3]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(base);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(MPU_ADDR, 6) != 6) {
    return false;
  }
  for (int i = 0; i < 3; ++i) {
    const uint8_t high = Wire.read();
    const uint8_t low = Wire.read();
    out[i] = static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
  }
  return true;
}

bool read_gyro_sample(vec3 &sample) {
  int16_t raw[3];
  if (!read_burst(MPU_GYRO_BASE, raw)) {
    return false;
  }
  sample = {raw[0] * GYRO_LSB_TO_DPS, raw[1] * GYRO_LSB_TO_DPS,
            raw[2] * GYRO_LSB_TO_DPS};
  return true;
}

bool read_accel_sample(vec3 &sample) {
  int16_t raw[3];
  if (!read_burst(MPU_ACCEL_BASE, raw)) {
    return false;
  }
  sample = {raw[0] * ACCEL_LSB_O_MPS2, raw[1] * ACCEL_LSB_O_MPS2,
            raw[2] * ACCEL_LSB_O_MPS2};
  return true;
}

void imu_init() {
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);
  wake_mpu();
  // ~100 Hz internal rate (1 kHz / (1 + 9)) with DLPF ~42 Hz gyro / 44 Hz
  // accel.
  write_reg(SMPRT_DIV_REG, 9);
  write_reg(CONFIG_REG, 0x03);
}
