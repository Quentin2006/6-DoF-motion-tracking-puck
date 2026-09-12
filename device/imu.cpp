#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "imu.h"

static constexpr uint8_t SMPRT_DIV_REG = 0x19;
static constexpr uint8_t CONFIG_REG = 0x1A;
static constexpr uint8_t GYRO_CONFIG_REG = 0x1B;
static constexpr uint8_t ACCEL_CONFIG_REG = 0x1C;
static constexpr uint8_t WHO_AM_I_REG = 0x75;
static constexpr uint8_t WHO_AM_I_EXPECTED = 0x68;
static constexpr uint8_t AD0_HIGH_ADDR = 0x69;

static bool write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}

void imu_init() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(100);
  bool ok = true;
  ok &= write_reg(MPU_PWR_MGMT_1_REG, 0x00); // wake the MPU6050
  // ~100 Hz internal rate (1 kHz / (1 + 9)) with DLPF ~42 Hz gyro / 44 Hz
  // accel.
  ok &= write_reg(SMPRT_DIV_REG, 9);
  ok &= write_reg(CONFIG_REG, 0x03);
  ok &= write_reg(GYRO_CONFIG_REG, 0x00);  // pin ±250 dps to match GYRO_LSB_TO_DPS
  ok &= write_reg(ACCEL_CONFIG_REG, 0x00); // pin ±2 g to match ACCEL_LSB_O_MPS2
  if (!ok) {
    Serial.println("MPU6050 config write failed (check wiring)");
  }
}

bool imu_read(vec3 &gyro_dps, vec3 &accel_mps2) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_BASE);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(MPU_ADDR, 14) != 14) {
    return false;
  }
  uint8_t b[14];
  for (int i = 0; i < 14; ++i) {
    b[i] = static_cast<uint8_t>(Wire.read());
  }
  const auto s16 = [](uint8_t hi, uint8_t lo) {
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
  };
  accel_mps2 = {s16(b[0], b[1]) * ACCEL_LSB_O_MPS2,
                s16(b[2], b[3]) * ACCEL_LSB_O_MPS2,
                s16(b[4], b[5]) * ACCEL_LSB_O_MPS2};
  gyro_dps = {s16(b[8], b[9]) * GYRO_LSB_TO_DPS,
              s16(b[10], b[11]) * GYRO_LSB_TO_DPS,
              s16(b[12], b[13]) * GYRO_LSB_TO_DPS};
  return true;
}

static bool who_am_i(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(WHO_AM_I_REG);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(addr, 1) != 1) {
    return false;
  }
  return static_cast<uint8_t>(Wire.read()) == WHO_AM_I_EXPECTED;
}

bool imu_self_test() {
  if (who_am_i(MPU_ADDR)) {
    return true;
  }
  if (who_am_i(AD0_HIGH_ADDR)) {
    Serial.println("MPU6050 found at 0x69; set MPU_ADDR to 0x69");
  }
  return false;
}
