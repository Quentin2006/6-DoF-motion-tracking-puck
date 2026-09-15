#include <math.h>
#include "mpu6050.h"
#include "config.h"

Mpu6050::Mpu6050()
    : addr(MPU_ADDR), sda(SDA_PIN), scl(SCL_PIN),
      mpu_gyro_base(MPU_GYRO_BASE), mpu_accel_base(MPU_ACCEL_BASE),
      gyro_bias{0, 0, 0}, accel_bias{0, 0, 0} {}

bool Mpu6050::begin() {
  Wire.begin(static_cast<uint8_t>(sda), static_cast<uint8_t>(scl));
  Wire.setClock(400000);

  if (!writeReg(MPU_PWR_MGMT_1_REG, 0x00)) {
    return false;
  }
  delay(100);

  Wire.beginTransmission(addr);
  Wire.write(MPU_WHO_AM_I_REG);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(addr, static_cast<uint8_t>(1)) != 1) {
    return false;
  }
  const int who = Wire.read();
  if (who != MPU_WHO_AM_I_EXPECT && who != MPU_WHO_AM_I_EXPECT_ALT) {
    return false;
  }

  if (!writeReg(MPU_SMPLRT_DIV_REG, MPU_SMPLRT_DIV_VAL)) {
    return false;
  }
  if (!writeReg(MPU_CONFIG_REG, MPU_CONFIG_VAL)) {
    return false;
  }
  if (!writeReg(MPU_GYRO_CONFIG_REG, MPU_GYRO_CONFIG_VAL)) {
    return false;
  }
  if (!writeReg(MPU_ACCEL_CONFIG_REG, MPU_ACCEL_CONFIG_VAL)) {
    return false;
  }
  return true;
}

bool Mpu6050::readGyro(vec3 &out) {
  vec3 raw;
  if (!readBurst6(mpu_gyro_base, GYRO_LSB_TO_DPS, raw)) {
    return false;
  }
  out = raw;
  out -= gyro_bias;
  return true;
}

bool Mpu6050::readAccel(vec3 &out) {
  vec3 raw;
  if (!readBurst6(mpu_accel_base, ACCEL_LSB_O_MPS2, raw)) {
    return false;
  }
  out = raw;
  out -= accel_bias;
  return true;
}

void Mpu6050::getGyroBias(int samples) {
  if (samples <= 0) {
    return;
  }
  vec3 total;
  vec3 s;
  int ok = 0;
  for (int i = 0; i < samples; ++i) {
    delay(MPU_CALIB_DELAY_MS);
    if (readBurst6(mpu_gyro_base, GYRO_LSB_TO_DPS, s)) {
      total += s;
      ++ok;
    }
  }
  if (ok > 0) {
    total /= static_cast<float>(ok);
    gyro_bias = total;
  }
}

void Mpu6050::getAccelBias(int samples) {
  if (samples <= 0) {
    return;
  }
  vec3 total;
  vec3 s;
  int ok = 0;
  for (int i = 0; i < samples; ++i) {
    delay(MPU_CALIB_DELAY_MS);
    if (readBurst6(mpu_accel_base, ACCEL_LSB_O_MPS2, s)) {
      total += s;
      ++ok;
    }
  }
  if (ok > 0) {
    total /= static_cast<float>(ok);
    // Tilt-robust: subtract gravity along the measured direction so
    // accel_bias holds sensor offset only at any rest tilt and readAccel preserves gravity.
    const float n = sqrtf(total.x * total.x + total.y * total.y + total.z * total.z);
    if (n < 1e-9f) {
      return;
    }
    const float s = GRAVITY_MPS2 / n;
    accel_bias.x = total.x - total.x * s;
    accel_bias.y = total.y - total.y * s;
    accel_bias.z = total.z - total.z * s;
  }
}

bool Mpu6050::writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}

bool Mpu6050::readBurst6(uint8_t baseReg, float scale, vec3 &out) {
  Wire.beginTransmission(addr);
  Wire.write(baseReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(addr, static_cast<uint8_t>(6)) != 6) {
    return false;
  }
  const uint8_t xh = Wire.read();
  const uint8_t xl = Wire.read();
  const uint8_t yh = Wire.read();
  const uint8_t yl = Wire.read();
  const uint8_t zh = Wire.read();
  const uint8_t zl = Wire.read();
  const int16_t xr =
      static_cast<int16_t>((static_cast<uint16_t>(xh) << 8) | xl);
  const int16_t yr =
      static_cast<int16_t>((static_cast<uint16_t>(yh) << 8) | yl);
  const int16_t zr =
      static_cast<int16_t>((static_cast<uint16_t>(zh) << 8) | zl);
  vec3 tmp{static_cast<float>(xr) * scale, static_cast<float>(yr) * scale,
           static_cast<float>(zr) * scale};
  out = tmp;
  return true;
}
