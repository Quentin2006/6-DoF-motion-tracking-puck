#include <math.h>
#include "mpu6050.h"
#include "config.h"

Mpu6050::Mpu6050()
    : addr(MPU_ADDR), sda(SDA_PIN), scl(SCL_PIN),
      mpu_gyro_base(MPU_GYRO_BASE), mpu_accel_base(MPU_ACCEL_BASE),
      gyro_bias{0, 0, 0}, accel_bias{0, 0, 0} {}

bool Mpu6050::begin() {
  Wire.begin(static_cast<uint8_t>(sda), static_cast<uint8_t>(scl));

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
    // total includes 1 g of gravity: remove it from the dominant axis so
    // accel_bias holds sensor offset only and readAccel preserves gravity.
    const float ax = fabsf(total.x);
    const float ay = fabsf(total.y);
    const float az = fabsf(total.z);
    if (ax >= ay && ax >= az) {
      total.x -= (total.x >= 0 ? GRAVITY_MPS2 : -GRAVITY_MPS2);
    } else if (ay >= ax && ay >= az) {
      total.y -= (total.y >= 0 ? GRAVITY_MPS2 : -GRAVITY_MPS2);
    } else {
      total.z -= (total.z >= 0 ? GRAVITY_MPS2 : -GRAVITY_MPS2);
    }
    accel_bias = total;
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
