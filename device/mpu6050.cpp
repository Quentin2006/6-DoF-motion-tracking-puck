#include "mpu6050.h"
#include "config.h"

Mpu6050::Mpu6050()
    : addr(MPU_ADDR), sda(SDA_PIN), scl(SCL_PIN), mpu_gyro_base(MPU_GYRO_BASE),
      mpu_accel_base(MPU_ACCEL_BASE) {
  // wake the mpu
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT_1_REG);
  Wire.write(0x00); // Wake the MPU6050.
  Wire.endTransmission(true);

  now_gyro_us = now_accel_us = micros();
}

bool Mpu6050::readGyro(vec3 &sample) {
  now_gyro_us = micros();
  const float dt = (now_gyro_us - last_gyro_us) / 1000000.0f;
  last_gyro_us = now_gyro_us;

  sample = {read_data(mpu_gyro_base, GYRO_LSB_TO_DPS) * dt,
            read_data(mpu_gyro_base + 0x2, GYRO_LSB_TO_DPS) * dt,
            read_data(mpu_gyro_base + 0x4, GYRO_LSB_TO_DPS) * dt};

  return !isnan(sample.x) && !isnan(sample.y) && !isnan(sample.z);
}

bool Mpu6050::readAccel(vec3 &sample) {
  now_accel_us = micros();
  const float dt = (now_accel_us - last_accel_us) / 1000000.0f;
  last_accel_us = now_accel_us;

  sample = {read_data(mpu_accel_base, ACCEL_LSB_O_MPS2) * dt,
            read_data(mpu_accel_base + 0x2, ACCEL_LSB_O_MPS2) * dt,
            read_data(mpu_accel_base + 0x4, ACCEL_LSB_O_MPS2) * dt};

  return !isnan(sample.x) && !isnan(sample.y) && !isnan(sample.z);
}

float Mpu6050::read_data(size_t registerAddress, float scale) {
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
  return static_cast<float>(data);
}
