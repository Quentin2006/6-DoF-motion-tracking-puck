#include "HardwareSerial.h"
#include <Arduino.h>
#include <Wire.h>

struct vec3 {
  int x;
  int y;
  int z;
};

struct vec3_f {
  float x;
  float y;
  float z;

  // Overload the += operator
  vec3_f &operator+=(const vec3_f &rhs) {
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

// MPU6050 power-on defaults: ±2 g accelerometer and ±250 °/s gyroscope.
constexpr float ACCEL_LSB_TO_MPS2 = 9.80665f / 16384.0f;
constexpr float GYRO_LSB_TO_DPS = 1.0f / 131.0f;

vec3_f cur_gyro = {0, 0, 0};
vec3_f gyro_bias = {0, 0, 0};
unsigned long last_gyro_us = 0;

constexpr size_t GYRO_CALIBRATION_SAMPLES = 100;
constexpr unsigned long GYRO_CALIBRATION_DELAY_MS = 5;

void wake_mpu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT_1_REG);
  Wire.write(0x00); // Wake the MPU6050.
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
bool read_gyro_sample(vec3_f &sample) {
  sample = {read_data(MPU_GYRO_REGS.x, GYRO_LSB_TO_DPS),
            read_data(MPU_GYRO_REGS.y, GYRO_LSB_TO_DPS),
            read_data(MPU_GYRO_REGS.z, GYRO_LSB_TO_DPS)};

  return !isnan(sample.x) && !isnan(sample.y) && !isnan(sample.z);
}

void calibrate_gyro_bias() {
  Serial.println("GYRO calibration: keep puck stationary");

  vec3_f gyro_sum = {0, 0, 0};
  size_t valid_samples = 0;
  size_t invalid_samples = 0;

  while (valid_samples < GYRO_CALIBRATION_SAMPLES) {
    vec3_f sample;
    if (read_gyro_sample(sample)) {
      gyro_sum += sample;
      ++valid_samples;
    } else {
      ++invalid_samples;
    }
    delay(GYRO_CALIBRATION_DELAY_MS);
  }

  gyro_bias = {gyro_sum.x / GYRO_CALIBRATION_SAMPLES,
               gyro_sum.y / GYRO_CALIBRATION_SAMPLES,
               gyro_sum.z / GYRO_CALIBRATION_SAMPLES};
  Serial.printf("GYRO calibration complete: bias %.4f %.4f %.4f deg/s; "
                "%lu valid, %lu invalid\n",
                gyro_bias.x, gyro_bias.y, gyro_bias.z,
                static_cast<unsigned long>(valid_samples),
                static_cast<unsigned long>(invalid_samples));
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // SDA = GPIO 5, SCL = GPIO 4.
  delay(100);
  wake_mpu();
  calibrate_gyro_bias();
  last_gyro_us = micros();
  Serial.println();
}

void loop() {
  // Gyro readings are angular velocity in deg/s; multiply by elapsed seconds
  // to integrate a relative angle. Residual noise and bias still cause drift.
  vec3_f gyro_sample;
  if (read_gyro_sample(gyro_sample)) {
    const unsigned long now_us = micros();
    const float dt = (now_us - last_gyro_us) / 1000000.0f;
    last_gyro_us = now_us;

    gyro_sample.x -= gyro_bias.x;
    gyro_sample.y -= gyro_bias.y;
    gyro_sample.z -= gyro_bias.z;
    cur_gyro.x += gyro_sample.x * dt;
    cur_gyro.y += gyro_sample.y * dt;
    cur_gyro.z += gyro_sample.z * dt;
  }

  Serial.printf("%.2f %.2f %.2f\r", cur_gyro.x, cur_gyro.y, cur_gyro.z);
}
