#include "HardwareSerial.h"
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

struct vec4 {
  float w;
  float x;
  float y;
  float z;

  // Overload the += operator
  vec4 &operator+=(const vec4 &rhs) {
    this->w += rhs.w;
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

vec3 cur_gyro = {0, 0, 0};
vec3 gyro_bias = {0, 0, 0};
unsigned long last_us = 0;

vec3 cur_pos = {0, 0, 0};

constexpr size_t GYRO_CALIBRATION_SAMPLES = 1000;
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

vec4 rpy_to_quaternion(float roll, float pitch, float yaw) {
  // math shit for conversion
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);

  float w = cy * cp * cr + sy * sp * sr;
  float x = cy * cp * sr - sy * sp * cr;
  float y = cy * sp * cr + sy * cp * sr;
  float z = sy * cp * cr - cy * sp * sr;
  return vec4{w, x, y, z};
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // SDA = GPIO 5, SCL = GPIO 4.
  delay(100);
  wake_mpu();
  last_us = micros();
  Serial.println();
}

void loop() {
  // want to retun the relative world pos
  //
  //
  // we need to get the delta time for future calulations
  const unsigned long now_us = micros();
  const float dt = (now_us - last_us) / 1000000.0f;
  last_us = now_us;

  // #1 get gyro info
  vec3 gyro_sample;
  if (read_gyro_sample(gyro_sample)) {
    // gyro is in deg/sec, so we need to mulitpy by seconds elapsed to get the
    // delta deg
    // TODO: remove bias
    cur_gyro.x += gyro_sample.x * dt;
    cur_gyro.y += gyro_sample.y * dt;
    cur_gyro.z += gyro_sample.z * dt;
  }

  // #2 get acceleration sample (m/s^2)
  vec3 accel_sample;
  if (read_accel_sample(accel_sample)) {
    // TODO: remove bias
  };

  // #3 rotate with quaternion
  vec4 q = rpy_to_quaternion(cur_gyro.x, cur_gyro.y, cur_gyro.z);
  vec3 u = {q.x, q.y, q.z};
  float s = q.w;

  // Calculate temporary cross product: u x v
  vec3 u_cross_v = {u.y * accel_sample.z - u.z * accel_sample.y,
                    u.z * accel_sample.x - u.x * accel_sample.z,
                    u.x * accel_sample.y - u.y * accel_sample.x};

  // Calculate: u x (u x v + s*v)
  vec3 internal_part = {u_cross_v.x + s * accel_sample.x,
                        u_cross_v.y + s * accel_sample.y,
                        u_cross_v.z + s * accel_sample.z};

  vec3 u_cross_internal = {u.y * internal_part.z - u.z * internal_part.y,
                           u.z * internal_part.x - u.x * internal_part.z,
                           u.x * internal_part.y - u.y * internal_part.x};

  // Final vector: v + 2 * (u x (u x v + s*v))
  vec3 rotated_accel = {accel_sample.x + 2.0f * u_cross_internal.x,
                        accel_sample.y + 2.0f * u_cross_internal.y,
                        accel_sample.z + 2.0f * u_cross_internal.z};

  // #4 Subtract 9.81 from world z.
  rotated_accel.z -= 9.81;

  // #5 double integrate acceleration(m/s^2) to get m
  vec3 delta_pos{
      0.5f * rotated_accel.x * dt * dt,
      0.5f * rotated_accel.y * dt * dt,
      0.5f * rotated_accel.z * dt * dt,
  };

  cur_pos += delta_pos;

  Serial.printf("%f %f %f\n", cur_pos.x, cur_pos.y, cur_pos.z);
}
