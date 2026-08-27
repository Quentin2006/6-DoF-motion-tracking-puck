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

void loop() {

  /*
  This is the datasheet im using for all info on the IMU (MPU6050):
  https://www.hestore.hu/prod_getfile.php?id=8301

  provides us both gyro (deg/sec) and accel (m/s^2) data. These two peices
  alone can provide us a decent estamate on the devices rotation.

  To get the rotaiton from the device, we cannot just integrate over gyro due
  potental inaccuracies and drift.

  So inorder to get a better estamtaion, we need to use a sensor fusion
  algorithm.

  There are a few main algos we can use that will output rotation info (via
  quaternion) though inputing both accel and gyro data.

  --------- VQF ---------
  Pros: Good accuracy, handles gyro bias pretty well, works well when there is
  some extra acceleration, and is still relatively lightweight.
  Cons: More complicated than Mahony or Madgwick, and not as flexible as making
  our own EKF.

  --------- Mahony ---------
  Pros: Very lightweight, pretty simple to implement, doesn't take much
  processing power, and would work well on an embedded system.
  Cons: Needs some tuning to get good results, can have problems when the device
  is accelerating, and isn't as advanced as VQF or an EKF.

  --------- Madgwick ---------
  Pros: Lightweight, fairly simple to implement, widely used, and there are
  already a lot of implementations we could use as a starting point.
  Cons: Still needs some tuning, can have problems when there is extra
  acceleration, and doesn't give us as much control as an EKF.

  --------- EKF ---------
  Pros: Gives us a lot more control over how we handle the sensor data, can
  account for things like sensor noise and gyro bias, and can be extended later
  if we want to add more information or constraints.
  Cons: More computationally expensive, much more complicated to implement and
  tune, and there are more things that can go wrong if we implement it
  incorrectly.

  --------- Invariant EKF (IEKF) ---------
  Pros: Has some better theoretical properties than a normal EKF, can be more
  robust and consistent, and would be good if we wanted to do a more advanced
  implementation.
  Cons: More complicated mathematically, takes more work to implement, and would
  probably be overkill for what we are trying to do.


  VQF would provide the best accuracy, but Im not choosing it. This is only due
  to the fact this algo is very complex, and I want to finish the project.
  So instead Im choosing the Madgwick filter due to its relative eaise to
  implement
   */
}
