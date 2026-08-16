#include "HardwareSerial.h"
#include <Arduino.h>
#include <Wire.h>
constexpr int SDA_PIN = 5;
constexpr int SCL_PIN = 4;

const int MPU_ADDR = 0x68;
const int MPU_PWR_MGMT_1_REG = 0x6B;
const int MPU_ACCEL_XOUT_REG = 0x3B;
const int MPU_ACCEL_YOUT_REG = 0x3D;
const int MPU_ACCEL_ZOUT_REG = 0x3F;
const int MPU_GYRO_XOUT_REG = 0x43;
const int MPU_GYRO_YOUT_REG = 0x45;
const int MPU_GYRO_ZOUT_REG = 0x47;

void wake_mpu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT_1_REG);
  Wire.write(0x00); // Wake the MPU6050.
}

void read_data(size_t registerAddress) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(registerAddress);
  if (Wire.endTransmission(false) != 0) {
    Serial.println("MPU register select failed");
    return;
  }

  if (Wire.requestFrom(MPU_ADDR, 2) != 2) {
    Serial.println("MPU read failed");
    return;
  }

  const uint8_t high = Wire.read();
  const uint8_t low = Wire.read();

  // swap
  const int16_t data =
      static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);

  Serial.print(data);
  Serial.print(" ");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // SDA = GPIO 5, SCL = GPIO 4.
  delay(100);
  wake_mpu();
}

void loop() {
  Serial.println("ACCEL: ");
  read_data(MPU_ACCEL_XOUT_REG);
  read_data(MPU_ACCEL_YOUT_REG);
  read_data(MPU_ACCEL_ZOUT_REG);
  Serial.println();

  Serial.println("GYRO: ");
  read_data(MPU_GYRO_XOUT_REG);
  read_data(MPU_GYRO_YOUT_REG);
  read_data(MPU_GYRO_ZOUT_REG);
  Serial.println();
}
