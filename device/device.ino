#include <Arduino.h>

#include "config.h"
#include "fusion.h"
#include "imu.h"
#include "net.h"
#include "proto.h"

Quat data{ 0, 0, 0, 1 };

void setup() {
  Serial.begin(115200);
  Serial.flush();
  imu_init();
  Serial.println("Mpu awake");
  fusion_calibrate();
  fusion_init();
  net_connect();
  Serial.println("UDP ready");
}

void loop() {
  static uint32_t next = 0;
  static constexpr uint32_t PERIOD_US = SEND_INTERVAL_MS * 1000;
  static uint32_t sample_fails = 0;
  if (micros() < next) return;  // SEND_INTERVAL_MS grid; must match DELTA_T in madgwick.h
  next += PERIOD_US;
  if (micros() - next > PERIOD_US) next = micros();
  vec3 gyro, accel;
  if (!read_gyro_sample(gyro) || !read_accel_sample(accel)) {
    if (++sample_fails % 100 == 1) {
      Serial.print("IMU read failed x");
      Serial.println(sample_fails);
    }
  } else {
    fusion_update(gyro, accel);
    data = fusion_quat();
  }
  net_send_quat(data);
}
