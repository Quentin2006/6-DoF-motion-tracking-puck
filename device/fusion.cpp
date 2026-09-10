#include <Arduino.h>

#include "config.h"
#include "fusion.h"
#include "madgwick.h"

namespace {
constexpr float DPS_TO_RPS = PI / 180.0f;
vec3 gyro_bias{ 0, 0, 0 };
}  // namespace

void fusion_init() {
  q_est.q1 = 1.0f;
  q_est.q2 = q_est.q3 = q_est.q4 = 0.0f;
}

void fusion_calibrate() {
  Serial.println("Calibrating gyro, hold still...");
  vec3 sum{ 0, 0, 0 };
  int ok = 0;
  const int max_attempts = 2 * GYRO_CALIB_SAMPLES;
  for (int i = 0; i < max_attempts && ok < GYRO_CALIB_SAMPLES; ++i) {
    vec3 s;
    if (read_gyro_sample(s)) {
      sum.x += s.x;
      sum.y += s.y;
      sum.z += s.z;
      ++ok;
      if (ok % 50 == 0) {
        Serial.print("Calibrating gyro: ");
        Serial.print(ok);
        Serial.print("/");
        Serial.println(GYRO_CALIB_SAMPLES);
      }
    }
    delay(10);
  }
  if (ok == 0) {
    Serial.println("Gyro calibration failed, bias=0");
    return;
  }
  gyro_bias.x = sum.x / ok;
  gyro_bias.y = sum.y / ok;
  gyro_bias.z = sum.z / ok;
  Serial.print("Gyro calibration done: ");
  Serial.print(ok);
  Serial.print("/");
  Serial.println(GYRO_CALIB_SAMPLES);
  Serial.print("Gyro bias dps: ");
  Serial.print(gyro_bias.x);
  Serial.print(" ");
  Serial.print(gyro_bias.y);
  Serial.print(" ");
  Serial.println(gyro_bias.z);
}

void fusion_update(vec3 gyro_dps, vec3 accel) {
  // imu_filter takes gyro in rad/s; accel is normalized internally.
  gyro_dps.x -= gyro_bias.x;
  gyro_dps.y -= gyro_bias.y;
  gyro_dps.z -= gyro_bias.z;
  imu_filter(accel.x, accel.y, accel.z, gyro_dps.x * DPS_TO_RPS,
             gyro_dps.y * DPS_TO_RPS, gyro_dps.z * DPS_TO_RPS);
}

Quat fusion_quat() {
  // Vendored filter stores (w, x, y, z) as (q1, q2, q3, q4).
  return Quat{ q_est.q2, q_est.q3, q_est.q4, q_est.q1 };
}
