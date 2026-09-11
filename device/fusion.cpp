#include <Arduino.h>

#include "config.h"
#include "imu.h"
#include "madgwick.h"
#include "proto.h"

namespace {
constexpr float DPS_TO_RPS = PI / 180.0f;
constexpr float STILL_GYRO_DPS = 1.5f;
constexpr float STILL_ACCEL_TOL_MPS2 = 0.50f; // uncalibrated MPU6050 zero-g offset is tens of mg; pure rotation keeps |a|~=g so the gyro half still catches rotation
constexpr int STILL_FREEZE_SAMPLES = 50;
constexpr float STILL_BIAS_ADAPT = 0.002f; // leaky-average gain, MUST stay << 1 (k>=1 overshoots, k=2 never converges)

vec3 gyro_bias{0, 0, 0};
int still_count = 0;
uint32_t sample_fails = 0;

void print_progress(const char *label, int ok) {
  Serial.print(label);
  Serial.print(": ");
  Serial.print(ok);
  Serial.print("/");
  Serial.println(GYRO_CALIB_SAMPLES);
}
} // namespace

void fusion_init() {
  q_est.q1 = 1.0f;
  q_est.q2 = q_est.q3 = q_est.q4 = 0.0f;
  gyro_bias = {0, 0, 0};
  still_count = 0;
}

void fusion_calibrate() {
  Serial.println("Calibrating gyro, hold still...");
  neopixelWrite(RGB_LED_PIN, 255, 0, 0); // red while calibrating
  vec3 sum{0, 0, 0};
  int ok = 0;
  const int max_attempts = 2 * GYRO_CALIB_SAMPLES;
  for (int i = 0; i < max_attempts && ok < GYRO_CALIB_SAMPLES; ++i) {
    vec3 s;
    if (read_gyro_sample(s)) {
      sum.x += s.x;
      sum.y += s.y;
      sum.z += s.z;
      ++ok;
      if (ok % 50 == 0)
        print_progress("Calibrating gyro", ok);
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
  print_progress("Gyro calibration done", ok);
  Serial.print("Gyro bias dps: ");
  Serial.print(gyro_bias.x);
  Serial.print(" ");
  Serial.print(gyro_bias.y);
  Serial.print(" ");
  Serial.println(gyro_bias.z);
  // Stationary accel magnitude: the still gate compares |a| to gravity, and
  // uncalibrated MPU6050 zero-g offset shifts it. Print it so a gate that
  // can never engage is visible in the log.
  {
    double mag_sum = 0;
    int mag_ok = 0;
    for (int i = 0; i < 100; ++i) {
      vec3 a;
      if (read_accel_sample(a)) {
        mag_sum += sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
        ++mag_ok;
      }
      delay(10);
    }
    Serial.print("Accel mag while still: ");
    if (mag_ok > 0) {
      Serial.print(mag_sum / mag_ok, 3);
      Serial.print(" m/s2 (still gate needs 9.807 +- ");
      Serial.print(STILL_ACCEL_TOL_MPS2, 2);
      Serial.println(")");
    } else {
      Serial.println("no accel samples");
    }
  }
  neopixelWrite(RGB_LED_PIN, 0, 255, 0); // green: bias locked in
}

void fusion_update() {
  vec3 gyro_dps, accel;
  if (!read_gyro_sample(gyro_dps) || !read_accel_sample(accel)) {
    if (++sample_fails % 100 == 1) {
      Serial.print("IMU read failed x");
      Serial.println(sample_fails);
    }
    return; // keep last quat; no orientation change on bad read
  }
  static uint32_t prev_us = 0;
  const uint32_t now_us = micros();
  float dt;
  if (prev_us == 0) {
    dt = 0.01f; // first call after calibrate: skip the huge gap
  } else {
    dt = (now_us - prev_us) / 1000000.0f; // unsigned diff: micros() wrap-safe
    if (dt > 0.05f)
      dt = 0.01f;
    else if (dt < 0.0005f)
      dt = 0.0005f;
  }
  prev_us = now_us;
  const vec3 raw = gyro_dps;
  // imu_filter takes gyro in rad/s; accel is normalized internally.
  gyro_dps.x -= gyro_bias.x;
  gyro_dps.y -= gyro_bias.y;
  gyro_dps.z -= gyro_bias.z;
  const float gyro_norm_dps =
      sqrtf(gyro_dps.x * gyro_dps.x + gyro_dps.y * gyro_dps.y +
            gyro_dps.z * gyro_dps.z);
  const float accel_mag =
      sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
  if (gyro_norm_dps < STILL_GYRO_DPS &&
      fabsf(accel_mag - GRAVITY_MPS2) < STILL_ACCEL_TOL_MPS2) {
    ++still_count;
    // Still this sample: don't integrate residual gyro. Yaw is unobservable
    // without a magnetometer, so any residual bias integrates into drift;
    // accel still levels pitch/roll below. Track temperature wander with a
    // slow leaky average toward the raw gyro sample.
    gyro_dps = {0, 0, 0};
    gyro_bias.x += STILL_BIAS_ADAPT * (raw.x - gyro_bias.x);
    gyro_bias.y += STILL_BIAS_ADAPT * (raw.y - gyro_bias.y);
    gyro_bias.z += STILL_BIAS_ADAPT * (raw.z - gyro_bias.z);
  } else {
    still_count = 0;
  }
  if (still_count >= STILL_FREEZE_SAMPLES) {
    return; // stationary: freeze orientation, zero drift
  }
  imu_filter(accel.x, accel.y, accel.z, gyro_dps.x * DPS_TO_RPS,
             gyro_dps.y * DPS_TO_RPS, gyro_dps.z * DPS_TO_RPS, dt);
}

Quat fusion_quat() {
  // Vendored filter stores (w, x, y, z) as (q1, q2, q3, q4).
  return Quat{q_est.q2, q_est.q3, q_est.q4, q_est.q1};
}
