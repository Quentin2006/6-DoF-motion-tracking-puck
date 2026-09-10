#include <Arduino.h>

#include "config.h"
#include "fusion.h"
#include "madgwick.h"

namespace {
constexpr float DPS_TO_RPS = PI / 180.0f;
constexpr float STILL_GYRO_DPS = 1.5f;
constexpr float STILL_ACCEL_TOL_MPS2 = 0.15f;
constexpr int STILL_FREEZE_SAMPLES = 50;
constexpr float STILL_BIAS_ADAPT = 0.002f;
vec3 gyro_bias{ 0, 0, 0 };
int still_count = 0;
void print_progress(const char *label, int ok) {
  Serial.print(label);
  Serial.print(": ");
  Serial.print(ok);
  Serial.print("/");
  Serial.println(GYRO_CALIB_SAMPLES);
}
}  // namespace

void fusion_init() {
  q_est.q1 = 1.0f;
  q_est.q2 = q_est.q3 = q_est.q4 = 0.0f;
}

void fusion_calibrate() {
  Serial.println("Calibrating gyro, hold still...");
  neopixelWrite(RGB_LED_PIN, 255, 0, 0);  // red while calibrating
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
      if (ok % 50 == 0) print_progress("Calibrating gyro", ok);
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
  neopixelWrite(RGB_LED_PIN, 0, 255, 0);  // green: bias locked in
}

void fusion_update(vec3 gyro_dps, vec3 accel) {
  const vec3 raw = gyro_dps;
  // imu_filter takes gyro in rad/s; accel is normalized internally.
  gyro_dps.x -= gyro_bias.x;
  gyro_dps.y -= gyro_bias.y;
  gyro_dps.z -= gyro_bias.z;
  const float gyro_norm_dps =
      sqrtf(gyro_dps.x * gyro_dps.x + gyro_dps.y * gyro_dps.y + gyro_dps.z * gyro_dps.z);
  const float accel_mag =
      sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
  if (gyro_norm_dps < STILL_GYRO_DPS &&
      fabsf(accel_mag - GRAVITY_MPS2) < STILL_ACCEL_TOL_MPS2) {
    ++still_count;
  } else {
    still_count = 0;
  }
  if (still_count >= STILL_FREEZE_SAMPLES) {
    // Stationary: freeze orientation (zero drift) and track temperature
    // wander with a slow leaky average toward the raw gyro sample.
    gyro_bias.x += STILL_BIAS_ADAPT * (raw.x - gyro_bias.x);
    gyro_bias.y += STILL_BIAS_ADAPT * (raw.y - gyro_bias.y);
    gyro_bias.z += STILL_BIAS_ADAPT * (raw.z - gyro_bias.z);
    return;
  }
  imu_filter(accel.x, accel.y, accel.z, gyro_dps.x * DPS_TO_RPS,
             gyro_dps.y * DPS_TO_RPS, gyro_dps.z * DPS_TO_RPS);
}

Quat fusion_quat() {
  // Vendored filter stores (w, x, y, z) as (q1, q2, q3, q4).
  return Quat{ q_est.q2, q_est.q3, q_est.q4, q_est.q1 };
}
