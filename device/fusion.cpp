#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "imu.h"
#include "madgwick.h"
#include "proto.h"

namespace {
constexpr float DPS_TO_RPS = PI / 180.0f;
constexpr float STILL_GYRO_DPS = 1.5f;
constexpr float STILL_ACCEL_TOL_MPS2 = 0.50f; // uncalibrated MPU6050 zero-g offset is tens of mg; pure rotation keeps |a|~=g so the gyro half still catches rotation
constexpr float STILL_BIAS_ADAPT = 0.002f; // leaky-average gain, MUST stay << 1 (k>=1 overshoots, k=2 never converges)
constexpr float GYRO_CALIB_REJECT_DPS = 8.0f; // samples shaking harder than this never enter the bias mean
constexpr float DT = 0.01f; // fixed 100 Hz scheduler in loop()

vec3 gyro_bias{0, 0, 0};
float rest_gravity = GRAVITY_MPS2;
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
  rest_gravity = GRAVITY_MPS2;
}

void fusion_calibrate() {
  Serial.println("Calibrating gyro, hold still...");
  neopixelWrite(RGB_LED_PIN, 255, 0, 0); // red while calibrating
  vec3 sum{0, 0, 0};
  int ok = 0;
  int discards = 0;
  const int max_attempts = 2 * GYRO_CALIB_SAMPLES;
  for (int i = 0; i < max_attempts && ok < GYRO_CALIB_SAMPLES; ++i) {
    vec3 g, a;
    if (imu_read(g, a)) {
      const float norm =
          sqrtf(g.x * g.x + g.y * g.y + g.z * g.z);
      if (norm > GYRO_CALIB_REJECT_DPS) {
        ++discards; // held while moving: never let motion into the bias mean
      } else {
        sum.x += g.x;
        sum.y += g.y;
        sum.z += g.z;
        ++ok;
        if (ok % 50 == 0)
          print_progress("Calibrating gyro", ok);
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
  if (discards > 0) {
    Serial.print("Gyro calibration discarded ");
    Serial.print(discards);
    Serial.println(" shaky samples");
  }
  print_progress("Gyro calibration done", ok);
  Serial.print("Gyro bias dps: ");
  Serial.print(gyro_bias.x);
  Serial.print(" ");
  Serial.print(gyro_bias.y);
  Serial.print(" ");
  Serial.println(gyro_bias.z);
  // Stationary accel magnitude: the still gate compares |a| to resting
  // gravity, and uncalibrated MPU6050 zero-g offset shifts it. Capture the
  // mean here so a gate that can never engage is impossible.
  {
    double mag_sum = 0;
    int mag_ok = 0;
    for (int i = 0; i < 100; ++i) {
      vec3 g, a;
      if (imu_read(g, a)) {
        mag_sum += sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
        ++mag_ok;
      }
      delay(10);
    }
    Serial.print("Accel mag while still: ");
    if (mag_ok > 0) {
      rest_gravity = static_cast<float>(mag_sum / mag_ok);
      Serial.print(rest_gravity, 3);
      Serial.print(" m/s2 (still gate needs +- ");
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
  if (!imu_read(gyro_dps, accel)) {
    if (++sample_fails % 100 == 1) {
      Serial.print("IMU read failed x");
      Serial.println(sample_fails);
    }
    return; // keep last quat; no orientation change on bad read
  }
  const float accel_mag =
      sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
  if (!isfinite(accel.x + accel.y + accel.z) || accel_mag < 1e-6f ||
      !isfinite(gyro_dps.x + gyro_dps.y + gyro_dps.z)) {
    if (++sample_fails % 100 == 1) {
      Serial.print("IMU sample rejected x");
      Serial.println(sample_fails);
    }
    return; // never feed NaN/degenerate vectors into the filter
  }
  const vec3 raw = gyro_dps;
  // imu_filter takes gyro in rad/s; accel is normalized internally.
  gyro_dps.x -= gyro_bias.x;
  gyro_dps.y -= gyro_bias.y;
  gyro_dps.z -= gyro_bias.z;
  const float gyro_norm_dps =
      sqrtf(gyro_dps.x * gyro_dps.x + gyro_dps.y * gyro_dps.y +
            gyro_dps.z * gyro_dps.z);
  if (gyro_norm_dps < STILL_GYRO_DPS &&
      fabsf(accel_mag - rest_gravity) < STILL_ACCEL_TOL_MPS2) {
    // Still this sample: don't integrate residual gyro. Yaw is unobservable
    // without a magnetometer, so any residual bias integrates into drift.
    // Zero the rate but still run the filter so accel levels pitch/roll from
    // any attitude (a frozen branch would lock in boot tilt). Track
    // temperature wander with a slow leaky average toward the raw sample.
    gyro_dps = {0, 0, 0};
    gyro_bias.x += STILL_BIAS_ADAPT * (raw.x - gyro_bias.x);
    gyro_bias.y += STILL_BIAS_ADAPT * (raw.y - gyro_bias.y);
    gyro_bias.z += STILL_BIAS_ADAPT * (raw.z - gyro_bias.z);
  }
  imu_filter(accel.x, accel.y, accel.z, gyro_dps.x * DPS_TO_RPS,
             gyro_dps.y * DPS_TO_RPS, gyro_dps.z * DPS_TO_RPS, DT);
}

Quat fusion_quat() {
  // Vendored filter stores (w, x, y, z) as (q1, q2, q3, q4).
  return Quat{q_est.q2, q_est.q3, q_est.q4, q_est.q1};
}
