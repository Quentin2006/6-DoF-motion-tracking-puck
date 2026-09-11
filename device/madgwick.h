// Vendored from https://github.com/bjohnsonfl/Madgwick_Filter (MIT License,
// Copyright (c) Blake Johnson). Adapted for Arduino/ESP32: dropped the
// stdio debug helper and the unused Euler-angle helper, and imu_filter takes
// the measured sample interval (the loop free-spins; no fixed rate exists).

#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#ifndef PI
#define PI 3.14159265358979f
#endif

#ifndef GYRO_MEAN_ERROR
#define GYRO_MEAN_ERROR                                                        \
  PI * (5.0f / 180.0f) // 5 deg/s gyro error (rad/s), from paper
#endif

#ifndef BETA
#define BETA sqrt(3.0f / 4.0f) * GYRO_MEAN_ERROR // from paper
#endif

#include <math.h>

struct quaternion {
  float q1;
  float q2;
  float q3;
  float q4;
};

// Global orientation estimate: (w, x, y, z) = (q1, q2, q3, q4).
extern struct quaternion q_est;

// Multiply two quaternions and return a copy of the result, prod = L * R
struct quaternion quat_mult(struct quaternion q_L, struct quaternion q_R);

// Multiply a reference of a quaternion by a scalar, q = s*q
static inline void quat_scalar(struct quaternion *q, float scalar) {
  q->q1 *= scalar;
  q->q2 *= scalar;
  q->q3 *= scalar;
  q->q4 *= scalar;
}

// Adds two quaternions, Sum = L + R
static inline void quat_add(struct quaternion *Sum, struct quaternion L,
                            struct quaternion R) {
  Sum->q1 = L.q1 + R.q1;
  Sum->q2 = L.q2 + R.q2;
  Sum->q3 = L.q3 + R.q3;
  Sum->q4 = L.q4 + R.q4;
}

// Subtracts two quaternions, Sum = L - R
static inline void quat_sub(struct quaternion *Sum, struct quaternion L,
                            struct quaternion R) {
  Sum->q1 = L.q1 - R.q1;
  Sum->q2 = L.q2 - R.q2;
  Sum->q3 = L.q3 - R.q3;
  Sum->q4 = L.q4 - R.q4;
}

// Norm: sqrt(q1^2 + q2^2 + q3^2 + q4^2)
static inline float quat_Norm(struct quaternion q) {
  return sqrt(q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3 + q.q4 * q.q4);
}

// Normalizes *q in place.
static inline void quat_Normalization(struct quaternion *q) {
  float norm = quat_Norm(*q);
  q->q1 /= norm;
  q->q2 /= norm;
  q->q3 /= norm;
  q->q4 /= norm;
}

// IMU (gyro + accel) sensor fusion. Result accumulates in q_est.
// Gyro components in radians per second; accel is normalized internally.
// dt is the measured sample interval in seconds.
void imu_filter(float ax, float ay, float az, float gx, float gy, float gz,
                float dt);

#endif /* MADGWICK_FILTER_H */
