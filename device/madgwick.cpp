// Vendored from https://github.com/bjohnsonfl/Madgwick_Filter (MIT License,
// Copyright (c) Blake Johnson). Filter body unchanged except imu_filter
// takes the fixed sample interval from the 100 Hz scheduler.

#include "madgwick.h"

struct quaternion q_est = {1, 0, 0, 0}; // unit vector, real component = 1

struct quaternion quat_mult(struct quaternion L, struct quaternion R) {
  struct quaternion product;
  product.q1 = (L.q1 * R.q1) - (L.q2 * R.q2) - (L.q3 * R.q3) - (L.q4 * R.q4);
  product.q2 = (L.q1 * R.q2) + (L.q2 * R.q1) + (L.q3 * R.q4) - (L.q4 * R.q3);
  product.q3 = (L.q1 * R.q3) - (L.q2 * R.q4) + (L.q3 * R.q1) + (L.q4 * R.q2);
  product.q4 = (L.q1 * R.q4) + (L.q2 * R.q3) - (L.q3 * R.q2) + (L.q4 * R.q1);
  return product;
}

// The resulting quaternion is a global variable (q_est), so it is not
// returned or passed by reference/pointer.
// Gyroscope Angular Velocity components are in Radians per Second.
// Accelerometer components will be normalized.
void imu_filter(float ax, float ay, float az, float gx, float gy, float gz,
                float dt) {
  if (!isfinite(ax + ay + az + gx + gy + gz) || dt <= 0 || !isfinite(dt))
    return; // NaN input or bogus interval: keep last estimate
  // Variables and constants
  struct quaternion q_est_prev = q_est;

  float F_g[3] = {0};    // equation (15/21/25) objective function
  float J_g[3][4] = {0}; // Jacobian matrix

  struct quaternion gradient = {0};

  /* Integrate angular velocity to obtain position in angles. */
  struct quaternion q_w; // equation (10): gyro readings as a quaternion
  q_w.q1 = 0;            // real component is zero, simplifies quat. mult.
  q_w.q2 = gx;
  q_w.q3 = gy;
  q_w.q4 = gz;

  quat_scalar(&q_w, 0.5);           // equation (12) dq/dt = (1/2)q*w
  q_w = quat_mult(q_est_prev, q_w); // equation (12)

  /* Gradient = J_g' * F_g (equation 20), hard-coded transpose. */
  quat_Normalization(&q_a);
  if (!isfinite(q_a.q1))
    return; // degenerate accel (all zeros): keep last estimate
  // Objective function for gravity, eq. (15) simplified to eq. (25).
  F_g[0] = 2 * (q_est_prev.q2 * q_est_prev.q4 - q_est_prev.q1 * q_est_prev.q3) -
           q_a.q2;
  F_g[1] = 2 * (q_est_prev.q1 * q_est_prev.q2 + q_est_prev.q3 * q_est_prev.q4) -
           q_a.q3;
  F_g[2] = 2 * (0.5 - q_est_prev.q2 * q_est_prev.q2 -
                q_est_prev.q3 * q_est_prev.q3) -
           q_a.q4;

  // Jacobian matrix, equation (26), for gravity.
  J_g[0][0] = -2 * q_est_prev.q3;
  J_g[0][1] = 2 * q_est_prev.q4;
  J_g[0][2] = -2 * q_est_prev.q1;
  J_g[0][3] = 2 * q_est_prev.q2;

  J_g[1][0] = 2 * q_est_prev.q2;
  J_g[1][1] = 2 * q_est_prev.q1;
  J_g[1][2] = 2 * q_est_prev.q4;
  J_g[1][3] = 2 * q_est_prev.q3;

  J_g[2][0] = 0;
  J_g[2][1] = -4 * q_est_prev.q2;
  J_g[2][2] = -4 * q_est_prev.q3;
  J_g[2][3] = 0;

  gradient.q1 = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1] + J_g[2][0] * F_g[2];
  gradient.q2 = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2];
  gradient.q3 = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2];
  gradient.q4 = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1] + J_g[2][3] * F_g[2];

  // Normalize the gradient, equation (44).
  quat_Normalization(&gradient);

  /* Sensor fusion, equations 42-44: gyro rate minus beta-scaled gradient,
     integrated over the measured sample interval. */
  quat_scalar(&gradient, BETA);
  quat_sub(&q_est_dot, q_w, gradient);
  quat_scalar(&q_est_dot, dt);
  quat_add(&q_est, q_est_prev, q_est_dot);
  quat_Normalization(&q_est);
}
