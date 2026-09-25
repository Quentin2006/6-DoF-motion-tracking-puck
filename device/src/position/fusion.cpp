#include "fusion.h"

#include <math.h>

#include "../../config.h"

void Ekf6Dof::begin(const vec3 &bg0) {
  q0_ = 1;
  q1_ = 0;
  q2_ = 0;
  q3_ = 0;
  bg_[0] = bg0.x;
  bg_[1] = bg0.y;
  bg_[2] = bg0.z;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      P_[i][j] = 0;
    }
  }
  P_[0][0] = EKF_P_ATT_INIT;
  P_[1][1] = EKF_P_ATT_INIT;
  P_[2][2] = EKF_P_ATT_INIT;
  P_[3][3] = EKF_P_BG_INIT;
  P_[4][4] = EKF_P_BG_INIT;
  P_[5][5] = EKF_P_BG_INIT;
}

void Ekf6Dof::predict(float gx, float gy, float gz, float dt) {
  if (dt <= 0) {
    return;
  }

  // Bias-corrected rate.
  const float wx = gx - bg_[0];
  const float wy = gy - bg_[1];
  const float wz = gz - bg_[2];

  // Exact rotation-vector integration: q *= dq.
  const float omega = sqrtf(wx * wx + wy * wy + wz * wz);
  float dq0 = 1, dq1 = 0, dq2 = 0, dq3 = 0;
  if (omega > 1e-12f) {
    const float half = omega * dt * 0.5f;
    dq0 = cosf(half);
    const float s = sinf(half) / omega;
    dq1 = wx * s;
    dq2 = wy * s;
    dq3 = wz * s;
  }
  const float nq0 = q0_ * dq0 - q1_ * dq1 - q2_ * dq2 - q3_ * dq3;
  const float nq1 = q0_ * dq1 + q1_ * dq0 + q2_ * dq3 - q3_ * dq2;
  const float nq2 = q0_ * dq2 - q1_ * dq3 + q2_ * dq0 + q3_ * dq1;
  const float nq3 = q0_ * dq3 + q1_ * dq2 - q2_ * dq1 + q3_ * dq0;
  const float nq = sqrtf(nq0 * nq0 + nq1 * nq1 + nq2 * nq2 + nq3 * nq3);
  if (nq > 1e-12f) {
    q0_ = nq0 / nq;
    q1_ = nq1 / nq;
    q2_ = nq2 / nq;
    q3_ = nq3 / nq;
  }

  // Covariance: F = [[I - skew(w)*dt, -I*dt],[0, I]].
  float F[6][6] = {0};
  // Top-left A = I - skew(w)*dt.
  F[0][0] = 1;
  F[0][1] = wz * dt;
  F[0][2] = -wy * dt;
  F[1][0] = -wz * dt;
  F[1][1] = 1;
  F[1][2] = wx * dt;
  F[2][0] = wy * dt;
  F[2][1] = -wx * dt;
  F[2][2] = 1;
  // Top-right B = -I*dt.
  F[0][3] = -dt;
  F[1][4] = -dt;
  F[2][5] = -dt;
  // Bottom-right identity.
  F[3][3] = 1;
  F[4][4] = 1;
  F[5][5] = 1;

  const float qg = EKF_SIGMA_GYRO * EKF_SIGMA_GYRO * dt;
  const float qb = EKF_SIGMA_BG * EKF_SIGMA_BG * dt;

  // P = F*P*F' + Q via T = F*P, Pnew = T*F'.
  float T[6][6] = {0};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      float s = 0;
      for (int k = 0; k < 6; ++k) {
        s += F[i][k] * P_[k][j];
      }
      T[i][j] = s;
    }
  }
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      float s = 0;
      for (int k = 0; k < 6; ++k) {
        s += T[i][k] * F[j][k];
      }
      P_[i][j] = s;
    }
  }
  P_[0][0] += qg;
  P_[1][1] += qg;
  P_[2][2] += qg;
  P_[3][3] += qb;
  P_[4][4] += qb;
  P_[5][5] += qb;
}

bool Ekf6Dof::updateAccel(float ax, float ay, float az) {
  const float n = sqrtf(ax * ax + ay * ay + az * az);
  if (n == 0) {
    return false;
  }
  const float normDev = fabsf(n - GRAVITY_MPS2) / GRAVITY_MPS2;
  // Gate 1: far reject only -> coast on gyro; soft R deweight handles the rest below.
  if (normDev > EKF_GATE_NORM_FAR) {
    return false;
  }

  const float anx = ax / n;
  const float any = ay / n;
  const float anz = az / n;

  // Predicted body-gravity g_b = R(q)'*[0,0,1] = third row of R(q).
  const float w = q0_, x = q1_, y = q2_, z = q3_;
  const float gbx = 2 * (x * z - w * y);
  const float gby = 2 * (y * z + w * x);
  const float gbz = 1 - 2 * (x * x + y * y);

  // Innovation y = a_n - g_b.
  const float yv[3] = {anx - gbx, any - gby, anz - gbz};

  // H = [+skew(g_b), 0_3x3].
  float Hs[3][3];
  Hs[0][0] = 0;
  Hs[0][1] = -gbz;
  Hs[0][2] = gby;
  Hs[1][0] = gbz;
  Hs[1][1] = 0;
  Hs[1][2] = -gbx;
  Hs[2][0] = -gby;
  Hs[2][1] = gbx;
  Hs[2][2] = 0;

  // S = Hs*Ptt*Hs' + sa^2*I, Ptt = P_[0..2][0..2].
  float HP[3][3] = {0}; // Hs * Ptt
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      float s = 0;
      for (int k = 0; k < 3; ++k) {
        s += Hs[i][k] * P_[k][j];
      }
      HP[i][j] = s;
    }
  }
  float r = normDev / EKF_GATE_NORM;
  float ra = EKF_SIGMA_ACCEL * EKF_SIGMA_ACCEL * (1.0f + EKF_GATE_SOFT_K * r * r);
  float S[3][3] = {0};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      float s = 0;
      for (int k = 0; k < 3; ++k) {
        s += HP[i][k] * Hs[j][k];
      }
      S[i][j] = s;
    }
    S[i][i] += ra;
  }

  // Closed-form 3x3 inverse.
  const float a00 = S[0][0], a01 = S[0][1], a02 = S[0][2];
  const float a10 = S[1][0], a11 = S[1][1], a12 = S[1][2];
  const float a20 = S[2][0], a21 = S[2][1], a22 = S[2][2];
  const float c00 = a11 * a22 - a12 * a21;
  const float c01 = a12 * a20 - a10 * a22;
  const float c02 = a10 * a21 - a11 * a20;
  const float det = a00 * c00 + a01 * c01 + a02 * c02;
  if (det == 0) {
    return false;
  }
  const float inv = 1.0f / det;
  float Sinv[3][3];
  Sinv[0][0] = c00 * inv;
  Sinv[0][1] = (a02 * a21 - a01 * a22) * inv;
  Sinv[0][2] = (a01 * a12 - a02 * a11) * inv;
  Sinv[1][0] = c01 * inv;
  Sinv[1][1] = (a00 * a22 - a02 * a20) * inv;
  Sinv[1][2] = (a02 * a10 - a00 * a12) * inv;
  Sinv[2][0] = c02 * inv;
  Sinv[2][1] = (a01 * a20 - a00 * a21) * inv;
  Sinv[2][2] = (a00 * a11 - a01 * a10) * inv;

  // Gate 2: Mahalanobis chi2 3-dof.
  float maha = 0;
  for (int i = 0; i < 3; ++i) {
    float s = 0;
    for (int j = 0; j < 3; ++j) {
      s += Sinv[i][j] * yv[j];
    }
    maha += yv[i] * s;
  }
  if (maha > EKF_GATE_MAHA) {
    return false;
  }

  // K = P*H'*Sinv: M = P*H' (6x3), K = M*Sinv.
  float M[6][3] = {0};
  for (int i = 0; i < 6; ++i) {
    for (int k = 0; k < 3; ++k) {
      float s = 0;
      for (int j = 0; j < 3; ++j) {
        s += P_[i][j] * Hs[k][j];
      }
      M[i][k] = s;
    }
  }
  float K[6][3] = {0};
  for (int i = 0; i < 6; ++i) {
    for (int k = 0; k < 3; ++k) {
      float s = 0;
      for (int j = 0; j < 3; ++j) {
        s += M[i][j] * Sinv[j][k];
      }
      K[i][k] = s;
    }
  }

  // dx = K*y.
  float dx[6] = {0};
  for (int i = 0; i < 6; ++i) {
    dx[i] = K[i][0] * yv[0] + K[i][1] * yv[1] + K[i][2] * yv[2];
  }

  // Apply rotation correction: q *= quat(dx_theta/2), normalize.
  {
    const float dq0 = 1, dq1 = dx[0] * 0.5f, dq2 = dx[1] * 0.5f, dq3 = dx[2] * 0.5f;
    const float r0 = q0_ * dq0 - q1_ * dq1 - q2_ * dq2 - q3_ * dq3;
    const float r1 = q0_ * dq1 + q1_ * dq0 + q2_ * dq3 - q3_ * dq2;
    const float r2 = q0_ * dq2 - q1_ * dq3 + q2_ * dq0 + q3_ * dq1;
    const float r3 = q0_ * dq3 + q1_ * dq2 - q2_ * dq1 + q3_ * dq0;
    const float rn = sqrtf(r0 * r0 + r1 * r1 + r2 * r2 + r3 * r3);
    if (rn > 1e-12f) {
      q0_ = r0 / rn;
      q1_ = r1 / rn;
      q2_ = r2 / rn;
      q3_ = r3 / rn;
    }
  }
  // Bias unobservable when the correction is dominated by linear accel: freeze bg there.
  if (maha < EKF_BIAS_FREEZE_MAHA_FRAC * EKF_GATE_MAHA) {
    bg_[0] += dx[3];
    bg_[1] += dx[4];
    bg_[2] += dx[5];
  }
  // Joseph form: P = (I-KH)*P*(I-KH)' + K*R*K', R = ra*I_3.
  float IKH[6][6] = {0};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 3; ++j) {
      float s = 0;
      for (int k = 0; k < 3; ++k) {
        s += K[i][k] * Hs[k][j];
      }
      IKH[i][j] = -s;
    }
    IKH[i][i] += 1.0f;
  }
  float T[6][6] = {0};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      float s = 0;
      for (int l = 0; l < 6; ++l) {
        s += IKH[i][l] * P_[l][j];
      }
      T[i][j] = s;
    }
  }
  float Pn[6][6] = {0};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      float s = ra * (K[i][0] * K[j][0] + K[i][1] * K[j][1] + K[i][2] * K[j][2]);
      for (int l = 0; l < 6; ++l) {
        s += T[i][l] * IKH[j][l];
      }
      Pn[i][j] = s;
    }
  }
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      P_[i][j] = Pn[i][j];
    }
  }
  return true;
}

void Ekf6Dof::getQuat(float &x, float &y, float &z, float &w) const {
  x = q1_;
  y = q2_;
  z = q3_;
  w = q0_;
}

void Ekf6Dof::getBias(float &x, float &y, float &z) const {
  x = bg_[0];
  y = bg_[1];
  z = bg_[2];
}
