#include "still_detect.h"

void StillDetector::reset() {
  sum_[0] = sum_[1] = sum_[2] = 0.0f;
  sumsq_[0] = sumsq_[1] = sumsq_[2] = 0.0f;
  normSum_ = 0.0f;
  n_ = 0;
  idx_ = 0;
}

void StillDetector::push(float dx, float dy, float dz, float norm) {
  const float v[3] = {dx, dy, dz};
  if (n_ < kWinTicks) {
    for (int b = 0; b < 3; ++b) {
      ring_[idx_][b] = v[b];
      sum_[b] += v[b];
      sumsq_[b] += v[b] * v[b];
    }
    normRing_[idx_] = norm;
    normSum_ += norm;
    ++n_;
    idx_ = (idx_ + 1) % kWinTicks;
    return;
  }
  for (int b = 0; b < 3; ++b) {
    sum_[b] += v[b] - ring_[idx_][b];
    sumsq_[b] += v[b] * v[b] - ring_[idx_][b] * ring_[idx_][b];
    ring_[idx_][b] = v[b];
  }
  normSum_ += norm - normRing_[idx_];
  normRing_[idx_] = norm;
  idx_ = (idx_ + 1) % kWinTicks;
}

// Full-window still test: per-axis gyro variance + mean-norm deviation.
bool StillDetector::calm() const {
  if (n_ < kWinTicks) return false;
  const float inv = 1.0f / (float)kWinTicks;
  for (int b = 0; b < 3; ++b) {
    const float mean = sum_[b] * inv;
    const float var = sumsq_[b] * inv - mean * mean;
    if (!(var < kGyroVarDps2)) return false; // NaN-safe: NaN fails calm
  }
  const float meanNorm = normSum_ * inv;
  const float dev = fabsf(meanNorm - GRAVITY_MPS2) / GRAVITY_MPS2;
  return dev < kNormDev;
}

// Driver-frame gyro residual from the still window: direct window mean.
// The window stores driver-frame gyro and nudgeGyroBias expects driver-frame
// bias, so no remap round-trip is needed.
void StillDetector::meanDps(vec3 &out) const {
  const float inv = 1.0f / (float)kWinTicks;
  out = {sum_[0] * inv, sum_[1] * inv, sum_[2] * inv};
}

bool StillDetector::update(float gx, float gy, float gz, float norm) {
  const float omegaMag = sqrtf(gx * gx + gy * gy + gz * gz);
  const float normDev = fabsf(norm - GRAVITY_MPS2) / GRAVITY_MPS2;
  if (latched_ && (omegaMag > kExitDps || normDev > kExitNorm)) {
    latched_ = false;
    reset();
    cooldown_ = kCooldown;
    justExited_ = true;
    return false;
  }
  push(gx, gy, gz, norm);
  bool nudge = latched_;
  if (!latched_) {
    if (cooldown_ > 0) {
      --cooldown_;
    } else if (calm()) {
      latched_ = true;
      nudge = true;
    }
  }
  justExited_ = false;
  return nudge;
}
