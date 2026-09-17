#pragma once
#include "config.h"
#include "mpu6050.h"   // vec3
#include <math.h>

class StillDetector {
public:
  // Returns true if a bias nudge is warranted this tick (latched and still, or
  // just entered still). On a nudge-return, caller should call meanDps() and
  // nudgeGyroBias(mean, alpha()).
  bool update(float gx, float gy, float gz, float norm);
  void reset();
  bool latched() const { return latched_; }
  bool justExited() const { return justExited_; }
  vec3 meanDps() const;                 // window mean, driver frame
  float alpha() const { return kAlpha; }

private:
  void push(float dx, float dy, float dz, float norm);
  bool calm() const;

  static constexpr unsigned kWinTicks = 50;
  static constexpr float kGyroVarDps2 = 0.25f;
  static constexpr float kNormDev = 0.02f;
  static constexpr float kAlpha = 0.02f;
  static constexpr float kExitDps = 1.5f;
  static constexpr float kExitNorm = 0.03f;
  static constexpr unsigned kCooldown = 25;

  float ring_[kWinTicks][3];
  float normRing_[kWinTicks];
  float sum_[3] = {0, 0, 0}, sumsq_[3] = {0, 0, 0};
  float normSum_ = 0.0f;
  unsigned n_ = 0, idx_ = 0, cooldown_ = 0;
  bool latched_ = false, justExited_ = false;
};
