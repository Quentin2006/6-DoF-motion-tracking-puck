#include "tracker.h"

#include <Arduino.h> // Serial0, millis, micros, delay
#include <math.h>
#include <string.h>

#include "../../config.h"

// "… off x y z scale x y z" — shared by the boot report and the CAL6 solve.
static void printCal(const char *prefix, const float off[3], const float scale[3]) {
  Serial0.printf("%s off %.4f %.4f %.4f scale %.4f %.4f %.4f\n", prefix, off[0], off[1],
                off[2], scale[0], scale[1], scale[2]);
}

// Wait up to timeoutMs for one serial line and match it exactly. Returns true
// only on an exact `want` match (CAL6's "G"/"CAL6"/"SAVE" tokens).
static bool waitForToken(const char *want, unsigned long timeoutMs) {
  char buf[16];
  size_t n = 0;
  buf[0] = '\0';
  const unsigned long t0 = millis();
  while ((unsigned long)(millis() - t0) < timeoutMs) {
    while (Serial0.available() > 0 && n + 1 < sizeof buf) {
      const char c = (char)Serial0.read();
      if (c == '\n' || c == '\r') {
        if (n > 0) {
          return strcmp(buf, want) == 0;
        }
      } else {
        buf[n++] = c;
        buf[n] = '\0';
      }
    }
    delay(10);
  }
  return false;
}

// One-time serial-guided 6-point accel cal. Blocking, setup-time only. Any
// unexpected input or timeout aborts WITHOUT saving (NVS untouched).
static void runAccelCal(Mpu6050 &mpu) {
  static const char *kPose[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
  vec3 means[6];
  for (int p = 0; p < 6; ++p) {
    Serial0.printf("CAL6 pose %s down: hold still, send G\n", kPose[p]);
    if (!waitForToken("G", CAL6_TOKEN_TIMEOUT_MS)) {
      Serial0.println("CAL6 aborted, nothing saved");
      return;
    }
    // Raw mean (~200 samples @5 ms, bypassing all driver-side accel cal).
    vec3 mean;
    if (!mpu.meanRawAccel(mean)) {
      Serial0.println("CAL6 aborted, nothing saved (no samples)");
      return;
    }
    means[p] = mean;
    Serial0.printf("CAL6 mean %s %.4f %.4f %.4f\n", kPose[p], mean.x, mean.y, mean.z);
  }
  float off[3], scale[3];
  for (int i = 0; i < 3; ++i) {
    const vec3 &pa = means[2 * i];
    const vec3 &pb = means[2 * i + 1];
    const float a = i == 0 ? pa.x : (i == 1 ? pa.y : pa.z);
    const float b = i == 0 ? pb.x : (i == 1 ? pb.y : pb.z);
    const float mx = a > b ? a : b;
    const float mn = a < b ? a : b;
    if ((mx - mn) < 1e-6f) {
      Serial0.println("CAL6 aborted, nothing saved (degenerate span)");
      return;
    }
    off[i] = (mx + mn) * 0.5f;
    scale[i] = 2.0f * GRAVITY_MPS2 / (mx - mn);
  }
  printCal("CAL6 solved", off, scale);
  Serial0.println("CAL6 send SAVE to flash, anything else aborts");
  if (!waitForToken("SAVE", CAL6_TOKEN_TIMEOUT_MS)) {
    Serial0.println("CAL6 aborted, nothing saved");
    return;
  }
  mpu.setAccelCal(off, scale);
  mpu.getAccelBias(); // re-seed the runtime offset residual
  Serial0.println("CAL6 saved; reboot");
}

// Driver -> body swizzle: body=(sz,sy,sx). Sensor +X (up, ~9.8 at rest) lands
// on body +Z, matching the EKF gravity [0,0,1]; pitch/roll order verified in
// live test.
static void toBody(const vec3 &v, float out[3]) {
  out[0] = v.z;
  out[1] = v.y;
  out[2] = v.x;
}

static void quatMul(float ax, float ay, float az, float aw,
                    float bx, float by, float bz, float bw,
                    float &ox, float &oy, float &oz, float &ow) {
  ox = aw * bx + ax * bw + ay * bz - az * by;
  oy = aw * by - ax * bz + ay * bw + az * bx;
  oz = aw * bz + ax * by - ay * bx + az * bw;
  ow = aw * bw - ax * bx - ay * by - az * bz;
}

static void maybeRunAccelCal(Mpu6050 &mpu, unsigned long timeoutMs) {
  if (waitForToken("CAL6", timeoutMs)) {
    runAccelCal(mpu);
  }
}

bool Tracker::boot() {
  if (!mpu_.begin()) {
    return false;
  }
  delay(500);
  Serial0.println("calibrating...");
  mpu_.getAccelBias();
  mpu_.getGyroBias();
  Serial0.println("calibrated");
  float off[3], scale[3];
  mpu_.getAccelCal(off, scale);
  printCal("accCal", off, scale);
  Serial0.printf("send CAL6 within %lu ms for accel cal\n", CAL6_WINDOW_MS);
  maybeRunAccelCal(mpu_, CAL6_WINDOW_MS);
  vec3 gb;
  mpu_.readGyro(gb); // seed only; first loop() tick is a clean skip anyway
  ekf_.begin(vec3{gb.x * DPS_TO_RPS, gb.y * DPS_TO_RPS, gb.z * DPS_TO_RPS});
  lastTickUs_ = micros();   // first loop() tick is a clean "too-soon" skip
  lastReportMs_ = millis(); // 1 Hz status cadence starts with streaming
  return true;
}

void Tracker::report(uint32_t seq) {
  const unsigned long now = millis();
  if ((unsigned long)(now - lastReportMs_) < STATUS_PERIOD_MS) {
    return;
  }
  lastReportMs_ = now;
  float bx, by, bz;
  ekf_.getBias(bx, by, bz);
  Serial0.printf("q %.4f %.4f %.4f %.4f seq %lu tick %lu over %lu gate %lu bg %.6f %.6f %.6f st %d\n",
                quat_[0], quat_[1], quat_[2], quat_[3], (unsigned long)seq,
                (unsigned long)ticks_, (unsigned long)overruns_, (unsigned long)gated_,
                bx, by, bz, still_.latched() ? 1 : 0);
  gated_ = 0;
}

bool Tracker::step() {
  const unsigned long period = 1000000UL / FUSION_RATE_HZ;
  const unsigned long nowTick = micros();
  if ((unsigned long)(nowTick - lastTickUs_) > 2 * period) {
    lastTickUs_ = nowTick;
    overruns_++;
    return false;
  }
  if ((unsigned long)(nowTick - lastTickUs_) < period) {
    return false;
  }
  lastTickUs_ += period;
  vec3 accel, gyro;
  // Both reads always execute (as before) so a dead accel path cannot hide a
  // dead gyro path.
  const bool okA = mpu_.readAccel(accel);
  const bool okG = mpu_.readGyro(gyro);
  if (!okA || !okG) {
    Serial0.println("MPU read failed");
    return false;
  }
  float bg[3], ba[3];
  toBody(gyro, bg);
  toBody(accel, ba);
  const float bnorm = sqrtf(ba[0] * ba[0] + ba[1] * ba[1] + ba[2] * ba[2]);
  // Still detector: driver-frame gyro (pre-remap) + body accel norm.
  if (still_.update(gyro.x, gyro.y, gyro.z, bnorm)) {
    vec3 mean;
    still_.meanDps(mean);
    mpu_.nudgeGyroBias(mean, StillDetector::kAlpha);
  }
  if (still_.justExited()) {
    Serial0.println("still exit");
  }
  const float dt = 1.0f / (float)FUSION_RATE_HZ;
  ekf_.predict(bg[0] * DPS_TO_RPS, bg[1] * DPS_TO_RPS, bg[2] * DPS_TO_RPS, dt);
  if (!ekf_.updateAccel(ba[0], ba[1], ba[2])) {
    gated_++;
  }
  float ex, ey, ez, ew;
  ekf_.getQuat(ex, ey, ez, ew);
  // Display quat: q_disp = QW_ALIGN * q_ekf * QB_ALIGN.
  float tx, ty, tz, tw;
  quatMul(-DISPLAY_ALIGN_HALF, 0.0f, 0.0f, DISPLAY_ALIGN_HALF, ex, ey, ez, ew, tx, ty, tz, tw);
  quatMul(tx, ty, tz, tw, DISPLAY_ALIGN_HALF, 0.0f, 0.0f, DISPLAY_ALIGN_HALF,
          quat_[0], quat_[1], quat_[2], quat_[3]);
  gyro_[0] = bg[0] * DPS_TO_RPS;
  gyro_[1] = bg[1] * DPS_TO_RPS;
  gyro_[2] = bg[2] * DPS_TO_RPS;
  ++ticks_;
  return true;
}
