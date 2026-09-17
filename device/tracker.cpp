#include "tracker.h"
#include <math.h>
#include <string.h>

// "… off x y z scale x y z" — shared by the boot report and the CAL6 solve.
static void printCal(const char *prefix, const float off[3], const float scale[3]) {
  Serial.printf("%s off %.4f %.4f %.4f scale %.4f %.4f %.4f\n", prefix, off[0], off[1],
                off[2], scale[0], scale[1], scale[2]);
}

// Wait up to timeoutMs for a serial line. Returns true when a non-empty line
// arrived (also on timeout with partial content), false on silence.
static bool waitLine(char *out, size_t cap, unsigned long timeoutMs) {
  size_t n = 0;
  out[0] = '\0';
  const unsigned long t0 = millis();
  while ((unsigned long)(millis() - t0) < timeoutMs) {
    while (Serial.available() > 0 && n + 1 < cap) {
      const char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        if (n > 0) {
          return true;
        }
      } else {
        out[n++] = c;
        out[n] = '\0';
      }
    }
    delay(10);
  }
  return n > 0;
}

// One-time serial-guided 6-point accel cal. Blocking, setup-time only. Any
// unexpected input or timeout aborts WITHOUT saving (NVS untouched).
static void runAccelCal(Mpu6050 &mpu) {
  static const char *kPose[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
  vec3 means[6];
  for (int p = 0; p < 6; ++p) {
    Serial.printf("CAL6 pose %s down: hold still, send G\n", kPose[p]);
    char cmd[16];
    if (!waitLine(cmd, sizeof cmd, 120000UL) || strcmp(cmd, "G") != 0) {
      Serial.println("CAL6 aborted, nothing saved");
      return;
    }
    // ~5 s raw mean, bypassing all driver-side accel calibration.
    vec3 total;
    unsigned long n = 0;
    const unsigned long t0 = millis();
    while ((unsigned long)(millis() - t0) < 5000UL) {
      vec3 s;
      if (mpu.readRawAccel(s)) {
        total += s;
        ++n;
      }
      delay(5);
    }
    if (n == 0) {
      Serial.println("CAL6 aborted, nothing saved (no samples)");
      return;
    }
    total /= (float)n;
    means[p] = total;
    Serial.printf("CAL6 mean %s %.4f %.4f %.4f\n", kPose[p], total.x, total.y, total.z);
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
      Serial.println("CAL6 aborted, nothing saved (degenerate span)");
      return;
    }
    off[i] = (mx + mn) * 0.5f;
    scale[i] = 2.0f * GRAVITY_MPS2 / (mx - mn);
  }
  printCal("CAL6 solved", off, scale);
  Serial.println("CAL6 send SAVE to flash, anything else aborts");
  char save[16];
  if (!waitLine(save, sizeof save, 120000UL) || strcmp(save, "SAVE") != 0) {
    Serial.println("CAL6 aborted, nothing saved");
    return;
  }
  mpu.setAccelCal(off, scale);
  mpu.getAccelBias(); // re-seed the runtime offset residual
  Serial.println("CAL6 saved; reboot");
}

// Driver -> body remap from FUSION_AXIS_MAP (1/2/3 = +x/+y/+z; negative
// negates). Index 0 aliases z so an out-of-range 0 keeps the historical
// selAxis() fall-through instead of reading out of bounds.
static void toBody(const vec3 &v, float out[3]) {
  const float s[4] = {v.z, v.x, v.y, v.z};
  for (int i = 0; i < 3; ++i) {
    const int8_t m = FUSION_AXIS_MAP[i];
    const int a = m < 0 ? -m : m;
    out[i] = s[a] * (m < 0 ? -1.0f : 1.0f);
  }
}

static void quatMul(float ax, float ay, float az, float aw,
                    float bx, float by, float bz, float bw,
                    float &ox, float &oy, float &oz, float &ow) {
  ox = aw * bx + ax * bw + ay * bz - az * by;
  oy = aw * by - ax * bz + ay * bw + az * bx;
  oz = aw * bz + ax * by - ay * bx + az * bw;
  ow = aw * bw - ax * bx - ay * by - az * bz;
}

bool Tracker::begin() {
  return mpu_.begin();
}

void Tracker::calibrate() {
  mpu_.getAccelBias();
  mpu_.getGyroBias();
}

void Tracker::printAccelCal() {
  float off[3], scale[3];
  mpu_.getAccelCal(off, scale);
  printCal("accCal", off, scale);
}

void Tracker::maybeRunAccelCal(unsigned long timeoutMs) {
  char cmd[16];
  if (waitLine(cmd, sizeof cmd, timeoutMs) && strcmp(cmd, "CAL6") == 0) {
    runAccelCal(mpu_);
  }
}

void Tracker::start() {
  vec3 gb;
  mpu_.readGyro(gb);
  ekf_.begin(vec3{gb.x * DPS_TO_RPS, gb.y * DPS_TO_RPS, gb.z * DPS_TO_RPS});
  lastTickUs_ = micros();   // first loop() tick is a clean "too-soon" skip
  lastReportMs_ = millis(); // 1 Hz status cadence starts with streaming
}

void Tracker::report(uint32_t seq) {
  const unsigned long now = millis();
  if ((unsigned long)(now - lastReportMs_) < STATUS_PERIOD_MS) {
    return;
  }
  lastReportMs_ = now;
  float bx, by, bz;
  ekf_.getBias(bx, by, bz);
  Serial.printf("q %.4f %.4f %.4f %.4f seq %lu tick %lu over %lu gate %lu bg %.6f %.6f %.6f st %d\n",
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
    Serial.println("MPU read failed");
    return false;
  }
  float bg[3], ba[3];
  toBody(gyro, bg);
  toBody(accel, ba);
  const float bnorm = sqrtf(ba[0] * ba[0] + ba[1] * ba[1] + ba[2] * ba[2]);
  // Still detector: driver-frame gyro (pre-remap) + body accel norm.
  if (still_.update(gyro.x, gyro.y, gyro.z, bnorm)) {
    mpu_.nudgeGyroBias(still_.residualDps(), still_.alpha());
  }
  if (still_.justExited()) {
    Serial.println("still exit");
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
  quatMul(-0.70710678f, 0.0f, 0.0f, 0.70710678f, ex, ey, ez, ew, tx, ty, tz, tw);
  quatMul(tx, ty, tz, tw, 0.70710678f, 0.0f, 0.0f, 0.70710678f,
          quat_[0], quat_[1], quat_[2], quat_[3]);
  gyro_[0] = bg[0] * DPS_TO_RPS;
  gyro_[1] = bg[1] * DPS_TO_RPS;
  gyro_[2] = bg[2] * DPS_TO_RPS;
  ++ticks_;
  return true;
}
