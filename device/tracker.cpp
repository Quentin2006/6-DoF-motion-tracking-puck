#include "tracker.h"
#include <math.h>

// Driver->body remap; single source for the axis convention.
static float selAxis(int8_t m, float x, float y, float z) {
  float v = (m == 1 || m == -1) ? x : ((m == 2 || m == -2) ? y : z);
  return m < 0 ? -v : v;
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
  mpu_.getAccelBias(GYRO_CALIB_SAMPLES);
  mpu_.getGyroBias(GYRO_CALIB_SAMPLES);
}

void Tracker::printAccelCal() {
  // Persistence proof: NVS-loaded 6-point accel cal (0/1 defaults first boot).
  float accOff[3], accScale[3];
  mpu_.getAccelCal(accOff, accScale);
  Serial.print("accCal off ");
  Serial.print(accOff[0], 4);
  Serial.print(' ');
  Serial.print(accOff[1], 4);
  Serial.print(' ');
  Serial.print(accOff[2], 4);
  Serial.print(" scale ");
  Serial.print(accScale[0], 4);
  Serial.print(' ');
  Serial.print(accScale[1], 4);
  Serial.print(' ');
  Serial.println(accScale[2], 4);
}

bool Tracker::maybeRunAccelCal(unsigned long timeoutMs) {
  if (!cal_.awaitTrigger(timeoutMs)) return false;
  cal_.run(mpu_);
  return true;
}

void Tracker::seedEKF() {
  vec3 gb;
  mpu_.readGyro(gb);
  vec3 gbRps{gb.x * DPS_TO_RPS, gb.y * DPS_TO_RPS, gb.z * DPS_TO_RPS};
  ekf_.begin(gbRps);
  lastTickUs_ = micros(); // anchor scheduler: first loop() tick is a clean "too-soon" Skip
}

void Tracker::bias(float &x, float &y, float &z) const {
  ekf_.getBias(x, y, z);
}

Tracker::Step Tracker::step(float dt) {
  const unsigned long period = 1000000UL / FUSION_RATE_HZ;
  unsigned long nowTick = micros();
  if ((unsigned long)(nowTick - lastTickUs_) > 2 * period) {
    lastTickUs_ = nowTick;
    overruns_++;
    return Step::Skip;
  }
  if ((unsigned long)(nowTick - lastTickUs_) < period) {
    return Step::Skip;
  }
  lastTickUs_ += period;
  vec3 accel, gyro;
  bool okA = mpu_.readAccel(accel);
  bool okG = mpu_.readGyro(gyro);
  if (!okA || !okG) {
    return Step::Failed;
  }
  float bgx = selAxis(FUSION_AXIS_MAP[0], gyro.x, gyro.y, gyro.z);
  float bgy = selAxis(FUSION_AXIS_MAP[1], gyro.x, gyro.y, gyro.z);
  float bgz = selAxis(FUSION_AXIS_MAP[2], gyro.x, gyro.y, gyro.z);
  float bax = selAxis(FUSION_AXIS_MAP[0], accel.x, accel.y, accel.z);
  float bay = selAxis(FUSION_AXIS_MAP[1], accel.x, accel.y, accel.z);
  float baz = selAxis(FUSION_AXIS_MAP[2], accel.x, accel.y, accel.z);
  float bnorm = sqrtf(bax * bax + bay * bay + baz * baz);
  // Still detector: driver-frame gyro (pre-remap) + body accel norm.
  if (still_.update(gyro.x, gyro.y, gyro.z, bnorm)) {
    vec3 resid = still_.residualDps();
    mpu_.nudgeGyroBias(resid.x, resid.y, resid.z, still_.alpha());
  }
  if (still_.justExited()) {
    Serial.println("still exit");
  }
  ekf_.predict(bgx * DPS_TO_RPS, bgy * DPS_TO_RPS, bgz * DPS_TO_RPS, dt);
  bool accAccepted = ekf_.updateAccel(bax, bay, baz);
  if (!accAccepted) {
    gated_++;
  }
  float ex, ey, ez, ew;
  ekf_.getQuat(ex, ey, ez, ew);
  // Display quat: q_disp = QW_ALIGN * q_ekf * QB_ALIGN.
  float tx, ty, tz, tw;
  quatMul(-0.70710678f, 0.0f, 0.0f, 0.70710678f, ex, ey, ez, ew, tx, ty, tz, tw);
  float qx, qy, qz, qw;
  quatMul(tx, ty, tz, tw, 0.70710678f, 0.0f, 0.0f, 0.70710678f, qx, qy, qz, qw);
  quat_[0] = qx;
  quat_[1] = qy;
  quat_[2] = qz;
  quat_[3] = qw;
  gyro_[0] = bgx * DPS_TO_RPS;
  gyro_[1] = bgy * DPS_TO_RPS;
  gyro_[2] = bgz * DPS_TO_RPS;
  ++ticks_;
  return Step::Processed;
}
