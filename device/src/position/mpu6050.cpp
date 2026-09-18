#include <math.h>
#include <string.h>
#include <Preferences.h>
#include "mpu6050.h"
#include "../../config.h"

// Register map: driver-internal, never tuned. FSR/DLPF choices (MPU_*_VAL)
// stay in config.h next to the LSB scales they must agree with.
namespace {
constexpr uint8_t kPwrMgmt1Reg = 0x6B;
constexpr uint8_t kAccelBase = 0x3B;
constexpr uint8_t kGyroBase = 0x43;
constexpr uint8_t kWhoAmIReg = 0x75;
constexpr uint8_t kWhoAmIExpect = 0x68;
constexpr uint8_t kWhoAmIExpectAlt = 0x74; // observed clone ID; map matches 6050
constexpr uint8_t kSmplrtDivReg = 0x19;
constexpr uint8_t kConfigReg = 0x1A;
constexpr uint8_t kGyroConfigReg = 0x1B;
constexpr uint8_t kAccelConfigReg = 0x1C;
} // namespace

bool Mpu6050::begin() {
  Wire.begin(static_cast<uint8_t>(SDA_PIN), static_cast<uint8_t>(SCL_PIN));
  Wire.setClock(400000);

  if (!writeReg(kPwrMgmt1Reg, 0x00)) {
    return false;
  }
  delay(100);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(kWhoAmIReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(MPU_ADDR, static_cast<uint8_t>(1)) != 1) {
    return false;
  }
  const int who = Wire.read();
  if (who != kWhoAmIExpect && who != kWhoAmIExpectAlt) {
    return false;
  }

  if (!writeReg(kSmplrtDivReg, MPU_SMPLRT_DIV_VAL)) {
    return false;
  }
  if (!writeReg(kConfigReg, MPU_CONFIG_VAL)) {
    return false;
  }
  if (!writeReg(kGyroConfigReg, MPU_GYRO_CONFIG_VAL)) {
    return false;
  }
  if (!writeReg(kAccelConfigReg, MPU_ACCEL_CONFIG_VAL)) {
    return false;
  }
  loadAccelCal(); // non-fatal: identity defaults when NVS keys absent
  return true;
}

bool Mpu6050::readGyro(vec3 &out) {
  vec3 raw;
  if (!readBurst6(kGyroBase, GYRO_LSB_TO_DPS, raw)) {
    return false;
  }
  out = {raw.x - gyro_bias.x, raw.y - gyro_bias.y, raw.z - gyro_bias.z};
  return true;
}

bool Mpu6050::readAccel(vec3 &out) {
  vec3 raw;
  if (!readBurst6(kAccelBase, ACCEL_LSB_O_MPS2, raw)) {
    return false;
  }
  const vec3 c = applyAccelCal(raw); // identity on first boot
  // runtime offset seed only; never folded into accOff_
  out = {c.x - accel_bias.x, c.y - accel_bias.y, c.z - accel_bias.z};
  return true;
}

bool Mpu6050::readRawAccel(vec3 &out) {
  // CAL6-only: raw m/s², bypasses accOff_/accScale_/accel_bias.
  return readBurst6(kAccelBase, ACCEL_LSB_O_MPS2, out);
}

// Mean of GYRO_CALIB_SAMPLES bursts spaced MPU_CALIB_DELAY_MS apart.
bool Mpu6050::meanBurst(uint8_t baseReg, float scale, vec3 &out) {
  vec3 total, s;
  int ok = 0;
  for (int i = 0; i < GYRO_CALIB_SAMPLES; ++i) {
    delay(MPU_CALIB_DELAY_MS);
    if (readBurst6(baseReg, scale, s)) {
      total.x += s.x;
      total.y += s.y;
      total.z += s.z;
      ++ok;
    }
  }
  if (ok == 0) {
    return false;
  }
  const float inv = 1.0f / (float)ok;
  out = {total.x * inv, total.y * inv, total.z * inv};
  return true;
}

bool Mpu6050::getGyroBias() {
  return meanBurst(kGyroBase, GYRO_LSB_TO_DPS, gyro_bias);
}

bool Mpu6050::meanRawAccel(vec3 &out) {
  return meanBurst(kAccelBase, ACCEL_LSB_O_MPS2, out);
}

bool Mpu6050::getAccelBias() {
  vec3 m;
  if (!meanBurst(kAccelBase, ACCEL_LSB_O_MPS2, m)) {
    return false;
  }
  // The 6-point cal is per-axis affine, so applying it to the mean equals
  // applying it per sample (the old getAccelBias did the latter).
  m = applyAccelCal(m);
  const float n = sqrtf(m.x * m.x + m.y * m.y + m.z * m.z);
  if (n < 1e-9f) {
    return false;
  }
  // Tilt-robust: keep gravity, subtract the sensor offset along the measured
  // direction, so accel_bias holds the offset only and readAccel preserves g.
  const float k = 1.0f - GRAVITY_MPS2 / n;
  accel_bias = {m.x * k, m.y * k, m.z * k};
  return true;
}

vec3 Mpu6050::applyAccelCal(const vec3 &raw) const {
  return vec3{(raw.x - accOff_[0]) * accScale_[0], (raw.y - accOff_[1]) * accScale_[1],
              (raw.z - accOff_[2]) * accScale_[2]};
}

bool Mpu6050::writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}

bool Mpu6050::readBurst6(uint8_t baseReg, float scale, vec3 &out) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(baseReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(MPU_ADDR, static_cast<uint8_t>(6)) != 6) {
    return false;
  }
  const uint8_t xh = Wire.read();
  const uint8_t xl = Wire.read();
  const uint8_t yh = Wire.read();
  const uint8_t yl = Wire.read();
  const uint8_t zh = Wire.read();
  const uint8_t zl = Wire.read();
  const int16_t xr =
      static_cast<int16_t>((static_cast<uint16_t>(xh) << 8) | xl);
  const int16_t yr =
      static_cast<int16_t>((static_cast<uint16_t>(yh) << 8) | yl);
  const int16_t zr =
      static_cast<int16_t>((static_cast<uint16_t>(zh) << 8) | zl);
  vec3 tmp{static_cast<float>(xr) * scale, static_cast<float>(yr) * scale,
           static_cast<float>(zr) * scale};
  out = tmp;
  return true;
}

void Mpu6050::setAccelCal(const float off[3], const float scale[3]) {
  memcpy(accOff_, off, sizeof(accOff_));
  memcpy(accScale_, scale, sizeof(accScale_));
  saveAccelCal();
}

void Mpu6050::getAccelCal(float off[3], float scale[3]) const {
  memcpy(off, accOff_, sizeof(accOff_));
  memcpy(scale, accScale_, sizeof(accScale_));
}

void Mpu6050::loadAccelCal() {
  static const char *kOff[3] = {"ax0", "ax1", "ax2"};
  static const char *kScale[3] = {"as0", "as1", "as2"};
  Preferences prefs;
  if (!prefs.begin("mpu", true)) {
    return; // NVS unavailable: keep identity defaults
  }
  for (int i = 0; i < 3; ++i) {
    accOff_[i] = prefs.getFloat(kOff[i], 0.0f);
    accScale_[i] = prefs.getFloat(kScale[i], 1.0f);
  }
  prefs.end();
}

void Mpu6050::saveAccelCal() const {
  static const char *kOff[3] = {"ax0", "ax1", "ax2"};
  static const char *kScale[3] = {"as0", "as1", "as2"};
  Preferences prefs;
  if (!prefs.begin("mpu", false)) {
    return;
  }
  for (int i = 0; i < 3; ++i) {
    prefs.putFloat(kOff[i], accOff_[i]);
    prefs.putFloat(kScale[i], accScale_[i]);
  }
  prefs.end();
}

// One still-detect bias step (dps), clamped so a single noisy window cannot
// yank the bias.
void Mpu6050::nudgeGyroBias(const vec3 &meanDps, float alpha) {
  constexpr float kMaxStepDps = 0.002f;
  float *bias[3] = {&gyro_bias.x, &gyro_bias.y, &gyro_bias.z};
  const float step[3] = {alpha * meanDps.x, alpha * meanDps.y, alpha * meanDps.z};
  for (int i = 0; i < 3; ++i) {
    const float s = step[i] > kMaxStepDps ? kMaxStepDps : (step[i] < -kMaxStepDps ? -kMaxStepDps : step[i]);
    *bias[i] += s;
  }
}
