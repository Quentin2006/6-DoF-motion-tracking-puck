#include <math.h>
#include <string.h>
#include <Preferences.h>
#include "mpu6050.h"
#include "config.h"

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

Mpu6050::Mpu6050()
    : addr(MPU_ADDR), sda(SDA_PIN), scl(SCL_PIN),
      gyro_bias{0, 0, 0}, accel_bias{0, 0, 0} {}

bool Mpu6050::begin() {
  Wire.begin(static_cast<uint8_t>(sda), static_cast<uint8_t>(scl));
  Wire.setClock(400000);

  if (!writeReg(kPwrMgmt1Reg, 0x00)) {
    return false;
  }
  delay(100);

  Wire.beginTransmission(addr);
  Wire.write(kWhoAmIReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(addr, static_cast<uint8_t>(1)) != 1) {
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
  out = raw;
  out -= gyro_bias;
  return true;
}

bool Mpu6050::readAccel(vec3 &out) {
  vec3 raw;
  if (!readBurst6(kAccelBase, ACCEL_LSB_O_MPS2, raw)) {
    return false;
  }
  // 6-point factory cal (m/s² domain): offset then per-axis scale. Defaults
  // are identity, so first-boot behavior is unchanged from the pure path.
  raw.x = (raw.x - accOff_[0]) * accScale_[0];
  raw.y = (raw.y - accOff_[1]) * accScale_[1];
  raw.z = (raw.z - accOff_[2]) * accScale_[2];
  out = raw;
  out -= accel_bias; // runtime offset seed only; never folded into accOff_
  return true;
}

bool Mpu6050::readRawAccel(vec3 &out) {
  // CAL6-only: raw m/s², bypasses accOff_/accScale_/accel_bias.
  return readBurst6(kAccelBase, ACCEL_LSB_O_MPS2, out);
}

void Mpu6050::getGyroBias(int samples) {
  if (samples <= 0) {
    return;
  }
  vec3 total;
  vec3 s;
  int ok = 0;
  for (int i = 0; i < samples; ++i) {
    delay(MPU_CALIB_DELAY_MS);
    if (readBurst6(kGyroBase, GYRO_LSB_TO_DPS, s)) {
      total += s;
      ++ok;
    }
  }
  if (ok > 0) {
    total /= static_cast<float>(ok);
    gyro_bias = total;
  }
}

void Mpu6050::getAccelBias(int samples) {
  if (samples <= 0) {
    return;
  }
  vec3 total;
  vec3 s;
  int ok = 0;
  for (int i = 0; i < samples; ++i) {
    delay(MPU_CALIB_DELAY_MS);
    if (readBurst6(kAccelBase, ACCEL_LSB_O_MPS2, s)) {
      // Calibrated stream: factory scale/offset applied, runtime residual
      // excluded (accel_bias stays untouched until the new seed lands).
      s.x = (s.x - accOff_[0]) * accScale_[0];
      s.y = (s.y - accOff_[1]) * accScale_[1];
      s.z = (s.z - accOff_[2]) * accScale_[2];
      total += s;
      ++ok;
    }
  }
  if (ok > 0) {
    total /= static_cast<float>(ok);
    // Tilt-robust: subtract gravity along the measured direction so
    // accel_bias holds sensor offset only at any rest tilt and readAccel preserves gravity.
    const float n = sqrtf(total.x * total.x + total.y * total.y + total.z * total.z);
    if (n < 1e-9f) {
      return;
    }
    const float s = GRAVITY_MPS2 / n;
    accel_bias.x = total.x - total.x * s;
    accel_bias.y = total.y - total.y * s;
    accel_bias.z = total.z - total.z * s;
  }
}

bool Mpu6050::writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}

bool Mpu6050::readBurst6(uint8_t baseReg, float scale, vec3 &out) {
  Wire.beginTransmission(addr);
  Wire.write(baseReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(addr, static_cast<uint8_t>(6)) != 6) {
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
  Preferences prefs;
  if (!prefs.begin("mpu", true)) {
    return; // NVS unavailable: keep identity defaults
  }
  accOff_[0] = prefs.getFloat("ax0", 0.0f);
  accOff_[1] = prefs.getFloat("ax1", 0.0f);
  accOff_[2] = prefs.getFloat("ax2", 0.0f);
  accScale_[0] = prefs.getFloat("as0", 1.0f);
  accScale_[1] = prefs.getFloat("as1", 1.0f);
  accScale_[2] = prefs.getFloat("as2", 1.0f);
  prefs.end();
}

void Mpu6050::saveAccelCal() const {
  Preferences prefs;
  if (!prefs.begin("mpu", false)) {
    return;
  }
  prefs.putFloat("ax0", accOff_[0]);
  prefs.putFloat("ax1", accOff_[1]);
  prefs.putFloat("ax2", accOff_[2]);
  prefs.putFloat("as0", accScale_[0]);
  prefs.putFloat("as1", accScale_[1]);
  prefs.putFloat("as2", accScale_[2]);
  prefs.end();
}

void Mpu6050::nudgeGyroBias(float mx, float my, float mz, float alpha) {
  // Still-detect re-bias: lerp gyro_bias toward the still-window mean.
  // Per-tick clamp keeps a single noisy window from yanking the bias; no I2C.
  constexpr float kMaxStepDps = 0.002f;
  float dx = alpha * mx;
  float dy = alpha * my;
  float dz = alpha * mz;
  dx = dx > kMaxStepDps ? kMaxStepDps : (dx < -kMaxStepDps ? -kMaxStepDps : dx);
  dy = dy > kMaxStepDps ? kMaxStepDps : (dy < -kMaxStepDps ? -kMaxStepDps : dy);
  dz = dz > kMaxStepDps ? kMaxStepDps : (dz < -kMaxStepDps ? -kMaxStepDps : dz);
  gyro_bias.x += dx;
  gyro_bias.y += dy;
  gyro_bias.z += dz;
}
