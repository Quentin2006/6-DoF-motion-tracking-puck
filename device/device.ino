#include "config.h"
#include "mpu6050.h"
#include "fusion.h"
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>
#include <math.h>

Mpu6050 mpu;
Ekf6Dof ekf;
WiFiUDP udp;
uint32_t pktSeq = 0;
unsigned long tick = 0;
unsigned long lastStatusMs = 0;
unsigned long lastTickUs = 0;
unsigned long overruns_ = 0;
unsigned gated_ = 0;
// ---- Cycle-2: still-detect gyro re-bias (file-local tunables, never config.h)
static const unsigned STILL_WIN_TICKS = 50;
static const float STILL_GYRO_VAR_DPS2 = 0.25f;  // (0.5 dps)^2 per axis
static const float STILL_NORM_DEV = 0.02f;       // |(|a|-g|)/g
static const float STILL_ALPHA = 0.02f;          // nudge gain per tick
static const float STILL_EXIT_DPS = 1.5f;        // single-tick |omega| exit
static const float STILL_EXIT_NORM = 0.03f;      // single-tick norm-dev exit
static const unsigned STILL_COOLDOWN = 25;

// Driver->body remap; single source for the axis convention.
static inline float selAxis(int8_t m, float x, float y, float z);
static float stillRing[STILL_WIN_TICKS][3];
static float stillNormRing[STILL_WIN_TICKS];
static float stillSum[3] = {0.0f, 0.0f, 0.0f};
static float stillSumSq[3] = {0.0f, 0.0f, 0.0f};
static float stillNormSum = 0.0f;
static unsigned stillN = 0;    // samples in window (full at STILL_WIN_TICKS)
static unsigned stillIdx = 0;  // ring write cursor
static bool stillLatched = false;
static unsigned stillCooldown = 0;

static inline void stillWindowReset() {
  stillSum[0] = stillSum[1] = stillSum[2] = 0.0f;
  stillSumSq[0] = stillSumSq[1] = stillSumSq[2] = 0.0f;
  stillNormSum = 0.0f;
  stillN = 0;
  stillIdx = 0;
}

// Push one tick: driver-frame gyro dps (pre-remap) + body-frame accel norm.
static inline void stillWindowPush(float dx, float dy, float dz, float norm) {
  const float v[3] = {dx, dy, dz};
  if (stillN < STILL_WIN_TICKS) {
    for (int b = 0; b < 3; ++b) {
      stillRing[stillIdx][b] = v[b];
      stillSum[b] += v[b];
      stillSumSq[b] += v[b] * v[b];
    }
    stillNormRing[stillIdx] = norm;
    stillNormSum += norm;
    ++stillN;
    stillIdx = (stillIdx + 1) % STILL_WIN_TICKS;
    return;
  }
  for (int b = 0; b < 3; ++b) {
    stillSum[b] += v[b] - stillRing[stillIdx][b];
    stillSumSq[b] += v[b] * v[b] - stillRing[stillIdx][b] * stillRing[stillIdx][b];
    stillRing[stillIdx][b] = v[b];
  }
  stillNormSum += norm - stillNormRing[stillIdx];
  stillNormRing[stillIdx] = norm;
  stillIdx = (stillIdx + 1) % STILL_WIN_TICKS;
}

// Full-window still test: per-axis gyro variance + mean-norm deviation.
static inline bool stillWindowCalm() {
  if (stillN < STILL_WIN_TICKS) return false;
  const float inv = 1.0f / (float)STILL_WIN_TICKS;
  for (int b = 0; b < 3; ++b) {
    const float mean = stillSum[b] * inv;
    const float var = stillSumSq[b] * inv - mean * mean;
    if (!(var < STILL_GYRO_VAR_DPS2)) return false; // NaN-safe: NaN fails calm
  }
  const float meanNorm = stillNormSum * inv;
  const float dev = fabsf(meanNorm - GRAVITY_MPS2) / GRAVITY_MPS2;
  return dev < STILL_NORM_DEV;
}

// Driver-frame gyro residual from the still window, routed through the
// forward FUSION_AXIS_MAP (driver->body) and back via the inverse map
// (body->driver) so the remap convention stays single-sourced. The
// round-trip equals the driver mean; the explicit inverse documents intent.
static inline vec3 stillDriverResidDps() {
  const float inv = 1.0f / (float)STILL_WIN_TICKS;
  float md[3] = {stillSum[0] * inv, stillSum[1] * inv, stillSum[2] * inv};
  float mb[3];
  for (int b = 0; b < 3; ++b) {
    mb[b] = selAxis(FUSION_AXIS_MAP[b], md[0], md[1], md[2]);
  }
  float r[3] = {0, 0, 0};
  for (int b = 0; b < 3; ++b) {
    int8_t m = FUSION_AXIS_MAP[b];
    int d = (m == 1 || m == -1) ? 0 : ((m == 2 || m == -2) ? 1 : 2);
    r[d] = (m < 0 ? -mb[b] : mb[b]); // inverse-map: driver[d] = sign * body[b]
  }
  vec3 out;
  out.x = r[0];
  out.y = r[1];
  out.z = r[2];
  return out;
}

// Raw burst read of the accel registers in m/s^2, bypassing any driver
// calibration (no accOff_/accScale_/accel_bias). Used only by the CAL6
// one-time procedure. Returns false on I2C failure.
static bool readRawAccelMS2(vec3 &out) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_BASE);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)6) != 6) return false;
  uint8_t b[6];
  for (int i = 0; i < 6; ++i) b[i] = Wire.read();
  int16_t xr = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
  int16_t yr = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
  int16_t zr = (int16_t)(((uint16_t)b[4] << 8) | b[5]);
  out.x = (float)xr * ACCEL_LSB_O_MPS2;
  out.y = (float)yr * ACCEL_LSB_O_MPS2;
  out.z = (float)zr * ACCEL_LSB_O_MPS2;
  return true;
}

// Wait up to timeoutMs for a serial line; trims CR/LF. Returns true on line.
static bool waitSerialLine(String &line, unsigned long timeoutMs) {
  unsigned long t0 = millis();
  line = "";
  while ((unsigned long)(millis() - t0) < timeoutMs) {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        if (line.length() > 0) return true;
        // else keep waiting for content within the window
      } else {
        line += c;
      }
    }
    delay(10);
  }
  line.trim();
  return line.length() > 0;
}

static inline float selAxis(int8_t m, float x, float y, float z) {
  float v = (m == 1 || m == -1) ? x : ((m == 2 || m == -2) ? y : z);
  return m < 0 ? -v : v;
}

static inline void quatMul(float ax, float ay, float az, float aw,
                           float bx, float by, float bz, float bw,
                           float &ox, float &oy, float &oz, float &ow) {
  ox = aw * bx + ax * bw + ay * bz - az * by;
  oy = aw * by - ax * bz + ay * bw + az * bx;
  oz = aw * bz + ax * by - ay * bx + az * bw;
  ow = aw * bw - ax * bx - ay * by - az * bz;
}
static inline float compOf(const vec3 &v, int i) {
  return i == 0 ? v.x : (i == 1 ? v.y : v.z);
}

// One-time serial-guided 6-point accel cal. Blocking, setup-time only.
// Any unexpected input or timeout aborts WITHOUT saving (NVS untouched).
static void runCal6() {
  static const char *kPoseName[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
  vec3 means[6];
  for (int p = 0; p < 6; ++p) {
    Serial.print("CAL6 pose ");
    Serial.print(kPoseName[p]);
    Serial.println(" down: hold still, send G");
    String cmd;
    if (!waitSerialLine(cmd, 120000UL) || cmd != "G") {
      Serial.println("CAL6 aborted, nothing saved");
      return;
    }
    // ~5 s raw mean, bypassing all driver-side accel calibration.
    vec3 total;
    unsigned long n = 0;
    unsigned long t0 = millis();
    while ((unsigned long)(millis() - t0) < 5000UL) {
      vec3 s;
      if (readRawAccelMS2(s)) {
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
    Serial.print("CAL6 mean ");
    Serial.print(kPoseName[p]);
    Serial.print(' ');
    Serial.print(total.x, 4);
    Serial.print(' ');
    Serial.print(total.y, 4);
    Serial.print(' ');
    Serial.println(total.z, 4);
  }
  float off[3], scale[3];
  for (int i = 0; i < 3; ++i) {
    float a = compOf(means[2 * i], i);
    float b = compOf(means[2 * i + 1], i);
    float mx = a > b ? a : b;
    float mn = a < b ? a : b;
    if ((mx - mn) < 1e-6f) {
      Serial.println("CAL6 aborted, nothing saved (degenerate span)");
      return;
    }
    off[i] = (mx + mn) * 0.5f;
    scale[i] = 2.0f * GRAVITY_MPS2 / (mx - mn);
  }
  Serial.print("CAL6 solved off ");
  Serial.print(off[0], 4);
  Serial.print(' ');
  Serial.print(off[1], 4);
  Serial.print(' ');
  Serial.print(off[2], 4);
  Serial.print(" scale ");
  Serial.print(scale[0], 4);
  Serial.print(' ');
  Serial.print(scale[1], 4);
  Serial.print(' ');
  Serial.println(scale[2], 4);
  Serial.println("CAL6 send SAVE to flash, anything else aborts");
  String save;
  if (!waitSerialLine(save, 120000UL) || save != "SAVE") {
    Serial.println("CAL6 aborted, nothing saved");
    return;
  }
  mpu.setAccelCal(off, scale);
  mpu.getAccelBias(GYRO_CALIB_SAMPLES); // re-seed runtime offset residual
  Serial.println("CAL6 saved; reboot");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  if (!mpu.begin()) {
    Serial.println("MPU6050 init failed");
    while (true) {
      delay(100);
    }
  }
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(WIFI_PS_NONE); // modem-sleep off: kills 10-100ms jitter bursts
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, pwd);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    if (millis() - wifiStart > 15000UL) {
      Serial.println("WiFi failed");
      while (true) {
        delay(1000);
      }
    }
  }
  udp.begin(UDP_PORT);
  delay(500);
  Serial.println("calibrating...");
  mpu.getAccelBias(GYRO_CALIB_SAMPLES);
  mpu.getGyroBias(GYRO_CALIB_SAMPLES);
  Serial.println("calibrated");
  // Persistence proof: NVS-loaded 6-point accel cal (0/1 defaults first boot).
  float accOff[3], accScale[3];
  mpu.getAccelCal(accOff, accScale);
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
  // Dormant 6-pose procedure: time-bounded, explicit-input only.
  Serial.println("send CAL6 within 5 s for accel cal");
  String calCmd;
  if (waitSerialLine(calCmd, 5000UL) && calCmd == "CAL6") {
    runCal6();
  }
  while (Serial.available() > 0) Serial.read(); // flush stray RX bytes
  vec3 gb;
  mpu.readGyro(gb);
  vec3 gbRps{gb.x * DPS_TO_RPS, gb.y * DPS_TO_RPS, gb.z * DPS_TO_RPS};
  ekf.begin(gbRps);
  lastTickUs = micros();
  lastStatusMs = millis();
}

void loop() {
  const unsigned long period = 1000000UL / FUSION_RATE_HZ;
  unsigned long nowTick = micros();
  if ((unsigned long)(nowTick - lastTickUs) > 2 * period) {
    lastTickUs = nowTick;
    overruns_++;
    return;
  }
  if ((unsigned long)(nowTick - lastTickUs) < period) {
    return;
  }
  lastTickUs += period;
  vec3 accel, gyro;
  bool okA = mpu.readAccel(accel);
  bool okG = mpu.readGyro(gyro);
  if (!okA || !okG) {
    Serial.println("MPU read failed");
    return;
  }
  float bgx = selAxis(FUSION_AXIS_MAP[0], gyro.x, gyro.y, gyro.z);
  float bgy = selAxis(FUSION_AXIS_MAP[1], gyro.x, gyro.y, gyro.z);
  float bgz = selAxis(FUSION_AXIS_MAP[2], gyro.x, gyro.y, gyro.z);
  float bax = selAxis(FUSION_AXIS_MAP[0], accel.x, accel.y, accel.z);
  float bay = selAxis(FUSION_AXIS_MAP[1], accel.x, accel.y, accel.z);
  float baz = selAxis(FUSION_AXIS_MAP[2], accel.x, accel.y, accel.z);
  // Still detector: 50-tick window on driver-frame gyro (pre-remap) + body accel norm.
  // Latched -> nudge driver gyro bias toward the window mean each tick; EKF accel
  // updates keep running. Exit freezes nudging and holds last bias (never snaps back).
  float bnorm = sqrtf(bax * bax + bay * bay + baz * baz);
  float omegaMag = sqrtf(bgx * bgx + bgy * bgy + bgz * bgz);
  float normDevTick = fabsf(bnorm - GRAVITY_MPS2) / GRAVITY_MPS2;
  if (stillLatched) {
    if (omegaMag > STILL_EXIT_DPS || normDevTick > STILL_EXIT_NORM) {
      stillLatched = false;
      stillWindowReset();
      stillCooldown = STILL_COOLDOWN;
      Serial.println("still exit");
    } else {
      stillWindowPush(gyro.x, gyro.y, gyro.z, bnorm);
      vec3 resid = stillDriverResidDps();
      mpu.nudgeGyroBias(resid.x, resid.y, resid.z, STILL_ALPHA);
    }
  } else {
    stillWindowPush(gyro.x, gyro.y, gyro.z, bnorm);
    if (stillCooldown > 0) {
      stillCooldown--;
    } else if (stillWindowCalm()) {
      stillLatched = true;
      vec3 resid = stillDriverResidDps();
      mpu.nudgeGyroBias(resid.x, resid.y, resid.z, STILL_ALPHA);
    }
  }
  const float dt = 1.0f / (float)FUSION_RATE_HZ;
  ekf.predict(bgx * DPS_TO_RPS, bgy * DPS_TO_RPS, bgz * DPS_TO_RPS, dt);
  bool accAccepted = ekf.updateAccel(bax, bay, baz);
  if (!accAccepted) {
    gated_++;
  }
  float ex, ey, ez, ew;
  ekf.getQuat(ex, ey, ez, ew);
  // Display quat: q_disp = QW_ALIGN * q_ekf * QB_ALIGN.
  // QW_ALIGN = X -90 deg (-sqrt.5,0,0,+sqrt.5): world Z-up (EKF gravity
  // [0,0,1]) to world Y-up (engine render frame). QB_ALIGN = X +90 deg
  // (+sqrt.5,0,0,+sqrt.5): body thin-Z to model thin-Y. Identity maps to
  // identity.
  float tx, ty, tz, tw;
  quatMul(-0.70710678f, 0.0f, 0.0f, 0.70710678f, ex, ey, ez, ew, tx, ty, tz, tw);
  float qx, qy, qz, qw;
  quatMul(tx, ty, tz, tw, 0.70710678f, 0.0f, 0.0f, 0.70710678f, qx, qy, qz, qw);
  uint8_t pkt[QUAT_PACKET_SIZE_V2];
  uint32_t stamp = micros();
  memcpy(pkt + 0, &pktSeq, 4);
  memcpy(pkt + 4, &stamp, 4);
  memcpy(pkt + 8, &qx, 4);
  memcpy(pkt + 12, &qy, 4);
  memcpy(pkt + 16, &qz, 4);
  memcpy(pkt + 20, &qw, 4);
  float wx = bgx * DPS_TO_RPS, wy = bgy * DPS_TO_RPS, wz = bgz * DPS_TO_RPS;
  memcpy(pkt + 24, &wx, 4);
  memcpy(pkt + 28, &wy, 4);
  memcpy(pkt + 32, &wz, 4);
  if ((tick & 1) == 0) {
    udp.beginPacket(engine_ip, UDP_PORT);
    udp.write(pkt, QUAT_PACKET_SIZE_V2);
    udp.endPacket();
    pktSeq++;
  }
  tick++;
  unsigned long nowMs = millis();
  if ((unsigned long)(nowMs - lastStatusMs) >= 1000UL) {
    lastStatusMs = nowMs;
    float bbx, bby, bbz;
    ekf.getBias(bbx, bby, bbz);
    Serial.print("q ");
    Serial.print(qx, 4);
    Serial.print(' ');
    Serial.print(qy, 4);
    Serial.print(' ');
    Serial.print(qz, 4);
    Serial.print(' ');
    Serial.print(qw, 4);
    Serial.print(" seq ");
    Serial.print(pktSeq);
    Serial.print(" tick ");
    Serial.print(tick);
    Serial.print(" over ");
    Serial.print(overruns_);
    Serial.print(" gate ");
    Serial.print(gated_);
    Serial.print(" bg ");
    Serial.print(bbx, 6);
    Serial.print(' ');
    Serial.print(bby, 6);
    Serial.print(' ');
    Serial.print(bbz, 6);
    Serial.print(" st ");
    Serial.println(stillLatched ? 1 : 0);
    gated_ = 0;
  }
}
