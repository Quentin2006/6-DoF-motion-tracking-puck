#include "config.h"
#include "mpu6050.h"
#include "fusion.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>

Mpu6050 mpu;
Ekf6Dof ekf;
WiFiUDP udp;
uint32_t pktSeq = 0;
unsigned long tick = 0;
unsigned long lastStatusMs = 0;
unsigned long lastTickUs = 0;

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

void setup() {
  Serial.begin(115200);
  delay(500);
  if (!mpu.begin()) {
    Serial.println("MPU6050 init failed");
    while (true) {
      delay(100);
    }
  }
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
  const float dt = 1.0f / (float)FUSION_RATE_HZ;
  ekf.predict(bgx * DPS_TO_RPS, bgy * DPS_TO_RPS, bgz * DPS_TO_RPS, dt);
  ekf.updateAccel(bax, bay, baz);
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
  uint8_t pkt[QUAT_PACKET_SIZE];
  uint32_t stamp = micros();
  memcpy(pkt + 0, &pktSeq, 4);
  memcpy(pkt + 4, &stamp, 4);
  memcpy(pkt + 8, &qx, 4);
  memcpy(pkt + 12, &qy, 4);
  memcpy(pkt + 16, &qz, 4);
  memcpy(pkt + 20, &qw, 4);
  if ((tick & 1) == 0) {
    udp.beginPacket(engine_ip, UDP_PORT);
    udp.write(pkt, QUAT_PACKET_SIZE);
    udp.endPacket();
    pktSeq++;
  }
  tick++;
  unsigned long nowMs = millis();
  if ((unsigned long)(nowMs - lastStatusMs) >= 1000UL) {
    lastStatusMs = nowMs;
    Serial.print("q ");
    Serial.print(qx, 4);
    Serial.print(' ');
    Serial.print(qy, 4);
    Serial.print(' ');
    Serial.print(qz, 4);
    Serial.print(' ');
    Serial.print(qw, 4);
    Serial.print(' ');
    Serial.println(pktSeq);
  }
}
