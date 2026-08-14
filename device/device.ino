#include "MPU6050_6Axis_MotionApps612.h"
#include <Wire.h>

MPU6050 mpu;
Quaternion q;
uint8_t fifoBuffer[64];

void calibrateGyro() {
  const int SAMPLES = 200;
  int32_t sx = 0, sy = 0, sz = 0;
  int16_t x, y, z;
  for (int i = 0; i < SAMPLES; i++) {
    mpu.getRotation(&x, &y, &z);
    sx += x;
    sy += y;
    sz += z;
    delay(5);
  }
  mpu.setXGyroOffset(-sx / SAMPLES);
  mpu.setYGyroOffset(-sy / SAMPLES);
  mpu.setZGyroOffset(-sz / SAMPLES);
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  Wire.begin(5, 4);

  Serial.println("Initing MPU6050...");
  mpu.initialize();

  uint8_t err = mpu.dmpInitialize();
  if (err != 0) {
    Serial.print("DMP init failed: ");
    Serial.println(err);
    return;
  }
  calibrateGyro();
  mpu.setDMPEnabled(true);
  Serial.println("DMP ready");
}

void rotateByQuat(const float qw, const float qx, const float qy, const float qz,
                  float &vx, float &vy, float &vz) {
  const float qw2 = qw * qw - (qx * qx + qy * qy + qz * qz);  // 2*qw²−1 for unit q
  const float dot = qx * vx + qy * vy + qz * vz;
  const float cx = qy * vz - qz * vy;
  const float cy = qz * vx - qx * vz;
  const float cz = qx * vy - qy * vx;
  vx = qw2 * vx + 2.0f * dot * qx + 2.0f * qw * cx;
  vy = qw2 * vy + 2.0f * dot * qy + 2.0f * qw * cy;
  vz = qw2 * vz + 2.0f * dot * qz + 2.0f * qw * cz;
}

void loop() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    int16_t accel[3];
    mpu.dmpGetAccel(accel, fifoBuffer);
    constexpr float ACCEL_SCALE = 9.81f / 16384.f;
    float wx = accel[0] * ACCEL_SCALE, wy = accel[1] * ACCEL_SCALE, wz = accel[2] * ACCEL_SCALE;
    rotateByQuat(q.w, q.x, q.y, q.z, wx, wy, wz);
    wz -= 9.81f;  // world +Z = up; resting accel = g, so X/Y carry g·sin(tilt) offset
    float temp = mpu.getTemperature() / 340.0f + 36.53f;
    Serial.printf("%.4f %.4f %.4f %.4f %.3f %.3f %.3f %.2f\n",
                  q.w, q.x, q.y, q.z, wx, wy, wz, temp);
  }
}
