#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
Quaternion q;
uint16_t packetSize;
uint8_t fifoBuffer[64];

void calibrateGyro() {
  const int SAMPLES = 200;
  int32_t sx = 0, sy = 0, sz = 0;
  int16_t x, y, z;
  for (int i = 0; i < SAMPLES; i++) {
    mpu.getRotation(&x, &y, &z);
    sx += x; sy += y; sz += z;
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

  Wire.begin(4, 5);

  Serial.println("Initing MPU6050...");
  mpu.initialize();
  calibrateGyro();

  uint8_t err = mpu.dmpInitialize();
  if (err != 0) {
    Serial.print("DMP init failed: ");
    Serial.println(err);
    return;
  }
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  Serial.println("DMP ready");
}

void loop() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    float temp = mpu.getTemperature() / 340.0f + 36.53f;
    Serial.printf("%.4f %.4f %.4f %.4f %.2f\n", q.w, q.x, q.y, q.z, temp);
  }
}