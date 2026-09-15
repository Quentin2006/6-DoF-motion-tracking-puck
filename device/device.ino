#include "config.h"
#include "tracker.h"
#include "net.h"
#include "cal.h"
#include "led.h"
#include <Arduino.h>

Tracker tracker;
Net net;
AccelCalibrator cal;
StatusLed led;
unsigned long lastStatusMs = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  led.red(); // calibrating / not ready
  if (!tracker.begin()) {
    Serial.println("MPU6050 init failed");
    while (true) {
      delay(100);
    }
  }
  if (!net.begin(ssid, pwd, engine_ip, UDP_PORT)) {
    Serial.println("WiFi failed");
    while (true) {
      delay(1000);
    }
  }
  delay(500);
  Serial.println("calibrating...");
  tracker.calibrate();
  Serial.println("calibrated");
  tracker.printAccelCal();
  Serial.println("send CAL6 within 5 s for accel cal");
  if (cal.awaitTrigger(5000UL)) {
    cal.run(tracker.mpu());
  }
  while (Serial.available() > 0) {
    Serial.read();
  }
  tracker.seedEKF();
  led.green(); // calibrated, streaming
  lastStatusMs = millis();
}

void loop() {
  const float dt = 1.0f / (float)FUSION_RATE_HZ;
  switch (tracker.step(dt)) {
    case Tracker::Step::Processed:
      net.sendQuat(tracker.quat(), tracker.gyro(), micros());
      break;
    case Tracker::Step::Failed:
      Serial.println("MPU read failed");
      break;
    case Tracker::Step::Skip:
      break;
  }
  unsigned long nowMs = millis();
  if ((unsigned long)(nowMs - lastStatusMs) >= 1000UL) {
    lastStatusMs = nowMs;
    float bx, by, bz;
    tracker.bias(bx, by, bz);
    const float *q = tracker.quat();
    Serial.print("q ");
    Serial.print(q[0], 4);
    Serial.print(' ');
    Serial.print(q[1], 4);
    Serial.print(' ');
    Serial.print(q[2], 4);
    Serial.print(' ');
    Serial.print(q[3], 4);
    Serial.print(" seq ");
    Serial.print(net.sequence());
    Serial.print(" tick ");
    Serial.print(tracker.ticks());
    Serial.print(" over ");
    Serial.print(tracker.overruns());
    Serial.print(" gate ");
    Serial.print(tracker.gated());
    Serial.print(" bg ");
    Serial.print(bx, 6);
    Serial.print(' ');
    Serial.print(by, 6);
    Serial.print(' ');
    Serial.print(bz, 6);
    Serial.print(" st ");
    Serial.println(tracker.stillLatch() ? 1 : 0);
    tracker.resetGated();
  }
}
