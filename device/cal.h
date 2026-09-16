#pragma once
#include "mpu6050.h"
#include <Arduino.h>

class AccelCalibrator {
public:
  // Wait up to timeoutMs for a line reading exactly "CAL6" (trimmed). Returns true if seen.
  bool awaitTrigger(unsigned long timeoutMs);
  // Blocking 6-pose procedure. Any unexpected input/timeout aborts WITHOUT saving (NVS untouched).
  // On SAVE: mpu.setAccelCal(off,scale), then mpu.getAccelBias(GYRO_CALIB_SAMPLES) re-seed.
  void run(Mpu6050 &mpu);
private:
  static bool waitLine(String &line, unsigned long timeoutMs);
};
