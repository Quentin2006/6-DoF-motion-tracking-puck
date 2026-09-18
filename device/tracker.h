#pragma once
#include <stdint.h>
#include "mpu6050.h"
#include "fusion.h"
#include "still_detect.h"

struct UsbWin {
  uint32_t sent = 0, dropped = 0, rxBytes = 0, chunks = 0, decDrops = 0, frameDrops = 0;
};
struct LoopWin {
  unsigned tFusionMax = 0, tVideoMax = 0, loopMax = 0, tReportMax = 0;
  uint32_t exitIdle = 0, exitBudget = 0, exitMax = 0;
  uint8_t lastN = 0, lastExit = 0;
};
class Tracker {
public:
  bool begin();                                  // mpu_.begin(); false -> caller halts
  void calibrate();                              // blocking gyro + accel bias seeding
  void printAccelCal();                          // NVS-loaded 6-point accel cal (persistence proof)
  void maybeRunAccelCal(unsigned long timeoutMs); // "CAL6" within the window -> blocking 6-pose cal
  void start();                                  // seed EKF from current gyro; anchor scheduler + status cadence
  bool step();                                   // one 500 Hz tick; true = fresh quat()/gyro() to send
  bool report(uint32_t seq, const UsbWin &usb, const LoopWin &loop); // 1 Hz status; true when printed
  const float *quat() const { return quat_; } // display-frame x,y,z,w
  const float *gyro() const { return gyro_; } // body-frame rad/s
private:
  Mpu6050 mpu_;
  Ekf6Dof ekf_;
  StillDetector still_;
  float quat_[4] = {0, 0, 0, 1};
  float gyro_[3] = {0, 0, 0};
  uint32_t ticks_ = 0, overruns_ = 0, gated_ = 0;
  unsigned long lastTickUs_ = 0, lastReportMs_ = 0;
};
