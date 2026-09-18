#pragma once
#include "mpu6050.h"
#include "fusion.h"
#include "still_detect.h"

class Tracker {
public:
  // Bring up IMU + biases + EKF in one ordered call; false -> caller halts.
  // Prints the NVS accel cal, honors the CAL6 window, seeds the EKF, and
  // anchors the scheduler + status cadence.
  bool boot();
  bool step();               // one 500 Hz tick; true = fresh quat()/gyro() to send
  void report(uint32_t seq); // 1 Hz Serial status line (self-gated)

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
