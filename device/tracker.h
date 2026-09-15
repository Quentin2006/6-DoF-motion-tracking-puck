#pragma once
#include "config.h"
#include "mpu6050.h"
#include "fusion.h"
#include "still_detect.h"

class Tracker {
public:
  enum class Step { Skip, Processed, Failed };

  bool begin();                      // mpu_.begin(); false -> caller prints "MPU6050 init failed"
  void calibrate();                  // mpu_.getAccelBias(GYRO_CALIB_SAMPLES) + getGyroBias(...)
  void printAccelCal();              // getAccelCal() then print off/scale (persistence proof)
  void seedEKF();                    // mpu_.readGyro -> ekf_.begin(gbRps)
  Mpu6050 &mpu() { return mpu_; }    // for AccelCalibrator.run()

  // One 500 Hz tick. Skip = scheduler chose not to run (overrun or too soon).
  // Processed = full pipeline ran, quat()/gyro()/outputs valid -> caller sends.
  // Failed = IMU read failed -> caller logs.
  Step step(float dt);

  const float *quat() const { return quat_; }   // display-frame x,y,z,w (valid after Processed)
  const float *gyro() const { return gyro_; }   // body-frame gyro rad/s (valid after Processed)
  void bias(float &x, float &y, float &z) const;
  bool stillLatch() const { return still_.latched(); }
  bool stillJustExited() const { return still_.justExited(); }
  uint32_t ticks() const { return ticks_; }
  uint32_t overruns() const { return overruns_; }
  uint32_t gated() const { return gated_; }
  void resetGated() { gated_ = 0; }

private:
  Mpu6050 mpu_;
  Ekf6Dof ekf_;
  StillDetector still_;
  float quat_[4] = {0, 0, 0, 1};     // x,y,z,w display frame
  float gyro_[3] = {0, 0, 0};        // body-frame rad/s
  uint32_t ticks_ = 0, overruns_ = 0, gated_ = 0;
  unsigned long lastTickUs_ = 0;
};
