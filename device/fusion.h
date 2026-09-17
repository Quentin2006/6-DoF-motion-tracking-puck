#pragma once

#include "mpu6050.h"
#include "config.h"
#include <math.h>

class Ekf6Dof {
public:
  void begin(const vec3 &bg0);
  // gyro rad/s (bias-corrected inside via bg_), accel m/s^2 with gravity, dt seconds
  void predict(float gx, float gy, float gz, float dt);
  // returns true if accel accepted, false if gated out (predict-only tick)
  bool updateAccel(float ax, float ay, float az);
  void getQuat(float &x, float &y, float &z, float &w) const;
  void getBias(float &x, float &y, float &z) const;

private:
  float q0_ = 1, q1_ = 0, q2_ = 0, q3_ = 0; // (w,x,y,z)
  float bg_[3] = {0, 0, 0};
  float P_[6][6] = {0};
};
