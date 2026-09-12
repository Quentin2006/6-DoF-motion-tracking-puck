#pragma once

#include <stddef.h>
#include <stdint.h>

struct vec3 {
  float x;
  float y;
  float z;
};

void imu_init();
bool imu_read(vec3 &gyro_dps, vec3 &accel_mps2); // one atomic 14B burst
bool imu_self_test(); // WHO_AM_I == 0x68
