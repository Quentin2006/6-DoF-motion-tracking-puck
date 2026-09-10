#pragma once

#include <stddef.h>
#include <stdint.h>

struct vec3 {
  float x;
  float y;
  float z;
};

void imu_init();
bool read_gyro_sample(vec3 &sample);
bool read_accel_sample(vec3 &sample);
