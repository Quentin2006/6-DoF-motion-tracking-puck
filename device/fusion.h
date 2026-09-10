#pragma once

#include "imu.h"
#include "proto.h"

// Sensors in: gyro (dps) + accel. Madgwick chosen over VQF/Mahony/EKF as
// lightweight and implementable on the ESP32-S3.
// Correction is gravity-only (no magnetometer), so yaw relies on the gyro
// bias estimate handled in fusion.cpp.
// MPU6050 datasheet: https://www.hestore.hu/prod_getfile.php?id=8301

// Sensor-fusion stage: feeds gyro (deg/s) + accel samples into the Madgwick
// filter and exposes the orientation estimate as a wire-format Quat.
void fusion_init();
// Blocks ~2 s averaging gyro samples into a bias estimate. Requires the
// device held still, imu_init() already run, and Serial up.
void fusion_calibrate();
void fusion_update(vec3 gyro_dps, vec3 accel);
Quat fusion_quat();
