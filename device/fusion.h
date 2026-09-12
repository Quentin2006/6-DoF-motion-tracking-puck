#pragma once

#include "proto.h"

void fusion_init();
void fusion_calibrate(); // blocks ~10 s, sets gyro bias + resting gravity
void fusion_update();    // one fixed 10 ms sample, always runs imu_filter
Quat fusion_quat();
