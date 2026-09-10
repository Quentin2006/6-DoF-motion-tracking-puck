#pragma once

#include "imu.h"
#include "proto.h"

/*
  This is the datasheet im using for all info on the IMU (MPU6050):
  https://www.hestore.hu/prod_getfile.php?id=8301

  provides us both gyro (deg/sec) and accel (m/s^2) data. These two peices
  alone can provide us a decent estamate on the devices rotation.

  To get the rotaiton from the device, we cannot just integrate over gyro due
  potental inaccuracies and drift.

  So inorder to get a better estamtaion, we need to use a sensor fusion
  algorithm.

  There are a few main algos we can use that will output rotation info (via
  quaternion) though inputing both accel and gyro data.

  --------- VQF ---------
  Pros: Good accuracy, handles gyro bias pretty well, works well when there is
  some extra acceleration, and is still relatively lightweight.
  Cons: More complicated than Mahony or Madgwick, and not as flexible as making
  our own EKF.

  --------- Mahony ---------
  Pros: Very lightweight, pretty simple to implement, doesn't take much
  processing power, and would work well on an embedded system.
  Cons: Needs some tuning to get good results, can have problems when the device
  is accelerating, and isn't as advanced as VQF or an EKF.

  --------- Madgwick ---------
  Pros: Lightweight, fairly simple to implement, widely used, and there are
  already a lot of implementations we could use as a starting point.
  Cons: Still needs some tuning, can have problems when there is extra
  acceleration, and doesn't give us as much control as an EKF.

  --------- EKF ---------
  Pros: Gives us a lot more control over how we handle the sensor data, can
  account for things like sensor noise and gyro bias, and can be extended later
  if we want to add more information or constraints.
  Cons: More computationally expensive, much more complicated to implement and
  tune, and there are more things that can go wrong if we implement it
  incorrectly.

  --------- Invariant EKF (IEKF) ---------
  Pros: Has some better theoretical properties than a normal EKF, can be more
  robust and consistent, and would be good if we wanted to do a more advanced
  implementation.
  Cons: More complicated mathematically, takes more work to implement, and would
  probably be overkill for what we are trying to do.


  VQF would probably provide the best accuracy, but I'm not choosing it. This
  is mainly because the algorithm is more complex, and I want to be able to
  finish the project in a reasonable amount of time. Instead, I'm choosing the
  Madgwick filter because it is relatively easy to implement while still
  providing a good estimate of the device's orientation.
   */

// Sensor-fusion stage: feeds gyro (deg/s) + accel samples into the Madgwick
// filter and exposes the orientation estimate as a wire-format Quat.
void fusion_init();
// Blocks ~2 s averaging gyro samples into a bias estimate. Requires the
// device held still, imu_init() already run, and Serial up.
void fusion_calibrate();
void fusion_update(vec3 gyro_dps, vec3 accel);
Quat fusion_quat();
