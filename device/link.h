#pragma once
#include <Arduino.h>

// Pose out over Serial (the framed CDC link in frame.h). The CDC itself is
// owned by the video task (src/display/video_task.cpp), which initializes it
// on core 0 so its per-byte RX interrupt never runs on the fusion core.
class UsbLink {
public:
  // Build the 36 B v2 packet (stamp = micros()) and write one FRAME_POSE
  // frame to Serial. ~250 Hz: alternates send/skip per call, matching the
  // 500 Hz tick rate.
  void sendQuat(const float quat[4], const float gyro[3]);

  uint32_t sequence() const { return pktSeq_; }

private:
  uint32_t pktSeq_ = 0;
  bool sendNow_ = true;
};
