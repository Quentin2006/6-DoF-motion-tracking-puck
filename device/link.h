#pragma once
#include <cstdint>

// Pose out over the framed TinyUSB vendor-bulk pipe. The video task owns bulk
// receive; UART0 remains the independent console used for boot and calibration.
class BulkLink {
public:
  // Build the QUAT_PACKET_LEN packet (stamp = micros()) and write one
  // complete FRAME_POSE frame to bulk. Alternates send/skip per call at the
  // FUSION_RATE_HZ tick, so the wire rate is half of it (~250 Hz at 500 Hz).
  void sendQuat(const float quat[4], const float gyro[3]);

  uint32_t sequence() const { return pktSeq_; }

private:
  uint32_t pktSeq_ = 0;
  bool sendNow_ = true;
};
