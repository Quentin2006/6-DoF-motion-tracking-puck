#pragma once
#include "packet.h"
#include "video.h"
#include "frame.h"
#include <Arduino.h>

class UsbLink {
public:
  // Grow the CDC RX buffer (default 256 B) before Serial.begin so paced video
  // bursts fit; baud is nominal on S3 native USB, kept for UART fallback.
  // Always returns true (nothing to fail).
  bool begin(unsigned long baud = 115200, size_t rxBuf = 4096);

  // Moves at most one segment out of the Serial queue per call, then pumps
  // it through the frame parser, returning at most one video chunk (never
  // a drain — that would starve 500 Hz fusion under a back-to-back video
  // burst). Parses the 8 B header into hdr and copies the payload into
  // `out` (must hold VIDEO_PAYLOAD_MAX bytes). Returns payload bytes, or 0
  // when no complete FRAME_VIDEO frame arrived / the header is short / the
  // length is out of range.
  int pollVideoChunk(uint8_t *out, VideoChunkHdr &hdr);

  // Build the 36 B v2 packet (stamp = micros()) and write one FRAME_POSE
  // frame to Serial. ~250 Hz: alternates send/skip per call, matching the
  // 500 Hz tick rate.
  void sendQuat(const float quat[4], const float gyro[3]);

  uint32_t sequence() const { return pktSeq_; }

private:
  FrameParser parser_;
  uint32_t pktSeq_ = 0;
  bool sendNow_ = true;
};
