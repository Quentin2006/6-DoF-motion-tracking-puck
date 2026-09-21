#pragma once
#include <Arduino.h>

class UsbLink {
public:
  // Grow the CDC RX queue (core default 256 B) before Serial.begin so a
  // back-to-back video burst waits in the queue instead of being dropped —
  // HWCDC's ISR silently discards whatever does not fit. This board runs
  // Serial on the ESP32-S3 USB-Serial-JTAG peripheral (HWCDC), so baud is a
  // no-op for throughput: the pipe is capped by the USB delivery path, not by
  // a line rate. Always returns true (nothing to fail).
  bool begin(unsigned long baud = 921600, size_t rxBuf = 4096);

  // Build the 36 B v2 packet (stamp = micros()) and write one FRAME_POSE
  // frame to Serial. ~250 Hz: alternates send/skip per call, matching the
  // 500 Hz tick rate.
  void sendQuat(const float quat[4], const float gyro[3]);

  uint32_t sequence() const { return pktSeq_; }

private:
  uint32_t pktSeq_ = 0;
  bool sendNow_ = true;
};
