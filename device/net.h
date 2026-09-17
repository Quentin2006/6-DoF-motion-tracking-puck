#pragma once
#include "config.h"
#include "video.h"
#include <WiFi.h>
#include <WiFiUdp.h>
class Net {
public:
  // WiFi STA (modem sleep off, auto-reconnect), block up to 15 s for
  // WL_CONNECTED, then udp_.begin(UDP_PORT). false on WiFi failure.
  bool begin(const char *ssid, const char *pwd);
  // Bind the engine -> device video RX socket. Advisory: false keeps
  // tracking-only; never fails setup.
  bool beginVideo();

  // Exactly one non-blocking datagram per call. Parses the 8 B header into hdr
  // and copies the payload into `out` (which must hold VIDEO_PAYLOAD_MAX bytes).
  // Returns payload bytes, or 0 when there is no datagram / the header is short
  // / the declared length is out of range. A chunk is never split across calls.
  int pollVideoChunk(uint8_t *out, VideoChunkHdr &hdr);

  // Build the 36 B v2 packet (stamp = micros()) and send to ENGINE_IP:UDP_PORT.
  // ~250 Hz: alternates send/skip per call, matching the 500 Hz tick rate.
  void sendQuat(const float quat[4], const float gyro[3]);

  uint32_t sequence() const { return pktSeq_; }

private:
  WiFiUDP udp_;
  WiFiUDP video_;
  uint32_t pktSeq_ = 0;
  bool sendNow_ = true;
};
