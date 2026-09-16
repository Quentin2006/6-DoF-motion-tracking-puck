#pragma once
#include "config.h"
#include "video.h"
#include <WiFi.h>
#include <WiFiUdp.h>
class Net {
public:
  // WiFi.mode(WIFI_STA), setSleep(WIFI_PS_NONE), setAutoReconnect, WiFi.begin,
  // block up to 15 s for WL_CONNECTED, then udp.begin(port). Returns false on WiFi failure.
  bool begin(const char *ssid, const char *pwd, const char *ip, uint16_t port);
  // Bind the dedicated video RX socket (engine -> device). Advisory: false
  // keeps tracking-only; never fails setup.
  bool beginVideo(uint16_t port);

  // Build 36B packet (seq u32 + stamp u32 + quat xyz w + gyro xyz f32 LE, all
  // from config.h offset scheme) and send to ip:port. Send ~250 Hz: internally
  // alternates (first call sends, second skips, ...) matching the old `(tick & 1)`.
  // pktSeq_ increments only on an actual send. Also advances the alternating flag
  // on skipped calls so the decimation cadence is preserved.
  void sendQuat(const float quat[4], const float gyro[3], uint32_t stamp);

  // Video RX (engine -> device, port VIDEO_UDP_PORT): exactly one non-blocking
  // parsePacket + read per call. Parses the 8 B header into hdr, copies the
  // payload into payload_out (must be >= VIDEO_PAYLOAD_MAX bytes).
  // Returns payload bytes, or 0 when no datagram / header short.
  int pollVideoChunk(uint8_t *payload_out, VideoChunkHdr &hdr);

  uint32_t sequence() const { return pktSeq_; }

private:
  WiFiUDP udp_;
  WiFiUDP video_;
  const char *ip_ = nullptr;
  uint16_t port_ = 0;
  uint32_t pktSeq_ = 0;
  bool sendNow_ = true;
};
