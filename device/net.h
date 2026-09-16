#pragma once
#include "config.h"
#include <WiFi.h>
#include <WiFiUdp.h>

class Net {
public:
  // WiFi.mode(WIFI_STA), setSleep(WIFI_PS_NONE), setAutoReconnect, WiFi.begin,
  // block up to 15 s for WL_CONNECTED, then udp.begin(port). Returns false on WiFi failure.
  bool begin(const char *ssid, const char *pwd, const char *ip, uint16_t port);

  // Build 36B packet (seq u32 + stamp u32 + quat xyz w + gyro xyz f32 LE, all
  // from config.h offset scheme) and send to ip:port. Send ~250 Hz: internally
  // alternates (first call sends, second skips, ...) matching the old `(tick & 1)`.
  // pktSeq_ increments only on an actual send. Also advances the alternating flag
  // on skipped calls so the decimation cadence is preserved.
  void sendQuat(const float quat[4], const float gyro[3], uint32_t stamp);

  uint32_t sequence() const { return pktSeq_; }

private:
  WiFiUDP udp_;
  const char *ip_ = nullptr;
  uint16_t port_ = 0;
  uint32_t pktSeq_ = 0;
  bool sendNow_ = true;
};
