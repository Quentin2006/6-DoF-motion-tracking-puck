#include "net.h"
#include "packet.h"

bool Net::begin(const char *ssid, const char *pwd) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(WIFI_PS_NONE); // modem-sleep off: kills 10-100ms jitter bursts
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, pwd);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    if (millis() - wifiStart > 15000UL) {
      return false;
    }
  }
  udp_.begin(UDP_PORT);
  return true;
}

bool Net::beginVideo() {
  return video_.begin(VIDEO_UDP_PORT) == 1;
}

int Net::pollVideoChunk(uint8_t *out, VideoChunkHdr &hdr) {
  const int n = video_.parsePacket();
  if (n <= 0) {
    return 0;
  }
  uint8_t hb[VIDEO_HDR_LEN];
  bool ok = video_.read(hb, sizeof hb) == (int)sizeof hb;
  ok = ok && parse_video_header(hb, sizeof hb, hdr);
  ok = ok && hdr.len > 0 && hdr.len <= VIDEO_PAYLOAD_MAX && hdr.len <= n - (int)VIDEO_HDR_LEN;
  ok = ok && video_.read(out, hdr.len) == (int)hdr.len;
  video_.flush(); // remaining bytes (if any) belong to no chunk
  return ok ? (int)hdr.len : 0;
}

void Net::sendQuat(const float quat[4], const float gyro[3]) {
  const bool doSend = sendNow_;
  sendNow_ = !sendNow_;
  if (!doSend) {
    return;
  }
  uint8_t pkt[QUAT_PACKET_LEN_V2];
  build_quat_packet(pkt, pktSeq_, micros(), quat, gyro);
  udp_.beginPacket(ENGINE_IP, UDP_PORT);
  udp_.write(pkt, QUAT_PACKET_LEN_V2);
  udp_.endPacket();
  pktSeq_++;
}
