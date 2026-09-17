#include "net.h"
#include "packet.h"

bool Net::begin(const char *ssid, const char *pwd, const char *ip, uint16_t port) {
  ip_ = ip;
  port_ = port;
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
  udp_.begin(port);
  return true;
}

bool Net::beginVideo(uint16_t port) {
  return video_.begin(port) == 1;
}

int Net::pollVideoChunk(uint8_t *payload_out, VideoChunkHdr &hdr) {
  if (!payload_out) {
    return 0;
  }
  int n = video_.parsePacket();
  if (n <= 0) {
    return 0;
  }
  uint8_t hdrBuf[VIDEO_HDR_LEN];
  int hr = video_.read(hdrBuf, sizeof hdrBuf);
  if (hr < (int)VIDEO_HDR_LEN) {
    video_.flush();
    return 0;
  }
  if (!parse_video_header(hdrBuf, sizeof hdrBuf, hdr)) {
    video_.flush();
    return 0;
  }
  int want = (int)hdr.len;
  int cap = n - (int)VIDEO_HDR_LEN;
  if (want <= 0 || want > (int)VIDEO_PAYLOAD_MAX || want > cap) {
    video_.flush();
    return 0;
  }
  int got = video_.read(payload_out, (size_t)want);
  if (got < want) {
    video_.flush();
    return 0;
  }
  // One packet buffered at a time; remainder (if any) belongs to no chunk.
  video_.flush();
  return want;
}

void Net::sendQuat(const float quat[4], const float gyro[3], uint32_t stamp) {
  bool doSend = sendNow_;
  sendNow_ = !sendNow_;

  uint8_t pkt[QUAT_PACKET_LEN_V2];
  build_quat_packet(pkt, pktSeq_, stamp, quat, gyro);
  if (doSend) {
    udp_.beginPacket(ip_, port_);
    udp_.write(pkt, QUAT_PACKET_LEN_V2);
    udp_.endPacket();
    pktSeq_++;
  }
}
