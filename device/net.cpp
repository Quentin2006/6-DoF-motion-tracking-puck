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

void Net::sendQuat(const float quat[4], const float gyro[3], uint32_t stamp) {
  bool doSend = sendNow_;
  sendNow_ = !sendNow_;

  uint8_t pkt[QUAT_PACKET_SIZE_V2];
  build_quat_packet(pkt, pktSeq_, stamp, quat, gyro);
  if (doSend) {
    udp_.beginPacket(ip_, port_);
    udp_.write(pkt, QUAT_PACKET_SIZE_V2);
    udp_.endPacket();
    pktSeq_++;
  }
}
