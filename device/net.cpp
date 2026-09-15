#include "net.h"
#include <string.h>

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
  memcpy(pkt + 0, &pktSeq_, 4);
  memcpy(pkt + 4, &stamp, 4);
  memcpy(pkt + 8, &quat[0], 4);
  memcpy(pkt + 12, &quat[1], 4);
  memcpy(pkt + 16, &quat[2], 4);
  memcpy(pkt + 20, &quat[3], 4);
  float wx = gyro[0], wy = gyro[1], wz = gyro[2];
  memcpy(pkt + 24, &wx, 4);
  memcpy(pkt + 28, &wy, 4);
  memcpy(pkt + 32, &wz, 4);
  if (doSend) {
    udp_.beginPacket(ip_, port_);
    udp_.write(pkt, QUAT_PACKET_SIZE_V2);
    udp_.endPacket();
    pktSeq_++;
  }
}
