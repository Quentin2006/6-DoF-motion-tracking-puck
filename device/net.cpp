#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "config.h"
#include "net.h"

static WiFiUDP udp;

void net_connect() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, pwd);
  Serial.println("");
  Serial.print("Connecting to ");
  Serial.println(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  udp.begin(udpPort);
}

void net_send_quat(const Quat &q) {
  udp.beginPacket(udpAddress, udpPort);
  udp.write(reinterpret_cast<const uint8_t *>(&q), QUAT_PACKET_SIZE);
  udp.endPacket();
}
