#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "config.h"
#include "imu.h"
#include "proto.h"

void fusion_init();
void fusion_calibrate();
void fusion_update();
Quat fusion_quat();

static WiFiUDP udp;

void setup() {
  Serial.begin(115200);
  Serial.flush();
  imu_init();
  Serial.println("Mpu awake");
  fusion_init();
  fusion_calibrate();
  net_connect();
  Serial.println("UDP ready");
}

void loop() {
  fusion_update();
  const Quat q = fusion_quat();
  net_send_quat(q);
}

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
}

void net_send_quat(const Quat &q) {
  udp.beginPacket(UDP_DEST, UDP_PORT);
  udp.write(reinterpret_cast<const uint8_t *>(&q), QUAT_PACKET_SIZE);
  udp.endPacket();
}
