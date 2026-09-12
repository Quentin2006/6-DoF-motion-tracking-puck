#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "config.h"
#include "fusion.h"
#include "imu.h"
#include "proto.h"

static WiFiUDP udp;

void setup() {
  Serial.begin(115200);
  Serial.flush();
  imu_init();
  if (!imu_self_test()) {
    Serial.println("MPU6050 not found");
    for (;;) {
      neopixelWrite(RGB_LED_PIN, 255, 0, 0);
      delay(250);
      neopixelWrite(RGB_LED_PIN, 0, 0, 0);
      delay(250);
    }
  }
  Serial.println("Mpu awake");
  fusion_init();
  fusion_calibrate();
  net_connect();
  Serial.println("UDP ready");
}

void loop() {
  fusion_update();
  static uint32_t pkt_seq = 0;
  QuatPacket pkt;
  pkt.seq = pkt_seq++;
  pkt.micros = micros();
  pkt.quat = fusion_quat();
  net_send_quat(pkt);
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

void net_send_quat(const QuatPacket &pkt) {
  udp.beginPacket(UDP_DEST, UDP_PORT);
  udp.write(reinterpret_cast<const uint8_t *>(&pkt), sizeof(QuatPacket));
  udp.endPacket();
}
