#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "config.h"
#include "fusion.h"
#include "imu.h"
#include "proto.h"

static WiFiUDP udp;
static uint32_t pkt_seq = 0;
static uint32_t drop_count = 0;
static uint32_t last_reconnect_try_us = 0;
static bool link_reported = false;

bool net_connect(uint32_t timeout_ms = 15000);
void reconnect_tick();
bool net_send_quat(const QuatPacket &pkt);

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
  if (!net_connect()) {
    Serial.println("WiFi failed, continuing offline");
    neopixelWrite(RGB_LED_PIN, 255, 165, 0); // amber: tracking, no link
  } else {
    link_reported = true;
  }
  Serial.println("UDP ready");
}

void loop() {
  static uint32_t next_us = 0;
  const uint32_t now = micros();
  if ((int32_t)(now - next_us) < 0)
    return; // wait for slot (wrap-safe)
  next_us = now + 10000; // 100 Hz; late slots collapse, never catch up
  if (WiFi.status() != WL_CONNECTED) {
    link_reported = false;
    reconnect_tick();
    return; // skip sample, keep cadence
  }
  if (!link_reported) {
    link_reported = true;
    neopixelWrite(RGB_LED_PIN, 0, 255, 0); // amber->green
    Serial.println("WiFi (re)connected");
  }
  fusion_update();
  QuatPacket pkt;
  pkt.seq = pkt_seq++;
  pkt.micros = micros();
  pkt.quat = fusion_quat();
  net_send_quat(pkt);
}

bool net_connect(uint32_t timeout_ms) {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, pwd);
  Serial.println("");
  Serial.print("Connecting to ");
  Serial.println(ssid);

  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start >= timeout_ms) {
      return false;
    }
    delay(250);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  return true;
}

void reconnect_tick() {
  const uint32_t now = micros();
  if (now - last_reconnect_try_us < 5000000) {
    return; // one attempt per 5 s, wrap-safe
  }
  last_reconnect_try_us = now;
  Serial.println("WiFi lost, reconnecting...");
  WiFi.reconnect();
}

bool net_send_quat(const QuatPacket &pkt) {
  udp.beginPacket(UDP_DEST, UDP_PORT);
  udp.write(reinterpret_cast<const uint8_t *>(&pkt), sizeof(QuatPacket));
  if (udp.endPacket() != 1) {
    if (++drop_count % 100 == 1) {
      Serial.print("UDP drops x");
      Serial.println(drop_count);
    }
    return false;
  }
  return true;
}
