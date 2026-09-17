#include "config.h"
#include "tracker.h"
#include "net.h"
#include "led.h"
#include "display.h"
#include "video.h"
#include <Arduino.h>

Tracker tracker;
Net net;
StatusLed led;
Display display;
unsigned long lastStatusMs = 0;
unsigned long lastColorMs = 0;
unsigned long videoActiveMs = 0;
bool videoSeen = false;
uint8_t colorIdx = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  led.red(); // calibrating / not ready
  if (!tracker.begin()) {
    Serial.println("MPU6050 init failed");
    while (true) {
      delay(100);
    }
  }
  if (!net.begin(WIFI_SSID, WIFI_PASSWORD)) {
    Serial.println("WiFi failed");
    while (true) {
      delay(1000);
    }
  }
  delay(500);
  Serial.println("calibrating...");
  tracker.calibrate();
  Serial.println("calibrated");
  tracker.printAccelCal();
  Serial.println("send CAL6 within 5 s for accel cal");
  (void)tracker.maybeRunAccelCal(5000UL);
  while (Serial.available() > 0) {
    Serial.read();
  }
  tracker.start();
  if (!net.beginVideo()) {
    Serial.println("video bind failed; tracking-only");
  }
  (void)display.begin(); // best-effort: tracking is authoritative, panel advisory
  lastColorMs = millis();
  led.green(); // calibrated, streaming
  lastStatusMs = millis();
}

void loop() {
  if (tracker.step()) {
    net.sendQuat(tracker.quat(), tracker.gyro());
  }
  // Video: one datagram per loop max (never a drain — starves 500 Hz fusion).
  // Chunks overwrite in place; loss tears, next frame_id keeps overwriting.
  {
    VideoChunkHdr hdr;
    uint8_t buf[VIDEO_PAYLOAD_MAX];
    int n = net.pollVideoChunk(buf, hdr);
    if (n > 0) {
      videoActiveMs = millis();
      videoSeen = true;
      int rows = n / (VIDEO_W * 2);
      int y0 = (int)hdr.idx * 2;
      if (rows > 0 && y0 + rows <= VIDEO_H && n == (int)hdr.len) {
        display.writeChunk(0, y0, VIDEO_W, rows, (const uint16_t *)buf);
      }
    }
  }
  tracker.report(net.sequence());
  const unsigned long nowMs = millis();
  // RGB demo resumes only after 10 s of video silence so it never fights video.
  // videoSeen keeps the first 10 s demo intact (videoActiveMs starts at 0).
  if ((unsigned long)(nowMs - lastColorMs) >= DISPLAY_COLOR_PERIOD_MS &&
      (!videoSeen || (unsigned long)(nowMs - videoActiveMs) > 10000UL)) {
    lastColorMs = nowMs;
    colorIdx = (uint8_t)((colorIdx + 1) % 3);
    const uint16_t c = (colorIdx == 0) ? ST77XX_RED : (colorIdx == 1) ? ST77XX_GREEN : ST77XX_BLUE;
    display.fill(c);
  }
}
