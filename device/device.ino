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
  if (!net.begin(ssid, pwd, engine_ip, UDP_PORT)) {
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
  tracker.seedEKF();
  if (!net.beginVideo(VIDEO_UDP_PORT)) {
    Serial.println("video bind failed; tracking-only");
  }
  (void)display.begin(); // best-effort: tracking is authoritative, panel advisory
  lastColorMs = millis();
  led.green(); // calibrated, streaming
  lastStatusMs = millis();
}

void loop() {
  const float dt = 1.0f / (float)FUSION_RATE_HZ;
  switch (tracker.step(dt)) {
    case Tracker::Step::Processed:
      net.sendQuat(tracker.quat(), tracker.gyro(), micros());
      break;
    case Tracker::Step::Failed:
      Serial.println("MPU read failed");
      break;
    case Tracker::Step::Skip:
      break;
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
      int y0 = (int)hdr.idx * 3;
      if (rows > 0 && y0 + rows <= VIDEO_H && n == (int)hdr.len) {
        display.writeChunk(0, y0, VIDEO_W, rows, (const uint16_t *)buf);
      }
    }
  }
  unsigned long nowMs = millis();
  if ((unsigned long)(nowMs - lastStatusMs) >= 1000UL) {
    lastStatusMs = nowMs;
    float bx, by, bz;
    tracker.bias(bx, by, bz);
    const float *q = tracker.quat();
    Serial.print("q ");
    Serial.print(q[0], 4);
    Serial.print(' ');
    Serial.print(q[1], 4);
    Serial.print(' ');
    Serial.print(q[2], 4);
    Serial.print(' ');
    Serial.print(q[3], 4);
    Serial.print(" seq ");
    Serial.print(net.sequence());
    Serial.print(" tick ");
    Serial.print(tracker.ticks());
    Serial.print(" over ");
    Serial.print(tracker.overruns());
    Serial.print(" gate ");
    Serial.print(tracker.gated());
    Serial.print(" bg ");
    Serial.print(bx, 6);
    Serial.print(' ');
    Serial.print(by, 6);
    Serial.print(' ');
    Serial.print(bz, 6);
    Serial.print(" st ");
    Serial.println(tracker.stillLatch() ? 1 : 0);
    tracker.resetGated();
  }
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
