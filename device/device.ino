#include <Arduino.h> // first: defines RGB_BUILTIN, used by config.h
#include "config.h"
#include "tracker.h"
#include "packet.h"
#include "link.h"
#include "display.h"
#include "video.h"

Tracker tracker;
UsbLink usbLink;
Display display;

// Deterministic fail-stop: a puck with no IMU has nothing to do.
static void halt(const char *why) {
  Serial.println(why);
  while (true) {
    delay(1000);
  }
}

void setup() {
  usbLink.begin(); // Serial + 4 KB RX buffer; never fails
  delay(500);
  rgbLedWrite(RGB_LED_PIN, 255, 0, 0); // calibrating / not ready
  if (!tracker.begin()) {
    halt("MPU6050 init failed");
  }
  delay(500);
  Serial.println("calibrating...");
  tracker.calibrate();
  Serial.println("calibrated");
  tracker.printAccelCal();
  Serial.println("send CAL6 within 5 s for accel cal");
  tracker.maybeRunAccelCal(5000UL);
  tracker.start();
  display.begin();
  rgbLedWrite(RGB_LED_PIN, 0, 255, 0); // calibrated, streaming
}

void loop() {
  const unsigned long loopT0 = micros();
  // Step 2 window instrumentation: fusion/video/report spans + drain exit
  // reasons; link deltas vs cumulative UsbLink counters. Reset on print.
  static uint32_t pSent = 0, pDrop = 0, pRx = 0, pCh = 0, pDec = 0;
  static LoopWin win;

  const unsigned long tFuse0 = micros();
  const bool fresh = tracker.step();
  const unsigned tFuse = (unsigned)(micros() - tFuse0);
  if (fresh) {
    usbLink.sendQuat(tracker.quat(), tracker.gyro());
  }
  if (tFuse > win.tFusionMax) {
    win.tFusionMax = tFuse;
  }

  // Video: budgeted multi-chunk drain (never drain-all — it starves 500 Hz
  // fusion). Max 16 chunks under a 1200 us budget inside one SPI
  // transaction; fusion + ~0.35 ms I2C must still fit the 2000 us tick.
  // vidBuf must be 4-byte aligned: Display::chunkInner -> writePixels SPI
  // DMA reads 16-bit words, and a misaligned stack byte buffer corrupts on
  // word boundaries (periodic per-pixel pattern).
  alignas(4) static uint16_t vidBuf[VIDEO_PAYLOAD_MAX / 2];
  constexpr int kMaxChunks = 16;
  constexpr unsigned long kBudgetUs = 1200;
  const unsigned long tVid0 = micros();
  int n = 0;
  bool open = false;
  bool idleExit = false;
  while (n < kMaxChunks && (unsigned long)(micros() - tVid0) < kBudgetUs) {
    VideoChunkHdr hdr = {};
    const int got = usbLink.pollVideoChunk((uint8_t *)vidBuf, hdr);
    if (got <= 0) {
      idleExit = true;
      break;
    }
    if (!open) {
      display.beginBurst();
      open = true;
    }
    display.chunkInner(hdr, vidBuf, got);
    n++;
  }
  if (open) {
    display.endBurst();
  }
  const unsigned tVid = (unsigned)(micros() - tVid0);
  if (tVid > win.tVideoMax) {
    win.tVideoMax = tVid;
  }
  if (idleExit) {
    win.exitIdle++;
    win.lastExit = 0;
  } else if (n >= kMaxChunks) {
    win.exitMax++;
    win.lastExit = 2;
  } else {
    win.exitBudget++;
    win.lastExit = 1;
  }
  win.lastN = (uint8_t)n;

  const UsbWin delta{usbLink.sent() - pSent, usbLink.dropped() - pDrop,
                     usbLink.rxBytes() - pRx, usbLink.chunks() - pCh,
                     usbLink.decodeDrops() - pDec};
  const unsigned long tRep0 = micros();
  const bool printed = tracker.report(usbLink.sent(), delta, win);
  const unsigned tRep = (unsigned)(micros() - tRep0);
  if (printed) {
    pSent = usbLink.sent();
    pDrop = usbLink.dropped();
    pRx = usbLink.rxBytes();
    pCh = usbLink.chunks();
    pDec = usbLink.decodeDrops();
    win = LoopWin{};
  }
  // Current tRep/loop totals open the next window (report already printed).
  if (tRep > win.tReportMax) {
    win.tReportMax = tRep;
  }
  display.tick(millis());
  const unsigned loopUs = (unsigned)(micros() - loopT0);
  if (loopUs > win.loopMax) {
    win.loopMax = loopUs;
  }
}
