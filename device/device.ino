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
  static uint32_t pSent = 0, pDrop = 0, pRx = 0, pCh = 0, pDec = 0, pFd = 0;
  static LoopWin win;
  // Frame barrier: the engine bursts idx 0..119 ascending per frame with the
  // frame id wrapping u16. Paint only the active frame — once a newer frame
  // starts, rows from any older frame are stale and must never paint over
  // it. Stale rows are counted, never silently lost. Adoption is by newer
  // frame id (wrapping compare), never by idx 0: if idx 0 of a frame is lost
  // to a CRC/stash reject, waiting for idx 0 would drop every later chunk of
  // that frame forever and freeze the panel on a partial image. Adopting the
  // first live chunk instead tears at most one frame. A restart streak (8
  // consecutive drops of one id the wrap compare calls older) means the
  // engine's ids restarted, so adopt it rather than stall for good.
  static bool haveFrame = false;
  static uint16_t activeFrame = 0;
  static uint16_t dropFrame = 0;
  static uint8_t dropStreak = 0;
  constexpr uint8_t kAdoptStreak = 8;

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
  // fusion). Step 5: max 8 chunks under the same 1200 us budget — at
  // 62.5 MHz SPI one 1280 B chunk is ~164 us + setAddrWindow, so 16 was
  // ~2.6 ms > the 2000 us tick while 8 fits with fusion + ~0.35 ms I2C.
  // vidBuf must be 4-byte aligned: Display::chunkInner -> writePixels SPI
  // DMA reads 16-bit words, and a misaligned stack byte buffer corrupts on
  // word boundaries (periodic per-pixel pattern).
  alignas(4) static uint16_t vidBuf[VIDEO_PAYLOAD_MAX / 2];
  constexpr int kMaxChunks = 8;
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
    if (!haveFrame || hdr.frame != activeFrame) {
      // Newest-wins with u16 wrap: distance < 32768 means hdr.frame is newer.
      // (A chunk of the active frame skips this block and paints.) A newer id
      // adopts on any idx — waiting for idx 0 would freeze on a partial frame
      // forever when idx 0 is lost — so a lost idx 0 tears at most one frame.
      const bool newer = !haveFrame || (uint16_t)(hdr.frame - activeFrame) < 0x8000u;
      bool adopt = newer;
      if (!adopt) {
        // Older id: stale row, unless the engine's ids restarted (then every
        // new chunk reads "older" forever). Streak-adopt after 8 consecutive
        // drops of one id rather than stall for good.
        if (hdr.frame == dropFrame) {
          ++dropStreak;
        } else {
          dropFrame = hdr.frame;
          dropStreak = 1;
        }
        adopt = dropStreak >= kAdoptStreak;
      }
      if (!adopt) {
        usbLink.noteFrameDrop();
        continue;
      }
      activeFrame = hdr.frame;
      haveFrame = true;
      dropStreak = 0;
    } else {
      dropStreak = 0;
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
                     usbLink.decodeDrops() - pDec, usbLink.frameDrops() - pFd};
  const unsigned long tRep0 = micros();
  const bool printed = tracker.report(usbLink.sent(), delta, win);
  const unsigned tRep = (unsigned)(micros() - tRep0);
  if (printed) {
    pSent = usbLink.sent();
    pDrop = usbLink.dropped();
    pRx = usbLink.rxBytes();
    pCh = usbLink.chunks();
    pDec = usbLink.decodeDrops();
    pFd = usbLink.frameDrops();
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
