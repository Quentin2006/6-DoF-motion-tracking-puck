#include <Arduino.h> // first: Serial/millis + the FreeRTOS task API

#include "video_task.h"

#include "../../config.h"
#include "display.h"
#include "frame.h"
#include "video.h"
#include <JPEGDEC.h>
#include <cstring>

namespace {

// Cross-core stats: the loop core reads, this task writes. A torn 32-bit read
// at the 1 Hz boundary is harmless — these are rates, not accounting.
volatile uint32_t gFrames = 0, gDrops = 0, gBad = 0, gBytes = 0;

Display *gDisplay = nullptr;
SemaphoreHandle_t gSerialReady = nullptr;
volatile bool gGo = false;

// All of JPEGDEC's state (including its pixel block buffer) lives in the
// object, and the assembler holds the JPEG bytes: no heap, no per-frame
// allocation, nothing to close after openRAM (close() only closes a file
// handle, and there is none).
JPEGDEC gJpeg;
VideoFragAssembler gAsm;
uint32_t gAsmDropped = 0;

// The rectangles the frame in flight is made of, and which frame they belong
// to. The host computes them (it holds the previous RGBA frame anyway) and
// sends one JPEG per rectangle in the blob the fragments carry; the device
// paints each JPEG into its rectangle, so it decodes only what changed.
VideoRectsHdr gRects;
bool gRectsValid = false;

// Where the JPEG being decoded belongs: the rectangle's panel and its origin
// inside that panel. JPEGDEC hands the callback coordinates relative to the
// image it decoded, so the blit position is the rectangle's origin plus those.
int gRectPanel = 0;
int gRectX = 0;
int gRectY = 0;

// One 16x16 tile, staged because a JPEGDEC block's rows are iWidth apart while
// Display::blitTile wants 16 contiguous pixels per row.
uint16_t gTile[VIDEO_TILE * VIDEO_TILE];

// JPEGDEC hands out MCU-aligned blocks (<= 2048 px, so 8 MCUs x 1 at this
// geometry: 128x16), and a block is a whole number of 16x16 tiles — which is
// exactly the 4:2:0 MCU the sender encodes, so a tile never straddles two
// blocks. Each dirty tile is blitted as its own window on its own panel; the
// SPI path tops out at ~3.4 MB/s (measured; the ESP32 pixel write is a 64 B
// polling loop), so painting both panels in full would cost ~91 ms — 9 fps.
int drawBlock(JPEGDRAW *p) {
  const int y = gRectY + p->y;
  for (int k = 0; k * (int)VIDEO_TILE < p->iWidthUsed; k++) {
    const int x = gRectX + p->x + k * (int)VIDEO_TILE;
    const uint16_t *src = p->pPixels + k * (int)VIDEO_TILE;
    for (int row = 0; row < (int)VIDEO_TILE; row++) {
      memcpy(gTile + (size_t)row * VIDEO_TILE, src + (size_t)row * p->iWidth, VIDEO_TILE * 2);
    }
    gDisplay->blitTile(gRectPanel, x, y, gTile);
  }
  return 1;
}

// Decode the frame's blob straight into the panels, one JPEG per dirty
// rectangle, inside a single SPI transaction. `frame_id` is the id from the
// fragment header the blob arrived under, which the rect table must agree
// with. Returns false for anything the panels cannot show.
bool paint(const uint8_t *blob, size_t len, uint16_t frame_id) {
  if (!gRectsValid || gRects.frame != frame_id) {
    return false; // rects from another frame would paint the wrong place
  }
  gRectsValid = false;
  if (len != video_rects_blob_len(gRects)) {
    return false; // a rect's JPEG is missing or extra bytes rode along
  }
  gJpeg.setPixelType(RGB565_LITTLE_ENDIAN); // matches Display::blitTile's word reads
  // A frame arrived even if nothing ends up painted: that, not the blit, is
  // what must keep the idle RGB cycle away.
  gDisplay->markVideo(millis());
  gDisplay->beginBurst();
  size_t off = 0;
  bool ok = true;
  for (size_t i = 0; i < gRects.rects && ok; i++) {
    const VideoRect &r = gRects.rect[i];
    if (gJpeg.openRAM(const_cast<uint8_t *>(blob + off), (int)r.jpeg_len, drawBlock) == 0) {
      ok = false;
      break;
    }
    // The blit assumes whole MCU blocks: a rectangle is 16-px aligned, so it is
    // a whole number of 4:2:0 MCUs and iWidthUsed == iWidth everywhere.
    // Anything else is a sender bug, not a frame.
    if (gJpeg.getWidth() != r.w || gJpeg.getHeight() != r.h) {
      ok = false;
      break;
    }
    gRectPanel = r.eye;
    gRectX = r.x;
    gRectY = r.y;
    ok = gJpeg.decode(0, 0, 0) != 0;
    off += r.jpeg_len;
  }
  gDisplay->endBurst();
  return ok;
}

void videoTaskEntry(void *) {
  uint8_t scratch[1024];
  uint8_t payload[FRAME_MAX_PAYLOAD];
  FrameParser parser;

  // CDC init happens here so its RX interrupt lands on this core (see the
  // header). Grow the RX queue first, or HWCDC::begin creates its 256 B
  // default and a burst overflows it. A failed grow leaves the link unable to
  // carry a frame, so fail-stop like setup()'s MPU halt.
  if (Serial.setRxBufferSize(CDC_RX_BYTES) != CDC_RX_BYTES) {
    while (true) {
      delay(1000);
    }
  }
  Serial.begin(CDC_BAUD);
  xSemaphoreGive(gSerialReady);

  // boot() owns Serial until videoTaskGo() (it reads the CAL6 tokens).
  while (!gGo) {
    vTaskDelay(1);
  }

  for (;;) {
    const size_t n = Serial.read(scratch, sizeof scratch);
    if (n == 0) {
      // Nothing queued: yield the core (the CDC ISR and the idle task need
      // it) and let the panel fall back to its RGB cycle — tick self-gates on
      // VIDEO_IDLE_MS, so calling it every idle millisecond is free.
      vTaskDelay(1);
      gDisplay->tick(millis());
      continue;
    }
    gBytes += n;
    for (size_t i = 0; i < n; i++) {
      uint8_t type = 0;
      uint16_t len = 0;
      if (!parser.push(scratch[i], type, payload, len)) {
        continue;
      }
      if (type == FRAME_VIDEO_RECT) {
        VideoRectsHdr rhdr = {};
        // Only rects this device can paint are usable: anything else is dropped
        // here so the decode path can trust the table.
        if (parse_video_rects_hdr(payload, len, rhdr)) {
          gRects = rhdr;
          gRectsValid = true;
        }
        continue;
      }
      if (type != FRAME_VIDEO) {
        continue;
      }
      VideoFragHdr hdr = {};
      if (!parse_video_frag_hdr(payload, len, hdr)) {
        continue;
      }
      if (!gAsm.push(hdr, payload + VIDEO_HDR_LEN, hdr.len)) {
        continue;
      }
      // The whole blob arrived: decode and paint it before reading on. The RX
      // queue is sized to hold the next frame's fragments while this runs.
      if (paint(gAsm.buf(), gAsm.len(), hdr.frame)) {
        gFrames++;
      } else {
        gBad++;
      }
    }
    const uint32_t dropped = gAsm.dropped();
    gDrops += dropped - gAsmDropped;
    gAsmDropped = dropped;
    // The idle RGB cycle repaints the panels behind our back. Nothing here can
    // repair that — with rectangles the device never holds the whole canvas —
    // so the host's periodic keyframe does, within one keyframe interval.
    gDisplay->tick(millis());
  }
}

} // namespace

void videoTaskStart(Display &display) {
  gDisplay = &display;
  gSerialReady = xSemaphoreCreateBinary();
  if (xTaskCreatePinnedToCore(videoTaskEntry, "video", VIDEO_TASK_STACK, nullptr, VIDEO_TASK_PRIO,
                              nullptr, VIDEO_TASK_CORE) != pdPASS) {
    // Fail-stop, like setup()'s MPU halt: nothing can report this yet, because
    // the task that was going to initialize Serial is the one that failed.
    while (true) {
      delay(1000);
    }
  }
  xSemaphoreTake(gSerialReady, portMAX_DELAY);
}

void videoTaskGo() { gGo = true; }

VideoStats videoTakeStats() {
  VideoStats s;
  s.frames = gFrames;
  s.drops = gDrops;
  s.bad = gBad;
  s.kb = gBytes >> 10;
  gFrames = gDrops = gBad = 0;
  gBytes -= s.kb * 1024; // keep the sub-KiB remainder across intervals
  return s;
}
