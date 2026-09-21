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
volatile uint32_t gFrames = 0, gDrops = 0, gBad = 0, gNoMask = 0, gBytes = 0;

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

// What the host says changed since the last frame it sent (bit t = canvas tile
// t), and which frame that mask belongs to. The device cannot work this out
// for itself: a per-eye 320x240 RGB565 shadow is 307 KB against 82 KB free, so
// the dirty set is computed host-side (it holds the previous RGBA frame
// anyway) and costs 75 B a frame. Lives here rather than in the assembler:
// it is per-frame state, not per-fragment.
uint8_t gMask[VIDEO_MASK_BYTES];
uint16_t gMaskFrame = 0;
bool gMaskValid = false;

// The mask in force for the frame being decoded; nullptr means paint every
// tile (no mask arrived, or the panels were repainted by the idle RGB cycle).
const uint8_t *gPaintMask = nullptr;
// The idle RGB cycle repainted the panels behind our back, so the first frame
// after it must repaint everything: the host's mask is relative to the last
// frame it sent, not to a colour wheel.
bool gPaintAll = false;
// Last frame id that actually reached the panels, so a frame lost in transit
// (its fragments never completed) can be told apart from the heartbeat's
// repeat and from the expected successor.
uint16_t gLastPainted = 0;
bool gHavePainted = false;

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
  constexpr int kCols = VIDEO_W / VIDEO_TILE;
  for (int k = 0; k * (int)VIDEO_TILE < p->iWidthUsed; k++) {
    const int x = p->x + k * (int)VIDEO_TILE;
    const size_t t = (size_t)(p->y / (int)VIDEO_TILE) * kCols + (size_t)(x / (int)VIDEO_TILE);
    if (gPaintMask != nullptr && !(gPaintMask[t >> 3] & (uint8_t)(1u << (t & 7)))) {
      continue; // unchanged since the last frame the host sent
    }
    const uint16_t *src = p->pPixels + k * (int)VIDEO_TILE;
    for (int y = 0; y < (int)VIDEO_TILE; y++) {
      memcpy(gTile + (size_t)y * VIDEO_TILE, src + (size_t)y * p->iWidth, VIDEO_TILE * 2);
    }
    gDisplay->blitTile(x, p->y, gTile);
  }
  return 1;
}

// Decode one assembled JPEG straight into the panels, inside a single SPI
// transaction. `frame_id` is the id from the fragment header, which the mask
// must agree with. Returns false for anything the panels cannot show.
bool paint(const uint8_t *jpeg, size_t len, uint16_t frame_id) {
  if (gJpeg.openRAM(const_cast<uint8_t *>(jpeg), (int)len, drawBlock) == 0) {
    return false;
  }
  // The blit assumes whole MCU blocks: 640x240 is exactly 40x15 16x16 MCUs for
  // the 4:2:0 the sender emits, so iWidthUsed == iWidth everywhere and every
  // block is a clean rectangle of tiles. Anything else is a sender bug, not a
  // frame.
  if (gJpeg.getWidth() != VIDEO_W || gJpeg.getHeight() != VIDEO_H) {
    return false;
  }
  // A mask only means anything for the frame it names and only when the panels
  // are showing the frame it was computed against: a mask from another frame,
  // or one whose baseline frame never reached the panels, would leave tiles
  // stale — so anything untrustworthy paints everything.
  bool forced = gPaintAll;
  gPaintAll = false;
  if (!mask_trustworthy(frame_id, gLastPainted, gHavePainted)) {
    forced = true;
  }
  gLastPainted = frame_id;
  gHavePainted = true;
  gPaintMask = (!forced && gMaskValid && gMaskFrame == frame_id) ? gMask : nullptr;
  if (gPaintMask == nullptr && !forced) {
    gNoMask++;
  }
  gMaskValid = false;
  gJpeg.setPixelType(RGB565_LITTLE_ENDIAN); // matches Display::blitTile's word reads
  // A frame arrived even if nothing needs repainting: that, not the blit, is
  // what must keep the idle RGB cycle away.
  gDisplay->markVideo(millis());
  gDisplay->beginBurst();
  const int ok = gJpeg.decode(0, 0, 0);
  gDisplay->endBurst();
  gPaintMask = nullptr;
  return ok != 0;
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
      if (gDisplay->tick(millis())) {
        gPaintAll = true;
      }
      continue;
    }
    gBytes += n;
    for (size_t i = 0; i < n; i++) {
      uint8_t type = 0;
      uint16_t len = 0;
      if (!parser.push(scratch[i], type, payload, len)) {
        continue;
      }
      if (type == FRAME_VIDEO_MASK) {
        VideoMaskHdr mhdr = {};
        // Only a full, this-canvas mask is usable: parse rejects anything else,
        // and the frame it names is checked again at paint time.
        if (parse_video_mask_hdr(payload, len, mhdr)) {
          memcpy(gMask, payload + 4, VIDEO_MASK_BYTES);
          gMaskFrame = mhdr.frame;
          gMaskValid = true;
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
      // A whole JPEG arrived: decode and paint it before reading on. The RX
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
    // The idle RGB cycle repaints the panels behind our back, so nothing the
    // host last sent still describes them: the next frame repaints in full.
    if (gDisplay->tick(millis())) {
      gPaintAll = true;
    }
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
  s.nomask = gNoMask;
  s.kb = gBytes >> 10;
  gFrames = gDrops = gBad = gNoMask = 0;
  gBytes -= s.kb * 1024; // keep the sub-KiB remainder across intervals
  return s;
}
