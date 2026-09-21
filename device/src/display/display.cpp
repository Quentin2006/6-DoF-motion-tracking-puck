#include <Arduino.h>
#include "display.h"
#include "video.h" // VIDEO_ROTATION, VIDEO_TILE, panel_tile_x

namespace {
// Chip select per panel, in the same order as tft_ (tft_[0] = panel 0).
constexpr int kCsPins[2] = {TFT_CS, TFT_CS2};
} // namespace

Display::Display()
    : tft_{Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST), Adafruit_ST7789(TFT_CS2, TFT_DC, TFT_RST)} {}

void Display::begin() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  // Start the bus once, with the pins the panels are actually wired to: a
  // second init() calls SPI.begin() with all-default pins, and the core
  // returns immediately when the bus is already up (SPIClass::begin exits at
  // the `if (_spi)` check), so panel 1 inherits this bus rather than
  // reconfiguring it.
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  for (int i = 0; i < 2; i++) {
    tft_[i].init(TFT_W, TFT_H);
    // The library's own transactions (fillScreen in tick()) must run at the
    // same speed as the video burst, which sets it explicitly.
    tft_[i].setSPISpeed(TFT_SPI_FREQ);
    // Panels are mounted landscape, matching the engine's video frame layout.
    tft_[i].setRotation(VIDEO_ROTATION);
    tft_[i].fillScreen(ST77XX_BLACK);
  }
  lastColorMs_ = millis();
}

// The burst cannot use Adafruit's startWrite/endWrite: those drive the CS of
// the instance they belong to, and blitTile paints whichever panel owns the
// tile — so panel 1's pixels would be clocked in while panel 0 was selected
// and land on panel 0. The transaction is opened here instead, and blitTile
// selects the panel per tile.
void Display::beginBurst() { SPI.beginTransaction(SPISettings(TFT_SPI_FREQ, MSBFIRST, SPI_MODE0)); }

void Display::endBurst() { SPI.endTransaction(); }

void Display::blitTile(int x, int y, const uint16_t *px) {
  int localX = 0;
  const int panel = panel_tile_x(x / (int)VIDEO_TILE, &localX);
  if (panel < 0) {
    return;
  }
  Adafruit_ST7789 &tft = tft_[panel];
  digitalWrite(kCsPins[panel], LOW);
  tft.setAddrWindow((uint16_t)localX, (uint16_t)y, (uint16_t)VIDEO_TILE, (uint16_t)VIDEO_TILE);
  // Adafruit's 2nd argument is a PIXEL count (w*h*2 bytes on the wire).
  // bigEndian=false is the word swap the panel needs: SPIClass::writePixels
  // swaps in hardware on ESP32.
  tft.writePixels(const_cast<uint16_t *>(px), (uint32_t)(VIDEO_TILE * VIDEO_TILE), true, false);
  digitalWrite(kCsPins[panel], HIGH);
  lastVideoMs_ = millis();
}

bool Display::tick(unsigned long now) {
  const bool videoIdle = lastVideoMs_ == 0 || (unsigned long)(now - lastVideoMs_) > VIDEO_IDLE_MS;
  const bool due = (unsigned long)(now - lastColorMs_) >= DISPLAY_COLOR_PERIOD_MS;
  if (!videoIdle || !due) {
    return false;
  }
  lastColorMs_ = now;
  static const uint16_t kColors[3] = {ST77XX_RED, ST77XX_GREEN, ST77XX_BLUE};
  colorIdx_ = (uint8_t)((colorIdx_ + 1) % 3);
  for (int i = 0; i < 2; i++) {
    tft_[i].fillScreen(kColors[colorIdx_]);
  }
  return true;
}
