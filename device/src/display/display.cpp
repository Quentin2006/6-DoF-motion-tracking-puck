#include <Arduino.h>
#include "display.h"
#include "video.h" // VIDEO_TILE

namespace {
// Chip select per panel, in the same order as tft_ (tft_[0] = panel 0).
constexpr int kCsPins[2] = {TFT_CS, TFT_CS2};
} // namespace

// Neither instance owns the reset pin. The panels share TFT_RST, and
// Adafruit's init() *always* pulses the reset line it was given, so a second
// instance constructed with it resets the first panel again — after that
// panel's own init issued SLPOUT/DISPON — and leaves panel 0 asleep and blank.
// begin() pulses the shared line once instead, and each panel's own init still
// sends the software reset that is addressed to it.
Display::Display()
    : tft_{Adafruit_ST7789(TFT_CS, TFT_DC, -1), Adafruit_ST7789(TFT_CS2, TFT_DC, -1)} {}

void Display::begin() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  // One hardware reset for both panels, with the timings Adafruit_SPITFT's own
  // reset uses. It has to precede both inits and must not be repeated between
  // them: the line is shared, so a second pulse would clear whichever panel was
  // initialized first. The level is set before the driver is enabled, so
  // enabling it (which applies the port's current latch value) does not pulse
  // the line low first.
  digitalWrite(TFT_RST, HIGH);
  pinMode(TFT_RST, OUTPUT);
  delay(100);
  digitalWrite(TFT_RST, LOW);
  delay(100);
  digitalWrite(TFT_RST, HIGH);
  delay(200);
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
    // Landscape logical space, per panel: the mounting differs (config.h).
    tft_[i].setRotation(TFT_ROTATION[i]);
    tft_[i].fillScreen(ST77XX_BLACK);
  }
  lastColorMs_ = millis();
}

// The burst cannot use Adafruit's startWrite/endWrite: those drive the CS of
// the instance they belong to, and blitTile paints whichever panel the frame's
// rectangle named — so panel 1's pixels would be clocked in while panel 0 was
// selected and land on panel 0. The transaction is opened here instead, and
// blitTile selects the panel per tile.
void Display::beginBurst() { SPI.beginTransaction(SPISettings(TFT_SPI_FREQ, MSBFIRST, SPI_MODE0)); }

void Display::endBurst() { SPI.endTransaction(); }

void Display::blitTile(int panel, int x, int y, const uint16_t *px) {
  Adafruit_ST7789 &tft = tft_[panel];
  digitalWrite(kCsPins[panel], LOW);
  tft.setAddrWindow((uint16_t)x, (uint16_t)y, (uint16_t)VIDEO_TILE, (uint16_t)VIDEO_TILE);
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
