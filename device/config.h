#pragma once

#include <cstddef>
#include <cstdint>

// ------------------------------- MPU -------------------------------
constexpr int SDA_PIN = 5;
constexpr int SCL_PIN = 4;

constexpr int MPU_ADDR = 0x68; // AD0 low; register map lives in mpu6050.cpp
constexpr uint8_t MPU_SMPLRT_DIV_VAL = 0x00; // 1 kHz when DLPF enabled
constexpr uint8_t MPU_CONFIG_VAL =
    0x01; // DLPF 184 Hz gyro / 188 Hz accel, 1 kHz
constexpr uint8_t MPU_GYRO_CONFIG_VAL = 0x18;  // FS_SEL=3 → ±2000 dps
constexpr uint8_t MPU_ACCEL_CONFIG_VAL = 0x10; // AFS_SEL=2 → ±8 g
constexpr int MPU_CALIB_DELAY_MS = 5;
constexpr int GYRO_CALIB_SAMPLES = 200; // ±2000dps noise floor is 8x the old ±250dps scale
constexpr float ACCEL_LSB_O_MPS2 = 9.80665f / 4096.0f; // ±8 g
constexpr float GYRO_LSB_TO_DPS = 1.0f / 16.4f;         // ±2000 dps
constexpr float GRAVITY_MPS2 = 9.80665f;
constexpr int FUSION_RATE_HZ = 500;
constexpr unsigned long STATUS_PERIOD_MS = 1000; // 1 Hz Serial status line (Tracker::report)
constexpr float DPS_TO_RPS = 3.14159265f / 180.0f; // deg/s to rad/s
constexpr float EKF_SIGMA_GYRO = 0.002f;   // rad/s/sqrt(Hz): ±2000dps FSR stationary floor ±0.15 dps; quant 0.061 dps/LSB adds 0.018 dps rms (RSS 0.151) — negligible vs electronic noise, value holds
constexpr float EKF_SIGMA_BG = 5.0e-06f;   // rad/s^2/sqrt(Hz): bias random walk, one decade below gyro over 100 s drift
constexpr float EKF_SIGMA_ACCEL = 0.02f;   // normalized-accel units (~0.2 m/s²): ±8g quant 0.0024 m/s²/LSB far below; value holds. If tilt lags, raise x1.5 — never touch filter code
constexpr float EKF_GATE_MAHA = 11.34f;    // chi2 3-dof 99%: reject above this
constexpr float EKF_GATE_NORM = 0.35f;     // soft-gate scale: R deweights above this (step 5), reject far above
constexpr float EKF_GATE_NORM_FAR = 0.70f; // 2x EKF_GATE_NORM: coast above this

// ------------------------------- CDC LINK -------------------------------
// Serial is the ESP32-S3 USB-Serial-JTAG peripheral (FQBN CDCOnBoot=cdc), so
// baud is a no-op for throughput: the pipe is capped by USB delivery (~512
// kB/s measured), not by a line rate. The RX queue must be grown before
// Serial.begin or HWCDC creates its 256 B default — and HWCDC's ISR silently
// discards whatever does not fit.
//
// It has to hold a whole frame's fragments plus a full-canvas repaint: the
// video task decodes and paints one 640x240 JPEG (~20 ms decode + ~6 ms of
// dirty tiles) before it reads again, but when every tile is dirty (the first
// frame, or the first frame after the idle RGB cycle) the blit alone is ~102 ms
// at the measured SPI rate, and at ~85 kB/s that is ~9 kB arriving while the
// task is not reading. Measured on the puck: with 16 KB a full-repaint frame
// overran the queue and cost 2 frames; with 32 KB the queue never approaches
// the ceiling. Both are applied by the video task
// (src/display/video_task.cpp), on core 0.
constexpr unsigned long CDC_BAUD = 921600;
constexpr size_t CDC_RX_BYTES = 32768;

// ------------------------------- DISPLAY -------------------------------
// ESP32-S3 pin assignments. The stereo pair shares MOSI/SCLK/DC/RST/BL and
// has one chip select each: CS is what routes a tile to the left or the right
// eye (Display owns the per-panel CS handling; the panel carries no address).
constexpr int TFT_MOSI = 11;
constexpr int TFT_SCLK = 12;
constexpr int TFT_CS = 10;  // panel 0 (canvas left half)
constexpr int TFT_CS2 = 13; // panel 1 (canvas right half)
constexpr int TFT_DC = 9;
constexpr int TFT_RST = 8;
constexpr int TFT_BL = 7;
constexpr int TFT_W = 240;
constexpr int TFT_H = 320;
// Per-panel mounting, in the same order as the chip selects (0 = TFT_CS,
// 1 = TFT_CS2): both modules run landscape at 90 deg CW from portrait
// (rotation 1), but panel 1 is mounted the other way up, so it takes rotation
// 3 — the same landscape window with both axes flipped, i.e. 180 deg from
// panel 0. Indexing the rotation keeps the mount a fact of this file instead
// of a property of the drawing code.
//
// Both panels read 180 deg from where the modules are bolted (the headset is
// worn the other way up): rotation 1 and 3 are Adafruit's MY|MV and MX|MV, which
// differ by reversing both axes, so rotation ^ 2 is a 180 deg flip of the image.
// The flip is free and exact — 1 and 3 share the same 320x240 logical window and
// init(240, 320) leaves _colstart/_rowstart at 0, so nothing shifts by a pixel —
// and the rect/tile geometry is unaffected: a 180 deg flip maps the 16x16 tile
// grid onto itself and leaves a rect's eye-local coordinates meaning what they
// meant. A 90 deg rotation would move the tiles instead and is not available.
constexpr uint8_t TFT_ROTATION[2] = {1, 3};
// 62.5 MHz: ST7789VW §7.4.3 Table 6 TSCYCW min 16 ns write cycle. Note the
// core's integer divider on the 80 MHz APB cannot reach it — any request from
// 40 to 70 MHz lands on the same divider and yields 40 MHz, measured at
// 3.38 MB/s of pixels through the polling write path (80 MHz, SPI_CLK_EQU_SYSCLK,
// measures 5.17 MB/s but is past the panel's write-cycle spec). One panel's
// full frame is therefore ~45 ms, and the stereo pair's is ~91 ms — 9 fps on
// its own, which is why the host sends a dirty-tile mask and the video task
// blits only the tiles it names (src/display/video_task.cpp).
constexpr uint32_t TFT_SPI_FREQ = 62500000;
constexpr unsigned long DISPLAY_COLOR_PERIOD_MS = 5000; // solid RGB cycle step
constexpr unsigned long VIDEO_IDLE_MS = 10000; // RGB cycle takes over after this long with no chunks

// Video RX + panel blit task (src/display/video_task.cpp). It owns the whole
// Serial RX path, and it is pinned to core 0 because the CDC stack pops one
// FreeRTOS queue item per byte: at ~400 kB/s that is ~400k pops/s on whatever
// core calls read(), and on core 1 (the Arduino loop core, where the 500 Hz
// fusion tick lives) it starves Tracker::step(). Core 1 keeps the loop task to
// itself; the burst cost moves to a core that has nothing else to do.
constexpr int VIDEO_TASK_CORE = 0;
constexpr int VIDEO_TASK_PRIO = 5;
constexpr uint32_t VIDEO_TASK_STACK = 8192; // bytes: scratch + FrameParser + payload copy

// ------------------------------- STATUS LED -------------------------------
// Onboard WS2812 addressable RGB (GPIO48 on the stock S3 devkit). Driven via
// the core's rgbLedWrite(pin, R, G, B) (GRB reordered internally). Used as a
// calibration/debug light: red while calibrating, green when done.
#ifdef RGB_BUILTIN
constexpr int RGB_LED_PIN = RGB_BUILTIN;
#else
constexpr int RGB_LED_PIN = 48; // onboard WS2812 on most S3 boards
#endif

