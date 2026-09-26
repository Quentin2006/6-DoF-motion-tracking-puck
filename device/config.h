#pragma once

#include "wire.h"
#include <cstdint>

// ------------------------------- MPU -------------------------------
constexpr int SDA_PIN = 5;
constexpr int SCL_PIN = 4;
constexpr uint32_t I2C_CLOCK_HZ = 400000; // MPU6050 I2C rate: the fastest it holds

constexpr int MPU_ADDR = 0x68; // AD0 low; register map lives in mpu6050.cpp
constexpr uint8_t MPU_SMPLRT_DIV_VAL = 0x00; // 1 kHz when DLPF enabled
constexpr uint8_t MPU_CONFIG_VAL =
    0x01; // DLPF 184 Hz gyro / 188 Hz accel, 1 kHz
constexpr uint8_t MPU_GYRO_CONFIG_VAL = 0x18;  // FS_SEL=3 → ±2000 dps
constexpr uint8_t MPU_ACCEL_CONFIG_VAL = 0x10; // AFS_SEL=2 → ±8 g
constexpr int MPU_CALIB_DELAY_MS = 5;
constexpr int GYRO_CALIB_SAMPLES = 200; // ±2000dps stationary noise floor, averaged
constexpr float GRAVITY_MPS2 = 9.80665f;
constexpr float ACCEL_FS_COUNTS = 4096.0f; // ±8 g full scale in counts (AFS_SEL=2)
constexpr float ACCEL_LSB_O_MPS2 = GRAVITY_MPS2 / ACCEL_FS_COUNTS; // m/s² per LSB at ±8 g
constexpr float GYRO_LSB_TO_DPS = 1.0f / 16.4f;         // ±2000 dps
constexpr int FUSION_RATE_HZ = 500;
constexpr unsigned long STATUS_PERIOD_MS = 1000; // 1 Hz UART0 status line (Tracker::report)
constexpr unsigned long VIDEO_COUNTER_PERIOD_MS = 1000; // 1 Hz video-counter line; independent of STATUS_PERIOD_MS by design (one measures the pose path, the other the video task)
constexpr unsigned long CONSOLE_BAUD = 921600;    // UART0 rate for logs and CAL6 tokens
constexpr unsigned long CAL6_WINDOW_MS = 5000;    // boot delay before a CAL6 token is accepted
constexpr unsigned long CAL6_TOKEN_TIMEOUT_MS = 120000; // per-pose, and again for the SAVE confirmation
constexpr float DPS_TO_RPS = 3.14159265f / 180.0f; // deg/s to rad/s
constexpr float EKF_SIGMA_GYRO = 0.002f;   // rad/s/sqrt(Hz): ±2000dps FSR stationary floor ±0.15 dps; quant 0.061 dps/LSB adds 0.018 dps rms (RSS 0.151) — negligible vs electronic noise, value holds
constexpr float EKF_SIGMA_BG = 5.0e-06f;   // rad/s^2/sqrt(Hz): bias random walk, one decade below gyro over 100 s drift
constexpr float EKF_SIGMA_ACCEL = 0.02f;   // normalized-accel units (~0.2 m/s²): ±8g quant 0.0024 m/s²/LSB far below; value holds. If tilt lags, raise x1.5 — never touch filter code
constexpr float EKF_GATE_MAHA = 11.34f;    // chi2 3-dof 99%: reject above this
constexpr float EKF_GATE_NORM = 0.35f;     // soft-gate scale: R deweights above this (step 5), reject far above
constexpr float EKF_GATE_NORM_FAR = 0.70f; // 2x EKF_GATE_NORM: coast above this
constexpr float EKF_GATE_SOFT_K = 10.0f;   // R inflation: R *= 1 + K*(normDev/EKF_GATE_NORM)^2
constexpr float EKF_BIAS_FREEZE_MAHA_FRAC = 0.75f; // below this fraction of EKF_GATE_MAHA the bias is still updated
constexpr float EKF_P_ATT_INIT = 1e-2f;    // attitude covariance seed, rad^2
constexpr float EKF_P_BG_INIT = 1e-6f;     // gyro-bias covariance seed, (rad/s)^2
// Filter-tuning rule for this file: EKF sigmas and gates live here; thresholds
// that only one class needs (StillDetector::k*, Mpu6050's dps/tick clamp) stay
// class-local. Do not merge a class-local constant into this block.
// The driver->body axis map is toBody() in src/position/tracker.cpp; it is a
// swizzle, not a table, so it stays a function.
// ------------------------------- USB BULK LINK -------------------------------
// PUCK_USB_VID / PUCK_USB_PID come from wire.h (generated).

// ------------------------------- DISPLAY -------------------------------
// ESP32-S3 pin assignments. The stereo pair shares MOSI/SCLK/DC/RST/BL and
// has one chip select each: CS is what routes a tile to the left or the right
// eye (Display owns the per-panel CS handling; the panel carries no address).
constexpr int TFT_MOSI = 11;
constexpr int TFT_MISO = -1; // the panels are write-only: no MISO is wired
constexpr int TFT_SCLK = 12;
// The glasses are crossed on the ribbon: the canvas's left half is on
// TFT_CS2 and the right half on TFT_CS. Confirmed on hardware by flooding
// panel 0 alone and watching which glass lit up.
constexpr int TFT_CS = 13;  // panel 0 (canvas left half)
constexpr int TFT_CS2 = 10; // panel 1 (canvas right half)
constexpr int TFT_DC = 9;
constexpr int TFT_RST = 8;
constexpr int TFT_BL = 7;
constexpr int TFT_W = 240;
constexpr int TFT_H = 320;
// Shared hardware reset, Adafruit_SPITFT's own timings. The line is shared, so
// it is pulsed once for both panels, never between their inits.
constexpr unsigned long TFT_RESET_SETUP_MS = 100;  // output enabled before the pulse
constexpr unsigned long TFT_RESET_PULSE_MS = 100; // RST held low
constexpr unsigned long TFT_RESET_WAIT_MS = 200;  // RST high before the first SPI traffic
// Body -> view alignment, the fixed sandwich Tracker::step() bakes into the
// display quaternion (q_disp = A * q_ekf * A^-1). A maps the body's X/Y/Z
// onto the view's yaw/pitch/roll, so the puck turns the way the wearer does:
// A = 180 deg about (1,1,0)/sqrt(2), whose quaternion is
// (DISPLAY_ALIGN_AXIS, DISPLAY_ALIGN_AXIS, 0, 0) — a unit quaternion only
// because sin(90 deg) = 1 and the axis carries the 1/sqrt(2). This is a
// conjugation, not a left-multiply, so the rest orientation the EKF starts
// from is preserved and only the axis labels change.
constexpr float DISPLAY_ALIGN_AXIS = 0.70710678f;
// Per-panel mounting, in the same order as the chip selects (0 = TFT_CS,
// 1 = TFT_CS2): both modules run landscape at 90 deg CW from portrait, and the
// two glasses are bolted 180 deg apart, so one takes rotation 1 and the other
// rotation 3 — both in the same 320x240 logical window. Indexing the rotation
// by panel keeps the mount a fact of this file instead of a property of the
// drawing code.
//
// The table is indexed by *panel*, not by glass, so it swapped along with the
// chip selects above: each physical glass keeps the 180 deg orientation its
// own mounting needs while now serving the opposite eye. Rotations 1 and 3
// are Adafruit's MY|MV and MX|MV, which differ by reversing both axes, so
// rotation ^ 2 is a 180 deg flip of the image. The flip is free and exact —
// 1 and 3 share the same 320x240 logical window and init(240, 320) leaves
// _colstart/_rowstart at 0, so nothing shifts by a pixel — and the rect/tile
// geometry is unaffected: a 180 deg flip maps the 16x16 tile grid onto itself
// and leaves a rect's eye-local coordinates meaning what they meant. A 90 deg
// rotation would move the tiles instead and is not available.
constexpr uint8_t TFT_ROTATION[2] = {1, 3};
// 62.5 MHz: ST7789VW §7.4.3 Table 6 TSCYCW min 16 ns write cycle. Note the
// core's integer divider on the 80 MHz APB cannot reach it — any request from
// 40 to 70 MHz lands on the same divider and yields 40 MHz, measured at
// 3.38 MB/s of pixels through the polling write path (80 MHz, SPI_CLK_EQU_SYSCLK,
// measures 5.17 MB/s but is past the panel's write-cycle spec). One panel's
// full frame is therefore ~45 ms, and the stereo pair's is ~91 ms — 9 fps on
// its own, which is why the host sends a rectangle table and the video task
// blits only the tiles it names (src/display/video.h parse_video_rects_hdr).
constexpr uint32_t TFT_SPI_FREQ = 62500000;
constexpr unsigned long DISPLAY_COLOR_PERIOD_MS = 5000; // solid RGB cycle step
constexpr unsigned long VIDEO_IDLE_MS = 10000; // RGB cycle takes over after this long with no chunks

// Video bulk RX + panel blit task. It stays pinned to core 0 so JPEG decode and
// SPI painting never compete with the core-1 500 Hz fusion loop. BulkUsb uses
// fixed 64-byte chunks, so receive does not perform a FreeRTOS operation per byte.
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

