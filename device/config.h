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
// discards whatever does not fit, which is exactly what a video burst does.
// Both are applied by the video task (src/display/video_task.cpp), on core 0.
constexpr unsigned long CDC_BAUD = 921600;
constexpr size_t CDC_RX_BYTES = 4096;

// ------------------------------- DISPLAY -------------------------------
// ESP32-S3 pin assignments
constexpr int TFT_MOSI = 11;
constexpr int TFT_SCLK = 12;
constexpr int TFT_CS = 10;
constexpr int TFT_DC = 9;
constexpr int TFT_RST = 8;
constexpr int TFT_BL = 7;
constexpr int TFT_W = 240;
constexpr int TFT_H = 320;
constexpr uint32_t TFT_SPI_FREQ = 62500000; // 62.5 MHz: ST7789VW §7.4.3 Table 6 TSCYCW min 16 ns write cycle; fallback 40 MHz on any tear
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

