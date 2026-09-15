#pragma once

#include <cstddef>
#include <cstdint>

// ------------------------------- MPU -------------------------------
constexpr int SDA_PIN = 5;
constexpr int SCL_PIN = 4;

constexpr int MPU_ADDR = 0x68;
constexpr int MPU_PWR_MGMT_1_REG = 0x6B;
constexpr uint8_t MPU_ACCEL_BASE = 0x3B;
constexpr uint8_t MPU_GYRO_BASE = 0x43;
constexpr uint8_t MPU_WHO_AM_I_REG = 0x75;
constexpr uint8_t MPU_WHO_AM_I_EXPECT = 0x68;
constexpr uint8_t MPU_WHO_AM_I_EXPECT_ALT = 0x74; // observed clone ID; registers match MPU-6050 map
constexpr uint8_t MPU_SMPLRT_DIV_REG = 0x19;
constexpr uint8_t MPU_CONFIG_REG = 0x1A;
constexpr uint8_t MPU_GYRO_CONFIG_REG = 0x1B;
constexpr uint8_t MPU_ACCEL_CONFIG_REG = 0x1C;
constexpr uint8_t MPU_SMPLRT_DIV_VAL = 0x00;   // 1 kHz when DLPF enabled
constexpr uint8_t MPU_CONFIG_VAL = 0x01;       // DLPF 184 Hz gyro / 188 Hz accel, 1 kHz
constexpr uint8_t MPU_GYRO_CONFIG_VAL = 0x18;  // FS_SEL=3 → ±2000 dps
constexpr uint8_t MPU_ACCEL_CONFIG_VAL = 0x10; // AFS_SEL=2 → ±8 g
constexpr int MPU_CALIB_DELAY_MS = 5;
constexpr uint8_t MPU_I2C_TIMEOUT_MS = 50;
constexpr int GYRO_CALIB_SAMPLES = 200; // ±2000dps noise floor is 8x the old ±250dps scale
constexpr float ACCEL_LSB_O_MPS2 = 9.80665f / 4096.0f; // ±8 g
constexpr float GYRO_LSB_TO_DPS = 1.0f / 16.4f;         // ±2000 dps
constexpr float GRAVITY_MPS2 = 9.80665f;
constexpr uint16_t UDP_PORT = 3333;      // must match engine/src/tracking.rs UDP_PORT
constexpr int FUSION_RATE_HZ = 500;
constexpr float DPS_TO_RPS = 3.14159265f / 180.0f; // deg/s to rad/s
constexpr size_t QUAT_PACKET_SIZE = 24;  // seq u32 + micros u32 + xyzw f32 LE
constexpr float EKF_SIGMA_GYRO = 0.002f;   // rad/s/sqrt(Hz): ±2000dps FSR stationary floor ±0.15 dps; quant 0.061 dps/LSB adds 0.018 dps rms (RSS 0.151) — negligible vs electronic noise, value holds
constexpr float EKF_SIGMA_BG = 5.0e-06f;   // rad/s^2/sqrt(Hz): bias random walk, one decade below gyro over 100 s drift
constexpr float EKF_SIGMA_ACCEL = 0.02f;   // normalized-accel units (~0.2 m/s²): ±8g quant 0.0024 m/s²/LSB far below; value holds. If tilt lags, raise x1.5 — never touch filter code
constexpr float EKF_GATE_MAHA = 11.34f;    // chi2 3-dof 99%: reject above this
constexpr float EKF_GATE_NORM = 0.35f;     // soft-gate scale: R deweights above this (step 5), reject far above
constexpr float EKF_GATE_NORM_FAR = 0.70f; // 2x EKF_GATE_NORM: coast above this
constexpr int8_t FUSION_AXIS_MAP[3] = {3, 2, 1}; // body=(+sz,+sy,+sx): pitch/roll inputs swapped per live test; sensor +X (up, ~9.8 at rest) to body +Z matching EKF gravity [0,0,1]

// ------------------------------- DISPLAY -------------------------------
// Pins are FIXED — user refused rewire. NEVER move them.
//   CLK=18 CS=8 DIN=3 DC=6 RST=9 (BL hardwired to 3V3 rail, NOT GPIO5)
// (a) BL is rail-tied: panel backlight always on. NEVER drive BL_PIN as
//     GPIO — confirmed 2026-09-15 (PWM 0/70/140/255 no change).
// (b) DIN=3 strapping: GPIO3 is a boot-strap pin — no pull-up/down here,
//     and the display must never back-drive DIN at boot or the S3 may
//     fail to enter normal boot.
// (c) GPIO43/44 are UART0 (CH343 console) and GPIO19/20 are native-USB
//     D-/D+. Keep them free — display control lines must never use them.
// (d) Backlight is active HIGH on this panel (proven: LOW=float black
//     ~18.6, HIGH=glow ~34.2). Drive BL_PIN HIGH to light the panel.
// (e) VCC->3V3 + GND common required. MPU unplugged isolates the rail
//     during display bring-up.
constexpr uint8_t CLK_PIN = 18;
constexpr uint8_t CS_PIN = 8;
constexpr uint8_t DIN_PIN = 3;
constexpr uint8_t DC_PIN = 6;
constexpr uint8_t RST_PIN = 9;
// BL rail-tied: no GPIO. Kept as -1 sentinel so stale code fails loudly.
constexpr int BL_PIN = -1;

// WiFi credentials live in secrets.h (gitignored, see secrets.h.example).
#include "secrets.h"
constexpr const char *ssid = WIFI_SSID;
constexpr const char *pwd = WIFI_PASSWORD;
constexpr const char *engine_ip = ENGINE_IP;
