#pragma once

#include <cstdint>
#include <stdint.h>

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
constexpr uint8_t MPU_CONFIG_VAL = 0x03;       // DLPF 44 Hz
constexpr uint8_t MPU_GYRO_CONFIG_VAL = 0x00;  // ±250 dps → matches GYRO_LSB_TO_DPS
constexpr uint8_t MPU_ACCEL_CONFIG_VAL = 0x00; // ±2 g → matches ACCEL_LSB_O_MPS2
constexpr int MPU_CALIB_DELAY_MS = 5;
constexpr uint8_t MPU_I2C_TIMEOUT_MS = 50;
constexpr int GYRO_CALIB_SAMPLES = 100;
constexpr float ACCEL_LSB_O_MPS2 = 9.80665f / 16384.0f;
constexpr float GYRO_LSB_TO_DPS = 1.0f / 131.0f;
constexpr float GRAVITY_MPS2 = 9.80665f;

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
