#pragma once

#include <stdint.h>

constexpr int SDA_PIN = 5;
constexpr int SCL_PIN = 4;

constexpr int MPU_ADDR = 0x68;
constexpr int MPU_PWR_MGMT_1_REG = 0x6B;
constexpr uint8_t MPU_ACCEL_BASE = 0x3B;
constexpr uint8_t MPU_GYRO_BASE = 0x43;

constexpr float ACCEL_LSB_O_MPS2 = 9.80665f / 16384.0f;
constexpr float GYRO_LSB_TO_DPS = 1.0f / 131.0f;
constexpr float GRAVITY_MPS2 = 9.80665f;

// WiFi credentials live in secrets.h (gitignored, see secrets.h.example).
#include "secrets.h"
constexpr const char *ssid = WIFI_SSID;
constexpr const char *pwd = WIFI_PASSWORD;

// UDP destination: unicast to the engine host (broadcast jittered badly).
constexpr const char *UDP_DEST = "192.168.0.233";
// must match engine UDP_PORT
constexpr uint16_t UDP_PORT = 3333;

constexpr int GYRO_CALIB_SAMPLES = 1000;
#ifdef RGB_BUILTIN
constexpr int RGB_LED_PIN = RGB_BUILTIN;
#else
constexpr int RGB_LED_PIN = 48; // onboard WS2812 on most S3 boards
#endif
