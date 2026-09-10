#pragma once

#include "imu.h"
#include <stdint.h>

constexpr int SDA_PIN = 5;
constexpr int SCL_PIN = 4;

constexpr int MPU_ADDR = 0x68;
constexpr int MPU_PWR_MGMT_1_REG = 0x6B;
constexpr vec3 MPU_ACCEL_REGS{0x3B, 0x3D, 0x3F};
constexpr vec3 MPU_GYRO_REGS{0x43, 0x45, 0x47};

constexpr float ACCEL_LSB_O_MPS2 = 9.80665f / 16384.0f;
constexpr float GYRO_LSB_TO_DPS = 1.0f / 131.0f;
constexpr float GRAVITY_MPS2 = 9.80665f;

// WiFi credentials live in secrets.h (gitignored, see secrets.h.example).
#include "secrets.h"
constexpr const char *ssid = WIFI_SSID;
constexpr const char *pwd = WIFI_PASSWORD;

// UDP destination: server IP or broadcast address
constexpr const char *udpAddress = "192.168.0.233";
constexpr const int udpPort = 3333;

constexpr uint32_t SEND_INTERVAL_MS = 10;
constexpr int GYRO_CALIB_SAMPLES = 1000;
#ifdef RGB_BUILTIN
constexpr int RGB_LED_PIN = RGB_BUILTIN;
#else
constexpr int RGB_LED_PIN = 48; // onboard WS2812 on most S3 boards
#endif
