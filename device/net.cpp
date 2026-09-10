#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "config.h"
#include "net.h"

// The udp library class
static WiFiUDP udp;

static const char *net_status_name(wl_status_t status) {
  switch (status) {
    case WL_CONNECTED: return "CONNECTED";
    case WL_IDLE_STATUS: return "IDLE";
    case WL_NO_SSID_AVAIL: return "NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "SCAN_COMPLETED";
    case WL_CONNECT_FAILED: return "CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED: return "DISCONNECTED";
    default: return "UNKNOWN";
  }
}

static const char *net_auth_name(wifi_auth_mode_t mode) {
  switch (mode) {
    case WIFI_AUTH_OPEN: return "OPEN";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA_PSK";
    case WIFI_AUTH_WPA2_PSK: return "WPA2_PSK";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA_WPA2_PSK";
    case WIFI_AUTH_ENTERPRISE: return "ENTERPRISE";
    case WIFI_AUTH_WPA3_PSK: return "WPA3_PSK";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2_WPA3_PSK";
    case WIFI_AUTH_WAPI_PSK: return "WAPI_PSK";
    default: return "UNKNOWN";
  }
}

static void net_scan() {
  Serial.println("Scanning nearby networks...");
  const int16_t count = WiFi.scanNetworks();
  if (count < 0) {
    Serial.println("Scan failed");
    return;
  }
  if (count == 0) {
    Serial.println("No networks visible (AP must be 2.4 GHz for ESP32)");
    return;
  }
  for (int16_t i = 0; i < count; ++i) {
    Serial.print(WiFi.SSID(i));
    Serial.print(" ch=");
    Serial.print(WiFi.channel(i));
    Serial.print(" rssi=");
    Serial.print(WiFi.RSSI(i));
    Serial.print(" auth=");
    Serial.println(net_auth_name(WiFi.encryptionType(i)));
  }
  WiFi.scanDelete();
}

void net_connect() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  net_scan();
  WiFi.begin("Asgard", "13Hubble!23");
  Serial.println("");
  Serial.print("Connecting to ");
  Serial.println(ssid);

  uint32_t attempts = 0;
  uint32_t reports = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++attempts % 20 == 0) {
      const wl_status_t status = WiFi.status();
      Serial.println("");
      Serial.print("WiFi status: ");
      Serial.print(net_status_name(status));
      Serial.print(" (");
      Serial.print(static_cast<int>(status));
      Serial.println(")");
      if (++reports % 6 == 0) {
        net_scan();
      }
    }
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // This initializes udp and transfer buffer
  udp.begin(udpPort);
}

void net_send_quat(const Quat &q) {
  udp.beginPacket(udpAddress, udpPort);
  udp.write(reinterpret_cast<const uint8_t *>(&q), QUAT_PACKET_SIZE);
  udp.endPacket();
}
