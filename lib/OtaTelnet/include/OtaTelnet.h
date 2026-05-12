#pragma once

#include <Arduino.h>
#include <functional>

namespace OtaTelnet
{
struct Callbacks
{
  // Return true when the command is accepted and start should proceed.
  // Return false to reject (e.g. already running).
  std::function<bool()> onPlay;
  std::function<void()> onStop;
  std::function<String()> status;

  // Optional: enable/disable FFT filters from the telnet console.
  std::function<void(bool enabled)> setFiltersEnabled;
  std::function<bool()> getFiltersEnabled;

  // Optional: enable/disable verbose measurement status logs.
  std::function<void(bool enabled)> setMeasurementLogsEnabled;
  std::function<bool()> getMeasurementLogsEnabled;
};

void begin(const char *wifiSsid,
           const char *wifiPass,
           const char *otaHostname,
           const char *otaPassword,
           uint16_t otaPort,
           uint16_t telnetPort,
           const char *telnetPassword);

void setCallbacks(Callbacks callbacks);
void loop();

bool isWifiConfigured();
Print *console();

// Fully disable/enable WiFi radio (no transmit when disabled).
// When disabled, telnet and OTA are unavailable.
void enableRadio();
void disableRadio();
bool isRadioEnabled();
} // namespace OtaTelnet
