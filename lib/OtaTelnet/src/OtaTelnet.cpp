#include "OtaTelnet.h"

#if defined(ARDUINO_ARCH_ESP32)

#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>

#include <LogMirror.h>

namespace OtaTelnet
{
namespace
{
enum class WifiState : uint8_t
{
  RadioOff,
  Connecting,
  Ready,
};

Callbacks callbacks;
WiFiClient telnetClient;
String telnetLine;
bool telnetAuthenticated = false;
bool wifiConfigured = false;

uint16_t configuredTelnetPort = 23;
String configuredTelnetPassword;
WiFiServer *telnetServer = nullptr;

String configuredSsid;
String configuredPass;
String configuredHostname;
String configuredOtaPassword;
uint16_t configuredOtaPort = 3232;

WifiState wifiState = WifiState::RadioOff;
unsigned long connectStartedAtMs = 0;
constexpr unsigned long CONNECT_TIMEOUT_MS = 12000;

void stopRadio()
{
  telnetClient.stop();
  telnetAuthenticated = false;
  wifiConfigured = false;
  wifiState = WifiState::RadioOff;

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

void startRadio()
{
  if (configuredSsid.length() == 0)
  {
    stopRadio();
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(configuredSsid.c_str(), configuredPass.c_str());
  connectStartedAtMs = millis();
  wifiState = WifiState::Connecting;
}

void onConnected()
{
  auto &log = LogMirror::out();
  log.print("WiFi OK, IP: ");
  log.println(WiFi.localIP());

  if (configuredHostname.length() > 0 && MDNS.begin(configuredHostname.c_str()))
  {
    log.print("mDNS: ");
    log.print(configuredHostname);
    log.println(".local");
  }

  if (configuredHostname.length() > 0)
  {
    ArduinoOTA.setHostname(configuredHostname.c_str());
  }
  ArduinoOTA.setPort(configuredOtaPort);
  if (configuredOtaPassword.length() > 0)
  {
    ArduinoOTA.setPassword(configuredOtaPassword.c_str());
  }
  ArduinoOTA.onStart([]() { LogMirror::out().println("OTA start"); });
  ArduinoOTA.onEnd([]() { LogMirror::out().println("OTA end"); });
  ArduinoOTA.onError([](ota_error_t error) {
    auto &log = LogMirror::out();
    log.print("OTA error: ");
    log.println(static_cast<int>(error));
  });
  ArduinoOTA.begin();
  log.print("OTA ready (ArduinoOTA), port: ");
  log.println(configuredOtaPort);

  if (telnetServer)
  {
    telnetServer->begin();
    log.print("Telnet port: ");
    log.println(configuredTelnetPort);
  }

  wifiConfigured = true;
  wifiState = WifiState::Ready;
}

void telnetPrintln(const String &line)
{
  if (telnetClient && telnetClient.connected())
  {
    telnetClient.println(line);
  }
}

void telnetPrompt()
{
  if (telnetClient && telnetClient.connected())
  {
    telnetClient.print("> ");
  }
}

void handleTelnetCommand(String command)
{
  command.trim();
  String lower = command;
  lower.toLowerCase();

  if (!telnetAuthenticated)
  {
    if (lower.startsWith("pass "))
    {
      String supplied = command.substring(5);
      supplied.trim();
      if (supplied == configuredTelnetPassword)
      {
        telnetAuthenticated = true;
        telnetPrintln("OK");
      }
      else
      {
        telnetPrintln("Spatne heslo");
      }
      return;
    }

    telnetPrintln("Neautorizovano. Zadejte: pass <heslo>");
    return;
  }

  if (lower == "help")
  {
    telnetPrintln("help                - tato napoveda");
    telnetPrintln("status              - stav ladeni");
    telnetPrintln("play / start        - spustit ladeni");
    telnetPrintln("stop                - zastavit ladeni");
    telnetPrintln("filters on/off      - FFT filtry zap/vyp");
    telnetPrintln("reboot              - restart ESP32");
    telnetPrintln("ip                  - IP adresa");
    return;
  }

  if (lower == "status")
  {
    if (callbacks.status)
    {
      telnetPrintln(callbacks.status());
    }
    else
    {
      telnetPrintln(String("wifi: ") + (WiFi.status() == WL_CONNECTED ? "connected" : "disconnected"));
    }
    return;
  }

  if (lower == "ip")
  {
    telnetPrintln(WiFi.localIP().toString());
    return;
  }

  if (lower == "filters")
  {
    if (!callbacks.getFiltersEnabled)
    {
      telnetPrintln("NOT SUPPORTED");
      return;
    }
    telnetPrintln(String("filters: ") + (callbacks.getFiltersEnabled() ? "on" : "off"));
    return;
  }

  if (lower == "filters on" || lower == "filters off")
  {
    if (!callbacks.setFiltersEnabled)
    {
      telnetPrintln("NOT SUPPORTED");
      return;
    }
    callbacks.setFiltersEnabled(lower == "filters on");
    telnetPrintln("OK");
    return;
  }

  if (lower == "play" || lower == "start")
  {
    bool accepted = true;
    if (callbacks.onPlay)
    {
      accepted = callbacks.onPlay();
    }
    telnetPrintln(accepted ? "OK" : "BUSY");
    return;
  }

  if (lower == "stop")
  {
    if (callbacks.onStop)
    {
      callbacks.onStop();
    }
    telnetPrintln("OK");
    return;
  }

  if (lower == "reboot")
  {
    telnetPrintln("Restartuji...");
    delay(50);
    ESP.restart();
  }

  telnetPrintln("Neznamy prikaz. Zkuste: help");
}

void serviceTelnet()
{
  if (!wifiConfigured || wifiState != WifiState::Ready || WiFi.status() != WL_CONNECTED || !telnetServer)
  {
    return;
  }

  if (!telnetClient || !telnetClient.connected())
  {
    WiFiClient newClient = telnetServer->available();
    if (newClient)
    {
      telnetClient.stop();
      telnetClient = newClient;
      telnetClient.setNoDelay(true);
      telnetLine = "";
      telnetAuthenticated = (configuredTelnetPassword.length() == 0);
      telnetPrintln("");
      telnetPrintln("Auto tuner konzole (telnet) - prikazy: help, status, play/start, stop, reboot");
      if (!telnetAuthenticated)
      {
        telnetPrintln("Zadejte: pass <heslo>");
      }
      telnetPrompt();
    }
    return;
  }

  while (telnetClient.available() > 0)
  {
    const char c = static_cast<char>(telnetClient.read());
    if (c == '\r')
    {
      continue;
    }
    if (c == '\n')
    {
      String line = telnetLine;
      telnetLine = "";
      line.trim();
      if (line.length() > 0)
      {
        handleTelnetCommand(line);
      }
      telnetPrompt();
      continue;
    }

    if (telnetLine.length() < 160)
    {
      telnetLine += c;
    }
  }
}
} // namespace

void begin(const char *wifiSsid,
           const char *wifiPass,
           const char *otaHostname,
           const char *otaPassword,
           uint16_t otaPort,
           uint16_t telnetPort,
           const char *telnetPassword)
{
  configuredTelnetPort = telnetPort;
  configuredTelnetPassword = telnetPassword ? telnetPassword : "";
  configuredSsid = wifiSsid ? wifiSsid : "";
  configuredPass = wifiPass ? wifiPass : "";
  configuredHostname = otaHostname ? otaHostname : "";
  configuredOtaPassword = otaPassword ? otaPassword : "";
  configuredOtaPort = otaPort;

  if (telnetServer)
  {
    delete telnetServer;
    telnetServer = nullptr;
  }
  telnetServer = new WiFiServer(configuredTelnetPort);

  if (configuredSsid.length() == 0)
  {
    LogMirror::out().println("WiFi neni nakonfigurovana (chybi WIFI_SSID). Preskakuji WiFi/OTA/telnet.");
    stopRadio();
    return;
  }

  auto &log = LogMirror::out();
  log.print("Pripojuji WiFi: ");
  log.println(configuredSsid);
  startRadio();
}

void setCallbacks(Callbacks newCallbacks) { callbacks = std::move(newCallbacks); }

void loop()
{
  if (wifiState == WifiState::RadioOff)
  {
    return;
  }

  if (wifiState == WifiState::Connecting)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      onConnected();
      return;
    }

    if (millis() - connectStartedAtMs > CONNECT_TIMEOUT_MS)
    {
      LogMirror::out().println("WiFi pripojeni selhalo - vypinam radio (bez vysilani).");
      stopRadio();
      return;
    }

    return;
  }

  if (wifiState != WifiState::Ready || WiFi.status() != WL_CONNECTED)
  {
    return;
  }

  ArduinoOTA.handle();
  serviceTelnet();
}

bool isWifiConfigured() { return wifiConfigured && wifiState == WifiState::Ready && WiFi.status() == WL_CONNECTED; }

Print *console()
{
  if (!telnetClient || !telnetClient.connected() || !telnetAuthenticated)
  {
    return nullptr;
  }
  return &telnetClient;
}

void enableRadio()
{
  if (wifiState != WifiState::RadioOff)
  {
    return;
  }
  auto &log = LogMirror::out();
  log.print("WiFi radio ON, pripojuji: ");
  log.println(configuredSsid);
  startRadio();
}

void disableRadio()
{
  if (wifiState == WifiState::RadioOff)
  {
    return;
  }
  LogMirror::out().println("WiFi radio OFF (tichy rezim pro mereni).");
  stopRadio();
}

bool isRadioEnabled() { return wifiState != WifiState::RadioOff; }
} // namespace OtaTelnet

#else

namespace OtaTelnet
{
void begin(const char *, const char *, const char *, const char *, uint16_t, uint16_t, const char *) {}
void setCallbacks(Callbacks) {}
void loop() {}
bool isWifiConfigured() { return false; }
Print *console() { return nullptr; }
void enableRadio() {}
void disableRadio() {}
bool isRadioEnabled() { return false; }
} // namespace OtaTelnet

#endif
