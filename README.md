# Auto_tuner_SW

## WiFi + vzdaleny pristup (ESP32)

- Vytvorte `include/wifi_secrets.h` podle `include/wifi_secrets.example.h` a doplnte SSID/heslo.
- Po nahrani firmware se ESP32 pripoji k WiFi, zapne OTA a telnet konzoli.
  - OTA hostname: `auto-tuner.local` (lze zmenit `OTA_HOSTNAME`)
  - Telnet port: `23` (lze zmenit `TELNET_PORT`)

### OTA (PlatformIO `espota`) troubleshooting

- Pokud vidite `No response from device`, nejcasteji je to tim, ze ESP32 nedokaze navazat TCP spojeni zpet na PC (Windows firewall / spatny `--host_ip` pri vice sitovkach).
- V `platformio.ini` pro `[env:esp32_ota]` je nastavene pevne `--host_ip` a `--host_port=32323`, aby slo snadno povolit firewall.
- Firewall (PowerShell jako admin):
  - `New-NetFirewallRule -DisplayName "PlatformIO ESP OTA 32323" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 32323 -Profile Private`
  - Pokud mate Wi‑Fi jako *Public*, nastavte profil na `Public,Private` (nebo zmente sit na Private ve Windows nastaveni):
    - `Set-NetFirewallRule -DisplayName "PlatformIO ESP OTA 32323" -Profile Public,Private`
