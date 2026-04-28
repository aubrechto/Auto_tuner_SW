#pragma once

#include <Arduino.h>
#include <functional>

namespace DisplayComm
{
struct Config
{
  uint8_t waveformId = 1;
  uint16_t graphPoints = 300;
  uint8_t amplitude = 127;
  uint8_t offset = 78;
};

struct Callbacks
{
  std::function<void()> onPlay;
  std::function<void()> onStop;
};

void begin(HardwareSerial &serial, uint32_t baud, int8_t rxPin, int8_t txPin);
void setCallbacks(Callbacks callbacks);
void setConfig(const Config &config);

bool poll();

void clearWaveform();
void resetWaveform();

void writeText(const String &object, const String &value);
void writeNumber(const String &object, int value);
void setVisible(const String &object, bool visible);

void sendWaveformChunk(char note, float measuredFrequency, int chunkSize);
void updateDisplay(char note, float targetFrequency, float measuredFrequency);
} // namespace DisplayComm

