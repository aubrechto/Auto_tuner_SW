#include "DisplayComm.h"

#include <math.h>

namespace DisplayComm
{
namespace
{
HardwareSerial *displaySerial = nullptr;
Callbacks callbacks;
Config config;
uint16_t waveformIndex = 0;

void endCommand()
{
  if (!displaySerial)
  {
    return;
  }
  displaySerial->write(0xFF);
  displaySerial->write(0xFF);
  displaySerial->write(0xFF);
}

float referenceFrequencyForNote(char note)
{
  switch (note)
  {
  case 'E':
    return 82.41f;
  case 'A':
    return 110.00f;
  case 'D':
    return 146.83f;
  case 'G':
    return 196.00f;
  case 'H':
    return 246.94f;
  case 'e':
    return 329.63f;
  default:
    return 100.0f;
  }
}

void addPoint(uint8_t channel, uint8_t value)
{
  if (!displaySerial)
  {
    return;
  }
  displaySerial->print("add ");
  displaySerial->print(config.waveformId);
  displaySerial->print(",");
  displaySerial->print(channel);
  displaySerial->print(",");
  displaySerial->print(value);
  endCommand();
}
} // namespace

void begin(HardwareSerial &serial, uint32_t baud, int8_t rxPin, int8_t txPin)
{
  displaySerial = &serial;
  serial.begin(baud, SERIAL_8N1, rxPin, txPin);
  waveformIndex = 0;
}

void setCallbacks(Callbacks newCallbacks) { callbacks = std::move(newCallbacks); }

void setConfig(const Config &newConfig) { config = newConfig; }

bool poll()
{
  if (!displaySerial || !displaySerial->available())
  {
    return false;
  }

  const String command = displaySerial->readStringUntil('\n');
  String normalized = command;
  normalized.trim();
  normalized.toUpperCase();

  if (normalized == "PLAY" || normalized == "START")
  {
    if (callbacks.onPlay)
    {
      callbacks.onPlay();
    }
    return true;
  }

  if (normalized == "STOP")
  {
    if (callbacks.onStop)
    {
      callbacks.onStop();
    }
    return true;
  }

  return false;
}

void clearWaveform()
{
  if (!displaySerial)
  {
    return;
  }
  displaySerial->print("cle ");
  displaySerial->print(config.waveformId);
  displaySerial->print(",255");
  endCommand();
}

void resetWaveform()
{
  waveformIndex = 0;
  clearWaveform();
}

void writeText(const String &object, const String &value)
{
  if (!displaySerial)
  {
    return;
  }
  displaySerial->print(object);
  displaySerial->print(".txt=\"");
  displaySerial->print(value);
  displaySerial->print("\"");
  endCommand();
}

void writeNumber(const String &object, int value)
{
  if (!displaySerial)
  {
    return;
  }
  displaySerial->print(object);
  displaySerial->print(".val=");
  displaySerial->print(value);
  endCommand();
}

void writeAttributeNumber(const String &object, const String &attribute, int value)
{
  if (!displaySerial)
  {
    return;
  }
  displaySerial->print(object);
  displaySerial->print(".");
  displaySerial->print(attribute);
  displaySerial->print("=");
  displaySerial->print(value);
  endCommand();
}

void setVisible(const String &object, bool visible)
{
  if (!displaySerial)
  {
    return;
  }
  displaySerial->print("vis ");
  displaySerial->print(object);
  displaySerial->print(",");
  displaySerial->print(visible ? 1 : 0);
  endCommand();
}

void sendWaveformChunk(char note, float measuredFrequency, int chunkSize)
{
  const float referenceFrequency = referenceFrequencyForNote(note);

  for (int i = 0; i < chunkSize; ++i)
  {
    const uint16_t index = waveformIndex + static_cast<uint16_t>(i);
    if (index >= config.graphPoints)
    {
      break;
    }

    const float t = static_cast<float>(index) / static_cast<float>(config.graphPoints - 1);
    const uint8_t referenceValue =
        static_cast<uint8_t>(abs(config.offset + static_cast<int>(config.amplitude * sinf(2.0f * PI * t))));
    addPoint(0, referenceValue);

    const float scaledT = t * (measuredFrequency / referenceFrequency);
    const uint8_t measuredValue =
        static_cast<uint8_t>(abs(config.offset + static_cast<int>(config.amplitude * sinf(2.0f * PI * scaledT))));
    addPoint(1, measuredValue);
  }

  waveformIndex += static_cast<uint16_t>(chunkSize);
  if (waveformIndex >= config.graphPoints)
  {
    waveformIndex = 0;
    clearWaveform();
  }
}

void updateDisplay(char note, float targetFrequency, float measuredFrequency)
{
  writeText("Note", String(note));
  writeNumber("desiredf", static_cast<int>(lroundf(targetFrequency)));
  writeNumber("currentf", static_cast<int>(lroundf(measuredFrequency)));
  sendWaveformChunk(note, measuredFrequency, 10);
}
} // namespace DisplayComm
