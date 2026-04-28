#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#include <DisplayComm.h>
#include <LogMirror.h>
#include <OtaTelnet.h>
#include <TimeFrequencyTracker.h>

#if defined(__has_include)
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif
#endif

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASS
#define WIFI_PASS ""
#endif
#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME "auto-tuner"
#endif
#ifndef OTA_PASSWORD
#define OTA_PASSWORD ""
#endif
#ifndef TELNET_PORT
#define TELNET_PORT 23
#endif
#ifndef TELNET_PASSWORD
#define TELNET_PASSWORD ""
#endif

namespace
{
constexpr uint8_t RXD2 = 16;
constexpr uint8_t TXD2 = 17;
constexpr uint8_t ADC_PIN = 32;
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;

constexpr uint16_t SERVO_CENTER = 375;
constexpr uint16_t SERVO_MAX_OFFSET = 255;
constexpr uint16_t SERVO_COARSE_OFFSET = 160;
constexpr uint16_t SERVO_MEDIUM_OFFSET = 110;
constexpr uint16_t SERVO_FINE_OFFSET = 70;
constexpr unsigned long SERVO_COARSE_PULSE_MS = 220;
constexpr unsigned long SERVO_MEDIUM_PULSE_MS = 140;
constexpr unsigned long SERVO_FINE_PULSE_MS = 90;
constexpr unsigned long SERVO_SETTLE_MS = 220;
constexpr float SERVO_COARSE_ERROR_HZ = 2.5f;
constexpr float SERVO_MEDIUM_ERROR_HZ = 0.8f;

constexpr float COARSE_TOLERANCE = 0.50f;
constexpr float FINE_TOLERANCE = 0.01f;
constexpr uint8_t REQUIRED_STABLE_READS = 3;
constexpr unsigned long POST_STRING_DELAY_MS = 250;
constexpr unsigned long WIFI_OFF_DELAY_MS = 150;

constexpr float TRACKER_SAMPLE_RATE_HZ = 4000.0f;
constexpr uint32_t TRACKER_SAMPLE_PERIOD_US = static_cast<uint32_t>(1000000.0f / TRACKER_SAMPLE_RATE_HZ);
constexpr size_t TRACKER_WINDOW_SIZE = 512;
constexpr size_t TRACKER_HOP_SIZE = 256;
constexpr float TRACKER_FREQUENCY_STEP_HZ = 1.0f;
constexpr float TRACKER_LOW_PASS_ALPHA = 0.18f;
constexpr float TRACKER_EMA_ALPHA = 0.90f;
constexpr float TRACKER_MIN_SIGNAL_RMS = 35.0f;
constexpr float TRACKER_MIN_PEAK_TO_PEAK = 260.0f;
constexpr float TRACKER_MIN_SWEEP_HZ = 60.0f;
constexpr float TRACKER_MAX_SWEEP_HZ = 360.0f;
constexpr float TRACKER_SWEEP_LOW_RATIO = 0.55f;
constexpr float TRACKER_SWEEP_HIGH_RATIO = 1.55f;
constexpr float TRACKER_HARMONIC_WEIGHT_2 = 0.75f;
constexpr float TRACKER_HARMONIC_WEIGHT_3 = 0.35f;
constexpr float TRACKER_REJECT_CENTER_HZ = 100.0f;
constexpr float TRACKER_REJECT_HALF_WIDTH_HZ = 4.0f;
constexpr uint8_t TRACKER_MAX_SAMPLES_PER_SERVICE = 16;
constexpr uint8_t TRACKER_MAX_TIMING_LAG_SAMPLES = 8;

struct StringConfig
{
  float desiredHz;
  char note;
  const char *name;
  uint8_t servoChannel;
};

constexpr StringConfig STRINGS[] = {
    {82.41f, 'E', "E2", 0},
    {110.00f, 'A', "A2", 1},
    {146.83f, 'D', "D3", 2},
    {196.00f, 'G', "G3", 3},
    {246.94f, 'H', "B3", 4},
    {329.63f, 'e', "E4", 5},
};
constexpr size_t STRING_COUNT = sizeof(STRINGS) / sizeof(STRINGS[0]);

struct StringTuningResult
{
  float desiredHz = 0.0f;
  float tunedHz = 0.0f;
  bool hasMeasurement = false;
};

enum class TunerState : uint8_t
{
  Idle,
  Preparing,
  Tuning,
  Finished,
};

Adafruit_PWMServoDriver pwmDriver(0x41);
TimeFrequencyTracker::GoertzelSweepTracker frequencyTracker;

TunerState tunerState = TunerState::Idle;
bool pausedByStop = false;
int currentStringIndex = 0;
int stableCount = 0;
unsigned long lastDisplayUpdateMs = 0;
bool servoPulseActive = false;
uint8_t activeServoChannel = 0;
unsigned long servoPulseStopAtMs = 0;
unsigned long nextMeasurementAllowedAtMs = 0;
bool filtersEnabled = true;
unsigned long preparingStartedAtMs = 0;
bool preparingInitialized = false;
bool pendingFinishedReport = false;
StringTuningResult results[STRING_COUNT];
bool trackerConfigured = false;
int trackerConfiguredStringIndex = -1;
bool trackerConfiguredLowPass = false;
uint32_t nextTrackerSampleAtUs = 0;
uint32_t trackerEstimateSequence = 0;
uint32_t consumedEstimateSequence = 0;
TimeFrequencyTracker::Estimate latestTrackerEstimate{};

namespace WifiControl
{
void enable() { OtaTelnet::enableRadio(); }
void disable() { OtaTelnet::disableRadio(); }
bool isEnabled() { return OtaTelnet::isRadioEnabled(); }
} // namespace WifiControl

void pollConsoleCommands();
bool ensureFrequencyTrackerReady();
void resetFrequencyTracker(bool reconfigure = false);
void serviceFrequencyTracker();
bool takeLatestFrequencyEstimate(TimeFrequencyTracker::Estimate &estimate);
} // namespace
bool requestStartTuning();
void abortTuning(bool byStop);
void resetUiToIdle();

void serviceBackground()
{
  DisplayComm::poll();
  OtaTelnet::loop();
  LogMirror::setMirror(OtaTelnet::console());
  pollConsoleCommands();
}

namespace
{
void handleConsoleCommand(String line)
{
  line.trim();
  if (line.length() == 0)
  {
    return;
  }

  String lower = line;
  lower.toLowerCase();

  if (lower == "filters on")
  {
    filtersEnabled = true;
    LogMirror::out().println("Filters: ON");
    return;
  }

  if (lower == "filters off")
  {
    filtersEnabled = false;
    LogMirror::out().println("Filters: OFF");
    return;
  }

  if (lower == "filters")
  {
    LogMirror::out().println(String("Filters: ") + (filtersEnabled ? "ON" : "OFF"));
    return;
  }
}

void pollConsoleCommands()
{
  static String line;

  while (Serial.available() > 0)
  {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r')
    {
      continue;
    }

    if (c == '\n')
    {
      handleConsoleCommand(line);
      line = "";
      continue;
    }

    if (line.length() < 160)
    {
      line += c;
    }
  }
}

float getTrackerMinFrequency(float targetFrequency)
{
  const float scaledMin = targetFrequency * TRACKER_SWEEP_LOW_RATIO;
  return max(TRACKER_MIN_SWEEP_HZ, scaledMin);
}

float getTrackerMaxFrequency(float targetFrequency)
{
  const float scaledMax = targetFrequency * TRACKER_SWEEP_HIGH_RATIO;
  return min(TRACKER_MAX_SWEEP_HZ, scaledMax);
}

bool configureFrequencyTrackerForString(int stringIndex)
{
  if (stringIndex < 0 || stringIndex >= static_cast<int>(STRING_COUNT))
  {
    trackerConfigured = false;
    trackerConfiguredStringIndex = -1;
    return false;
  }

  const float targetFrequency = STRINGS[stringIndex].desiredHz;
  TimeFrequencyTracker::Config config;
  config.sampleRateHz = TRACKER_SAMPLE_RATE_HZ;
  config.windowSize = TRACKER_WINDOW_SIZE;
  config.hopSize = TRACKER_HOP_SIZE;
  config.minFrequencyHz = getTrackerMinFrequency(targetFrequency);
  config.maxFrequencyHz = getTrackerMaxFrequency(targetFrequency);
  config.frequencyStepHz = TRACKER_FREQUENCY_STEP_HZ;
  config.enableLowPass = filtersEnabled;
  config.lowPassAlpha = TRACKER_LOW_PASS_ALPHA;
  config.emaAlpha = TRACKER_EMA_ALPHA;
  config.minSignalRms = TRACKER_MIN_SIGNAL_RMS;
  config.minPeakToPeak = TRACKER_MIN_PEAK_TO_PEAK;
  config.harmonicWeight2 = TRACKER_HARMONIC_WEIGHT_2;
  config.harmonicWeight3 = TRACKER_HARMONIC_WEIGHT_3;
  config.enableRejectBand = filtersEnabled;
  config.rejectBandCenterHz = TRACKER_REJECT_CENTER_HZ;
  config.rejectBandHalfWidthHz = TRACKER_REJECT_HALF_WIDTH_HZ;

  trackerConfigured = frequencyTracker.begin(config);
  trackerConfiguredStringIndex = trackerConfigured ? stringIndex : -1;
  trackerConfiguredLowPass = filtersEnabled;
  trackerEstimateSequence = 0;
  consumedEstimateSequence = 0;
  latestTrackerEstimate = {};
  nextTrackerSampleAtUs = micros();

  auto &log = LogMirror::out();
  if (trackerConfigured)
  {
    log.print("Tracker pripraven pro ");
    log.print(STRINGS[stringIndex].name);
    log.print(" | rozsah: ");
    log.print(config.minFrequencyHz, 1);
    log.print("-");
    log.print(config.maxFrequencyHz, 1);
    log.println(" Hz");
  }
  else
  {
    log.println("Chyba: tracker se nepodarilo inicializovat.");
  }

  return trackerConfigured;
}

void resetFrequencyTracker(bool reconfigure)
{
  trackerEstimateSequence = 0;
  consumedEstimateSequence = 0;
  latestTrackerEstimate = {};
  nextTrackerSampleAtUs = micros();

  if (reconfigure)
  {
    configureFrequencyTrackerForString(currentStringIndex);
    return;
  }

  if (trackerConfigured)
  {
    frequencyTracker.reset();
  }
}

bool ensureFrequencyTrackerReady()
{
  if (currentStringIndex < 0 || currentStringIndex >= static_cast<int>(STRING_COUNT))
  {
    return false;
  }

  if (!trackerConfigured ||
      trackerConfiguredStringIndex != currentStringIndex ||
      trackerConfiguredLowPass != filtersEnabled)
  {
    return configureFrequencyTrackerForString(currentStringIndex);
  }

  return true;
}

bool measurementAllowed()
{
  return tunerState == TunerState::Tuning &&
         !servoPulseActive &&
         millis() >= nextMeasurementAllowedAtMs &&
         currentStringIndex >= 0 &&
         currentStringIndex < static_cast<int>(STRING_COUNT);
}

void serviceFrequencyTracker()
{
  if (!measurementAllowed())
  {
    return;
  }

  if (!ensureFrequencyTrackerReady())
  {
    return;
  }

  const uint32_t nowUs = micros();
  const uint32_t maxLagUs = TRACKER_SAMPLE_PERIOD_US * TRACKER_MAX_TIMING_LAG_SAMPLES;
  if (static_cast<int32_t>(nowUs - nextTrackerSampleAtUs) > static_cast<int32_t>(maxLagUs))
  {
    nextTrackerSampleAtUs = nowUs;
  }

  uint8_t samplesProcessed = 0;
  while (samplesProcessed < TRACKER_MAX_SAMPLES_PER_SERVICE &&
         static_cast<int32_t>(micros() - nextTrackerSampleAtUs) >= 0)
  {
    nextTrackerSampleAtUs += TRACKER_SAMPLE_PERIOD_US;

    TimeFrequencyTracker::Estimate estimate;
    const float sample = static_cast<float>(analogRead(ADC_PIN));
    if (frequencyTracker.pushSample(sample, estimate))
    {
      latestTrackerEstimate = estimate;
      ++trackerEstimateSequence;
    }

    ++samplesProcessed;
  }
}

bool takeLatestFrequencyEstimate(TimeFrequencyTracker::Estimate &estimate)
{
  if (trackerEstimateSequence == 0 || trackerEstimateSequence == consumedEstimateSequence)
  {
    return false;
  }

  estimate = latestTrackerEstimate;
  consumedEstimateSequence = trackerEstimateSequence;
  return true;
}
} // namespace

uint8_t getServoChannelForCurrentString()
{
  return STRINGS[currentStringIndex].servoChannel;
}

void centerServoChannel(uint8_t channel)
{
  pwmDriver.setPWM(channel, 0, SERVO_CENTER);
}

void disableServoChannel(uint8_t channel)
{
  pwmDriver.setPWM(channel, 0, 0);
}

void disableInactiveServos(uint8_t activeChannel)
{
  for (size_t i = 0; i < STRING_COUNT; ++i)
  {
    const uint8_t channel = STRINGS[i].servoChannel;
    if (channel != activeChannel)
    {
      disableServoChannel(channel);
    }
  }
}

void stopAllServos()
{
  for (size_t i = 0; i < STRING_COUNT; ++i)
  {
    disableServoChannel(STRINGS[i].servoChannel);
  }
}

void startServoPulse(uint8_t channel, bool tightenDirection, uint16_t servoOffset, unsigned long pulseDurationMs)
{
  const int signedOffset = tightenDirection ? static_cast<int>(servoOffset) : -static_cast<int>(servoOffset);
  const uint16_t boundedPulse = constrain(SERVO_CENTER + signedOffset,
                                          static_cast<int>(SERVO_CENTER - SERVO_MAX_OFFSET),
                                          static_cast<int>(SERVO_CENTER + SERVO_MAX_OFFSET));

  disableInactiveServos(channel);
  pwmDriver.setPWM(channel, 0, boundedPulse);
  servoPulseActive = true;
  activeServoChannel = channel;
  servoPulseStopAtMs = millis() + pulseDurationMs;
  resetFrequencyTracker();
}

void serviceServoMotion()
{
  if (!servoPulseActive)
  {
    return;
  }

  if (millis() < servoPulseStopAtMs)
  {
    return;
  }

  disableServoChannel(activeServoChannel);
  servoPulseActive = false;
  nextMeasurementAllowedAtMs = millis() + SERVO_SETTLE_MS;
}

bool commandServoStepFromError(float targetFrequency, float measuredFrequency)
{
  const float errorHz = measuredFrequency - targetFrequency;
  const float absErrorHz = fabsf(errorHz);
  if (absErrorHz <= FINE_TOLERANCE * targetFrequency)
  {
    return false;
  }

  uint16_t servoOffset = SERVO_FINE_OFFSET;
  unsigned long pulseDurationMs = SERVO_FINE_PULSE_MS;
  if (absErrorHz >= SERVO_COARSE_ERROR_HZ)
  {
    servoOffset = SERVO_COARSE_OFFSET;
    pulseDurationMs = SERVO_COARSE_PULSE_MS;
  }
  else if (absErrorHz >= SERVO_MEDIUM_ERROR_HZ)
  {
    servoOffset = SERVO_MEDIUM_OFFSET;
    pulseDurationMs = SERVO_MEDIUM_PULSE_MS;
  }

  const uint8_t channel = getServoChannelForCurrentString();
  const bool tightenDirection = errorHz < 0.0f;
  startServoPulse(channel, tightenDirection, servoOffset, pulseDurationMs);
  return true;
}

void centerCurrentServo()
{
  const uint8_t activeChannel = getServoChannelForCurrentString();
  disableInactiveServos(activeChannel);
  centerServoChannel(activeChannel);
}

void resetTuningState()
{
  currentStringIndex = 0;
  stableCount = 0;
  servoPulseActive = false;
  activeServoChannel = 0;
  servoPulseStopAtMs = 0;
  nextMeasurementAllowedAtMs = 0;
  trackerConfigured = false;
  trackerConfiguredStringIndex = -1;
  trackerConfiguredLowPass = false;
  resetFrequencyTracker();
  DisplayComm::resetWaveform();
  stopAllServos();
}

void updateDisplay(char note, float targetFrequency, float measuredFrequency)
{
  DisplayComm::updateDisplay(note, targetFrequency, measuredFrequency);
}

const char *getTuningDirection(float targetFrequency, float measuredFrequency)
{
  if (measuredFrequency < targetFrequency * (1.0f - FINE_TOLERANCE))
  {
    return "NIZKO";
  }

  if (measuredFrequency > targetFrequency * (1.0f + FINE_TOLERANCE))
  {
    return "VYSOKO";
  }

  return "OK";
}

void printTuningStatus(const char *stringName,
                       float targetFrequency,
                       float measuredFrequency,
                       const TimeFrequencyTracker::Estimate &estimate)
{
  auto &log = LogMirror::out();
  log.print("Ladim strunu: ");
  log.print(stringName);
  log.print(" | aktivni LED: ");
  log.print("LED");
  log.print(getServoChannelForCurrentString());
  log.print(" | cil: ");
  log.print(targetFrequency, 2);
  log.print(" Hz | freq: ");
  log.print(measuredFrequency, 2);
  log.print(" Hz | raw: ");
  log.print(estimate.rawPeakFrequencyHz, 2);
  log.print(" Hz | mag: ");
  log.print(estimate.magnitude, 1);
  log.print(" | rms: ");
  log.print(estimate.signalRms, 1);
  log.print(" | p2p: ");
  log.print(estimate.peakToPeak, 1);
  log.print(" (");
  log.print(getTuningDirection(targetFrequency, measuredFrequency));
  log.println(")");
}

void printWeakSignalStatus(const char *stringName,
                           float targetFrequency,
                           const TimeFrequencyTracker::Estimate &estimate)
{
  auto &log = LogMirror::out();
  log.print("Ladim strunu: ");
  log.print(stringName);
  log.print(" | aktivni LED: ");
  log.print("LED");
  log.print(getServoChannelForCurrentString());
  log.print(" | cil: ");
  log.print(targetFrequency, 2);
  log.print(" Hz | raw: ");
  log.print(estimate.rawPeakFrequencyHz, 2);
  log.print(" Hz | rms: ");
  log.print(estimate.signalRms, 1);
  log.print(" | p2p: ");
  log.print(estimate.peakToPeak, 1);
  log.print(" | minimum rms/p2p: ");
  log.print(TRACKER_MIN_SIGNAL_RMS, 1);
  log.print("/");
  log.print(TRACKER_MIN_PEAK_TO_PEAK, 1);
  log.println(" | signal slaby");
}

void printNoFrequencyStatus(const char *stringName, float targetFrequency)
{
  auto &log = LogMirror::out();
  log.print("Ladim strunu: ");
  log.print(stringName);
  log.print(" | aktivni LED: ");
  log.print("LED");
  log.print(getServoChannelForCurrentString());
  log.print(" | cil: ");
  log.print(targetFrequency, 2);
  log.println(" Hz | frekvence zatim neni k dispozici");
}

void processCurrentString()
{
  serviceServoMotion();
  serviceFrequencyTracker();

  if (currentStringIndex >= static_cast<int>(STRING_COUNT))
  {
    LogMirror::out().println("Hotovo! Vsechny struny jsou naladene.");
    stopAllServos();
    DisplayComm::setVisible("stop", 0);
    tunerState = TunerState::Finished;
    pendingFinishedReport = true;
    WifiControl::enable();
    return;
  }

  if (servoPulseActive || millis() < nextMeasurementAllowedAtMs)
  {
    return;
  }

  const float targetFrequency = STRINGS[currentStringIndex].desiredHz;
  const char note = STRINGS[currentStringIndex].note;

  if (!ensureFrequencyTrackerReady())
  {
    stableCount = 0;
    printNoFrequencyStatus(STRINGS[currentStringIndex].name, targetFrequency);
    return;
  }

  TimeFrequencyTracker::Estimate estimate;
  if (!takeLatestFrequencyEstimate(estimate))
  {
    return;
  }

  if (!estimate.valid)
  {
    stableCount = 0;
    disableServoChannel(getServoChannelForCurrentString());
    printWeakSignalStatus(STRINGS[currentStringIndex].name, targetFrequency, estimate);
    return;
  }

  const float measuredFrequency = estimate.estimatedFrequencyHz;

  results[currentStringIndex].tunedHz = measuredFrequency;
  results[currentStringIndex].hasMeasurement = true;

  if (tunerState == TunerState::Tuning && millis() - lastDisplayUpdateMs >= 120)
  {
    updateDisplay(note, targetFrequency, measuredFrequency);
    lastDisplayUpdateMs = millis();
  }

  printTuningStatus(STRINGS[currentStringIndex].name, targetFrequency, measuredFrequency, estimate);

  const float coarseLower = targetFrequency * (1.0f - COARSE_TOLERANCE);
  const float coarseUpper = targetFrequency * (1.0f + COARSE_TOLERANCE);
  const float fineLower = targetFrequency * (1.0f - FINE_TOLERANCE);
  const float fineUpper = targetFrequency * (1.0f + FINE_TOLERANCE);
  const bool withinFineRange = measuredFrequency >= fineLower && measuredFrequency <= fineUpper;

  if (measuredFrequency >= coarseLower && measuredFrequency <= coarseUpper &&
      withinFineRange)
  {
    disableServoChannel(getServoChannelForCurrentString());
    ++stableCount;
    auto &log = LogMirror::out();
    log.print("Stabilni: ");
    log.print(stableCount);
    log.print("/");
    log.println(REQUIRED_STABLE_READS);

    if (stableCount >= REQUIRED_STABLE_READS)
    {
      const uint8_t finishedServoChannel = getServoChannelForCurrentString();
      log.print("NALAZENO: ");
      log.println(STRINGS[currentStringIndex].name);
      disableServoChannel(finishedServoChannel);
      ++currentStringIndex;
      stableCount = 0;
      trackerConfigured = false;
      trackerConfiguredStringIndex = -1;
      resetFrequencyTracker();
      nextMeasurementAllowedAtMs = millis() + POST_STRING_DELAY_MS;
      if (tunerState == TunerState::Tuning)
      {
        DisplayComm::resetWaveform();
      }
    }
  }
  else
  {
    stableCount = 0;
    if (commandServoStepFromError(targetFrequency, measuredFrequency))
    {
      auto &log = LogMirror::out();
      log.print("Servo krok pro ");
      log.print(STRINGS[currentStringIndex].name);
      log.print(" | chyba: ");
      log.print(measuredFrequency - targetFrequency, 2);
      log.println(" Hz");
    }
  }
}

void setup()
{
  Serial.begin(115200);
  LogMirror::begin(Serial);

  DisplayComm::setConfig(DisplayComm::Config{});
  DisplayComm::setCallbacks(DisplayComm::Callbacks{
      []() { requestStartTuning(); },
      []() {
        LogMirror::out().println("Prijat prikaz STOP.");
        abortTuning(true);
      },
  });
  DisplayComm::begin(Serial2, 9600, RXD2, TXD2);

  OtaTelnet::setCallbacks(OtaTelnet::Callbacks{
      []() { return requestStartTuning(); },
      []() {
        abortTuning(true);
      },
      []() {
        String status;
        status += String("state: ");
        switch (tunerState)
        {
        case TunerState::Idle:
          status += "IDLE";
          break;
        case TunerState::Preparing:
          status += "PREPARING";
          break;
        case TunerState::Tuning:
          status += "TUNING";
          break;
        case TunerState::Finished:
          status += "FINISHED";
          break;
        }
        status += "\n";
        status += String("wifiRadio: ") + (WifiControl::isEnabled() ? "on" : "off") + "\n";
        status += String("pausedByStop: ") + (pausedByStop ? "true" : "false") + "\n";
        status += String("currentStringIndex: ") + currentStringIndex;
        status += "\n";
        status += String("filters: ") + (filtersEnabled ? "on" : "off");
        return status;
      },
      [](bool enabled) {
        filtersEnabled = enabled;
        LogMirror::out().println(String("Filters: ") + (filtersEnabled ? "ON" : "OFF"));
      },
      []() { return filtersEnabled; },
  });
  OtaTelnet::begin(WIFI_SSID, WIFI_PASS, OTA_HOSTNAME, OTA_PASSWORD, 3232, TELNET_PORT, TELNET_PASSWORD);

  Wire.begin(I2C_SDA, I2C_SCL);

  pwmDriver.begin();
  pwmDriver.setPWMFreq(50);
  stopAllServos();

  analogReadResolution(12);

  LogMirror::out().println("Auto tuner startuje...");
  resetUiToIdle();
}

void loop()
{
  serviceBackground();

  if (tunerState == TunerState::Idle)
  {
    delay(20);
    return;
  }

  if (tunerState == TunerState::Preparing)
  {
    if (!preparingInitialized)
    {
      preparingInitialized = true;
      pausedByStop = false;
      resetTuningState();
      for (size_t i = 0; i < STRING_COUNT; ++i)
      {
        results[i] = {};
        results[i].desiredHz = STRINGS[i].desiredHz;
      }
      DisplayComm::setVisible("stop", 1);
      preparingStartedAtMs = millis();
      LogMirror::out().println("Priprava ladeni...");
      return;
    }

    if (millis() - preparingStartedAtMs < WIFI_OFF_DELAY_MS)
    {
      delay(10);
      return;
    }

    WifiControl::disable();
    tunerState = TunerState::Tuning;
    lastDisplayUpdateMs = 0;
    LogMirror::out().println("WiFi vypnuta, zacinam ladit...");
    return;
  }

  if (tunerState == TunerState::Tuning)
  {
    if (WifiControl::isEnabled())
    {
      WifiControl::disable();
    }
    processCurrentString();
    return;
  }

  if (tunerState == TunerState::Finished)
  {
    if (pendingFinishedReport)
    {
      Print *telnet = OtaTelnet::console();
      if (telnet)
      {
        telnet->println("STATUS: OK");
        for (size_t i = 0; i < STRING_COUNT; ++i)
        {
          telnet->print("String ");
          telnet->print(STRINGS[i].note);
          telnet->print(": desired ");
          telnet->print(results[i].desiredHz, 1);
          telnet->print(" Hz, tuned ");
          if (results[i].hasMeasurement)
          {
            telnet->print(results[i].tunedHz, 1);
            telnet->println(" Hz");
          }
          else
          {
            telnet->println("N/A");
          }
        }
        pendingFinishedReport = false;
        tunerState = TunerState::Idle;
        resetUiToIdle();
      }
    }
    delay(50);
    return;
  }
}

bool requestStartTuning()
{
  if (tunerState == TunerState::Preparing || tunerState == TunerState::Tuning)
  {
    return false;
  }

  preparingInitialized = false;
  pendingFinishedReport = false;
  tunerState = TunerState::Preparing;
  return true;
}

void abortTuning(bool byStop)
{
  pausedByStop = byStop;
  pendingFinishedReport = false;
  tunerState = TunerState::Idle;
  preparingInitialized = false;
  resetTuningState();
  WifiControl::enable();
  resetUiToIdle();
}

void resetUiToIdle()
{
  DisplayComm::setVisible("stop", 0);
  DisplayComm::writeText("Note", "-");
  DisplayComm::writeNumber("desiredf", 0);
  DisplayComm::writeNumber("currentf", 0);
}
