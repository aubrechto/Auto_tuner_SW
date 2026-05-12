#include <Arduino.h>
// Unused in the current firmware; kept commented in case SPI hardware is added later.
// #include <SPI.h>
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
constexpr uint8_t BQ25895M_I2C_ADDRESS = 0x6A;

constexpr uint16_t SERVO_CENTER = 375;
constexpr uint16_t SERVO_MAX_OFFSET = 255;
constexpr uint16_t SERVO_STEP_FINE_OFFSET = 25;
constexpr unsigned long SERVO_STEP_FINE_PULSE_MS = 40;
constexpr uint16_t SERVO_STEP_MEDIUM_OFFSET = 32;
constexpr unsigned long SERVO_STEP_MEDIUM_PULSE_MS = 50;
constexpr uint16_t SERVO_STEP_COARSE_OFFSET = 50;
constexpr unsigned long SERVO_STEP_COARSE_PULSE_MS = 75;
constexpr uint16_t SERVO_STEP_AGGRESSIVE_OFFSET = 72;
constexpr unsigned long SERVO_STEP_AGGRESSIVE_PULSE_MS = 105;
constexpr float SERVO_FINE_STEP_THRESHOLD_HZ = 2.0f;
constexpr float SERVO_MEDIUM_STEP_THRESHOLD_HZ = 3.0f;
constexpr float SERVO_COARSE_STEP_THRESHOLD_HZ = 6.0f;
constexpr unsigned long SERVO_SETTLE_MS = 220;

constexpr float COARSE_TOLERANCE = 0.5f;
constexpr float FINE_TOLERANCE_HZ = 0.5f;
constexpr uint8_t REQUIRED_STABLE_READS = 1;
constexpr uint8_t MEASUREMENT_AVERAGE_COUNT = 1;
constexpr unsigned long POST_STRING_DELAY_MS = 1200;
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
constexpr unsigned long SYSTEM_STATUS_UPDATE_MS = 15000;
constexpr uint16_t DISPLAY_DEFAULT_BCO = 0;
constexpr uint16_t DISPLAY_TUNED_BCO = 1024;
constexpr uint8_t BQ25895M_REG_ADC_CONTROL = 0x02;
constexpr uint8_t BQ25895M_REG_CHARGER_STATUS = 0x0B;
constexpr uint8_t BQ25895M_REG_BATTERY_VOLTAGE = 0x0E;
constexpr uint8_t BQ25895M_ADC_CONTINUOUS_MASK = 0x40;
constexpr float BQ25895M_BATTERY_VOLTAGE_BASE_V = 2.304f;
constexpr float BQ25895M_BATTERY_VOLTAGE_STEP_V = 0.020f;

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

struct MeasurementAccumulator
{
  float estimatedFrequencySum = 0.0f;
  float rawPeakFrequencySum = 0.0f;
  float magnitudeSum = 0.0f;
  float scoreSum = 0.0f;
  float signalRmsSum = 0.0f;
  float peakToPeakSum = 0.0f;
  uint8_t sampleCount = 0;
};

struct BatteryStatus
{
  bool available = false;
  bool charging = false;
  float voltageV = 0.0f;
  int percent = 0;
};

struct BatteryPercentPoint
{
  float voltageV;
  uint8_t percent;
};

// Approximate open-circuit state-of-charge curve for a 1S Li-ion / LiPo cell with 3.7V nominal voltage.
constexpr BatteryPercentPoint BATTERY_PERCENT_CURVE[] = {
    {4.20f, 100},
    {4.15f, 95},
    {4.11f, 90},
    {4.08f, 85},
    {4.02f, 80},
    {3.98f, 75},
    {3.95f, 70},
    {3.91f, 65},
    {3.87f, 60},
    {3.85f, 55},
    {3.84f, 50},
    {3.82f, 45},
    {3.80f, 40},
    {3.79f, 35},
    {3.77f, 30},
    {3.75f, 25},
    {3.73f, 20},
    {3.71f, 15},
    {3.69f, 10},
    {3.61f, 5},
    {3.27f, 0},
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
bool measurementStatusLogsEnabled = true;
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
MeasurementAccumulator measurementAccumulator{};
unsigned long lastSystemStatusUpdateMs = 0;
int lastDisplayedBatteryPercent = -1;
int lastDisplayedChargingVisible = -1;
int lastDisplayedWifiVisible = -1;
int lastDisplayedDispBco = -1;

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
void resetMeasurementAccumulator();
bool takeAveragedFrequencyEstimate(const TimeFrequencyTracker::Estimate &estimate,
                                   TimeFrequencyTracker::Estimate &averagedEstimate);
void updateSystemStatusIndicators(bool force = false);
void setTunedDisplayHighlight(bool enabled);
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

  if (lower == "measurelogs on")
  {
    measurementStatusLogsEnabled = true;
    LogMirror::out().println("Measure logs: ON");
    return;
  }

  if (lower == "measurelogs off")
  {
    measurementStatusLogsEnabled = false;
    LogMirror::out().println("Measure logs: OFF");
    return;
  }

  if (lower == "measurelogs")
  {
    LogMirror::out().println(String("Measure logs: ") + (measurementStatusLogsEnabled ? "ON" : "OFF"));
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

void resetMeasurementAccumulator()
{
  measurementAccumulator = {};
}

bool takeAveragedFrequencyEstimate(const TimeFrequencyTracker::Estimate &estimate,
                                   TimeFrequencyTracker::Estimate &averagedEstimate)
{
  if (!estimate.valid)
  {
    resetMeasurementAccumulator();
    return false;
  }

  measurementAccumulator.estimatedFrequencySum += estimate.estimatedFrequencyHz;
  measurementAccumulator.rawPeakFrequencySum += estimate.rawPeakFrequencyHz;
  measurementAccumulator.magnitudeSum += estimate.magnitude;
  measurementAccumulator.scoreSum += estimate.score;
  measurementAccumulator.signalRmsSum += estimate.signalRms;
  measurementAccumulator.peakToPeakSum += estimate.peakToPeak;
  ++measurementAccumulator.sampleCount;

  if (measurementAccumulator.sampleCount < MEASUREMENT_AVERAGE_COUNT)
  {
    return false;
  }

  const float sampleCount = static_cast<float>(measurementAccumulator.sampleCount);
  averagedEstimate = estimate;
  averagedEstimate.estimatedFrequencyHz = measurementAccumulator.estimatedFrequencySum / sampleCount;
  averagedEstimate.rawPeakFrequencyHz = measurementAccumulator.rawPeakFrequencySum / sampleCount;
  averagedEstimate.magnitude = measurementAccumulator.magnitudeSum / sampleCount;
  averagedEstimate.score = measurementAccumulator.scoreSum / sampleCount;
  averagedEstimate.signalRms = measurementAccumulator.signalRmsSum / sampleCount;
  averagedEstimate.peakToPeak = measurementAccumulator.peakToPeakSum / sampleCount;
  averagedEstimate.valid = true;

  resetMeasurementAccumulator();
  return true;
}

bool bq25895mReadRegister(uint8_t reg, uint8_t &value)
{
  Wire.beginTransmission(BQ25895M_I2C_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  if (Wire.requestFrom(static_cast<int>(BQ25895M_I2C_ADDRESS), 1) != 1)
  {
    return false;
  }

  value = Wire.read();
  return true;
}

bool bq25895mWriteRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(BQ25895M_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool ensureBq25895mAdcContinuous()
{
  uint8_t adcControl = 0;
  if (!bq25895mReadRegister(BQ25895M_REG_ADC_CONTROL, adcControl))
  {
    return false;
  }

  if ((adcControl & BQ25895M_ADC_CONTINUOUS_MASK) != 0)
  {
    return true;
  }

  adcControl |= BQ25895M_ADC_CONTINUOUS_MASK;
  return bq25895mWriteRegister(BQ25895M_REG_ADC_CONTROL, adcControl);
}

int batteryVoltageToPercent(float voltageV)
{
  constexpr size_t pointCount = sizeof(BATTERY_PERCENT_CURVE) / sizeof(BATTERY_PERCENT_CURVE[0]);
  if (voltageV >= BATTERY_PERCENT_CURVE[0].voltageV)
  {
    return BATTERY_PERCENT_CURVE[0].percent;
  }

  for (size_t i = 1; i < pointCount; ++i)
  {
    const BatteryPercentPoint &upper = BATTERY_PERCENT_CURVE[i - 1];
    const BatteryPercentPoint &lower = BATTERY_PERCENT_CURVE[i];
    if (voltageV >= lower.voltageV)
    {
      const float rangeV = upper.voltageV - lower.voltageV;
      if (rangeV <= 0.0f)
      {
        return lower.percent;
      }

      const float ratio = (voltageV - lower.voltageV) / rangeV;
      const float percent = lower.percent + ratio * static_cast<float>(upper.percent - lower.percent);
      return constrain(static_cast<int>(lroundf(percent)), 0, 100);
    }
  }

  return BATTERY_PERCENT_CURVE[pointCount - 1].percent;
}

bool readBatteryStatus(BatteryStatus &status)
{
  if (!ensureBq25895mAdcContinuous())
  {
    return false;
  }

  uint8_t chargerStatus = 0;
  uint8_t batteryVoltage = 0;
  if (!bq25895mReadRegister(BQ25895M_REG_CHARGER_STATUS, chargerStatus) ||
      !bq25895mReadRegister(BQ25895M_REG_BATTERY_VOLTAGE, batteryVoltage))
  {
    return false;
  }

  const uint8_t chargeState = (chargerStatus >> 3) & 0x03;
  const uint8_t batteryAdc = batteryVoltage & 0x7F;

  status.available = true;
  status.charging = (chargeState == 0x01 || chargeState == 0x02);
  status.voltageV = BQ25895M_BATTERY_VOLTAGE_BASE_V +
                    static_cast<float>(batteryAdc) * BQ25895M_BATTERY_VOLTAGE_STEP_V;
  status.percent = batteryVoltageToPercent(status.voltageV);
  return true;
}

void updateSystemStatusIndicators(bool force)
{
  if (!force && millis() - lastSystemStatusUpdateMs < SYSTEM_STATUS_UPDATE_MS)
  {
    return;
  }
  lastSystemStatusUpdateMs = millis();

  BatteryStatus batteryStatus;
  const bool batteryStatusValid = readBatteryStatus(batteryStatus);
  const int batteryPercent = batteryStatusValid ? batteryStatus.percent : 0;
  const bool chargingVisible = batteryStatusValid && batteryStatus.charging;
  const bool wifiVisible = OtaTelnet::isWifiConfigured();

  if (force || batteryPercent != lastDisplayedBatteryPercent)
  {
    DisplayComm::writeNumber("jBat", batteryPercent);
    lastDisplayedBatteryPercent = batteryPercent;
  }

  const int chargingVisibleInt = chargingVisible ? 1 : 0;
  if (force || chargingVisibleInt != lastDisplayedChargingVisible)
  {
    DisplayComm::setVisible("pChargeIcon", chargingVisible);
    lastDisplayedChargingVisible = chargingVisibleInt;
  }

  const int wifiVisibleInt = wifiVisible ? 1 : 0;
  if (force || wifiVisibleInt != lastDisplayedWifiVisible)
  {
    DisplayComm::setVisible("wifi", wifiVisible);
    lastDisplayedWifiVisible = wifiVisibleInt;
  }
}

void setTunedDisplayHighlight(bool enabled)
{
  const int targetBco = enabled ? DISPLAY_TUNED_BCO : DISPLAY_DEFAULT_BCO;
  if (lastDisplayedDispBco == targetBco)
  {
    return;
  }

  DisplayComm::writeAttributeNumber("disp", "bco", targetBco);
  lastDisplayedDispBco = targetBco;
}
} // namespace

uint8_t getServoChannelForCurrentString()
{
  return STRINGS[currentStringIndex].servoChannel;
}

// Unused manual-centering helper kept for possible servo calibration.
// void centerServoChannel(uint8_t channel)
// {
//   pwmDriver.setPWM(channel, 0, SERVO_CENTER);
// }

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
  resetMeasurementAccumulator();
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
  if (absErrorHz <= FINE_TOLERANCE_HZ)
  {
    return false;
  }

  const uint8_t channel = getServoChannelForCurrentString();
  // Match the original servo polarity: when the measured frequency is above target,
  // drive the PWM to the positive side of center; below target goes to the negative side.
  const bool tightenDirection = errorHz > 0.0f;

  uint16_t servoOffset = SERVO_STEP_AGGRESSIVE_OFFSET;
  unsigned long pulseDurationMs = SERVO_STEP_AGGRESSIVE_PULSE_MS;
  if (absErrorHz <= SERVO_FINE_STEP_THRESHOLD_HZ)
  {
    servoOffset = SERVO_STEP_FINE_OFFSET;
    pulseDurationMs = SERVO_STEP_FINE_PULSE_MS;
  }
  else if (absErrorHz <= SERVO_MEDIUM_STEP_THRESHOLD_HZ)
  {
    servoOffset = SERVO_STEP_MEDIUM_OFFSET;
    pulseDurationMs = SERVO_STEP_MEDIUM_PULSE_MS;
  }
  else if (absErrorHz <= SERVO_COARSE_STEP_THRESHOLD_HZ)
  {
    servoOffset = SERVO_STEP_COARSE_OFFSET;
    pulseDurationMs = SERVO_STEP_COARSE_PULSE_MS;
  }

  startServoPulse(channel, tightenDirection, servoOffset, pulseDurationMs);
  return true;
}

// Unused manual-centering helper kept for possible servo calibration.
// void centerCurrentServo()
// {
//   const uint8_t activeChannel = getServoChannelForCurrentString();
//   disableInactiveServos(activeChannel);
//   centerServoChannel(activeChannel);
// }

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
  resetMeasurementAccumulator();
  resetFrequencyTracker();
  DisplayComm::resetWaveform();
  setTunedDisplayHighlight(false);
  stopAllServos();
}

void updateDisplay(char note, float targetFrequency, float measuredFrequency)
{
  DisplayComm::updateDisplay(note, targetFrequency, measuredFrequency);
}

const char *getTuningDirection(float targetFrequency, float measuredFrequency)
{
  if (measuredFrequency < targetFrequency - FINE_TOLERANCE_HZ)
  {
    return "NIZKO";
  }

  if (measuredFrequency > targetFrequency + FINE_TOLERANCE_HZ)
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
  if (!measurementStatusLogsEnabled)
  {
    return;
  }

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
  if (!measurementStatusLogsEnabled)
  {
    return;
  }

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
  if (!measurementStatusLogsEnabled)
  {
    return;
  }

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

  if (lastDisplayedDispBco == DISPLAY_TUNED_BCO)
  {
    setTunedDisplayHighlight(false);
  }

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
    resetMeasurementAccumulator();
    disableServoChannel(getServoChannelForCurrentString());
    printWeakSignalStatus(STRINGS[currentStringIndex].name, targetFrequency, estimate);
    return;
  }

  TimeFrequencyTracker::Estimate averagedEstimate;
  if (!takeAveragedFrequencyEstimate(estimate, averagedEstimate))
  {
    return;
  }

  const float measuredFrequency = averagedEstimate.estimatedFrequencyHz;

  results[currentStringIndex].tunedHz = measuredFrequency;
  results[currentStringIndex].hasMeasurement = true;

  if (tunerState == TunerState::Tuning && millis() - lastDisplayUpdateMs >= 120)
  {
    updateDisplay(note, targetFrequency, measuredFrequency);
    lastDisplayUpdateMs = millis();
  }

  printTuningStatus(STRINGS[currentStringIndex].name, targetFrequency, measuredFrequency, averagedEstimate);

  const float coarseLower = targetFrequency * (1.0f - COARSE_TOLERANCE);
  const float coarseUpper = targetFrequency * (1.0f + COARSE_TOLERANCE);
  const float fineLower = targetFrequency - FINE_TOLERANCE_HZ;
  const float fineUpper = targetFrequency + FINE_TOLERANCE_HZ;
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
      setTunedDisplayHighlight(true);
      ++currentStringIndex;
      stableCount = 0;
      trackerConfigured = false;
      trackerConfiguredStringIndex = -1;
      resetMeasurementAccumulator();
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
      []() { return requestStartTuning(); },
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
        status += String("filters: ") + (filtersEnabled ? "on" : "off") + "\n";
        status += String("measurelogs: ") + (measurementStatusLogsEnabled ? "on" : "off");
        return status;
      },
      [](bool enabled) {
        filtersEnabled = enabled;
        LogMirror::out().println(String("Filters: ") + (filtersEnabled ? "ON" : "OFF"));
      },
      []() { return filtersEnabled; },
      [](bool enabled) {
        measurementStatusLogsEnabled = enabled;
        LogMirror::out().println(String("Measure logs: ") + (measurementStatusLogsEnabled ? "ON" : "OFF"));
      },
      []() { return measurementStatusLogsEnabled; },
  });
  OtaTelnet::begin(WIFI_SSID, WIFI_PASS, OTA_HOSTNAME, OTA_PASSWORD, 3232, TELNET_PORT, TELNET_PASSWORD);

  Wire.begin(I2C_SDA, I2C_SCL);

  pwmDriver.begin();
  pwmDriver.setPWMFreq(50);
  stopAllServos();

  analogReadResolution(12);

  LogMirror::out().println("Auto tuner startuje...");
  resetUiToIdle();
  updateSystemStatusIndicators(true);
}

void loop()
{
  serviceBackground();

  if (tunerState == TunerState::Idle)
  {
    updateSystemStatusIndicators();
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
      updateSystemStatusIndicators();
      return;
    }

    if (millis() - preparingStartedAtMs < WIFI_OFF_DELAY_MS)
    {
      updateSystemStatusIndicators();
      delay(10);
      return;
    }

    WifiControl::disable();
    tunerState = TunerState::Tuning;
    lastDisplayUpdateMs = 0;
    LogMirror::out().println("WiFi vypnuta, zacinam ladit...");
    updateSystemStatusIndicators();
    return;
  }

  if (tunerState == TunerState::Tuning)
  {
    if (WifiControl::isEnabled())
    {
      WifiControl::disable();
    }
    processCurrentString();
    updateSystemStatusIndicators();
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
    updateSystemStatusIndicators();
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
  setTunedDisplayHighlight(false);
}
