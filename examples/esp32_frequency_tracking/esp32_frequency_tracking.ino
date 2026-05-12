#include <Arduino.h>

#include <TimeFrequencyTracker.h>

namespace
{
constexpr uint8_t ADC_PIN = 32;
constexpr float SAMPLE_RATE_HZ = 4000.0f;
constexpr uint32_t SAMPLE_PERIOD_US = static_cast<uint32_t>(1000000.0f / SAMPLE_RATE_HZ);

TimeFrequencyTracker::GoertzelSweepTracker goertzelTracker;
uint32_t nextSampleAtUs = 0;
}

void setup()
{
  Serial.begin(115200);
  analogReadResolution(12);

  TimeFrequencyTracker::Config config;
  config.sampleRateHz = SAMPLE_RATE_HZ;
  config.windowSize = 512;
  config.hopSize = 256;
  config.minFrequencyHz = 60.0f;
  config.maxFrequencyHz = 200.0f;
  config.frequencyStepHz = 1.0f;
  config.enableLowPass = true;
  config.lowPassAlpha = 0.18f;
  config.emaAlpha = 0.90f;
  config.minSignalRms = 12.0f;

  goertzelTracker.begin(config);
  Serial.println("tracker=goertzel");

  Serial.println("timestamp_ms,estimated_frequency_hz,magnitude,raw_peak_hz,signal_rms,valid");
  nextSampleAtUs = micros();
}

void loop()
{
  const uint32_t nowUs = micros();
  if (static_cast<int32_t>(nowUs - nextSampleAtUs) < 0)
  {
    return;
  }

  nextSampleAtUs += SAMPLE_PERIOD_US;

  const float sample = static_cast<float>(analogRead(ADC_PIN));
  TimeFrequencyTracker::Estimate estimate;
  const bool ready = goertzelTracker.pushSample(sample, estimate);

  if (!ready)
  {
    return;
  }

  Serial.print(estimate.timestampMs);
  Serial.print(',');
  Serial.print(estimate.estimatedFrequencyHz, 2);
  Serial.print(',');
  Serial.print(estimate.magnitude, 2);
  Serial.print(',');
  Serial.print(estimate.rawPeakFrequencyHz, 2);
  Serial.print(',');
  Serial.print(estimate.signalRms, 2);
  Serial.print(',');
  Serial.println(estimate.valid ? 1 : 0);
}
