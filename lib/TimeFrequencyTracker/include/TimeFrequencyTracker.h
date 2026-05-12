#pragma once

#include <Arduino.h>

namespace TimeFrequencyTracker
{
struct Config
{
  size_t windowSize = 512;
  size_t hopSize = 256;
  float sampleRateHz = 4000.0f;
  float minFrequencyHz = 60.0f;
  float maxFrequencyHz = 200.0f;
  float frequencyStepHz = 1.0f;
  bool enableLowPass = true;
  float lowPassAlpha = 0.18f;
  float emaAlpha = 0.90f;
  float minSignalRms = 8.0f;
  float minPeakToPeak = 0.0f;
  float minMagnitude = 0.0f;
  float harmonicWeight2 = 0.75f;
  float harmonicWeight3 = 0.35f;
  bool enableRejectBand = false;
  float rejectBandCenterHz = 100.0f;
  float rejectBandHalfWidthHz = 4.0f;
};

struct Estimate
{
  uint32_t timestampMs = 0;
  float estimatedFrequencyHz = 0.0f;
  float rawPeakFrequencyHz = 0.0f;
  float magnitude = 0.0f;
  float score = 0.0f;
  float signalRms = 0.0f;
  float peakToPeak = 0.0f;
  bool valid = false;
};

class TrackerBase
{
public:
  TrackerBase();
  virtual ~TrackerBase();

  bool begin(const Config &config);
  void reset();

  bool pushSample(float rawSample, Estimate &estimate);

  const Config &config() const { return config_; }

protected:
  const float *windowedSamples() const { return preparedWindow_; }
  size_t windowSize() const { return config_.windowSize; }
  float sampleRateHz() const { return config_.sampleRateHz; }

  virtual bool onBegin() = 0;
  virtual void onReset() = 0;
  virtual bool analyzePreparedWindow(float &rawPeakFrequencyHz, float &magnitude, float &score) = 0;

private:
  bool validateAndStoreConfig(const Config &config);
  bool allocateCommonBuffers();
  void freeCommonBuffers();
  void buildHannWindow();
  void prepareWindow();
  bool finalizeEstimate(float rawPeakFrequencyHz, float magnitude, float score, Estimate &estimate);

  Config config_{};
  float *ringBuffer_ = nullptr;
  float *preparedWindow_ = nullptr;
  float *hannWindow_ = nullptr;
  size_t writeIndex_ = 0;
  size_t samplesFilled_ = 0;
  size_t samplesSinceLastHop_ = 0;
  uint64_t totalSamplesPushed_ = 0;
  float lowPassState_ = 0.0f;
  bool lowPassInitialized_ = false;
  float smoothedFrequencyHz_ = 0.0f;
  bool smoothingInitialized_ = false;
  float signalRms_ = 0.0f;
  float signalPeakToPeak_ = 0.0f;
};

class GoertzelSweepTracker : public TrackerBase
{
public:
  GoertzelSweepTracker() = default;
  ~GoertzelSweepTracker() override;

private:
  struct GoertzelCoeff
  {
    float coeff = 0.0f;
  };

  struct Candidate
  {
    float frequencyHz = 0.0f;
    GoertzelCoeff fundamental{};
    GoertzelCoeff harmonic2{};
    GoertzelCoeff harmonic3{};
    bool hasHarmonic2 = false;
    bool hasHarmonic3 = false;
  };

  bool onBegin() override;
  void onReset() override;
  bool analyzePreparedWindow(float &rawPeakFrequencyHz, float &magnitude, float &score) override;

  static GoertzelCoeff makeCoeff(float frequencyHz, float sampleRateHz);
  float magnitudeSquared(const GoertzelCoeff &coeff) const;
  void freeCandidates();

  Candidate *candidates_ = nullptr;
  size_t candidateCount_ = 0;
};

// Example use:
//
//   TimeFrequencyTracker::Config cfg;
//   cfg.sampleRateHz = 4000.0f;
//   cfg.windowSize = 512;
//   cfg.hopSize = 256; // 50% overlap
//
//   TimeFrequencyTracker::GoertzelSweepTracker tracker;
//   tracker.begin(cfg);
//
//   TimeFrequencyTracker::Estimate est;
//   const float raw = static_cast<float>(analogRead(32));
//   if (tracker.pushSample(raw, est) && est.valid)
//   {
//     Serial.print(est.timestampMs);
//     Serial.print(',');
//     Serial.print(est.estimatedFrequencyHz, 2);
//     Serial.print(',');
//     Serial.println(est.magnitude, 2);
//   }

} // namespace TimeFrequencyTracker
