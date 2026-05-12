#include "TimeFrequencyTracker.h"

// Unused while SlidingFftTracker is commented out.
// #include <arduinoFFT.h>
#include <math.h>
#include <new>

namespace TimeFrequencyTracker
{
namespace
{
constexpr float kTwoPi = 6.28318530717958647692f;

float clampFloat(float value, float low, float high)
{
  if (value < low)
  {
    return low;
  }

  if (value > high)
  {
    return high;
  }

  return value;
}

size_t frequencyCountForSweep(float minFrequencyHz, float maxFrequencyHz, float stepHz)
{
  if (stepHz <= 0.0f || maxFrequencyHz < minFrequencyHz)
  {
    return 0;
  }

  const float span = maxFrequencyHz - minFrequencyHz;
  return static_cast<size_t>(floorf(span / stepHz)) + 1U;
}

bool isInsideBand(float frequencyHz, float centerHz, float halfWidthHz)
{
  return halfWidthHz > 0.0f && fabsf(frequencyHz - centerHz) <= halfWidthHz;
}
} // namespace

TrackerBase::TrackerBase() = default;

TrackerBase::~TrackerBase()
{
  freeCommonBuffers();
}

bool TrackerBase::begin(const Config &config)
{
  freeCommonBuffers();
  reset();

  if (!validateAndStoreConfig(config))
  {
    return false;
  }

  if (!allocateCommonBuffers())
  {
    freeCommonBuffers();
    return false;
  }

  buildHannWindow();
  onReset();
  return onBegin();
}

void TrackerBase::reset()
{
  writeIndex_ = 0;
  samplesFilled_ = 0;
  samplesSinceLastHop_ = 0;
  totalSamplesPushed_ = 0;
  lowPassState_ = 0.0f;
  lowPassInitialized_ = false;
  smoothedFrequencyHz_ = 0.0f;
  smoothingInitialized_ = false;
  signalRms_ = 0.0f;
  signalPeakToPeak_ = 0.0f;
  onReset();
}

bool TrackerBase::pushSample(float rawSample, Estimate &estimate)
{
  if (!ringBuffer_ || !preparedWindow_)
  {
    return false;
  }

  float sample = rawSample;
  if (config_.enableLowPass)
  {
    if (!lowPassInitialized_)
    {
      lowPassState_ = sample;
      lowPassInitialized_ = true;
    }
    else
    {
      lowPassState_ += config_.lowPassAlpha * (sample - lowPassState_);
    }
    sample = lowPassState_;
  }

  ringBuffer_[writeIndex_] = sample;
  writeIndex_ = (writeIndex_ + 1U) % config_.windowSize;
  ++totalSamplesPushed_;

  if (samplesFilled_ < config_.windowSize)
  {
    ++samplesFilled_;
  }

  ++samplesSinceLastHop_;
  if (samplesFilled_ < config_.windowSize || samplesSinceLastHop_ < config_.hopSize)
  {
    return false;
  }

  samplesSinceLastHop_ = 0;
  prepareWindow();

  if (signalRms_ < config_.minSignalRms || signalPeakToPeak_ < config_.minPeakToPeak)
  {
    return finalizeEstimate(0.0f, 0.0f, 0.0f, estimate);
  }

  float rawPeakFrequencyHz = 0.0f;
  float magnitude = 0.0f;
  float score = 0.0f;
  if (!analyzePreparedWindow(rawPeakFrequencyHz, magnitude, score))
  {
    return false;
  }

  return finalizeEstimate(rawPeakFrequencyHz, magnitude, score, estimate);
}

// Unused batch API in the current firmware; streaming pushSample() is used instead.
// size_t TrackerBase::processSamples(const float *samples, size_t sampleCount, Estimate *estimates, size_t maxEstimates)
// {
//   if (!samples || !estimates || maxEstimates == 0)
//   {
//     return 0;
//   }
//
//   size_t produced = 0;
//   for (size_t i = 0; i < sampleCount; ++i)
//   {
//     Estimate estimate;
//     if (pushSample(samples[i], estimate))
//     {
//       if (produced >= maxEstimates)
//       {
//         break;
//       }
//       estimates[produced++] = estimate;
//     }
//   }
//
//   return produced;
// }

bool TrackerBase::validateAndStoreConfig(const Config &config)
{
  if (config.windowSize < 256U || config.windowSize > 1024U)
  {
    return false;
  }

  if (config.sampleRateHz <= 0.0f ||
      config.minFrequencyHz <= 0.0f ||
      config.maxFrequencyHz <= config.minFrequencyHz ||
      config.frequencyStepHz <= 0.0f)
  {
    return false;
  }

  const float nyquistHz = config.sampleRateHz * 0.5f;
  if (config.maxFrequencyHz >= nyquistHz)
  {
    return false;
  }

  config_ = config;
  if (config_.hopSize == 0U)
  {
    config_.hopSize = config_.windowSize / 2U;
  }

  if (config_.hopSize > (config_.windowSize / 2U))
  {
    config_.hopSize = config_.windowSize / 2U;
  }

  config_.lowPassAlpha = clampFloat(config_.lowPassAlpha, 0.0f, 1.0f);
  config_.emaAlpha = clampFloat(config_.emaAlpha, 0.0f, 0.999f);
  if (config_.minPeakToPeak < 0.0f)
  {
    config_.minPeakToPeak = 0.0f;
  }
  if (config_.rejectBandHalfWidthHz < 0.0f)
  {
    config_.rejectBandHalfWidthHz = 0.0f;
  }
  return true;
}

bool TrackerBase::allocateCommonBuffers()
{
  ringBuffer_ = new (std::nothrow) float[config_.windowSize];
  preparedWindow_ = new (std::nothrow) float[config_.windowSize];
  hannWindow_ = new (std::nothrow) float[config_.windowSize];

  return ringBuffer_ && preparedWindow_ && hannWindow_;
}

void TrackerBase::freeCommonBuffers()
{
  delete[] ringBuffer_;
  delete[] preparedWindow_;
  delete[] hannWindow_;

  ringBuffer_ = nullptr;
  preparedWindow_ = nullptr;
  hannWindow_ = nullptr;
}

void TrackerBase::buildHannWindow()
{
  if (!hannWindow_)
  {
    return;
  }

  for (size_t i = 0; i < config_.windowSize; ++i)
  {
    const float angle = (config_.windowSize > 1U)
                            ? (kTwoPi * static_cast<float>(i) / static_cast<float>(config_.windowSize - 1U))
                            : 0.0f;
    hannWindow_[i] = 0.5f * (1.0f - cosf(angle));
  }
}

void TrackerBase::prepareWindow()
{
  float mean = 0.0f;
  float minSample = ringBuffer_[writeIndex_];
  float maxSample = ringBuffer_[writeIndex_];
  for (size_t i = 0; i < config_.windowSize; ++i)
  {
    const size_t index = (writeIndex_ + i) % config_.windowSize;
    const float sample = ringBuffer_[index];
    preparedWindow_[i] = sample;
    mean += sample;
    if (sample < minSample)
    {
      minSample = sample;
    }
    if (sample > maxSample)
    {
      maxSample = sample;
    }
  }
  mean /= static_cast<float>(config_.windowSize);
  signalPeakToPeak_ = maxSample - minSample;

  float energy = 0.0f;
  for (size_t i = 0; i < config_.windowSize; ++i)
  {
    const float centered = preparedWindow_[i] - mean;
    energy += centered * centered;
    preparedWindow_[i] = centered * hannWindow_[i];
  }

  signalRms_ = sqrtf(energy / static_cast<float>(config_.windowSize));
}

bool TrackerBase::finalizeEstimate(float rawPeakFrequencyHz, float magnitude, float score, Estimate &estimate)
{
  estimate.timestampMs = static_cast<uint32_t>(
      ((static_cast<float>(totalSamplesPushed_) - (static_cast<float>(config_.windowSize) * 0.5f)) * 1000.0f / config_.sampleRateHz) +
      0.5f);
  estimate.rawPeakFrequencyHz = rawPeakFrequencyHz;
  estimate.magnitude = magnitude;
  estimate.score = score;
  estimate.signalRms = signalRms_;
  estimate.peakToPeak = signalPeakToPeak_;
  estimate.valid = signalRms_ >= config_.minSignalRms &&
                   signalPeakToPeak_ >= config_.minPeakToPeak &&
                   magnitude >= config_.minMagnitude &&
                   rawPeakFrequencyHz > 0.0f;

  if (estimate.valid)
  {
    if (!smoothingInitialized_)
    {
      smoothedFrequencyHz_ = rawPeakFrequencyHz;
      smoothingInitialized_ = true;
    }
    else
    {
      smoothedFrequencyHz_ = (config_.emaAlpha * smoothedFrequencyHz_) +
                             ((1.0f - config_.emaAlpha) * rawPeakFrequencyHz);
    }
  }

  estimate.estimatedFrequencyHz = estimate.valid
                                      ? (smoothingInitialized_ ? smoothedFrequencyHz_ : rawPeakFrequencyHz)
                                      : 0.0f;
  return true;
}

GoertzelSweepTracker::~GoertzelSweepTracker()
{
  freeCandidates();
}

bool GoertzelSweepTracker::onBegin()
{
  freeCandidates();

  candidateCount_ = frequencyCountForSweep(config().minFrequencyHz, config().maxFrequencyHz, config().frequencyStepHz);
  if (candidateCount_ == 0U)
  {
    return false;
  }

  candidates_ = new (std::nothrow) Candidate[candidateCount_];
  if (!candidates_)
  {
    candidateCount_ = 0;
    return false;
  }

  const float nyquistHz = sampleRateHz() * 0.5f;
  for (size_t i = 0; i < candidateCount_; ++i)
  {
    Candidate &candidate = candidates_[i];
    candidate.frequencyHz = config().minFrequencyHz + (config().frequencyStepHz * static_cast<float>(i));
    candidate.fundamental = makeCoeff(candidate.frequencyHz, sampleRateHz());

    const float harmonic2Hz = candidate.frequencyHz * 2.0f;
    if (harmonic2Hz < nyquistHz)
    {
      candidate.harmonic2 = makeCoeff(harmonic2Hz, sampleRateHz());
      candidate.hasHarmonic2 = true;
    }

    const float harmonic3Hz = candidate.frequencyHz * 3.0f;
    if (harmonic3Hz < nyquistHz)
    {
      candidate.harmonic3 = makeCoeff(harmonic3Hz, sampleRateHz());
      candidate.hasHarmonic3 = true;
    }
  }

  return true;
}

void GoertzelSweepTracker::onReset()
{
}

bool GoertzelSweepTracker::analyzePreparedWindow(float &rawPeakFrequencyHz, float &magnitude, float &score)
{
  if (!candidates_ || candidateCount_ == 0U)
  {
    return false;
  }

  float bestScore = -1.0f;
  float bestMagnitudeSquared = 0.0f;
  float bestFrequencyHz = 0.0f;

  for (size_t i = 0; i < candidateCount_; ++i)
  {
    const Candidate &candidate = candidates_[i];
    if (config().enableRejectBand &&
        isInsideBand(candidate.frequencyHz, config().rejectBandCenterHz, config().rejectBandHalfWidthHz))
    {
      continue;
    }

    const float magFundamental = magnitudeSquared(candidate.fundamental);
    float candidateScore = magFundamental;

    if (candidate.hasHarmonic2)
    {
      candidateScore += config().harmonicWeight2 * magnitudeSquared(candidate.harmonic2);
    }

    if (candidate.hasHarmonic3)
    {
      candidateScore += config().harmonicWeight3 * magnitudeSquared(candidate.harmonic3);
    }

    if (candidateScore > bestScore)
    {
      bestScore = candidateScore;
      bestMagnitudeSquared = magFundamental;
      bestFrequencyHz = candidate.frequencyHz;
    }
  }

  rawPeakFrequencyHz = bestFrequencyHz;
  magnitude = sqrtf(bestMagnitudeSquared);
  score = sqrtf(bestScore > 0.0f ? bestScore : 0.0f);
  return true;
}

GoertzelSweepTracker::GoertzelCoeff GoertzelSweepTracker::makeCoeff(float frequencyHz, float sampleRateHz)
{
  const float omega = kTwoPi * frequencyHz / sampleRateHz;
  GoertzelCoeff coeff{};
  coeff.coeff = 2.0f * cosf(omega);
  return coeff;
}

float GoertzelSweepTracker::magnitudeSquared(const GoertzelCoeff &coeff) const
{
  float s1 = 0.0f;
  float s2 = 0.0f;
  const float *samples = windowedSamples();
  for (size_t i = 0; i < windowSize(); ++i)
  {
    const float s0 = samples[i] + (coeff.coeff * s1) - s2;
    s2 = s1;
    s1 = s0;
  }

  const float power = (s1 * s1) + (s2 * s2) - (coeff.coeff * s1 * s2);
  return power > 0.0f ? power : 0.0f;
}

void GoertzelSweepTracker::freeCandidates()
{
  delete[] candidates_;
  candidates_ = nullptr;
  candidateCount_ = 0;
}

// Unused FFT-based tracker kept as a reference. The firmware uses GoertzelSweepTracker.
// SlidingFftTracker::~SlidingFftTracker()
// {
//   freeFftBuffers();
// }
//
// bool SlidingFftTracker::onBegin()
// {
//   freeFftBuffers();
//
//   fftReal_ = new (std::nothrow) float[windowSize()];
//   fftImag_ = new (std::nothrow) float[windowSize()];
//   return fftReal_ && fftImag_;
// }
//
// void SlidingFftTracker::onReset()
// {
//   if (!fftReal_ || !fftImag_)
//   {
//     return;
//   }
//
//   for (size_t i = 0; i < windowSize(); ++i)
//   {
//     fftReal_[i] = 0.0f;
//     fftImag_[i] = 0.0f;
//   }
// }
//
// bool SlidingFftTracker::analyzePreparedWindow(float &rawPeakFrequencyHz, float &magnitude, float &score)
// {
//   if (!fftReal_ || !fftImag_)
//   {
//     return false;
//   }
//
//   const float *samples = windowedSamples();
//   for (size_t i = 0; i < windowSize(); ++i)
//   {
//     fftReal_[i] = samples[i];
//     fftImag_[i] = 0.0f;
//   }
//
//   ArduinoFFT<float> fft(fftReal_, fftImag_, static_cast<uint_fast16_t>(windowSize()), sampleRateHz());
//   fft.compute(FFTDirection::Forward);
//   fft.complexToMagnitude();
//
//   float bestScore = -1.0f;
//   float bestMagnitude = 0.0f;
//   float bestFrequencyHz = 0.0f;
//
//   const size_t candidateCount = frequencyCountForSweep(config().minFrequencyHz, config().maxFrequencyHz, config().frequencyStepHz);
//   for (size_t i = 0; i < candidateCount; ++i)
//   {
//     const float candidateFrequencyHz = config().minFrequencyHz + (config().frequencyStepHz * static_cast<float>(i));
//     if (config().enableRejectBand &&
//         isInsideBand(candidateFrequencyHz, config().rejectBandCenterHz, config().rejectBandHalfWidthHz))
//     {
//       continue;
//     }
//
//     const float fundamentalMagnitude = magnitudeAtFrequency(candidateFrequencyHz);
//     float candidateScore = fundamentalMagnitude;
//     candidateScore += config().harmonicWeight2 * magnitudeAtFrequency(candidateFrequencyHz * 2.0f);
//     candidateScore += config().harmonicWeight3 * magnitudeAtFrequency(candidateFrequencyHz * 3.0f);
//
//     if (candidateScore > bestScore)
//     {
//       bestScore = candidateScore;
//       bestMagnitude = fundamentalMagnitude;
//       bestFrequencyHz = interpolatePeakFrequency(candidateFrequencyHz);
//     }
//   }
//
//   rawPeakFrequencyHz = bestFrequencyHz;
//   magnitude = bestMagnitude;
//   score = bestScore > 0.0f ? bestScore : 0.0f;
//   return true;
// }
//
// void SlidingFftTracker::freeFftBuffers()
// {
//   delete[] fftReal_;
//   delete[] fftImag_;
//   fftReal_ = nullptr;
//   fftImag_ = nullptr;
// }
//
// float SlidingFftTracker::scoreAtFrequency(float frequencyHz) const
// {
//   return magnitudeAtFrequency(frequencyHz);
// }
//
// float SlidingFftTracker::magnitudeAtFrequency(float frequencyHz) const
// {
//   if (!fftReal_)
//   {
//     return 0.0f;
//   }
//
//   const float nyquistHz = sampleRateHz() * 0.5f;
//   if (frequencyHz <= 0.0f || frequencyHz >= nyquistHz)
//   {
//     return 0.0f;
//   }
//
//   const float bin = (frequencyHz * static_cast<float>(windowSize())) / sampleRateHz();
//   int centerIndex = static_cast<int>(bin + 0.5f);
//   centerIndex = constrain(centerIndex, 1, static_cast<int>(windowSize() / 2U) - 2);
//
//   float localPeak = 0.0f;
//   for (int offset = -1; offset <= 1; ++offset)
//   {
//     const float value = fftReal_[centerIndex + offset];
//     if (value > localPeak)
//     {
//       localPeak = value;
//     }
//   }
//
//   return localPeak;
// }
//
// float SlidingFftTracker::interpolatePeakFrequency(float centerFrequencyHz) const
// {
//   const float bin = (centerFrequencyHz * static_cast<float>(windowSize())) / sampleRateHz();
//   int index = static_cast<int>(bin + 0.5f);
//   index = constrain(index, 1, static_cast<int>(windowSize() / 2U) - 2);
//
//   const float a = fftReal_[index - 1];
//   const float b = fftReal_[index];
//   const float c = fftReal_[index + 1];
//   const float denominator = a - (2.0f * b) + c;
//   float delta = 0.0f;
//   if (fabsf(denominator) > 1e-6f)
//   {
//     delta = 0.5f * (a - c) / denominator;
//     delta = clampFloat(delta, -0.5f, 0.5f);
//   }
//
//   return ((static_cast<float>(index) + delta) * sampleRateHz()) / static_cast<float>(windowSize());
// }
} // namespace TimeFrequencyTracker
