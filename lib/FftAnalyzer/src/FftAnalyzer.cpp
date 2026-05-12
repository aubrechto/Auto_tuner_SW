// Unused legacy analyzer kept for reference; the current firmware uses TimeFrequencyTracker::GoertzelSweepTracker.
#if 0

#include "FftAnalyzer.h"

#include <arduinoFFT.h>
#include <math.h>

namespace FftAnalyzer
{
bool FiltersEnabled = true;
bool Enable70HzNotch = true;

namespace
{
constexpr uint16_t SAMPLES = 1024;
constexpr uint16_t SAMPLING_FREQUENCY = 3000;
constexpr float FFT_CALIBRATION = 1.0f;
constexpr float TARGET_FREQUENCY_TOLERANCE = 0.1f;
constexpr float HARMONIC_WINDOW_HZ = 8.0f;
constexpr float MAINS_50HZ = 50.0f;
constexpr float MAINS_50HZ_BAND = 3.0f;
constexpr float MAINS_100HZ = 100.0f;
constexpr float MAINS_100HZ_BAND = 6.0f;
constexpr float INTERFERENCE_70HZ = 70.0f;
constexpr float INTERFERENCE_70HZ_BAND = 3.5f;
constexpr int ADC_MAX_VALUE = 4095;
constexpr double MIN_SIGNAL_RMS = 35.0;
constexpr double MIN_PEAK_MAGNITUDE = 500.0;
constexpr double MIN_PEAK_TO_RMS_RATIO = 6.0;
constexpr float FUNDAMENTAL_MIN_RATIO = 0.35f;

double vReal[SAMPLES];
double vImag[SAMPLES];
int rawSamples[SAMPLES];
ArduinoFFT<double> fft(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

bool sampleSignal(uint8_t adcPin,
                  int &minSample,
                  int &maxSample,
                  const std::function<void()> &pump,
                  const std::function<bool()> &isActive)
{
  minSample = ADC_MAX_VALUE;
  maxSample = 0;

  for (uint16_t i = 0; i < SAMPLES; ++i)
  {
    if (pump && (i & 0x1F) == 0)
    {
      pump();
    }
    if (isActive && !isActive())
    {
      return false;
    }

    const int sample = analogRead(adcPin);
    rawSamples[i] = sample;

    if (sample < minSample)
    {
      minSample = sample;
    }
    if (sample > maxSample)
    {
      maxSample = sample;
    }

    delayMicroseconds(1000000 / SAMPLING_FREQUENCY);
  }

  return true;
}

void preprocessClippedSignal()
{
  double mean = 0.0;
  for (uint16_t i = 0; i < SAMPLES; ++i)
  {
    mean += rawSamples[i];
  }
  mean /= static_cast<double>(SAMPLES);

  for (uint16_t i = 0; i < SAMPLES; ++i)
  {
    const double clippedHalfWave = mean - static_cast<double>(rawSamples[i]);
    vReal[i] = clippedHalfWave > 0.0 ? clippedHalfWave : 0.0;
    vImag[i] = 0.0;
  }
}

double calculateSignalRms()
{
  double energy = 0.0;
  for (uint16_t i = 0; i < SAMPLES; ++i)
  {
    energy += vReal[i] * vReal[i];
  }

  return sqrt(energy / static_cast<double>(SAMPLES));
}

int frequencyToIndex(double frequency)
{
  const double binWidth = static_cast<double>(SAMPLING_FREQUENCY) / static_cast<double>(SAMPLES);
  int index = static_cast<int>(lround(frequency / binWidth));
  index = constrain(index, 0, static_cast<int>(SAMPLES / 2) - 1);
  return index;
}

double findPeakInBand(double centerFrequency, double windowHz, int &peakIndex)
{
  const double lowerFrequency = centerFrequency - windowHz;
  const double upperFrequency = centerFrequency + windowHz;

  const int startIndex = max(2, frequencyToIndex(lowerFrequency));
  const int endIndex = min(static_cast<int>(SAMPLES / 2) - 1, frequencyToIndex(upperFrequency));

  double peakMagnitude = 0.0;
  peakIndex = -1;

  for (int i = startIndex; i <= endIndex; ++i)
  {
    const double freq = (static_cast<double>(i) * SAMPLING_FREQUENCY / SAMPLES) / FFT_CALIBRATION;
    const bool filter50Hz = freq >= (MAINS_50HZ - MAINS_50HZ_BAND) && freq <= (MAINS_50HZ + MAINS_50HZ_BAND);
    const bool filter100Hz = freq >= (MAINS_100HZ - MAINS_100HZ_BAND) && freq <= (MAINS_100HZ + MAINS_100HZ_BAND);
    const bool filter70Hz =
        Enable70HzNotch && freq >= (INTERFERENCE_70HZ - INTERFERENCE_70HZ_BAND) && freq <= (INTERFERENCE_70HZ + INTERFERENCE_70HZ_BAND);

    if (FiltersEnabled && (filter50Hz || filter100Hz || filter70Hz))
    {
      continue;
    }

    if (vReal[i] > peakMagnitude)
    {
      peakMagnitude = vReal[i];
      peakIndex = i;
    }
  }

  return peakMagnitude;
}

int findGlobalPeakIndex()
{
  const int startIndex = 2;
  const int endIndex = static_cast<int>(SAMPLES / 2) - 2;

  double peakMagnitude = 0.0;
  int peakIndex = -1;

  for (int i = startIndex; i <= endIndex; ++i)
  {
    const double magnitude = vReal[i];
    if (magnitude > peakMagnitude)
    {
      peakMagnitude = magnitude;
      peakIndex = i;
    }
  }

  return peakIndex;
}

double refinePeakBinParabolic(int peakIndex)
{
  // Parabolic interpolation estimates the true spectral peak between FFT bins using the
  // neighboring magnitudes, improving frequency resolution beyond the raw bin width.
  if (peakIndex <= 0 || peakIndex >= (static_cast<int>(SAMPLES / 2) - 1))
  {
    return static_cast<double>(peakIndex);
  }

  const double aMinus1 = vReal[peakIndex - 1];
  const double a0 = vReal[peakIndex];
  const double aPlus1 = vReal[peakIndex + 1];

  const double denom = (aMinus1 - 2.0 * a0 + aPlus1);
  if (fabs(denom) < 1e-12)
  {
    return static_cast<double>(peakIndex);
  }

  double delta = 0.5 * (aMinus1 - aPlus1) / denom;
  // Numerical safety: prevent large jumps due to noise or flat peaks.
  if (delta > 0.5)
  {
    delta = 0.5;
  }
  else if (delta < -0.5)
  {
    delta = -0.5;
  }

  return static_cast<double>(peakIndex) + delta;
}

double detectFrequencyFromPeakIndex(int peakIndex)
{
  const double refinedBin = refinePeakBinParabolic(peakIndex);
  return (refinedBin * SAMPLING_FREQUENCY / SAMPLES) / FFT_CALIBRATION;
}
} // namespace

bool findDominantFrequency(uint8_t adcPin,
                           float targetFrequency,
                           double &detectedFrequency,
                           int &signalAmplitude,
                           const std::function<void()> &pump,
                           const std::function<bool()> &isActive)
{
  detectedFrequency = 0.0;
  int minSample = ADC_MAX_VALUE;
  int maxSample = 0;
  if (!sampleSignal(adcPin, minSample, maxSample, pump, isActive))
  {
    return false;
  }

  signalAmplitude = (maxSample - minSample) / 2;
  if (signalAmplitude < MinSignalAmplitude)
  {
    return false;
  }

  preprocessClippedSignal();
  const double signalRms = calculateSignalRms();
  if (signalRms < MIN_SIGNAL_RMS)
  {
    return false;
  }

  fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  fft.compute(FFTDirection::Forward);
  fft.complexToMagnitude();

  const int globalPeakIndex = findGlobalPeakIndex();
  if (globalPeakIndex >= 0)
  {
    // Best-effort frequency for debug/logging even when the target-band selection fails
    // (e.g. because of notch filters or thresholds).
    detectedFrequency = detectFrequencyFromPeakIndex(globalPeakIndex);
  }

  const double searchWindowHz = targetFrequency * TARGET_FREQUENCY_TOLERANCE;
  int fundamentalIndex = -1;
  const double fundamentalMagnitude = findPeakInBand(targetFrequency, searchWindowHz, fundamentalIndex);
  if (fundamentalIndex < 0)
  {
    return false;
  }

  if (fundamentalMagnitude < MIN_PEAK_MAGNITUDE || (fundamentalMagnitude / signalRms) < MIN_PEAK_TO_RMS_RATIO)
  {
    return false;
  }

  int secondHarmonicIndex = -1;
  const double secondHarmonicMagnitude = findPeakInBand(targetFrequency * 2.0f, HARMONIC_WINDOW_HZ, secondHarmonicIndex);

  int selectedIndex = fundamentalIndex;
  if (secondHarmonicIndex >= 0 && fundamentalMagnitude > 0.0 &&
      secondHarmonicMagnitude < (fundamentalMagnitude / FUNDAMENTAL_MIN_RATIO))
  {
    selectedIndex = fundamentalIndex;
  }

  detectedFrequency = detectFrequencyFromPeakIndex(selectedIndex);
  return true;
}
} // namespace FftAnalyzer

#endif
