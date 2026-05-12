// Unused legacy detector kept for reference; the current firmware uses TimeFrequencyTracker::GoertzelSweepTracker.
#if 0

#include "GoertzelDetector.h"

#include <math.h>

namespace GoertzelDetector
{
namespace
{
constexpr float kTwoPiF = 6.28318530717958647692f;

inline int clampInt(int v, int lo, int hi)
{
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

inline float hannWindowFromCos(float cosTheta)
{
  // w[n] = 0.5 * (1 - cos(2*pi*n/(N-1)))
  return 0.5f * (1.0f - cosTheta);
}
} // namespace

Coeffs computeCoeffs(int N, float targetFreq, float sampleRate)
{
  Coeffs out;
  if (N <= 0 || sampleRate <= 0.0f)
  {
    return out;
  }

  const float kFloat = (static_cast<float>(N) * targetFreq) / sampleRate;
  const int k = clampInt(static_cast<int>(kFloat + 0.5f), 0, N / 2);
  const float omega = (kTwoPiF * static_cast<float>(k)) / static_cast<float>(N);

  out.k = k;
  out.omega = omega;
  out.sine = sinf(omega);
  out.cosine = cosf(omega);
  out.coeff = 2.0f * out.cosine;
  return out;
}

float goertzel_detect(float *samples, int N, float target_freq, float sample_rate)
{
  return goertzel_detect(static_cast<const float *>(samples), N, target_freq, sample_rate, false, 0.15f);
}

float goertzel_detect(const float *samples,
                      int N,
                      float target_freq,
                      float sample_rate,
                      bool enableLowPass,
                      float lowPassAlpha)
{
  if (!samples || N <= 2 || sample_rate <= 0.0f)
  {
    return 0.0f;
  }

  // Mean subtraction is crucial with ESP32 ADC input (large DC offset).
  // Without it, the DC component leaks into many bins, masking the tone energy.
  float mean = 0.0f;
  for (int i = 0; i < N; ++i)
  {
    mean += samples[i];
  }
  mean /= static_cast<float>(N);

  const Coeffs coeffs = computeCoeffs(N, target_freq, sample_rate);
  Stream g;
  g.begin(coeffs);

  // Optional one-pole low-pass to tame harsh ADC noise before narrow-band detection.
  // alpha ~ 0.1..0.2 is a good starting point at 8 kHz for guitar-ish signals.
  if (lowPassAlpha < 0.0f)
    lowPassAlpha = 0.0f;
  if (lowPassAlpha > 1.0f)
    lowPassAlpha = 1.0f;

  float lp = 0.0f;

  // Hann window generation using a cosine recurrence (avoids per-sample cosf()).
  // theta(n) = 2*pi*n/(N-1)
  const float step = (N > 1) ? (kTwoPiF / static_cast<float>(N - 1)) : 0.0f;
  const float cosStep = cosf(step);
  const float sinStep = sinf(step);
  float cosTheta = 1.0f; // cos(0)
  float sinTheta = 0.0f; // sin(0)

  for (int n = 0; n < N; ++n)
  {
    float x = samples[n] - mean;

    if (enableLowPass)
    {
      lp += lowPassAlpha * (x - lp);
      x = lp;
    }

    // Windowing reduces leakage when the input isn't a clean sine and when the tone
    // doesn't land exactly on a DFT bin (common with real instruments).
    const float w = hannWindowFromCos(cosTheta);
    x *= w;

    g.push(x);

    // Recurrence update for cos/sin(theta).
    const float nextCos = (cosTheta * cosStep) - (sinTheta * sinStep);
    const float nextSin = (sinTheta * cosStep) + (cosTheta * sinStep);
    cosTheta = nextCos;
    sinTheta = nextSin;
  }

  const float mag2 = g.magnitudeSquared();
  return (mag2 >= 0.0f) ? mag2 : 0.0f;
}
} // namespace GoertzelDetector

#endif
