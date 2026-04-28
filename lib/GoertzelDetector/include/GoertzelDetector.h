#pragma once

#include <Arduino.h>

// Optimized Goertzel tone detector for ESP32 Arduino (float).
// - Designed for distorted/noisy signals (e.g. rectified / half-wave) where DC offset is large.
// - Uses mean subtraction (DC removal) + Hann window to reduce spectral leakage into the target bin.
// - Computes magnitude squared (energy) at one target frequency without using FFT.

namespace GoertzelDetector
{
struct Coeffs
{
  int k = 0;          // nearest DFT bin for target frequency
  float omega = 0.0f; // 2*pi*k/N
  float sine = 0.0f;
  float cosine = 1.0f;
  float coeff = 2.0f; // 2*cos(omega)
};

// Computes the Goertzel coefficients for a given target frequency.
// Note: This quantizes the target to the nearest DFT bin (k). For robust detection
// you typically scan a small neighborhood of frequencies/bins around the target.
Coeffs computeCoeffs(int N, float targetFreq, float sampleRate);

// Returns magnitude^2 (energy) at target frequency.
// - DC removal: mean subtraction across the buffer.
// - Optional low-pass: simple one-pole IIR (alpha ~ 0.1..0.2), helpful to tame noise
//   on harsh ADC signals; keep enabled only if it improves your measurements.
// - Window: Hann (reduces leakage, critical when the signal is not a clean sine).
float goertzel_detect(float *samples, int N, float target_freq, float sample_rate);

float goertzel_detect(const float *samples,
                      int N,
                      float target_freq,
                      float sample_rate,
                      bool enableLowPass = false,
                      float lowPassAlpha = 0.15f);

// Streaming-friendly Goertzel core (no buffer required).
// NOTE: For true DC removal in streaming mode you usually use an IIR mean estimate
// (high-pass behavior). The batch API above performs exact mean subtraction.
struct Stream
{
  Coeffs c;
  float s1 = 0.0f;
  float s2 = 0.0f;

  void begin(const Coeffs &coeffs)
  {
    c = coeffs;
    s1 = 0.0f;
    s2 = 0.0f;
  }

  inline void reset()
  {
    s1 = 0.0f;
    s2 = 0.0f;
  }

  inline void push(float x)
  {
    const float s0 = x + c.coeff * s1 - s2;
    s2 = s1;
    s1 = s0;
  }

  inline float magnitudeSquared() const
  {
    // |X(k)|^2 = s1^2 + s2^2 - coeff*s1*s2
    return (s1 * s1) + (s2 * s2) - (c.coeff * s1 * s2);
  }
};

// Example usage (ADC -> buffer -> Goertzel energy):
//
//   constexpr int N = 1024;
//   constexpr float FS = 8000.0f;
//   float buf[N];
//   for (int i = 0; i < N; ++i) {
//     buf[i] = (float)analogRead(32);         // raw ADC (0..4095)
//     delayMicroseconds((int)(1000000.0f/FS));
//   }
//   const float energyA2 = GoertzelDetector::goertzel_detect(buf, N, 110.0f, FS);
//   Serial.print("A2 energy^2: ");
//   Serial.println(energyA2, 2);
//
} // namespace GoertzelDetector
