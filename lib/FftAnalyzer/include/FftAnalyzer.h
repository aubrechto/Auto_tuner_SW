#pragma once

// Unused legacy analyzer kept for reference; the current firmware uses TimeFrequencyTracker::GoertzelSweepTracker.
#if 0

#include <Arduino.h>
#include <functional>

namespace FftAnalyzer
{
constexpr int MinSignalAmplitude = 150;

// Master switch for all FFT notch filters (50/100 Hz + optional 70 Hz).
// Toggle from your code via: `FftAnalyzer::FiltersEnabled = true/false;`
extern bool FiltersEnabled;

// Optional interference notch around 70±3 Hz (helps reject EMI / "antenna" pickup).
// Toggle from your code via: `FftAnalyzer::Enable70HzNotch = true/false;`
extern bool Enable70HzNotch;

bool findDominantFrequency(uint8_t adcPin,
                           float targetFrequency,
                           double &detectedFrequency,
                           int &signalAmplitude,
                           const std::function<void()> &pump,
                           const std::function<bool()> &isActive);
} // namespace FftAnalyzer

#endif
