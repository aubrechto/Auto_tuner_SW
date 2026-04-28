#pragma once

#include <Arduino.h>

namespace LogMirror
{
void begin(Print &primary);
void setMirror(Print *mirrorOrNull);

Print &out();
} // namespace LogMirror

