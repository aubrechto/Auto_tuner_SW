#include "LogMirror.h"

namespace LogMirror
{
namespace
{
class MirrorPrint final : public Print
{
public:
  explicit MirrorPrint(Print *primary) : primary_(primary) {}

  void setMirror(Print *mirror) { mirror_ = mirror; }

  size_t write(uint8_t b) override
  {
    size_t written = 0;
    if (primary_)
    {
      written = primary_->write(b);
    }
    if (mirror_)
    {
      mirror_->write(b);
    }
    return written;
  }

  size_t write(const uint8_t *buffer, size_t size) override
  {
    size_t written = 0;
    if (primary_)
    {
      written = primary_->write(buffer, size);
    }
    if (mirror_)
    {
      mirror_->write(buffer, size);
    }
    return written;
  }

private:
  Print *primary_ = nullptr;
  Print *mirror_ = nullptr;
};

MirrorPrint *logger = nullptr;
} // namespace

void begin(Print &primary)
{
  static MirrorPrint instance(&primary);
  logger = &instance;
}

void setMirror(Print *mirrorOrNull)
{
  if (!logger)
  {
    return;
  }
  logger->setMirror(mirrorOrNull);
}

Print &out()
{
  if (logger)
  {
    return *logger;
  }
  return Serial;
}
} // namespace LogMirror

