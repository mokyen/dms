#pragma once
#include "../Config.h"
#include <Arduino.h>

class EncoderReader {
public:
  EncoderReader(uint8_t pinA, uint8_t pinB);
  void begin();
  float getPositionInches() const;
  long getPositionCounts() const;
  void zero();
  static void attachInstance(EncoderReader* inst);
};