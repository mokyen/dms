#pragma once
#include <Arduino.h>
#include "Config.h"

class EncoderReader {
public:
  EncoderReader(uint8_t pinA, uint8_t pinB)
    : pinA(pinA), pinB(pinB) {}

  void begin() {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinA), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB), isrB, CHANGE);
  }

  static void attachInstance(EncoderReader* inst) { instance = inst; }

  float getPositionInches() const {
    return position * INV_COUNTS_PER_IN;
  }

  void handleInterrupt() {
    bool a = digitalRead(pinA);
    bool b = digitalRead(pinB);
    if (a == b) position++;
    else position--;
  }

private:
  uint8_t pinA, pinB;
  volatile long position = 0;
  static EncoderReader* instance;

  static void isrA() { if (instance) instance->handleInterrupt(); }
  static void isrB() { if (instance) instance->handleInterrupt(); }
};

inline EncoderReader* EncoderReader::instance = nullptr;