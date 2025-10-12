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
    instance = this;
    attachInterrupt(digitalPinToInterrupt(pinA), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB), isrB, CHANGE);
  }

  static void attachInstance(EncoderReader* inst) { instance = inst; }

  float getPositionInches() const {
    return position * INV_COUNTS_PER_IN;
  }

  long getPositionCounts() const {
    return position;
  }

  void handleInterruptA() {
    bool a = digitalRead(pinA);
    bool b = digitalRead(pinB);
    if (a == b) {
      position++;
    } else {
      position--;
    }
  }

  void handleInterruptB() {
    bool a = digitalRead(pinA);
    bool b = digitalRead(pinB);
    if (a != b) {
      position++;
    } else {
      position--;
    }
  }

  void zero() {
    position = 0;
  }

private:
  uint8_t pinA, pinB;
  volatile long position = 0;
  static EncoderReader* instance;

  static void isrA() { if (instance) instance->handleInterruptA(); }
  static void isrB() { if (instance) instance->handleInterruptB(); }
};

inline EncoderReader* EncoderReader::instance = nullptr;