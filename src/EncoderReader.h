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

  static void isrA() { instance->handleInterrupt(); }
  static void isrB() { instance->handleInterrupt(); }

  void handleInterrupt() {
    bool a = digitalRead(pinA);
    bool b = digitalRead(pinB);
    if (a == b) position++;
    else position--;
  }

  float getPositionInches() const {
    return position * INV_COUNTS_PER_IN;
  }

  int getDirection() const {
    return (position >= lastPosition) ? 1 : -1;
  }

  static void attachInstance(EncoderReader* inst) { instance = inst; }

private:
  uint8_t pinA, pinB;
  volatile long position = 0;
  mutable long lastPosition = 0;
  static EncoderReader* instance;
};
