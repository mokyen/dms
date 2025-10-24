#pragma once
#include <Arduino.h>
#include "../app/Config.h"

class EncoderReader {
public:
  EncoderReader(uint8_t pinA, uint8_t pinB)
    : pinA(pinA), pinB(pinB) {}

  void begin() {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    instance = this;
    
    // Read initial state
    lastState = (digitalRead(pinA) << 1) | digitalRead(pinB);
    
    attachInterrupt(digitalPinToInterrupt(pinA), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB), isrB, CHANGE);
  }

  static void attachInstance(EncoderReader* inst) { instance = inst; }

  float getPositionInches() const {
    return getPositionCounts() * INV_COUNTS_PER_IN;
  }

  long getPositionCounts() const {
    long currentPosition;
    noInterrupts(); 
    currentPosition = position;
    interrupts(); 
    return currentPosition;
  }

  void handleInterrupt() {
    // Read current state
    uint8_t newState = (digitalRead(pinA) << 1) | digitalRead(pinB);
    
    // Use lookup table for state transitions
    // This is more robust than reading both pins separately
    int8_t delta = stateTable[lastState][newState];
    
    position += delta;
    lastState = newState;
  }

  void zero() {
    noInterrupts();
    position = 0;
    interrupts();
  }

private:
  uint8_t pinA, pinB;
  volatile long position = 0;
  volatile uint8_t lastState = 0;
  static EncoderReader* instance;

  // Quadrature state transition table
  // [old_state][new_state] = direction
  // States: 00, 01, 10, 11 (binary AB)
  static constexpr int8_t stateTable[4][4] = {
    //  00   01   10   11   <- new state
    {   0,  +1,  -1,   0 }, // 00 old state
    {  -1,   0,   0,  +1 }, // 01
    {  +1,   0,   0,  -1 }, // 10
    {   0,  -1,  +1,   0 }  // 11
  };

  static void isrA() { if (instance) instance->handleInterrupt(); }
  static void isrB() { if (instance) instance->handleInterrupt(); }
};

inline EncoderReader* EncoderReader::instance = nullptr;