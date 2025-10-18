#include "EncoderReader.h"
#include <Arduino.h>

EncoderReader::EncoderReader(uint8_t pinA, uint8_t pinB) {}
void EncoderReader::begin() {}
float EncoderReader::getPositionInches() const { return 0.0f; }
long EncoderReader::getPositionCounts() const { return 0; }
void EncoderReader::zero() {}
void EncoderReader::attachInstance(EncoderReader* inst) {}
