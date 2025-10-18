#include "MotorDriver.h"
#include <Arduino.h>

MotorDriver::MotorDriver(uint8_t pwm, uint8_t ina, uint8_t inb, uint8_t cs) {}
void MotorDriver::begin() {}
void MotorDriver::stop() {}
void MotorDriver::brake() {}
float MotorDriver::readCurrent() const { return 0.0f; }
