#pragma once
#include "../Config.h"
#include <Arduino.h>

class MotorDriver {
public:
  MotorDriver(uint8_t pwm, uint8_t ina, uint8_t inb, uint8_t cs);
  void begin();
  void stop();
  void brake();
  float readCurrent() const;
private:
  // ...existing code...
};