#pragma once
#include <Arduino.h>
#include "MotorDriver.h"
#include "EncoderReader.h"

// Represents where the decoration motor is positioned
enum class MotorPosition {
  Unknown,
  Top,
  Bottom,
  Moving
};

class MotorControl {
public:
  explicit MotorControl(MotorDriver& driver, EncoderReader& encoder);

  void begin();
  void moveToPosition(MotorPosition target);
  void moveToPositionPercent(float targetPercent);
  void stopAtTop();
  void stopAtBottom();
  void update(); // call periodically (e.g., in loop())

  MotorPosition currentPosition() const { return position; }

private:
  MotorDriver& motor;
  EncoderReader& encoder;
  MotorPosition position;
  MotorPosition target;
  unsigned long lastUpdateMs;
  static constexpr unsigned long MOVE_TIMEOUT_MS = 5000; // safety stop
};
