#pragma once
#include <Arduino.h>
#include "MotorDriver.h"
#include "EncoderReader.h"

enum class MotorPosition { Unknown, Top, Bottom, Moving };

class MotorControl {
public:
  MotorControl(MotorDriver& driver, EncoderReader& encoder);
  void begin();
  void moveToPosition(MotorPosition target);
  void moveToPositionPercent(float targetPercent);
  void stopAtTop();
  void stopAtBottom();
  void update();

  MotorPosition currentPosition() const { return position; }

private:
  MotorDriver& motor;
  EncoderReader& encoder;
  MotorPosition position;
  MotorPosition target;
  unsigned long lastUpdateMs;
};