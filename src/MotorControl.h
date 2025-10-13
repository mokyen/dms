#pragma once
#include <Arduino.h>
#include "MotorDriver.h"
#include "EncoderReader.h"

enum class MotorPosition { Unknown, Top, Bottom, Moving }; 

class MotorControl {
public:
  MotorControl(MotorDriver& driver, EncoderReader& encoder);
  void begin();
  
  // PRIMARY PUBLIC API: All moves now use counts
  void moveToPositionCounts(long targetCounts);
  
  void stopAtTop();
  void stopAtBottom();
  void update();

  MotorPosition currentPosition() const { return position; }

private:
  MotorDriver& motor;
  EncoderReader& encoder;
  MotorPosition position;
  MotorPosition target; // Tracks the *target* state (Top/Bottom/Unknown)
  long targetCounts;
  unsigned long lastUpdateMs;

  // Private helper function to consolidate the move initiation logic
  void setTargetAndStartMove(long counts, MotorPosition state);
};