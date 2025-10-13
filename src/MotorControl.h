#pragma once
#include <Arduino.h>
#include "MotorDriver.h"
#include "EncoderReader.h"
#include "Config.h"

class MotorControl {
public:
  MotorControl(MotorDriver& driver, EncoderReader& encoder);
  
  void begin();
  
  // Simple public API
  void moveToCounts(long targetCounts, int maxSpeedPercent = 100);
  void stop();
  void emergencyStop();
  
  // Call this in loop() - handles all control logic
  void update();
  
  // Status queries
  bool isMoving() const { return moving; }
  bool hasArrived() const { return arrived; }
  long getTargetCounts() const { return targetCounts; }
  long getError() const { return targetCounts - encoder.getPositionCounts(); }

private:
  MotorDriver& motor;
  EncoderReader& encoder;
  
  long targetCounts;
  float maxSpeedFraction;  // Stored as 0.0-1.0
  bool moving;
  bool arrived;
  unsigned long moveStartMs;
  
  // Control law - easy to swap out for PID later
  float computeControlOutput(long error);
};