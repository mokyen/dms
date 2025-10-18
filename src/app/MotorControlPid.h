#pragma once
#include "../adapters/MotorDriver.h"
#include "../adapters/EncoderReader.h"
#include "../Config.h"
#include <PID_v1.h>

// --- PID Profiles ---
enum class PidProfile { Gentle, Balanced, Responsive };

class MotorControlPid {
public:
  MotorControlPid(MotorDriver& driver, EncoderReader& encoder);
  void begin();
  void moveToCounts(long targetCounts, int maxSpeedPercent = 100);
  void stop();
  void emergencyStop();
  void update();
  bool isMoving() const { return moving_; }
  bool hasArrived() const { return arrived_; }
  long getTargetCounts() const { return targetCounts_; }
  long getError() const { return targetCounts_ - encoder_.getPositionCounts(); }

private:
  MotorDriver& motor_;
  EncoderReader& encoder_;
  long targetCounts_;
  float maxSpeedFraction_;
  bool moving_;
  bool arrived_;
  unsigned long moveStartMs_;
  // PID internals
  double pidInput_;
  double pidOutput_;
  double pidSetpoint_;
  PID* pid_;
  PidProfile activeProfile_;
  // Add any additional state as needed
};