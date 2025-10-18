// Application layer: orchestrates high-level motion control, PID, and pattern execution
#pragma once
#include "app/MotionController.h"
#include "adapters/SerialCommandAdapter.h"
#include "adapters/MotorDriver.h"
#include "adapters/EncoderReader.h"
#include "app/MotorControlPid.h"
#include "Config.h"

class MotionController {
public:
  MotionController(MotorControlPid& pid, EncoderReader& encoder, MotorDriver& motor);
  void handleCommand(char cmd, float value);
  void update();

  // Migrated test and pattern methods
  void moveToTop();
  void moveToBottom();
  void moveToPercent(float percent);
  void stopMotor();
  void printPosition();
  void zeroEncoder();
  void rampUpSpeed();
  void jogMode();
  void continuousMonitor();
  void currentLimitTest();
  void manualDutyMode();
  void resistanceMeasurement();
  void stepResponse();
  void backEMFTest();
  void rlsElectricalTest();
  void continuousRLSTest();
  void updateContinuousRLS();
  void printHelp();
  void printStatus();
  // Spider patterns
  void runPattern(int patternId);
  void hauntingMode();
  void runRandomPattern();
  void setFeedForward(float ff);
  void setPidProfile(int idx);

private:
  MotorControlPid& pid_;
  EncoderReader& encoder_;
  MotorDriver& motor_;
  // Add state for current mode, pattern, etc.
};
