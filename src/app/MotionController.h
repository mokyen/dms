// Application layer: orchestrates high-level motion control, PID, and pattern execution
#pragma once

#include "adapters/MotorDriver.h"
#include "adapters/EncoderReader.h"
#include "app/MotorControlPid.h"
#include "Config.h"
#include <Arduino.h>

// Thin wrapper / orchestrator around MotorControlPid, EncoderReader and MotorDriver.
// Designed to be small and delegating so MotorControlPid remains accessible for direct tests.
class MotionController {
public:
  MotionController(MotorControlPid& pid, EncoderReader& encoder, MotorDriver& motor);

  // Central command entry point used by dms.ino
  void handleCommand(char cmd, float value = 0.0f);

  // Called from main loop at high frequency
  void update();

  // Thin wrappers (also callable directly)
  void moveToTop();
  void moveToBottom();
  void moveToPercent(float percent);
  void stopMotor();
  void printPosition();
  void zeroEncoder();

  // Test / diagnostics wrappers (delegates/backups)
  void rampUpSpeed();
  void jogMode();
  void continuousMonitor();
  void currentLimitTest();
  void manualDutyMode();

  // System ID / tests
  void resistanceMeasurement();
  void stepResponse();
  void backEMFTest();
  void rlsElectricalTest();
  void continuousRLSTest();

  // Spider patterns
  void runPattern(int id);
  void hauntingMode();
  void runRandomPattern();

  // Configuration setters
  void setFeedForward(float ff);         // convenience single value (symmetric)
  void setFeedForward(float up, float down);
  void setPidProfile(int idx);

  // Expose underlying components for direct testing if needed
  MotorControlPid& pid() { return pid_; }
  EncoderReader& encoder() { return encoder_; }
  MotorDriver& motor() { return motor_; }

private:
  MotorControlPid& pid_;
  EncoderReader& encoder_;
  MotorDriver& motor_;
};
