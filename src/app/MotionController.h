// Application layer: orchestrates high-level motion control, PID, and pattern execution
#pragma once

#include "adapters/MotorDriver.h"
#include "adapters/EncoderReader.h"
#include "app/MotorControlPid.h"
#include "Config.h"
#include <Arduino.h>

inline void printHelp() {
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║    DMS Manual Control Menu         ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.println(F("=== Motion Commands ==="));
  Serial.println(F("u       - Move to TOP (min speed)"));
  Serial.println(F("d       - Move to BOTTOM (min speed)"));
  Serial.println(F("p <num> - Move to % of travel (0-100)"));
  Serial.println(F("s       - STOP motor immediately"));
  Serial.println(F("z       - Zero encoder position (set current as 0)"));
  Serial.println(F(""));
  Serial.println(F("=== Diagnostic Modes ==="));
  Serial.println(F("r       - READ encoder position & current"));
  Serial.println(F("j       - JOG MODE (incremental movement)"));
  Serial.println(F("c       - CONTINUOUS MONITOR (live data)"));
  Serial.println(F(""));
  Serial.println(F("=== Testing Modes ==="));
  Serial.println(F("m       - RAMP speed test (alternates dir)"));
  Serial.println(F("l       - CURRENT LIMIT test"));
  Serial.println(F("f       - MANUAL DUTY CONTROL mode"));
  Serial.println(F(""));
  Serial.println(F("=== System ID Tests ==="));
  Serial.println(F("R       - RESISTANCE measurement (R)"));
  Serial.println(F("K       - BACK-EMF constant test (Kt)"));
  Serial.println(F("S       - STEP RESPONSE test (L, R, J, b)"));
  Serial.println(F("E       - RLS electrical params (L, R, Kt)"));
  Serial.println(F("X       - TOGGLE continuous RLS (background)"));
  Serial.println(F(""));
  Serial.println(F("=== Spider Patterns ==="));
  Serial.println(F("1       - Stalker Pattern"));
  Serial.println(F("2       - Pounce Pattern"));
  Serial.println(F("3       - Patrol Pattern"));
  Serial.println(F("4       - Twitch Pattern"));
  Serial.println(F("5       - Lurker Pattern"));
  Serial.println(F("A       - Haunting Mode"));
  Serial.println(F("T       - Run Random Pattern"));
  Serial.println(F(""));
  Serial.println(F("=== Control Parameters ==="));
  Serial.println(F("F <val> - Set feed-forward (0.0–0.5)"));
  Serial.println(F("Q <num> - Set PID profile (1=Gentle, 2=Balanced, 3=Responsive)"));
  Serial.println(F(""));
  Serial.println(F("h       - Show this help menu"));
  Serial.println(F("?       - Show system status"));
  Serial.println();
}

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
