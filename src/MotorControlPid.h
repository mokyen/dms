#pragma once
#include <Arduino.h>
#include <PID_v1.h>
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
  
  // PID tuning (optional - can adjust at runtime)
  void setPIDGains(double kp, double ki, double kd);

private:
  MotorDriver& motor;
  EncoderReader& encoder;
  
  long targetCounts;
  float maxSpeedFraction;
  bool moving;
  bool arrived;
  unsigned long moveStartMs;
  
  // PID variables (must be double for PID library)
  double pidInput, pidOutput, pidSetpoint;
  PID* pid;
  
  // PID gains - conservative for minimal overshoot
  static constexpr double Kp = 0.0003;  // Was 0.0008 - too aggressive
  static constexpr double Ki = 0.00015;     // Keep zero
  static constexpr double Kd = 0.0001;  // Was 0.002 - WAY too high
};