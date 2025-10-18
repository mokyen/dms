#pragma once
#include <Arduino.h>
#include <PID_v1.h>
#include "MotorDriver.h"
#include "EncoderReader.h"
#include "Config.h"

// --- PID Profiles ---
enum class PidProfile { Gentle, Balanced, Responsive };

static constexpr double Kp_Gentle     = 0.00040;
static constexpr double Ki_Gentle     = 0.00050;
static constexpr double Kd_Gentle     = 0.00008;

static constexpr double Kp_Balanced   = 0.00055;
static constexpr double Ki_Balanced   = 0.00070;
static constexpr double Kd_Balanced   = 0.00012;

static constexpr double Kp_Responsive = 0.00090;
static constexpr double Ki_Responsive = 0.00090;
static constexpr double Kd_Responsive = 0.00018;

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
  void setFeedForward(float ff) { feedForward = ff; }
  float getFeedForward() const { return feedForward; }
  void setPidProfile(PidProfile profile);
  PidProfile getPidProfile() const { return activeProfile; }

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

  float feedForward = 0.18f;
  PidProfile activeProfile = PidProfile::Gentle;
  static constexpr float CMD_DEADBAND = 0.02f;
};