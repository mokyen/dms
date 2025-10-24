#pragma once
#include "../adapters/MotorDriver.h"
#include "../adapters/EncoderReader.h"
#include "Config.h"
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

  bool isMoving() const { return m_moving; }
  bool hasArrived() const { return m_arrived; }
  long getTargetCounts() const { return m_targetCounts; }
  long getError() const { return m_targetCounts - m_encoder.getPositionCounts(); }

  // --- Feed-forward configuration ---
  void setFeedForward(float up, float down) {
    m_feedForwardUp = up;
    m_feedForwardDown = down;
  }
  // new convenience overload
  void setFeedForward(float ff) { setFeedForward(ff, ff); }
  float getFeedForwardUp() const { return m_feedForwardUp; }
  float getFeedForwardDown() const { return m_feedForwardDown; }

  // --- PID profile control ---
  void setPidProfile(PidProfile profile);
  PidProfile getPidProfile() const { return m_activeProfile; }

  // --- Manual tuning ---
  void setPIDGains(double kp, double ki, double kd);

private:
  MotorDriver& m_motor;
  EncoderReader& m_encoder;
  long m_targetCounts;
  float m_maxSpeedDecimal;
  bool m_moving;
  bool m_arrived;
  unsigned long m_moveStartMs;
  bool m_isDescending;
  float m_lastCommand;

  // PID internals
  double m_pidInput;
  double m_pidOutput;
  double m_pidSetpoint;
  PID m_pid;

  // Feed-forward (asymmetric)
  float m_feedForwardUp;
  float m_feedForwardDown;
  static constexpr float CMD_DEADBAND = 0.02f;
  static constexpr float MAX_COMMAND_CHANGE_PER_UPDATE = 0.02f; // Ramp rate

  // PID profile control
  PidProfile m_activeProfile;

  // PID gain tables
  static constexpr double Kp_Gentle     = 0.00040;
  static constexpr double Ki_Gentle     = 0.00050;
  static constexpr double Kd_Gentle     = 0.00008;

  static constexpr double Kp_Balanced   = 0.00055;
  static constexpr double Ki_Balanced   = 0.00070;
  static constexpr double Kd_Balanced   = 0.00012;

  static constexpr double Kp_Responsive = 0.00090;
  static constexpr double Ki_Responsive = 0.00090;
  static constexpr double Kd_Responsive = 0.00018;
};
