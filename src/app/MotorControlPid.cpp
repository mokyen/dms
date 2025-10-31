#include "MotorControlPid.h"
#include <Arduino.h>


MotorControlPid::MotorControlPid(MotorDriver& driver, EncoderReader& encoder)
  : m_motor(driver), m_encoder(encoder), m_targetCounts(0), m_maxSpeedDecimal(1.0f), m_moving(false),
    m_arrived(false), m_moveStartMs(0), m_lastCommand(0.0f), m_pidInput(0), m_pidOutput(0),
    m_pidSetpoint(0), m_pid(&m_pidInput, &m_pidOutput, &m_pidSetpoint,
      Kp_Balanced, Ki_Balanced, Kd_Balanced, DIRECT), m_feedForwardUp(0.20f),
      m_feedForwardDown(0.20f), m_activeProfile(PidProfile::Balanced)
    
{
  m_pid.SetMode(AUTOMATIC);
  m_pid.SetOutputLimits(-1.0, 1.0);
  m_pid.SetSampleTime(10);  // 10 ms (100 Hz)
}

void MotorControlPid::begin() {
  m_motor.begin();
  m_encoder.begin();
  EncoderReader::attachInstance(&m_encoder);
}

void MotorControlPid::stop() {
  m_motor.stop();
  m_moving = false;
  m_arrived = false;
}

void MotorControlPid::emergencyStop() {
  m_motor.brake();
  m_moving = false;
  m_arrived = false;
}

void MotorControlPid::setPidProfile(PidProfile profile) {
  m_activeProfile = profile;
  double kp, ki, kd;

  switch (profile) {
    case PidProfile::Gentle:
      kp = Kp_Gentle; ki = Ki_Gentle; kd = Kd_Gentle; break;
    case PidProfile::Balanced:
      kp = Kp_Balanced; ki = Ki_Balanced; kd = Kd_Balanced; break;
    case PidProfile::Responsive:
      kp = Kp_Responsive; ki = Ki_Responsive; kd = Kd_Responsive; break;
  }

  m_pid.SetTunings(kp, ki, kd);

  Serial.print(F("PID profile set: "));
  Serial.println((profile == PidProfile::Gentle) ? "Gentle" :
                 (profile == PidProfile::Balanced) ? "Balanced" : "Responsive");
}

void MotorControlPid::setPIDGains(double kp, double ki, double kd) {
  m_pid.SetTunings(kp, ki, kd);
  Serial.print(F("PID gains manually set: Kp="));
  Serial.print(kp, 6);
  Serial.print(F(" Ki="));
  Serial.print(ki, 6);
  Serial.print(F(" Kd="));
  Serial.println(kd, 6);
}


void MotorControlPid::moveToCounts(long counts, int maxSpeedPercent) {
  m_targetCounts = constrain(counts, 0L, MAX_TRAVEL_COUNTS);
  maxSpeedPercent = constrain(maxSpeedPercent, 1, 100);
  static constexpr float TO_DECIMAL = 0.01f;
  m_maxSpeedDecimal = static_cast<float>(maxSpeedPercent) * TO_DECIMAL;
  m_pidSetpoint = static_cast<double>(m_targetCounts);
  m_pid.SetOutputLimits(-1.0, 1.0);

  // **Detect if this is a descent**
  long currentPos = m_encoder.getPositionCounts();
  m_isDescending = (counts < currentPos);

  m_moving = true;
  m_arrived = false;
  m_moveStartMs = millis();

  Serial.print(F("Moving to: "));
  Serial.print(m_targetCounts);
  Serial.print(F(" counts @ "));
  Serial.print(maxSpeedPercent);
  Serial.print(F("% max speed"));
  if (m_isDescending) {
    Serial.print(F(" (DESCENT MODE)"));
  }
  Serial.println();
}

void MotorControlPid::update() {
  if (!m_moving) return;

  // Timeout safety
  if (millis() - m_moveStartMs > MOVE_TIMEOUT_MS) {
    emergencyStop();
    Serial.println(F("ERROR: Move timeout!"));
    return;
  }

  long currentPos = m_encoder.getPositionCounts();
  long error = m_targetCounts - currentPos;
  long absError = labs(error);

  // Check if arrived
  if (absError <= POSITION_TOLERANCE_COUNTS) {
    if (m_targetCounts == 0) {
      // At the bottom, just stop. No holding torque needed.
      m_motor.stop();
      Serial.println(F("Arrived at bottom!"));
    } else {
      // Apply holding torque to counteract gravity/load.
      // float holdCommand = (error > 0) ? m_feedForwardUp : m_feedForwardDown;
      // holdCommand = constrain(holdCommand, -m_maxSpeedDecimal, m_maxSpeedDecimal);
      // m_motor.setSpeed(holdCommand);
      m_motor.stop();
      m_lastCommand = 0.0f;

      Serial.print(F("Arrived! Holding torque applied (cmd=0.0f"));
      // Serial.print(0, 3);
      Serial.println(F(")"));
    }

    m_moving = false;
    m_arrived = true;
    return;
  }


  // **NORMAL PID CONTROL for upward motion**
  m_pidInput = static_cast<double>(currentPos);
  m_pidSetpoint = static_cast<double>(m_targetCounts);
  m_pid.Compute();

  float command = static_cast<float>(m_pidOutput) + m_feedForwardUp;
  
  command = constrain(command, -m_maxSpeedDecimal, m_maxSpeedDecimal);
  m_motor.setSpeed(command);
  m_lastCommand = command;

  // Debug print (every 50 ms)
  static unsigned long lastPrintMs = 0;
  if (millis() - lastPrintMs > 50) {
    Serial.print(F("ASCENT ms=")); Serial.print(millis() - m_moveStartMs);
    Serial.print(F(" pos=")); Serial.print(currentPos);
    Serial.print(F(" tgt=")); Serial.print(m_targetCounts);
    Serial.print(F(" err=")); Serial.print(error);
    Serial.print(F(" pid=")); Serial.print(m_pidOutput, 3);
    // Serial.print(F(" ff=")); Serial.print(ff, 3);
    Serial.print(F(" cmd=")); Serial.print(command, 3);
    Serial.print(F(" amps=")); Serial.println(m_motor.readCurrent(), 3);
    lastPrintMs = millis();
  }
}