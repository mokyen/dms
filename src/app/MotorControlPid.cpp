#include "MotorControlPid.h"
#include <Arduino.h>


MotorControlPid::MotorControlPid(MotorDriver& driver, EncoderReader& encoder)
  : m_motor(driver), m_encoder(encoder), m_targetCounts(0), m_maxSpeedDecimal(1.0f), m_moving(false),
    m_arrived(false), m_moveStartMs(0), m_lastCommand(0.0f), m_pidInput(0), m_pidOutput(0),
    m_pidSetpoint(0), m_pid(&m_pidInput, &m_pidOutput, &m_pidSetpoint,
      Kp_Gentle, Ki_Gentle, Kd_Gentle, DIRECT), m_feedForwardUp(0.20f),
      m_feedForwardDown(0.18f), m_activeProfile(PidProfile::Gentle)
    
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

// void MotorControlPid::moveToCounts(long counts, int maxSpeedPercent) {
//   m_targetCounts = constrain(counts, 0L, MAX_TRAVEL_COUNTS);
//   maxSpeedPercent = constrain(maxSpeedPercent, 1, 100);
//   static constexpr float TO_DECIMAL = 0.01f;
//   m_maxSpeedDecimal = static_cast<float>(maxSpeedPercent) * TO_DECIMAL;
//   m_pidSetpoint = static_cast<double>(m_targetCounts);
//   m_pid.SetOutputLimits(-1.0, 1.0);

//   // **Detect if this is a descent**
//   long currentPos = m_encoder.getPositionCounts();
//   m_isDescending = (counts < currentPos);

//   m_moving = true;
//   m_arrived = false;
//   m_moveStartMs = millis();

//   Serial.print(F("Moving to: "));
//   Serial.print(m_targetCounts);
//   Serial.print(F(" counts @ "));
//   Serial.print(maxSpeedPercent);
//   Serial.print(F("% max speed"));
//   if (m_isDescending) {
//     Serial.print(F(" (DESCENT MODE)"));
//   }
//   Serial.println();
// }



// void MotorControlPid::update() {
//   if (!m_moving) return;

//   // Timeout safety
//   if (millis() - m_moveStartMs > MOVE_TIMEOUT_MS) {
//     emergencyStop();
//     Serial.println(F("ERROR: Move timeout!"));
//     return;
//   }

//   long currentPos = m_encoder.getPositionCounts();
//   long error = m_targetCounts - currentPos;
//   long absError = labs(error);

//   // Check if arrived
//   if (absError <= POSITION_TOLERANCE_COUNTS) {
//     if (m_targetCounts == 0) {
//       // At the bottom, just stop. No holding torque needed.
//       m_motor.stop();
//       Serial.println(F("Arrived at bottom!"));
//     } else {
//       // Apply holding torque to counteract gravity/load.
//       // If error > 0 (below target), push up. If error < 0 (above target), pull down.
//       float holdCommand = (error > 0) ? m_feedForwardUp : -m_feedForwardDown;
//       holdCommand = constrain(holdCommand, -m_maxSpeedDecimal, m_maxSpeedDecimal);
//       m_motor.setSpeed(holdCommand);
//       m_lastCommand = holdCommand; // Save the holding command for the next move's ramp

//       Serial.print(F("Arrived! Holding torque applied (cmd="));
//       Serial.print(holdCommand, 3);
//       Serial.println(F(")"));
//     }

//     m_moving = false;
//     m_arrived = true;
//     return;
//   }

//   // **DESCENT MODE: Open-loop ramp with light position correction**
//   if (m_isDescending) {
//     // Target a gentle downward command
//     float targetCommand = -0.05f;  // Adjust this for descent speed
    
//     // Ramp smoothly from current command (typically +0.20 from holding)
//     float commandDelta = targetCommand - m_lastCommand;
//     if (fabs(commandDelta) > 0.02f) {
//       m_lastCommand += copysignf(0.02f, commandDelta);
//     } else {
//       m_lastCommand = targetCommand;
//     }
    
//     // Add gentle proportional correction as we approach target
//     if (absError < 500) {  // Within ~13 inches of target
//       float pCorrection = -0.0002f * error;  // Gentle P term
//       m_lastCommand += pCorrection;
//     }
    
//     // Constrain to max speed
//     m_lastCommand = constrain(m_lastCommand, -m_maxSpeedDecimal, m_maxSpeedDecimal);
//     m_motor.setSpeed(m_lastCommand);
    
//     // Debug print (every 50 ms)
//     static unsigned long lastPrintMs = 0;
//     if (millis() - lastPrintMs > 50) {
//       Serial.print(F("DESCENT ms=")); Serial.print(millis() - m_moveStartMs);
//       Serial.print(F(" pos=")); Serial.print(currentPos);
//       Serial.print(F(" tgt=")); Serial.print(m_targetCounts);
//       Serial.print(F(" err=")); Serial.print(error);
//       Serial.print(F(" cmd=")); Serial.print(m_lastCommand, 3);
//       Serial.print(F(" amps=")); Serial.println(m_motor.readCurrent(), 3);
//       lastPrintMs = millis();
//     }
    
//     return;  // Skip normal PID
//   }

//   // **NORMAL PID CONTROL for upward motion**
//   m_pidInput = static_cast<double>(currentPos);
//   m_pid.Compute();

//   // Apply asymmetric feed-forward
//   float ff = (error > 0) ? m_feedForwardUp : -m_feedForwardDown;
//   float command = static_cast<float>(m_pidOutput) + ff;
  
//   command = constrain(command, -m_maxSpeedDecimal, m_maxSpeedDecimal);
//   m_motor.setSpeed(command);
//   m_lastCommand = command;

//   // Debug print (every 50 ms)
//   static unsigned long lastPrintMs = 0;
//   if (millis() - lastPrintMs > 50) {
//     Serial.print(F("ASCENT ms=")); Serial.print(millis() - m_moveStartMs);
//     Serial.print(F(" pos=")); Serial.print(currentPos);
//     Serial.print(F(" tgt=")); Serial.print(m_targetCounts);
//     Serial.print(F(" err=")); Serial.print(error);
//     Serial.print(F(" pid=")); Serial.print(m_pidOutput, 3);
//     Serial.print(F(" ff=")); Serial.print(ff, 3);
//     Serial.print(F(" cmd=")); Serial.print(command, 3);
//     Serial.print(F(" amps=")); Serial.println(m_motor.readCurrent(), 3);
//     lastPrintMs = millis();
//   }
// }

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

// void MotorControlPid::update() {
//   if (!m_moving) return;

//   // Timeout safety
//   if (millis() - m_moveStartMs > MOVE_TIMEOUT_MS) {
//     emergencyStop();
//     Serial.println(F("ERROR: Move timeout!"));
//     return;
//   }

//   long currentPos = m_encoder.getPositionCounts();
//   long error = m_targetCounts - currentPos;
//   long absError = labs(error);

//   // Check if arrived
//   if (absError <= POSITION_TOLERANCE_COUNTS) {
//     if (m_targetCounts == 0) {
//       // At the bottom, just stop. No holding torque needed.
//       m_motor.stop();
//       Serial.println(F("Arrived at bottom!"));
//     } else {
//       // Apply holding torque to counteract gravity/load.
//       float holdCommand = (error > 0) ? m_feedForwardUp : -m_feedForwardDown;
//       holdCommand = constrain(holdCommand, -m_maxSpeedDecimal, m_maxSpeedDecimal);
//       m_motor.setSpeed(holdCommand);
//       m_lastCommand = holdCommand;

//       Serial.print(F("Arrived! Holding torque applied (cmd="));
//       Serial.print(holdCommand, 3);
//       Serial.println(F(")"));
//     }

//     m_moving = false;
//     m_arrived = true;
//     return;
//   }

//   // **DESCENT MODE: Open-loop ramp with light position correction**
//   if (m_isDescending) {
//     // Target a gentle downward command
//     float targetCommand = -0.05f;  // Adjust this for descent speed
    
//     // // Ramp smoothly from current command (typically +0.20 from holding)
//     // float commandDelta = targetCommand - m_lastCommand;
//     // if (fabs(commandDelta) > 0.02f) {
//     //   m_lastCommand += copysignf(0.02f, commandDelta);
//     // } else {
//     //   m_lastCommand = targetCommand;
//     // }
    
//     // // Add gentle proportional correction as we approach target
//     // // if (absError < 500) {  // Within ~13 inches of target
//     // //   float pCorrection = -0.0002f * error;  // Gentle P term

//     //   // **Only add P correction very close to target**
//     // if (absError < 100) {  // Changed from 500 to 100 (within ~2.5 inches)
//     //   float pCorrection = -0.0005f * error;  // Increased gain from 0.0002 to 0.0005
  
//     //   m_lastCommand += pCorrection;
//     // }

//     // float commandDelta = targetCommand - m_lastCommand;
//     // if (fabs(commandDelta) > 0.02f) {
//     //   m_lastCommand += copysignf(0.02f, commandDelta);
//     // } else {
//     //   m_lastCommand = targetCommand;
//     // }
    
//     // // Gentle P correction for final approach
//     // if (absError < 200) {  // Start earlier (within ~5 inches)
//     //   float pCorrection = -0.0003f * error;  // Reduced from 0.0005 to 0.0003
//     //   m_lastCommand += pCorrection;
//     // }

//     // // Ramp smoothly from holding torque
//     // float commandDelta = targetCommand - m_lastCommand;
//     // if (fabs(commandDelta) > 0.02f) {
//     //   m_lastCommand += copysignf(0.02f, commandDelta);
//     // } else {
//     //   m_lastCommand = targetCommand;
//     // }
    
//     // // Very gentle P correction closer to target
//     // if (absError < 150) {  // Within ~4 inches
//     //   float pCorrection = -0.0001f * error;  // Much smaller gain
//     //   m_lastCommand += pCorrection;
//     // }
    
//     // Ramp smoothly
//     float commandDelta = targetCommand - m_lastCommand;
//     if (fabs(commandDelta) > 0.02f) {
//       m_lastCommand += copysignf(0.02f, commandDelta);
//     } else {
//       m_lastCommand = targetCommand;
//     }
    
//     // Gentle correction that scales with distance
//     if (absError < 100) {  // Within ~2.5 inches - very close
//       // At 100 counts: correction = 0.01, at 10 counts: 0.001
//       float pCorrection = -0.0001f * error;
//       m_lastCommand += pCorrection;
      
//       // Add damping when very close to prevent overshoot
//       if (absError < 50) {
//         m_lastCommand *= 0.7f;  // Reduce command by 30% when very close
//       }
//     }

//     // Constrain to max speed
//     m_lastCommand = constrain(m_lastCommand, -m_maxSpeedDecimal, m_maxSpeedDecimal);
//     m_motor.setSpeed(m_lastCommand);
    
//     // Debug print (every 50 ms)
//     static unsigned long lastPrintMs = 0;
//     if (millis() - lastPrintMs > 50) {
//       Serial.print(F("DESCENT ms=")); Serial.print(millis() - m_moveStartMs);
//       Serial.print(F(" pos=")); Serial.print(currentPos);
//       Serial.print(F(" tgt=")); Serial.print(m_targetCounts);
//       Serial.print(F(" err=")); Serial.print(error);
//       Serial.print(F(" cmd=")); Serial.print(m_lastCommand, 3);
//       Serial.print(F(" amps=")); Serial.println(m_motor.readCurrent(), 3);
//       lastPrintMs = millis();
//     }
    
//     return;  // Skip normal PID
//   }

//   // **NORMAL PID CONTROL for upward motion**
//   m_pidInput = static_cast<double>(currentPos);
//   m_pid.Compute();

//   // Apply asymmetric feed-forward
//   float ff = (error > 0) ? m_feedForwardUp : -m_feedForwardDown;
//   float command = static_cast<float>(m_pidOutput) + ff;
  
//   command = constrain(command, -m_maxSpeedDecimal, m_maxSpeedDecimal);
//   m_motor.setSpeed(command);
//   m_lastCommand = command;

//   // Debug print (every 50 ms)
//   static unsigned long lastPrintMs = 0;
//   if (millis() - lastPrintMs > 50) {
//     Serial.print(F("ASCENT ms=")); Serial.print(millis() - m_moveStartMs);
//     Serial.print(F(" pos=")); Serial.print(currentPos);
//     Serial.print(F(" tgt=")); Serial.print(m_targetCounts);
//     Serial.print(F(" err=")); Serial.print(error);
//     Serial.print(F(" pid=")); Serial.print(m_pidOutput, 3);
//     Serial.print(F(" ff=")); Serial.print(ff, 3);
//     Serial.print(F(" cmd=")); Serial.print(command, 3);
//     Serial.print(F(" amps=")); Serial.println(m_motor.readCurrent(), 3);
//     lastPrintMs = millis();
//   }
// }

//WORKED OKAY
// void MotorControlPid::update() {
//   if (!m_moving) return;

//   // Timeout safety
//   if (millis() - m_moveStartMs > MOVE_TIMEOUT_MS) {
//     emergencyStop();
//     Serial.println(F("ERROR: Move timeout!"));
//     return;
//   }

//   long currentPos = m_encoder.getPositionCounts();
//   long error = m_targetCounts - currentPos;
//   long absError = labs(error);

//   // Check if arrived
//   if (absError <= POSITION_TOLERANCE_COUNTS) {
//     if (m_targetCounts == 0) {
//       // At the bottom, just stop. No holding torque needed.
//       m_motor.stop();
//       Serial.println(F("Arrived at bottom!"));
//     } else {
//       // Apply holding torque to counteract gravity/load.
//       float holdCommand = (error > 0) ? m_feedForwardUp : -m_feedForwardDown;
//       holdCommand = constrain(holdCommand, -m_maxSpeedDecimal, m_maxSpeedDecimal);
//       m_motor.setSpeed(holdCommand);
//       m_lastCommand = holdCommand;

//       Serial.print(F("Arrived! Holding torque applied (cmd="));
//       Serial.print(holdCommand, 3);
//       Serial.println(F(")"));
//     }

//     m_moving = false;
//     m_arrived = true;
//     return;
//   }

//   // **DESCENT MODE with handoff to PID**
//   if (m_isDescending) {
//     // Open-loop ramp for most of the descent
//     if (absError > 700) {  // More than ~18 inches away
//       float targetCommand = -0.05f;
      
//       // Ramp smoothly from holding torque
//       float commandDelta = targetCommand - m_lastCommand;
//       if (fabs(commandDelta) > 0.02f) {
//         m_lastCommand += copysignf(0.02f, commandDelta);
//       } else {
//         m_lastCommand = targetCommand;
//       }
      
//       m_motor.setSpeed(m_lastCommand);
      
//       // Debug print (every 50 ms)
//       static unsigned long lastPrintMs = 0;
//       if (millis() - lastPrintMs > 50) {
//         Serial.print(F("DESCENT(RAMP) ms=")); Serial.print(millis() - m_moveStartMs);
//         Serial.print(F(" pos=")); Serial.print(currentPos);
//         Serial.print(F(" tgt=")); Serial.print(m_targetCounts);
//         Serial.print(F(" err=")); Serial.print(error);
//         Serial.print(F(" cmd=")); Serial.print(m_lastCommand, 3);
//         Serial.print(F(" amps=")); Serial.println(m_motor.readCurrent(), 3);
//         lastPrintMs = millis();
//       }
      
//       return;  // Stay in open-loop mode
//     }
    
//     // **Within 700 counts (~18 inches): Fall through to PID control below**
//   }

//   // **NORMAL PID CONTROL** (for ascent OR final descent approach within 18")
//   m_pidInput = static_cast<double>(currentPos);
//   m_pidSetpoint = static_cast<double>(m_targetCounts);
//   m_pid.Compute();

//   // Apply asymmetric feed-forward
//   float ff = (error > 0) ? m_feedForwardUp : -m_feedForwardDown;
//   float command = static_cast<float>(m_pidOutput) + ff;
  
//   command = constrain(command, -m_maxSpeedDecimal, m_maxSpeedDecimal);
//   m_motor.setSpeed(command);
//   m_lastCommand = command;

//   // Debug print (every 50 ms)
//   static unsigned long lastPrintMs = 0;
//   if (millis() - lastPrintMs > 50) {
//     if (m_isDescending) {
//       Serial.print(F("DESCENT(PID) ms="));
//     } else {
//       Serial.print(F("ASCENT ms="));
//     }
//     Serial.print(millis() - m_moveStartMs);
//     Serial.print(F(" pos=")); Serial.print(currentPos);
//     Serial.print(F(" tgt=")); Serial.print(m_targetCounts);
//     Serial.print(F(" err=")); Serial.print(error);
//     Serial.print(F(" pid=")); Serial.print(m_pidOutput, 3);
//     Serial.print(F(" ff=")); Serial.print(ff, 3);
//     Serial.print(F(" cmd=")); Serial.print(command, 3);
//     Serial.print(F(" amps=")); Serial.println(m_motor.readCurrent(), 3);
//     lastPrintMs = millis();
//   }
// }

// void MotorControlPid::update() {
//   if (!m_moving) return;

//   // Timeout safety
//   if (millis() - m_moveStartMs > MOVE_TIMEOUT_MS) {
//     emergencyStop();
//     Serial.println(F("ERROR: Move timeout!"));
//     return;
//   }

//   long currentPos = m_encoder.getPositionCounts();
//   long error = m_targetCounts - currentPos;
//   long absError = labs(error);

//   // Check if arrived
//   if (absError <= POSITION_TOLERANCE_COUNTS) {
//     if (m_targetCounts == 0) {
//       // At the bottom, just stop. No holding torque needed.
//       m_motor.stop();
//       Serial.println(F("Arrived at bottom!"));
//     } else {
//       // Apply holding torque to counteract gravity/load.
//       float holdCommand = (error > 0) ? m_feedForwardUp : -m_feedForwardDown;
//       holdCommand = constrain(holdCommand, -m_maxSpeedDecimal, m_maxSpeedDecimal);
//       m_motor.setSpeed(holdCommand);
//       m_lastCommand = holdCommand;

//       Serial.print(F("Arrived! Holding torque applied (cmd="));
//       Serial.print(holdCommand, 3);
//       Serial.println(F(")"));
//     }

//     m_moving = false;
//     m_arrived = true;
//     return;
//   }

//   // **DESCENT MODE with smooth blending**
//   if (m_isDescending) {
//     // Zone 1: Open-loop ramp (above 700 counts / ~18 inches)
//     if (absError > 700) {
//       float targetCommand = -0.05f;
      
//       // Ramp smoothly from holding torque
//       float commandDelta = targetCommand - m_lastCommand;
//       if (fabs(commandDelta) > 0.02f) {
//         m_lastCommand += copysignf(0.02f, commandDelta);
//       } else {
//         m_lastCommand = targetCommand;
//       }
      
//       m_motor.setSpeed(m_lastCommand);
      
//       // Debug print (every 50 ms)
//       static unsigned long lastPrintMs = 0;
//       if (millis() - lastPrintMs > 50) {
//         Serial.print(F("DESCENT(RAMP) ms=")); Serial.print(millis() - m_moveStartMs);
//         Serial.print(F(" pos=")); Serial.print(currentPos);
//         Serial.print(F(" tgt=")); Serial.print(m_targetCounts);
//         Serial.print(F(" err=")); Serial.print(error);
//         Serial.print(F(" cmd=")); Serial.print(m_lastCommand, 3);
//         Serial.print(F(" amps=")); Serial.println(m_motor.readCurrent(), 3);
//         lastPrintMs = millis();
//       }
      
//       return;
//     }
    
//     // Zone 2: Blended transition (700-200 counts / ~18-5 inches)
//     if (absError > 200) {
//       // Compute PID
//       m_pidInput = static_cast<double>(currentPos);
//       m_pidSetpoint = static_cast<double>(m_targetCounts);
//       m_pid.Compute();
      
//       float ff = -m_feedForwardDown;
//       float pidCommand = static_cast<float>(m_pidOutput) + ff;
      
//       // Blend: at 700 counts use 100% open-loop, at 200 counts use 100% PID
//       float blendFactor = (absError - 200.0f) / 500.0f;  // 1.0 at 700, 0.0 at 200
//       float openLoopCommand = -0.05f;
      
//       float command = blendFactor * openLoopCommand + (1.0f - blendFactor) * pidCommand;
//       command = constrain(command, -m_maxSpeedDecimal, m_maxSpeedDecimal);
      
//       m_motor.setSpeed(command);
//       m_lastCommand = command;
      
//       // Debug print (every 50 ms)
//       static unsigned long lastPrintMs = 0;
//       if (millis() - lastPrintMs > 50) {
//         Serial.print(F("DESCENT(BLEND) ms=")); Serial.print(millis() - m_moveStartMs);
//         Serial.print(F(" pos=")); Serial.print(currentPos);
//         Serial.print(F(" blend=")); Serial.print(blendFactor, 2);
//         Serial.print(F(" pid=")); Serial.print(m_pidOutput, 3);
//         Serial.print(F(" ff=")); Serial.print(ff, 3);
//         Serial.print(F(" cmd=")); Serial.print(command, 3);
//         Serial.print(F(" amps=")); Serial.println(m_motor.readCurrent(), 3);
//         lastPrintMs = millis();
//       }
      
//       return;
//     }
    
//     // Zone 3: Full PID with scaled FF (below 200 counts / ~5 inches)
//     m_pidInput = static_cast<double>(currentPos);
//     m_pidSetpoint = static_cast<double>(m_targetCounts);
//     m_pid.Compute();
    
//     // Reduce FF near bottom to give PID more authority
//     float ffScale = absError / 200.0f;  // 1.0 at 200 counts, 0.0 at 0 counts
//     float ff = -m_feedForwardDown * ffScale;
    
//     float command = static_cast<float>(m_pidOutput) + ff;
//     command = constrain(command, -m_maxSpeedDecimal, m_maxSpeedDecimal);
    
//     m_motor.setSpeed(command);
//     m_lastCommand = command;
    
//     // Debug print (every 50 ms)
//     static unsigned long lastPrintMs = 0;
//     if (millis() - lastPrintMs > 50) {
//       Serial.print(F("DESCENT(FINAL) ms=")); Serial.print(millis() - m_moveStartMs);
//       Serial.print(F(" pos=")); Serial.print(currentPos);
//       Serial.print(F(" ffScale=")); Serial.print(ffScale, 2);
//       Serial.print(F(" pid=")); Serial.print(m_pidOutput, 3);
//       Serial.print(F(" ff=")); Serial.print(ff, 3);
//       Serial.print(F(" cmd=")); Serial.print(command, 3);
//       Serial.print(F(" amps=")); Serial.println(m_motor.readCurrent(), 3);
//       lastPrintMs = millis();
//     }
    
//     return;
//   }

//   // **NORMAL PID CONTROL for upward motion**
//   m_pidInput = static_cast<double>(currentPos);
//   m_pidSetpoint = static_cast<double>(m_targetCounts);
//   m_pid.Compute();

//   // Apply asymmetric feed-forward
//   float ff = (error > 0) ? m_feedForwardUp : -m_feedForwardDown;
//   float command = static_cast<float>(m_pidOutput) + ff;
  
//   command = constrain(command, -m_maxSpeedDecimal, m_maxSpeedDecimal);
//   m_motor.setSpeed(command);
//   m_lastCommand = command;

//   // Debug print (every 50 ms)
//   static unsigned long lastPrintMs = 0;
//   if (millis() - lastPrintMs > 50) {
//     Serial.print(F("ASCENT ms=")); Serial.print(millis() - m_moveStartMs);
//     Serial.print(F(" pos=")); Serial.print(currentPos);
//     Serial.print(F(" tgt=")); Serial.print(m_targetCounts);
//     Serial.print(F(" err=")); Serial.print(error);
//     Serial.print(F(" pid=")); Serial.print(m_pidOutput, 3);
//     Serial.print(F(" ff=")); Serial.print(ff, 3);
//     Serial.print(F(" cmd=")); Serial.print(command, 3);
//     Serial.print(F(" amps=")); Serial.println(m_motor.readCurrent(), 3);
//     lastPrintMs = millis();
//   }
// }

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
      float holdCommand = (error > 0) ? m_feedForwardUp : -m_feedForwardDown;
      holdCommand = constrain(holdCommand, -m_maxSpeedDecimal, m_maxSpeedDecimal);
      m_motor.setSpeed(holdCommand);
      m_lastCommand = holdCommand;

      Serial.print(F("Arrived! Holding torque applied (cmd="));
      Serial.print(holdCommand, 3);
      Serial.println(F(")"));
    }

    m_moving = false;
    m_arrived = true;
    return;
  }

  // **DESCENT MODE - Simple constant speed**
  if (m_isDescending) {
    // Simple: constant duty until close, then stop
    if (absError > 150) {  // More than ~1.3 inches away
      float command = -0.24f;  // 15% downward - adjust this to taste
      m_motor.setSpeed(command);
      
      // Debug
      static unsigned long lastPrintMs = 0;
      if (millis() - lastPrintMs > 50) {
        Serial.print(F("DESCENT ms=")); Serial.print(millis() - m_moveStartMs);
        Serial.print(F(" pos=")); Serial.print(currentPos);
        Serial.print(F(" err=")); Serial.print(error);
        Serial.print(F(" cmd=")); Serial.println(command, 3);
        lastPrintMs = millis();
      }
      
      return;
    }
    
    // Close to bottom - just stop
    m_motor.stop();
    Serial.println(F("Near bottom - coasting to stop"));
    
    // Debug
    static unsigned long lastPrintMs = 0;
    if (millis() - lastPrintMs > 50) {
      Serial.print(F("DESCENT(COAST) ms=")); Serial.print(millis() - m_moveStartMs);
      Serial.print(F(" pos=")); Serial.print(currentPos);
      Serial.print(F(" err=")); Serial.println(error);
      lastPrintMs = millis();
    }
    
    return;
  }

  // **NORMAL PID CONTROL for upward motion**
  m_pidInput = static_cast<double>(currentPos);
  m_pidSetpoint = static_cast<double>(m_targetCounts);
  m_pid.Compute();

  // Apply asymmetric feed-forward
  float ff = (error > 0) ? m_feedForwardUp : -m_feedForwardDown;
  float command = static_cast<float>(m_pidOutput) + ff;
  
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
    Serial.print(F(" ff=")); Serial.print(ff, 3);
    Serial.print(F(" cmd=")); Serial.print(command, 3);
    Serial.print(F(" amps=")); Serial.println(m_motor.readCurrent(), 3);
    lastPrintMs = millis();
  }
}