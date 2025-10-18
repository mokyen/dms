#include "MotorControlPid.h"

MotorControl::MotorControl(MotorDriver& driver, EncoderReader& enc)
  : motor(driver), encoder(enc), targetCounts(0), maxSpeedFraction(1.0f),
    moving(false), arrived(false), moveStartMs(0),
    pidInput(0), pidOutput(0), pidSetpoint(0)
{
  // Initialize PID
  // Note: PID library uses pointer to variables, so they must persist
  pid = new PID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);
  
  // Configure PID
  pid->SetMode(AUTOMATIC);
  pid->SetOutputLimits(-1.0, 1.0);  // Motor speed range
  pid->SetSampleTime(10);  // 10ms update rate (100Hz)
}

void MotorControl::begin() {
  motor.begin();
  encoder.begin();
  EncoderReader::attachInstance(&encoder);
}

void MotorControl::moveToCounts(long counts, int maxSpeedPercent) {
  // Clamp to safe range
  targetCounts = constrain(counts, 0L, MAX_TRAVEL_COUNTS);
  maxSpeedPercent = constrain(maxSpeedPercent, 1, 100);
  maxSpeedFraction = maxSpeedPercent / 100.0f;
  
  // Update PID setpoint
  pidSetpoint = (double)targetCounts;
  
  // Reset PID output limits based on max speed
  //   pid->SetOutputLimits(-maxSpeedFraction, maxSpeedFraction);
  pid->SetOutputLimits(-1.0, 1.0); // DEBUG ONLY - revert after verifying motor capability
  
  moving = true;
  arrived = false;
  moveStartMs = millis();
  
  Serial.print(F("Moving to: "));
  Serial.print(targetCounts);
  Serial.print(F(" counts @ "));
  Serial.print(maxSpeedPercent);
  Serial.println(F("% max speed"));
}

void MotorControl::stop() {
  motor.stop();
  moving = false;
  pid->SetMode(MANUAL);  // Disable PID when stopped
}

void MotorControl::emergencyStop() {
  motor.brake();
  moving = false;
  arrived = false;
  pid->SetMode(MANUAL);
}

void MotorControl::setPIDGains(double kp, double ki, double kd) {
  pid->SetTunings(kp, ki, kd);
  Serial.print(F("PID gains updated: Kp="));
  Serial.print(kp, 4);
  Serial.print(F(" Ki="));
  Serial.print(ki, 4);
  Serial.print(F(" Kd="));
  Serial.println(kd, 4);
}

void MotorControl::update() {
  if (!moving) return;
  
  // Timeout check
  if (millis() - moveStartMs > MOVE_TIMEOUT_MS) {
    emergencyStop();
    Serial.println(F("ERROR: Move timeout!"));
    return;
  }
  
  // Get current position
  long currentPos = encoder.getPositionCounts();
  long error = targetCounts - currentPos;
  long absError = labs(error);
  
  // Debug output (throttled)
  static unsigned long lastDebugMs = 0;
  if (millis() - lastDebugMs > 500) {
    Serial.print(F("pos="));
    Serial.print(currentPos);
    Serial.print(F(" tgt="));
    Serial.print(targetCounts);
    Serial.print(F(" err="));
    Serial.print(error);
    Serial.print(F(" out="));
    Serial.println(pidOutput, 3);
    lastDebugMs = millis();
  }
  
  // Check if we've arrived BEFORE computing PID
  // This prevents integral windup at the target
  if (absError <= POSITION_TOLERANCE_COUNTS) {
    motor.brake();
    moving = false;
    arrived = true;
    pid->SetMode(MANUAL);  // Disable PID
    
    Serial.print(F("Arrived! Final error: "));
    Serial.print(error);
    Serial.println(F(" counts"));
    return;
  }
  
  // Update PID
  pidInput = (double)currentPos;
  pid->SetMode(AUTOMATIC);  // Ensure PID is active
  pid->Compute();
  
  // Apply control output
  motor.setSpeed((float)pidOutput);
}