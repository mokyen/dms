// #include "MotorControl.h"
// #include <PID_v1.h>

// MotorControl::MotorControl(MotorDriver& driver, EncoderReader& enc)
//   : motor(driver), encoder(enc), targetCounts(0), maxSpeedFraction(1.0f),
//     moving(false), arrived(false), moveStartMs(0) {}

// void MotorControl::begin() {
//   motor.begin();
//   encoder.begin();
//   EncoderReader::attachInstance(&encoder);
// }

// void MotorControl::moveToCounts(long counts, int maxSpeedPercent) {
//   // Clamp to safe range
//   targetCounts = constrain(counts, 0L, MAX_TRAVEL_COUNTS);
//   maxSpeedPercent = constrain(maxSpeedPercent, 1, 100);
//   maxSpeedFraction = maxSpeedPercent / 100.0f;
  
//   moving = true;
//   arrived = false;
//   moveStartMs = millis();
  
//   Serial.print(F("Moving to: "));
//   Serial.print(targetCounts);
//   Serial.print(F(" counts @ "));
//   Serial.print(maxSpeedPercent);
//   Serial.println(F("% max speed"));
// }

// void MotorControl::stop() {
//   motor.stop();
//   moving = false;
// }

// void MotorControl::emergencyStop() {
//   motor.brake();
//   moving = false;
//   arrived = false;
// }

// float MotorControl::computeControlOutput(long error) {
//   // Current: Bang-bang with deceleration zone
//   // TODO: Replace with PID when ready
  
//   long absError = labs(error);
  
//   // Check if arrived
//   if (absError <= POSITION_TOLERANCE_COUNTS) {
//     return 0.0f; // Will trigger brake in update()
//   }
  
//   // Deceleration zone - scale with max speed
//   // At 45% max speed, we need a shorter decel distance
//   const long BASE_DECEL_DISTANCE = (long)(18.0f * COUNTS_PER_IN);
//   long decelDistance = (long)(BASE_DECEL_DISTANCE * maxSpeedFraction);
  
//   // Ensure minimum decel distance
//   if (decelDistance < (long)(3.0f * COUNTS_PER_IN)) {
//     decelDistance = (long)(3.0f * COUNTS_PER_IN);
//   }
  
//   constexpr float MIN_SPEED_FRACTION = 0.15f;
  
//   float speed;
//   if (absError > decelDistance) {
//     // Full speed (scaled by maxSpeedFraction)
//     speed = maxSpeedFraction;
//   } else {
//     // Linear ramp down in deceleration zone
//     float fraction = (float)absError / (float)decelDistance;
//     speed = MIN_SPEED_FRACTION + fraction * (maxSpeedFraction - MIN_SPEED_FRACTION);
//   }
  
//   // Apply direction
//   return (error > 0) ? speed : -speed;
// }

// void MotorControl::update() {
//   if (!moving) return;
  
//   // Timeout check
//   if (millis() - moveStartMs > MOVE_TIMEOUT_MS) {
//     emergencyStop();
//     Serial.println(F("ERROR: Move timeout!"));
//     return;
//   }
  
//   // Get current error
//   long currentPos = encoder.getPositionCounts();
//   long error = targetCounts - currentPos;
//   long absError = labs(error);
  
//   // Debug output (throttled)
//   static unsigned long lastDebugMs = 0;
//   if (millis() - lastDebugMs > 500) {
//     Serial.print(F("pos="));
//     Serial.print(currentPos);
//     Serial.print(F(" tgt="));
//     Serial.print(targetCounts);
//     Serial.print(F(" err="));
//     Serial.println(error);
//     lastDebugMs = millis();
//   }
  
//   // Compute control output
//   float output = computeControlOutput(error);
  
//   // Check if we've arrived
//   if (absError <= POSITION_TOLERANCE_COUNTS) {
//     motor.brake();
//     moving = false;
//     arrived = true;
    
//     Serial.print(F("Arrived! Final error: "));
//     Serial.print(error);
//     Serial.println(F(" counts"));
//     return;
//   }
  
//   // Apply control
//   motor.setSpeed(output);
// }