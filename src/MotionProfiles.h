#pragma once
#include <Arduino.h>
#include "MotorDriver.h"
#include "EncoderReader.h"
#include "Config.h"

namespace MotionProfiles {

// Deceleration parameters - tune these based on your system
constexpr long DECEL_START_COUNTS = (long)(18.0f * COUNTS_PER_IN);  // Start slowing at 18 inches
constexpr float MIN_SPEED_FRACTION = 0.15f;  // Minimum speed in decel zone

inline void moveToPositionCounts(MotorDriver& motor, EncoderReader& encoder, long targetCounts, float maxSpeedFraction) {
  if (targetCounts < 0L) targetCounts = 0L;
  if (targetCounts > MAX_TRAVEL_COUNTS) targetCounts = MAX_TRAVEL_COUNTS;

  long currentCounts = encoder.getPositionCounts();
  long error = targetCounts - currentCounts;
  
  // DEBUG OUTPUT
  static long lastDebugMs = 0;
  if (millis() - lastDebugMs > 500) {
    Serial.print(F("DEBUG: cur="));
    Serial.print(currentCounts);
    Serial.print(F(" tgt="));
    Serial.print(targetCounts);
    Serial.print(F(" err="));
    Serial.println(error);
    lastDebugMs = millis();
  }
  
  long absError = labs(error);
  
  if (absError <= POSITION_TOLERANCE_COUNTS) {
    Serial.print(F("DEBUG: Within tolerance! labs(error)="));
    Serial.print(absError);
    Serial.print(F(" TOLERANCE="));
    Serial.println(POSITION_TOLERANCE_COUNTS);
    motor.brake();
    return;
  }

  // Calculate speed based on distance to target
  float speed;
  if (absError > DECEL_START_COUNTS) {
    // Full speed when far away
    speed = maxSpeedFraction;
  } else {
    // Linear ramp down in deceleration zone
    float fraction = (float)absError / (float)DECEL_START_COUNTS;
    speed = MIN_SPEED_FRACTION + fraction * (maxSpeedFraction - MIN_SPEED_FRACTION);
  }

  float direction = (error > 0) ? 1.0f : -1.0f;
  motor.setSpeed(speed * direction);
}

// Stop when near bottom (zero counts)
inline void stopAtBottom(MotorDriver& motor, EncoderReader& encoder) {
  if (encoder.getPositionCounts() <= POSITION_TOLERANCE_COUNTS)
    motor.stop();
}

// Stop when near top (MAX_TRAVEL_COUNTS)
inline void stopAtTop(MotorDriver& motor, EncoderReader& encoder) {
  if (encoder.getPositionCounts() >= MAX_TRAVEL_COUNTS - POSITION_TOLERANCE_COUNTS)
    motor.stop();
}

} // namespace MotionProfiles